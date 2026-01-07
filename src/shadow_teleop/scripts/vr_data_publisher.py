#!/usr/bin/env python3
import zmq
import sys
import os
import rospy
from std_msgs.msg import String
from datetime import datetime

def main():
    rospy.init_node('hand_data_zmq_to_ros', anonymous=True)

    pub = rospy.Publisher('/hand_data', String, queue_size=10)

    address = rospy.get_param('~zmq_address', "tcp://192.168.68.70:5555")
    if len(sys.argv) > 1:
        address = sys.argv[1]

    rospy.loginfo(f"Connecting to ZMQ: {address}")

    # Create logs directory if it doesn't exist
    logs_dir = "logs"
    if not os.path.exists(logs_dir):
        os.makedirs(logs_dir)

    # Store VR data in log files
    log_file = os.path.join(logs_dir, f"hand_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")

    context = zmq.Context()
    sub_socket = context.socket(zmq.SUB)
    sub_socket.setsockopt(zmq.RCVHWM, 1000)
    sub_socket.connect(address)
    sub_socket.subscribe(b"")

    rospy.loginfo("ZMQ Subscriber connected. Publishing to ROS topic '/hand_data'...")
    rospy.loginfo(f"Logging to: {log_file}")

    message_count = 0

    try:
        with open(log_file, 'a') as file_writer:
            while not rospy.is_shutdown():
                if sub_socket.poll(timeout=100):
                    message = sub_socket.recv_string()
                    message_count += 1

                    rospy.loginfo(f"[{message_count}] {message}")

                    pub.publish(message)

                    timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                    file_writer.write(f"{timestamp}|{message}\n")
                    file_writer.flush()

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")
    finally:
        sub_socket.close()
        context.term()

if __name__ == "__main__":
    main()