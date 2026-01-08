"""
This ROS Node is responsible for receiving the target joint angle values from
the IK solver node. It then commands the Shadow Robot hand to move by publishing
those joint angles over the following topics:

    - /rh_trajectory_controller/command (Finger Joints)
    - /rh_wr_trajectory_controller/command (Wrist Joints)
"""

#!/usr/bin/python3

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


HAND_JOINTS = [
    'rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4',
    'rh_LFJ1', 'rh_LFJ2', 'rh_LFJ3', 'rh_LFJ4', 'rh_LFJ5',
    'rh_MFJ1', 'rh_MFJ2', 'rh_MFJ3', 'rh_MFJ4',
    'rh_RFJ1', 'rh_RFJ2', 'rh_RFJ3', 'rh_RFJ4',
    'rh_THJ1', 'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5',
]

WRIST_JOINTS = ['rh_WRJ1', 'rh_WRJ2']


class ShadowHandController:
    def __init__(self):
        self.hand_pub = rospy.Publisher('/rh_trajectory_controller/command', JointTrajectory, queue_size=1)
        self.wrist_pub = rospy.Publisher('/rh_wr_trajectory_controller/command', JointTrajectory, queue_size=1)
        rospy.Subscriber('/shadow_ik/q_sol', JointState, self.callback)

    def callback(self, msg):
        joint_angles = {name: pos for name, pos in zip(msg.name, msg.position)}

        # Hand trajectory
        hand_traj = JointTrajectory()
        hand_traj.joint_names = []
        hand_positions = []
        for joint in HAND_JOINTS:
            if joint in joint_angles:
                hand_traj.joint_names.append(joint)
                hand_positions.append(joint_angles[joint])

        if hand_traj.joint_names:
            point = JointTrajectoryPoint()
            point.positions = hand_positions
            point.time_from_start = rospy.Duration(0.1)
            hand_traj.points = [point]
            self.hand_pub.publish(hand_traj)

        # Wrist trajectory
        wrist_traj = JointTrajectory()
        wrist_traj.joint_names = []
        wrist_positions = []
        for joint in WRIST_JOINTS:
            if joint in joint_angles:
                wrist_traj.joint_names.append(joint)
                wrist_positions.append(joint_angles[joint])

        if wrist_traj.joint_names:
            point = JointTrajectoryPoint()
            point.positions = wrist_positions
            point.time_from_start = rospy.Duration(0.1)
            wrist_traj.points = [point]
            self.wrist_pub.publish(wrist_traj)


def main():
    rospy.init_node('shadow_hand_controller')

    controller = ShadowHandController()

    rospy.sleep(0.5)

    rospy.loginfo("Shadow hand controller ready, listening to /shadow_ik/q_sol")

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Shutting down shadow_hand_controller node")


if __name__ == '__main__':
    main()