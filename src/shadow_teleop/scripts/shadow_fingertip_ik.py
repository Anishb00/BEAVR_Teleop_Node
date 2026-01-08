#!/usr/bin/python3
"""
Shadow Hand Fingertip IK Solver with Unity-to-ROS Coordinate Transformation.

This IK solver converts 5 VR fingertip positions in wrist-frame coordinate space
into 24 robot joint angles for the Shadow Dexterous Hand.

This node utilizes a Numerical IK solver with 2 primary components:

- Pinocchio: Forward Kinematics (FK) - computes fingertip positions from joint angles
- nlopt (SLSQP): Gradient-based optimizer - iteratively refines joint angles to minimize position error

The solver works by:
1. nlopt proposes candidate joint angles
2. Pinocchio computes where fingertips would be at those angles (FK)
3. Error is calculated between actual and target fingertip positions
4. nlopt uses the gradient to propose better joint angles
5. Repeat until error is minimized

Todo:
There have been some over-optimizations to this solution in order to achieve better teleoperation results. As a result,
this solution works best with my hand and the Shadow Robot. Future changes will be made to ensure a more generalized
fit for different hand shapes. Additionally, this system will be made more compatible with various robots, as our fingertip
keypoints + IK solver allow for a plug-and-play solution that can work with robot end effectors of varying DoF.
"""

import rospy
import numpy as np
import json
import threading
from collections import deque

import pinocchio as pin
import nlopt

from sensor_msgs.msg import JointState
from std_msgs.msg import String



from pathlib import Path
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent.parent.parent  # scripts -> shadow_teleop -> src -> BEAVR_Teleop_Node
URDF_PATH = str(PROJECT_ROOT / "shadowhand.urdf")

FINGERTIP_FRAMES = {
    "thumb":  "rh_thtip",
    "index":  "rh_FFtip",
    "middle": "rh_MFtip",
    "ring":   "rh_RFtip",
    "little": "rh_LFtip",
}

WRIST_FRAME = "rh_wrist"
FINGER_ORDER = ["thumb", "index", "middle", "ring", "little"]

# Per-finger scale factors derived from calibration (Shadow / Unity finger length ratio)
# These account for the Shadow Hand having longer fingers than the human hand
DEFAULT_SCALE_FACTORS = {
    "thumb":  1.20,
    "index":  1.22,
    "middle": 1.17,
    "ring":   1.22,
    "little": 1.35,
}

class CoordinateTransformer:
    """
    Handles Unity wrist-local to Shadow Hand wrist-local coordinate transformation.

    The transformation was derived empirically by comparing VR hand tracking data
    to Shadow Hand neutral pose fingertip positions.

    Unity XRHands Wrist-Local Frame (right hand):

      +Z: Points toward fingers
      +X: Points toward thumb
      +Y: Points up from back of hand

    Shadow Hand Wrist-Local Frame:
      +Z: Points toward fingers
      +X: Points toward pinky
      +Y: Points up from back of hand


    Calibration Observations:
    - Unity X and Shadow X have OPPOSITE signs (thumb direction flipped)
    - Unity Y and Shadow Y have OPPOSITE signs (small values, but flipped)
    - Unity Z and Shadow Z have SAME sign (finger pointing direction matches)
    """

    def __init__(self, transform_mode="calibrated", scale_factors=None):
        self.transform_mode = transform_mode
        self.scale_factors = scale_factors if scale_factors else {f: 1.0 for f in FINGER_ORDER}

    def transform_fingertip(self, pos_unity, finger_name):
        """Transform fingertip position from Unity wrist frame to Shadow wrist frame."""
        ux, uy, uz = pos_unity
        sf = self.scale_factors.get(finger_name, 1.0)

        if self.transform_mode == "calibrated":
            # Best transformation based on calibration: flip X, zero Y, keep Z
            return np.array([-ux * sf, 0.0, uz * sf])

        elif self.transform_mode == "calibrated_keepY":
            # Alternative: flip X, flip Y, keep Z
            return np.array([-ux * sf, -uy * sf, uz * sf])

        elif self.transform_mode == "none":
            return np.array(pos_unity) * sf

        else:
            # Default to calibrated
            return np.array([-ux * sf, 0.0, uz * sf])

    def transform_all_fingertips(self, fingertips_unity):
        """Transform all fingertip positions."""
        return {
            finger: self.transform_fingertip(np.array(pos), finger)
            for finger, pos in fingertips_unity.items()
        }

class TemporalSmoother:
    """Moving average filter to reduce jitter in fingertip tracking."""

    def __init__(self, window_size=3):
        self.window_size = window_size
        self.buffers = {}

    def smooth(self, fingertips):
        """Apply moving average to each fingertip position."""
        smoothed = {}
        for finger, pos in fingertips.items():
            if finger not in self.buffers:
                self.buffers[finger] = deque(maxlen=self.window_size)
            self.buffers[finger].append(pos.copy())
            smoothed[finger] = np.mean(self.buffers[finger], axis=0)
        return smoothed


class FingertipIKSolver:
    """
    Numerical IK solver using nlopt optimization and Pinocchio FK.

    Finds joint angles that place fingertips at target positions by
    iteratively minimizing position error.
    """

    def __init__(self, urdf_path=URDF_PATH):
        # Load robot model
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        # Get frame IDs
        self.wrist_id = self.model.getFrameId(WRIST_FRAME)
        self.tip_ids = {
            finger: self.model.getFrameId(frame, pin.FrameType.FIXED_JOINT)
            for finger, frame in FINGERTIP_FRAMES.items()
        }

        # Joint limits
        self.q_min = self.model.lowerPositionLimit.copy()
        self.q_max = self.model.upperPositionLimit.copy()
        self.nq = self.model.nq

        # Configure optimizer
        self.opt = nlopt.opt(nlopt.LD_SLSQP, self.nq)
        self.opt.set_lower_bounds(self.q_min.tolist())
        self.opt.set_upper_bounds(self.q_max.tolist())
        self.opt.set_min_objective(self._objective_wrapper)
        self.opt.set_xtol_rel(1e-4)
        self.opt.set_maxeval(100)

        # Solver state
        self._targets = None
        self._prev_q = None
        self.warm_start_weight = 0.01

        rospy.loginfo(f"IK Solver initialized with {self.nq} DOF")

    def get_fingertips_in_wrist_frame(self, q):
        """Compute fingertip positions in wrist frame using Pinocchio FK."""
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        oMw = self.data.oMf[self.wrist_id]
        p_wrist = oMw.translation
        R_wrist = oMw.rotation

        tips = {}
        for finger, tip_id in self.tip_ids.items():
            oMt = self.data.oMf[tip_id]
            p_tip_world = oMt.translation
            p_tip_wrist = R_wrist.T @ (p_tip_world - p_wrist)
            tips[finger] = p_tip_wrist.copy()
        return tips

    def _objective(self, q):
        """Compute optimization objective: position error + regularization."""
        tips = self.get_fingertips_in_wrist_frame(q)

        # Sum of squared position errors
        error = 0.0
        for finger in FINGER_ORDER:
            if finger in self._targets:
                diff = tips[finger] - self._targets[finger]
                error += np.dot(diff, diff)

        # Warm start regularization (smooth motion)
        if self._prev_q is not None:
            dq = q - self._prev_q
            error += self.warm_start_weight * np.dot(dq, dq)

        return error

    def _gradient(self, q):
        """Compute gradient using finite differences."""
        grad = np.zeros(self.nq)
        eps = 1e-6
        f0 = self._objective(q)

        for i in range(self.nq):
            q_plus = q.copy()
            q_plus[i] += eps
            q_plus = np.clip(q_plus, self.q_min, self.q_max)
            grad[i] = (self._objective(q_plus) - f0) / eps

        return grad

    def _objective_wrapper(self, q, grad):
        """Wrapper for nlopt callback interface."""
        q = np.array(q)
        obj = self._objective(q)
        if grad.size > 0:
            grad[:] = self._gradient(q)
        return obj

    def solve(self, targets, init_q=None):
        """
        Solve IK for given fingertip targets.

        Args:
            targets: dict mapping finger name to 3D position (wrist-relative)
            init_q: initial joint configuration (warm start)

        Returns:
            q_sol: optimized joint configuration
        """
        self._targets = {f: np.asarray(p, dtype=float) for f, p in targets.items()}

        if init_q is None:
            init_q = pin.neutral(self.model)
        init_q = np.clip(np.asarray(init_q), self.q_min, self.q_max)

        try:
            q_sol = np.array(self.opt.optimize(init_q.tolist()))
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"nlopt: {e}")
            q_sol = init_q

        self._prev_q = q_sol.copy()
        return q_sol

    def q_to_jointstate(self, q):
        """Convert joint angle vector to ROS JointState message."""
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        names, positions = [], []

        for i in range(1, self.model.njoints):
            joint_name = self.model.names[i]
            idx = self.model.idx_qs[i]
            nq = self.model.nqs[i]
            if nq == 1:
                names.append(joint_name)
                positions.append(float(q[idx]))
            else:
                for k in range(nq):
                    names.append(f"{joint_name}_{k}")
                    positions.append(float(q[idx + k]))

        msg.name = names
        msg.position = positions
        return msg

class ShadowHandIKNode:
    """ROS node that receives VR hand data and publishes Shadow Hand joint commands."""

    def __init__(self):
        # Transform mode: "calibrated" (zero Y) or "calibrated_keepY" (flip Y)
        transform_mode = rospy.get_param("~transform_mode", "calibrated")

        # Load scale factors from ROS params (with calibrated defaults)
        scale_factors = {
            finger: rospy.get_param(f"~scale_{finger}", default)
            for finger, default in DEFAULT_SCALE_FACTORS.items()
        }

        # Initialize components
        self.transformer = CoordinateTransformer(transform_mode, scale_factors)
        self.smoother = TemporalSmoother(window_size=3)
        self.solver = FingertipIKSolver()

        # ROS pub/sub
        self.pub = rospy.Publisher("/shadow_ik/q_sol", JointState, queue_size=1)
        rospy.Subscriber("/hand_data", String, self._hand_data_callback, queue_size=1)

        # Thread-safe target storage
        self.latest_targets = None
        self.targets_lock = threading.Lock()
        self.prev_q = None

        rospy.loginfo(f"Node initialized: transform={transform_mode}, scales={scale_factors}")

    def _hand_data_callback(self, msg):
        """Parse incoming VR hand data."""
        try:
            data = json.loads(msg.data)
            fingertips = data['observation.state']['fingertips']
            targets = {f: np.array(p) for f, p in fingertips.items()}
            with self.targets_lock:
                self.latest_targets = targets
        except (json.JSONDecodeError, KeyError) as e:
            rospy.logwarn_throttle(1.0, f"Parse error: {e}")

    def _get_targets(self):
        """Thread-safe getter for latest targets."""
        with self.targets_lock:
            if self.latest_targets is None:
                return None
            return {k: v.copy() for k, v in self.latest_targets.items()}

    def run(self, rate_hz=30):
        """Main loop: receive VR data, solve IK, publish joint commands."""
        rate = rospy.Rate(rate_hz)

        rospy.loginfo("Waiting for /hand_data...")
        while not rospy.is_shutdown():
            if self._get_targets() is not None:
                break
            rate.sleep()

        rospy.loginfo(f"Starting IK loop at {rate_hz} Hz")

        while not rospy.is_shutdown():
            targets_unity = self._get_targets()
            if targets_unity is None:
                rate.sleep()
                continue

            # Transform: Unity frame -> Shadow frame
            targets_shadow = self.transformer.transform_all_fingertips(targets_unity)

            # Smooth to reduce jitter
            targets_smooth = self.smoother.smooth(targets_shadow)

            # Solve IK
            q_sol = self.solver.solve(targets_smooth, init_q=self.prev_q)
            self.prev_q = q_sol

            # Publish
            self.pub.publish(self.solver.q_to_jointstate(q_sol))

            rate.sleep()

def main():
    rospy.init_node('shadow_unity_ik')
    node = ShadowHandIKNode()
    try:
        node.run(rate_hz=30)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()