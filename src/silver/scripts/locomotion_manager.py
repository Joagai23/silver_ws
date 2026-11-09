#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unified ROS 2 Locomotion Manager for SILVER2.
Supports: 'cpg' (NumPy Hopf CPG), 'odg' (Omnidirectional Gait)
"""

import os
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory

try:
    SCRIPTS_DIR = os.path.join(get_package_share_directory('silver'), 'scripts')
except Exception:
    SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))

if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from controllers.numpy_cpg_controller import NumpyHexapodCPGController
from controllers.numpy_odg_controller import NumpyOmnidirectionalGaitController
from silver2_constants.isaac_constants import (
    SILVER2_MOUNTS,
    SILVER2_LINKS,
    SILVER2_STANDING_ANGLES_DEG,
)

class SilverLocomotionManager(Node):
    def __init__(self):
        super().__init__('silver_locomotion_manager')

        # --- Declare ROS 2 Parameters ---
        self.declare_parameter('controller_type', 'cpg')        # 'cpg', 'odg'
        self.declare_parameter('gait_type', 'tripod')           # 'tripod', 'tetrapod', 'wave'
        self.declare_parameter('control_rate', 50.0)            # Hz (dt = 0.02s)
        self.declare_parameter('gait_period', 2.0)              # Seconds
        self.declare_parameter('step_height', 0.05)             # Meters
        self.declare_parameter('startup_pose', True)            # Smoothly interpolate to standing pose on boot

        self.controller_type = self.get_parameter('controller_type').value
        self.gait_type = self.get_parameter('gait_type').value
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.dt = 1.0 / self.control_rate
        self.gait_period = float(self.get_parameter('gait_period').value)
        self.step_height = float(self.get_parameter('step_height').value)

        # --- State Tracking ---
        self.joint_order = [
            'coxa_joint_0', 'femur_joint_0', 'tibia_joint_0',
            'coxa_joint_1', 'femur_joint_1', 'tibia_joint_1',
            'coxa_joint_2', 'femur_joint_2', 'tibia_joint_2',
            'coxa_joint_3', 'femur_joint_3', 'tibia_joint_3',
            'coxa_joint_4', 'femur_joint_4', 'tibia_joint_4',
            'coxa_joint_5', 'femur_joint_5', 'tibia_joint_5',
        ]
        self.q_current = np.zeros(18, dtype=np.float64)
        self.joint_states_received = False
        self.isaac_robot_signs = np.array([-1.0, -1.0, 1.0], dtype=np.float64)

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.t_elapsed = 0.0
        self.ramp = 0.0

        # --- Publishers & Subscribers ---
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/pid_position_controller/commands', 10
        )
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10
        )
        self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10
        )

        # --- Initialize Selected Controller Backend ---
        self._init_controller(self.controller_type)

        # Optional sequential startup transition to standing pose
        if self.get_parameter('startup_pose').value:
            self.move_to_standing_pose_slow()

        # --- Start Control Timer ---
        self.timer = self.create_timer(self.dt, self._control_loop)
        self.get_logger().info(
            f"Locomotion Manager ready | Controller: '{self.controller_type}' | "
            f"Gait: '{self.gait_type}' | Rate: {self.control_rate} Hz"
        )

    def _init_controller(self, mode: str):
        mode = mode.strip().lower()
        self.get_logger().info(f"Loading locomotion backend: {mode}...")

        if mode == 'cpg':
            self.controller = NumpyHexapodCPGController(
                leg_mounts=SILVER2_MOUNTS,
                link_lengths=SILVER2_LINKS,
                dt=self.dt,
                total_period=self.gait_period,
                gait=self.gait_type,
            )
            self.default_feet_body = self.controller.compute_default_feet_body(
                SILVER2_STANDING_ANGLES_DEG, SILVER2_MOUNTS, SILVER2_LINKS
            )

        elif mode == 'odg':
            self.controller = NumpyOmnidirectionalGaitController(
                gait_type=self.gait_type,
                step_height=self.step_height,
                gait_period=self.gait_period,
            )
            robot_standing_deg = SILVER2_STANDING_ANGLES_DEG * self.isaac_robot_signs
            self.controller.default_feet_body = self.controller.compute_default_feet_body(
                robot_standing_deg
            )

        else:
            raise ValueError(
                f"Unknown controller_type '{mode}'. Use 'cpg' or 'odg'."
            )

        self.controller_type = mode
        self.t_elapsed = 0.0
        self.ramp = 0.0

    # --- ROS Callbacks ---
    def _joint_state_cb(self, msg: JointState):
        pos_map = dict(zip(msg.name, msg.position))
        for i, name in enumerate(self.joint_order):
            if name in pos_map:
                self.q_current[i] = pos_map[name]
        self.joint_states_received = True

    def _cmd_vel_cb(self, msg: Twist):
        self.vx = float(msg.linear.x)
        self.vy = float(msg.linear.y)
        self.wz = float(msg.angular.z)

    # --- Safe Startup Pose Routine ---
    def move_to_standing_pose_slow(self):
        self.get_logger().info("Waiting for /joint_states before moving to standing pose...")
        while rclpy.ok() and not self.joint_states_received:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not rclpy.ok():
            return

        q_tracker = np.copy(self.q_current)
        q_target_deg = SILVER2_STANDING_ANGLES_DEG * self.isaac_robot_signs
        q_target = np.deg2rad(q_target_deg.flatten())

        for leg_id in range(6):
            self.get_logger().info(f"Interpolating Leg {leg_id} to standing configuration...")
            q_start = np.copy(q_tracker)
            steps = 30
            idx_s, idx_e = 3 * leg_id, 3 * leg_id + 3

            for s in range(steps):
                alpha = (s + 1) / steps
                q_interp = np.copy(q_start)
                q_interp[idx_s:idx_e] = (1.0 - alpha) * q_start[idx_s:idx_e] + alpha * q_target[idx_s:idx_e]
                self._publish_joints(q_interp)
                time.sleep(1.5 / steps)

            q_tracker = np.copy(q_interp)
            time.sleep(0.3)

        self.get_logger().info("Standing pose reached. Listening for /cmd_vel...")

    # --- Main Real-Time Control Loop ---
    def _control_loop(self):
        lin_mag = float(np.hypot(self.vx, self.vy))
        is_moving = (lin_mag > 1e-3) or (abs(self.wz) > 1e-3)

        if not is_moving:
            self.ramp = max(0.0, self.ramp - self.dt * 2.0)
            if self.ramp == 0.0:
                return
        else:
            self.ramp = min(1.0, self.ramp + self.dt * 1.5)

        dir_angle_rad = float(np.arctan2(self.vy, self.vx)) if lin_mag > 1e-3 else 0.0
        stride_fwd = 0.010 * np.clip(lin_mag, 0.0, 1.0)
        stride_lat = 0.010 * np.clip(lin_mag, 0.0, 1.0)

        # 1. NumPy CPG Backend
        if self.controller_type == 'cpg':
            q_targets_isaac = self.controller.compute_joint_targets(
                self.default_feet_body,
                dir_angle_rad=dir_angle_rad,
                stride_forward=stride_fwd,
                stride_lateral=stride_lat,
                yaw_rate=self.wz,
                yaw_gain=0.005,
                ramp=self.ramp,
            )
            q_targets_robot = q_targets_isaac * self.isaac_robot_signs
            self._publish_joints(q_targets_robot.flatten())

        # 2. NumPy Omnidirectional Backend
        elif self.controller_type == 'odg':
            feet_body = self.controller.compute_foot_trajectories(
                t=self.t_elapsed,
                vx=self.vx * self.ramp,
                vy=self.vy * self.ramp,
                wz=self.wz * self.ramp,
            )
            feet_local = self.controller.body_to_local_frame(feet_body)
            q_targets_robot, admissible = self.controller.batch_leg_inv_kine(feet_local)

            if bool(np.all(admissible)):
                self._publish_joints(np.asarray(q_targets_robot).flatten())
            else:
                self.get_logger().warn("Non-admissible trajectory step in odg", throttle_duration_sec=1.0)

        self.t_elapsed += self.dt

    def _publish_joints(self, q_flat: np.ndarray):
        msg = Float64MultiArray()
        msg.data = [float(x) for x in q_flat]
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SilverLocomotionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()