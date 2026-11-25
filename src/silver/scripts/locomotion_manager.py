#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unified ROS 2 Locomotion Manager for SILVER2.
Supports: 'cpg' (NumPy Hopf CPG), 'odg' (Omnidirectional Gait),
and synchronized named pose transitions on '/target_pose'.
"""

import math
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
    SILVER2_DRAGON_POSE_DEG,
    SILVER2_LOW_TORQUE_POSE_DEG,
    SILVER2_FOLDED_POSE_DEG
)


class SilverLocomotionManager(Node):
    def __init__(self):
        super().__init__('silver_locomotion_manager')

        # --- Declare ROS 2 Parameters ---
        self.declare_parameter('controller_type', 'cpg')       # 'cpg', 'odg'
        self.declare_parameter('gait_type', 'tripod')          # 'tripod', 'tetrapod', 'wave'
        self.declare_parameter('control_rate', 50.0)           # Hz (dt = 0.02s)
        self.declare_parameter('startup_pose', True)           # Move to 'dragon' on boot
        self.declare_parameter('pose_interp_duration', 3.0)    # Seconds for synchronized pose transition

        # --- Declare Locomotion Parameters ---
        self.declare_parameter('gait_period', 2.0)             # Seconds
        self.declare_parameter('step_height', 0.05)            # Meters

        self.controller_type = self.get_parameter('controller_type').value
        self.gait_type = self.get_parameter('gait_type').value
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.dt = 1.0 / self.control_rate
        self.gait_period = float(self.get_parameter('gait_period').value)
        self.step_height = float(self.get_parameter('step_height').value)
        self.pose_interp_duration = float(self.get_parameter('pose_interp_duration').value)

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

        # --- Predefined Poses (18 joint angles in radians, robot sign convention) ---
        dragon_rad = np.deg2rad(
            (SILVER2_DRAGON_POSE_DEG * self.isaac_robot_signs).flatten()
        )
        low_torque_rad = np.deg2rad(
            (SILVER2_LOW_TORQUE_POSE_DEG * self.isaac_robot_signs).flatten()
        )
        folded_rad = np.deg2rad(
            (SILVER2_FOLDED_POSE_DEG * self.isaac_robot_signs).flatten()
        )
        self.predefined_poses = {
            'dragon': dragon_rad,
            'low_torque': low_torque_rad,
            'folded': folded_rad,
        }

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.t_elapsed = 0.0
        self.ramp = 0.0

        # --- Non-Blocking Synchronized Pose Interpolation State ---
        self.pose_interp_active = False
        self.pose_active_name = ''
        self.pose_q_start = None
        self.pose_q_target = None
        self.pose_step = 0

        # --- Publishers & Subscribers ---
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/pid_position_controller/commands', 10
        )
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10
        )
        self.create_subscription(
            String, '/target_pose', self._target_pose_cb, 10
        )
        self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10
        )

        # --- Initialize Selected Controller Backend ---
        self._init_controller(self.controller_type)

        # Optional synchronized startup transition to 'dragon'
        if self.get_parameter('startup_pose').value:
            self._move_to_pose_blocking('dragon')

        # --- Start Control Timer ---
        self.timer = self.create_timer(self.dt, self._control_loop)
        available = ', '.join(self.predefined_poses.keys())
        self.get_logger().info(
            f"Locomotion Manager ready | Controller: '{self.controller_type}' | "
            f"Gait: '{self.gait_type}' | Rate: {self.control_rate} Hz | "
            f"Poses on /target_pose: [{available}]"
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
                SILVER2_DRAGON_POSE_DEG, SILVER2_MOUNTS, SILVER2_LINKS
            )

        elif mode == 'odg':
            self.controller = NumpyOmnidirectionalGaitController(
                gait_type=self.gait_type,
                step_height=self.step_height,
                gait_period=self.gait_period,
            )
            robot_standing_deg = SILVER2_DRAGON_POSE_DEG * self.isaac_robot_signs
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

    def _target_pose_cb(self, msg: String):
        pose_name = msg.data.strip().lower()

        if pose_name not in self.predefined_poses:
            available = ', '.join(self.predefined_poses.keys())
            self.get_logger().warn(
                f"Unknown pose '{msg.data}'. Available poses: [{available}]"
            )
            return

        if not self.joint_states_received:
            self.get_logger().warn(
                f"Ignoring /target_pose '{pose_name}': /joint_states not received yet."
            )
            return

        lin_mag = float(np.hypot(self.vx, self.vy))
        if lin_mag > 1e-3 or abs(self.wz) > 1e-3:
            self.get_logger().warn(
                f"Ignoring /target_pose '{pose_name}' while /cmd_vel is non-zero. Stop robot first."
            )
            return

        self.ramp = 0.0
        self.pose_q_start = np.copy(self.q_current)
        self.pose_q_target = np.copy(self.predefined_poses[pose_name])
        self.pose_active_name = pose_name
        self.pose_step = 0
        self.pose_interp_active = True

        self.get_logger().info(
            f"Transitioning all legs simultaneously to '{pose_name}' "
            f"over {self.pose_interp_duration:.1f}s..."
        )

    def _cmd_vel_cb(self, msg: Twist):
        self.vx = float(msg.linear.x)
        self.vy = float(msg.linear.y)
        self.wz = float(msg.angular.z)

    # --- Synchronized Startup Transition ---
    def _move_to_pose_blocking(self, pose_name: str):
        self.get_logger().info(f"Waiting for /joint_states before moving to '{pose_name}'...")
        while rclpy.ok() and not self.joint_states_received:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not rclpy.ok():
            return

        q_start = np.copy(self.q_current)
        q_target = self.predefined_poses[pose_name]
        steps = max(1, int(round(self.pose_interp_duration / self.dt)))

        self.get_logger().info(
            f"Interpolating all legs simultaneously to '{pose_name}' "
            f"over {self.pose_interp_duration:.1f}s..."
        )
        for s in range(1, steps + 1):
            raw_alpha = s / steps
            alpha = 0.5 * (1.0 - math.cos(math.pi * raw_alpha))
            q_interp = (1.0 - alpha) * q_start + alpha * q_target
            self._publish_joints(q_interp)
            time.sleep(self.dt)

        self.get_logger().info(f"Pose '{pose_name}' reached.")

    # --- Non-Blocking Synchronized Pose Step (50 Hz) ---
    def _step_pose_interpolation(self):
        interp_steps = max(1, int(round(self.pose_interp_duration / self.dt)))
        self.pose_step += 1

        raw_alpha = min(1.0, self.pose_step / interp_steps)
        alpha = 0.5 * (1.0 - math.cos(math.pi * raw_alpha))

        q_cmd = (1.0 - alpha) * self.pose_q_start + alpha * self.pose_q_target
        self._publish_joints(q_cmd)

        if raw_alpha >= 1.0:
            self.pose_interp_active = False
            self.get_logger().info(f"Pose '{self.pose_active_name}' reached.")

    # --- Main Real-Time Control Loop ---
    def _control_loop(self):
        lin_mag = float(np.hypot(self.vx, self.vy))
        is_moving = (lin_mag > 1e-3) or (abs(self.wz) > 1e-3)

        if self.pose_interp_active:
            if is_moving:
                self.pose_interp_active = False
                self.get_logger().info(
                    f"Transition to '{self.pose_active_name}' preempted by non-zero /cmd_vel."
                )
            else:
                self._step_pose_interpolation()
                return

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