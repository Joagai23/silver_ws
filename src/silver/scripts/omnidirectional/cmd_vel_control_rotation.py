#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: Giacomo Picardi (modified for cmd_vel integration)

import os, sys, time, signal
import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import robot

should_quit = False

def handler(signum, frame):
    global should_quit
    should_quit = True

class OmnidirectionalGaitController(Node):

    def __init__(self):
        super().__init__('omnidirectional_gait_controller')

        # Fixed gait parameters
        self.gait_width = 40.0
        self.gait_height = 30.0
        self.step_length = 20.0
        self.duty_cycle = 0.6
        self.ground_clearance = 10.0
        self.phase_lag = np.array([0.0, 180.0, 0.0, 180.0, 0.0, 180.0])
        self.nstep = 30
        self.rotation = 0.0
        self.direction = 0.0
        self.period = 3.0

        # Latest cmd_vel
        self.latest_cmd = None
        self.last_cmd = None

        # Robot model and communication setup
        self.robot = robot.Robot()
        self.Q_current = robot.static_poses_pos['zero']

        self.joint_order = [
            'coxa_joint_0', 'femur_joint_0', 'tibia_joint_0',
            'coxa_joint_1', 'femur_joint_1', 'tibia_joint_1',
            'coxa_joint_2', 'femur_joint_2', 'tibia_joint_2',
            'coxa_joint_3', 'femur_joint_3', 'tibia_joint_3',
            'coxa_joint_4', 'femur_joint_4', 'tibia_joint_4',
            'coxa_joint_5', 'femur_joint_5', 'tibia_joint_5'
        ]

        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_subscriber_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.pid_pos_publisher = self.create_publisher(Float64MultiArray, '/pid_position_controller/commands', 10)

    def joint_state_subscriber_callback(self, msg):
        joint_position_dict = dict(zip(msg.name, msg.position))
        for i, joint_name in enumerate(self.joint_order):
            if joint_name in joint_position_dict:
                self.Q_current[i] = joint_position_dict[joint_name]
            else:
                self.get_logger().warn(f"Joint {joint_name} not found in message")

    def cmd_vel_callback(self, msg):
        self.latest_cmd = msg

    def change_configuration_loop(self, Q_target):
        Q_cc, _, Admiss_cc, nstep_cc, ctrl_timestep = self.robot.change_configuration(Q_target, self.Q_current)
        if not all(Admiss_cc):
            self.get_logger().warn("Configuration change outside workspace")
            return

        for i in range(nstep_cc):
            if should_quit:
                break
            msg = Float64MultiArray()
            msg.data = Q_cc[:, i].tolist()
            self.pid_pos_publisher.publish(msg)
            time.sleep(ctrl_timestep)

    def omni_loop(self):
        i = 0
        Q_omni = np.zeros((18, self.nstep))
        ctrl_timestep = self.period / self.nstep

        while rclpy.ok() and not should_quit:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.latest_cmd is None:
                time.sleep(0.1)
                continue

            vx, vy, wz = self.latest_cmd.linear.x, self.latest_cmd.linear.y, self.latest_cmd.angular.z

            if np.isclose(vx, 0.0, atol=1e-3) and np.isclose(vy, 0.0, atol=1e-3) and np.isclose(wz, 0.0, atol=1e-3):
                time.sleep(0.1)
                continue

            cmd_changed = False
            if self.last_cmd is None:
                cmd_changed = True
            else:
                dvx = abs(vx - self.last_cmd.linear.x)
                dvy = abs(vy - self.last_cmd.linear.y)
                dwz = abs(wz - self.last_cmd.angular.z)
                if dvx > 1e-3 or dvy > 1e-3 or dwz > 1e-3:
                    cmd_changed = True

            if cmd_changed:
                self.last_cmd = self.latest_cmd

                if np.isclose(vx, 0.0, atol=1e-3) and np.isclose(vy, 0.0, atol=1e-3) and not np.isclose(wz, 0.0, atol=1e-3):
                    self.rotation = 1
                    self.direction = np.sign(wz) * 90.0  # +90 for clockwise, -90 for counter-clockwise
                    vel_mag = min(abs(wz), 1.0)
                else:
                    self.rotation = 0
                    self.direction = np.arctan2(vy, vx) * 180.0 / np.pi
                    vel_mag = np.clip(np.linalg.norm([vx, vy]), 0.0, 1.0)

                self.period = 5.0 - 4.0 * vel_mag  # range 5.0 (slow) to 1.0 (fast)
                ctrl_timestep = self.period / self.nstep

                Admiss_omni = [False] * 6
                for leg_id in range(6):
                    _, Q_omni[3*leg_id:3*leg_id+3, :], _, Admiss_omni[leg_id] = \
                        self.robot.trj_gen(self.gait_width, self.gait_height, self.direction, self.step_length,
                                           self.duty_cycle, self.ground_clearance, self.phase_lag[leg_id],
                                           self.nstep, self.period, leg_id, self.rotation)

                if not all(Admiss_omni):
                    self.get_logger().warn("Gait not feasible for current direction")
                    continue

                if i == 0:
                    self.change_configuration_loop(Q_omni[:, 0])

            for step in range(self.nstep):
                if should_quit:
                    break
                msg = Float64MultiArray()
                msg.data = Q_omni[:, step].tolist()
                self.pid_pos_publisher.publish(msg)
                time.sleep(ctrl_timestep)

            i += 1

    def go_to_dragon_pose(self):
        self.get_logger().info("Initializing: Moving to Dragon Pose safely in joint space...")

        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

        Q_start = np.copy(self.Q_current)
        Q_end = robot.static_poses_pos['dragon']

        steps = 60
        delay = 3.0 / steps 

        for i in range(steps):
            if should_quit:
                break
            alpha = (i + 1) / steps
            Q_interp = Q_start * (1 - alpha) + Q_end * alpha

            msg = Float64MultiArray()
            msg.data = Q_interp.tolist()
            self.pid_pos_publisher.publish(msg)
            time.sleep(delay)

        self.get_logger().info("Dragon Pose reached! Ready for /cmd_vel inputs.")

    def go_to_dragon_pose_slow(self):
        self.get_logger().info("Initializing: Moving to Dragon Pose ONE LEG AT A TIME...")

        # Wait a moment to ensure we have received the actual physical joint states
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

        # Grab the current real-world starting positions
        Q_current_tracker = np.copy(self.Q_current)
        Q_target = robot.static_poses_pos['dragon']

        # Loop through each of the 6 legs sequentially
        for leg_id in range(6):
            self.get_logger().info(f"Moving Leg {leg_id} to Dragon Pose...")
            Q_start = np.copy(Q_current_tracker)
            
            # Move each leg over 1.5 seconds
            steps = 30
            delay = 1.5 / steps 

            for i in range(steps):
                if should_quit:
                    break
                alpha = (i + 1) / steps
                
                # Calculate the indices for the current leg (3 joints per leg)
                idx_start = 3 * leg_id
                idx_end = 3 * leg_id + 3
                
                # Interpolate ONLY the joints for the current leg
                Q_interp = np.copy(Q_start)
                Q_interp[idx_start:idx_end] = Q_start[idx_start:idx_end] * (1 - alpha) + Q_target[idx_start:idx_end] * alpha
                
                # Publish the array
                msg = Float64MultiArray()
                msg.data = Q_interp.tolist()
                self.pid_pos_publisher.publish(msg)
                time.sleep(delay)
            
            # Update our tracker so the next leg keeps this leg in its new position
            Q_current_tracker = np.copy(Q_interp)
            
            # Pause for half a second before moving the next leg
            time.sleep(0.5)

        self.get_logger().info("Dragon Pose successfully reached! Ready for /cmd_vel inputs.")

if __name__ == '__main__':
    rclpy.init()
    prevhand = signal.signal(signal.SIGINT, handler)
    try:
        gait_controller = OmnidirectionalGaitController()
    except Exception as e:
        print(e)
        sys.exit(1)

    gait_controller.go_to_dragon_pose_slow()

    gait_controller.omni_loop()
    rclpy.shutdown()