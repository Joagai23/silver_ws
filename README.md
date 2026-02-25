# Silver Robot Locomotion Bringup

This package contains the control configurations and launch files to bring up the Silver robot's hardware interfaces, load the PID position controllers, and start the omnidirectional gait generator.

## Prerequisites
Ensure your workspace is built and sourced before running the commands below. Open a terminal in the root of your ROS 2 workspace and run:
```
colcon build --packages-select silver
source install/setup.bash
```
---

## 1. Launching the Robot

To start the robot's hardware, controller manager, and the custom locomotion node, run the main launch file:
```
ros2 launch silver silver2.launch.py
```

*Note: This terminal must remain open. The robot is now waiting for Twist commands on the /cmd_vel topic.*

---

## 2. Controlling the Robot

Open a **new terminal**, source your workspace (`source install/setup.bash`), and choose one of the following methods to drive the robot:

### Option A: Keyboard Control
If you want to use your computer's arrow keys to send movement commands, install the keyboard teleop package (if you haven't already):

```
sudo apt install ros-jazzy-teleop-twist-keyboard
```

Then run the node:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Follow the on-screen instructions in the terminal to move forward/backward, strafe, and rotate.

### Option B: Xbox Controller (Joystick)
To use a physical Xbox controller, connect it via Bluetooth or USB to the machine you are running this command from. Install the joystick packages:

```
sudo apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy
```

Then, launch the joystick teleop node:

```
ros2 launch teleop_twist_joy teleop.launch.py joy_config:=xbox
```

**⚠️ IMPORTANT:** The Xbox controller has a "Dead Man's Switch" enabled by default for safety. **You must hold down the Enable Button (usually the 'A' button or Left Bumper)** while moving the joysticks, or the robot will not move.

---

## Troubleshooting
* **Robot not moving with Xbox controller:** Verify the controller is paired and recognized by Linux by running `ros2 topic echo /joy`. If numbers change when you move the sticks, the connection is good. Make sure you are holding the enable button!
* **Controllers failing to load:** Ensure your real-world hardware is powered on and the USB/serial connections defined in your `robot.urdf.xacro` are plugged in and have the correct permissions (e.g., `sudo chmod 666 /dev/ttyUSB0`).