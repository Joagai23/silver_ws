# Silver Robot Locomotion Testing Setup

This guide outlines the step-by-step terminal commands required to launch the SILVER2 robot's hardware interfaces, start the locomotion controller, and send test movement commands.

## Terminal 1: Core Bringup
This terminal builds the workspace (if needed), activates the Python virtual environment, and launches the hardware controller manager.
```
# Navigate to your workspace (only if in a different directory)
cd ~/silver_ws

# Source the base ROS 2 Jazzy installation
source /opt/ros/jazzy/setup.bash

# (Optional) Build the workspace if you made recent changes
colcon build

# Activate the Python virtual environment
source venv/bin/activate

# Source the built workspace
source install/setup.bash

# Launch the main hardware and controller sequence
ros2 launch silver silver2.launch.py
```

*Note: Wait for the controllers to fully load and activate before moving to Terminal 2.*

---

## Terminal 2: Locomotion Node
Once Terminal 1 is fully running, open a second terminal to start the custom locomotion script. This will initialize the robot's legs and move them into the starting "Dragon" pose.
```
# Navigate to your workspace (only if in a different directory)
cd ~/silver_ws

# Source the ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Activate the virtual environment
source venv/bin/activate

# Run the locomotion controller script
python3 ./src/silver/scripts/omnidirectional/cmd_vel_control_rotation.py
```

*Note: Wait for the robot to finish its initialization sequence before sending movement commands.*

---

## Terminal 3: Movement Commands
With the robot awake and in its neutral pose, use this third terminal to send movement commands.

```
# Source the ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Command the robot to walk forward (linear x: 0.5)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# To stop the robot, send a command with all zeros
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```