# SILVER2 ROS 2 Workspace

Repository for controlling the **SILVER2** 18-DOF underwater hexapod robot.

---

## 0. Clone the Repository
Clone the repository into your development directory:
```bash
cd ~/Code_Directory
git clone [https://github.com/Joagai23/silver_ws.git](https://github.com/Joagai23/silver_ws.git)
cd silver_ws
```

## 1. Python Virtual Environment & Workspace Build
### Create the Virtual Environment

From the root of the workspace (```silver_ws```), create a Python virtual environment named ```venv``` (using ```--system-site-packages``` so it retains access to ROS 2 system packages):
```bash
cd ~/Code_Directory/silver_ws
python3 -m venv venv --system-site-packages
```

### Add ```COLCON_IGNORE```
Prevent ```colcon build``` from scanning the virtual environment:
```bash
touch venv/COLCON_IGNORE
```

### Install Dependencies & Build the Workspace
Activate ```venv```, install the required Python packages, and build the ROS 2 workspace:
```bash
source venv/bin/activate
pip install -r requirements.txt
# Or manually: pip install dynamixel-sdk smbus2 pyserial numpy scipy transforms3d

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

**Note for every new terminal:** Always source both the virtual environment and the ROS 2 workspace overlay before running any commands:
```bash
cd ~/Code_Directory/silver_ws
source venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 2. Rebooting the Dynamixel Servos
To safely issue a Protocol 2.0 reboot across all 18 Dynamixel servos on the RS485 serial bus (```/dev/ttyUSB0``` at ```1000000``` bps):
```bash
# Via ROS 2 executable
ros2 run silver reboot_servos.py

# Or directly via Python
python3 src/silver/scripts/dynamixel/reboot_servos.py
```

### Check Active Legs & Motor Status
To ping all 18 motors (IDs ```0``` to ```17```, grouped into Legs ```0```–```5```) and verify which legs are online before launching ```ros2_control```:
```bash
python3 src/dynamixel/debug_scripts/dynamixel_controller.py
```

## 3. Launching the Robot & Sensors
### Option A: Launch Sensors Only (Telemetry & Battery Test)
Use this mode to monitor the battery, internal canister humidity/pressure, external depth, and IMU without starting ```ros2_control``` or powering the 18 Dynamixel actuators:
```bash
ros2 launch silver silver2_full.launch.py sensors_only:=true
```

### Option B: Launch Controllers & Actuators Only (No I2C Sensors)
Starts ```robot_state_publisher```, ```ros2_control_node``` (```DynamixelHardware```), ```joint_state_broadcaster```, and ```pid_position_controller```:
```bash
ros2 launch silver silver2.launch.py
```

### Option C: Launch Full Robot (Actuators + Sensors + CSV Logger)
Starts the ```ros2_control``` hardware stack alongside all I2C sensors and automatically records telemetry to ```~/silver_logs/``` (```record_sensors:=true``` by default):
```bash
ros2 launch silver silver2_full.launch.py
```

- To launch the full robot without recording a CSV log:
```bash
ros2 launch silver silver2_full.launch.py record_sensors:=false
```

- To specify a custom directory for the CSV logs:
```bash
ros2 launch silver silver2_full.launch.py log_dir:=/path/to/logs
```

## 4. Running the Locomotion Manager
Once ```silver2.launch.py``` or ```silver2_full.launch.py``` is running and ```pid_position_controller``` is active, open a second terminal (with ```venv``` and ```install/setup.bash``` sourced) and start the locomotion manager:
```bash
# Default: Hopf CPG controller ('cpg') with 'tripod' gait at 50 Hz
ros2 run silver locomotion_manager.py
```

### Customizing Gait & Controller Parameters

You can switch between the **Hopf CPG** (```cpg```) and **Omnidirectional Gait** (```odg```) backends and select the gait (```tripod```, ```quadruped```, ```wave```) via ROS parameters:
```bash
ros2 run silver locomotion_manager.py --ros-args \
  -p controller_type:=cpg \
  -p gait_type:=tripod \
  -p gait_period:=2.0 \
  -p step_height:=0.05 \
  -p startup_pose:=true
```

### Sending Velocity Commands (```/cmd_vel```)
In another terminal, publish ```geometry_msgs/msg/Twist``` commands to ```/cmd_vel``` to walk or pivot:
```bash
# Walk forward (linear.x = 0.5 m/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Stop the robot
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
