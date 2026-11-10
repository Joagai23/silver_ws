# SILVER2 ROS 2 Workspace

Repository for controlling the **SILVER2** 18-DOF underwater hexapod robot.

---

## 1. Python Virtual Environment Setup

From the root of the workspace (`silver_ws`), create a Python virtual environment named `venv` (using `--system-site-packages` so it retains access to ROS 2 system packages):

```bash
cd ~/Documents/Code/silver_ws
python3 -m venv venv --system-site-packages
```

### Add `COLCON_IGNORE` and `.gitignore`
Prevent `colcon build` from scanning the virtual environment and ensure Git ignores both the `venv` and ROS 2 build artifacts:

```bash
# Tell colcon to ignore the virtual environment
touch venv/COLCON_IGNORE

# Add venv and colcon build directories to .gitignore
cat <<EOT >> .gitignore
venv/
build/
install/
log/
__pycache__/
*.pyc
EOT
```

### Activate the Virtual Environment & Install Dependencies
Activate `venv` and install the Dynamixel SDK:

```bash
source venv/bin/activate
pip install dynamixel-sdk
```

---

## 2. Rebooting the Dynamixel Servos

To safely reboot all 18 Dynamixel servos on the serial bus (`/dev/ttyUSB0`), run the reboot utility script from the workspace root:

```bash
python3 src/silver/scripts/dynamixel/reboot_servos.py
```

*(Optional)* If you are testing on a workstation without the physical robot connected, pass the `--dry-run` flag:

```bash
python3 src/silver/scripts/dynamixel/reboot_servos.py --dry-run
```