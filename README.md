# Prompt to Drawing

A ROS2 pipeline that converts drawing commands into joint trajectories for a UR3e robotic arm. The system uses inverse kinematics to translate 2D drawing primitives (lines, circles, pen up/down) into joint-space trajectories, which are exported as CSV files and executed on the real robot via a custom ros2-control controller.

## Packages

| Package | Description |
| --- | --- |
| `csv_controller` | Custom ros2-control controller that reads joint trajectories from CSV files |
| `drawing_pipeline` | Drawing primitives and IK solver for converting 2D drawings to joint trajectories |
| `py_joint_pub` | Python node that publishes joint states from CSV for visualization |
| `msee22_description` | URDF description of the MSEE 22 lab environment |
| `table_description` | URDF description of the table |
| `wall_description` | URDF description of the wall |

## Drawing Pipeline

The `drawing_pipeline` package provides two main modules:

- **`primitives.py`** — A `DrawingProgram` class with commands: `pen_up()`, `pen_down()`, `move_to(x, y)`, `line_to(x, y)`, and `circle(cx, cy, radius)`. These are sampled into evenly-spaced waypoints with `sample_program()`, and can be scaled/rotated/translated with `apply_params()`.
- **`ik_solver.py`** — UR3e forward/inverse kinematics (body frame). `solve_trajectory()` takes 2D waypoints and returns a full 6-joint trajectory. `export_csv()` saves the result in a format compatible with the `csv_controller`.

## Prerequisites

### Establishing Communication with the UR3e

1. Connect the UR3e to your computer via ethernet. Configure a **static address** on the robot:

<div align="center">

| Setting | Value |
| --- | --- |
| IP address | 192.168.1.102 |
| Subnet Mask | 255.255.255.0 |
| Default gateway | 192.168.1.1 |
| Preferred DNS server | 192.168.1.1 |
| Alternative DNS server | 0.0.0.0 |

</div>

2. On your Linux computer (Ubuntu 24.04 / ROS2 Jazzy), configure a static address of `192.168.1.101` with the same subnet mask and default gateway.
3. Verify connectivity: `ping 192.168.1.102`

### Installing Drivers

1. Install [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html).
2. Install ros2-control (replace `DISTRO` with your ROS2 version):
```bash
sudo apt-get install ros-DISTRO-ros2-control
sudo apt-get install ros-DISTRO-ros2-controllers
```
3. Install the [UR ROS2 driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver).
4. Install the `External Control` URCap on the robot via USB stick.
5. Configure the URCap on the teaching pendant:

<div align="center">

| Setting | Value |
| --- | --- |
| Host IP | 192.168.1.101 |
| Custom Port | 50002 |
| Host Name | 192.168.1.101 |

</div>

6. Install the [generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library) (used by the csv_controller).

## Usage

### Build

```bash
mkdir -p ~/ros2ws/src
cd ~/ros2ws/src
git clone <this repo>
cd ~/ros2ws
colcon build --symlink-install
source install/setup.bash
```

### Visualize in RViz

View the room model with joint state GUI:
```bash
ros2 launch msee22_description view_room.launch.py
```

Replay a CSV trajectory in RViz:
```bash
ros2 launch msee22_description move_robot.launch.py
```

### Run on the Real Robot

Put the robot into **Remote Control** mode (upper right corner of the teaching pendant), then in two terminals:

```bash
# Terminal 1: Start the UR driver and csv_controller
ros2 launch csv_controller ur3e_csv.launch.py launch_rviz:=false csv_name:=ldihel_bonus

# Terminal 2 (optional): Launch MoveIt for visualization
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true
```

CSV files should be placed in the [csv_files](csv_files) folder. Specify the file name (without `.csv`) via the `csv_name` launch argument.

## Helpful Aliases

```bash
export ROS_WS=$HOME/ros2ws
alias rosd="cd $ROS_WS"
alias rosds="cd $ROS_WS/src"
alias cb="cd $ROS_WS && colcon build --symlink-install && source $ROS_WS/install/setup.bash"
alias rosclean="rosd && rm -rf build install log"
alias _ws="source $ROS_WS/install/setup.bash"
export ROBOT_IP="192.168.1.102"

source /opt/ros/jazzy/setup.bash
```
