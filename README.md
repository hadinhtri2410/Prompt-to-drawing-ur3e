# CSV Controller
This is an example of using ros2-control in order to write a custom controller in order to move the ur3e arm in lab. 
Put the `.csv` files into the [csv_files](csv_files) folder, rebuild the package, and then use a launch argument in order to specify which file to use.

### Example Launch Command
```bash
ros2 launch csv_controller ur3e_csv.launch.py launch_rviz:=false csv_name:=ldihel_bonus
```

## Pre-requesites
This is probably not a complete list of things you will need to install, but it will help get you started.

### Establishing Communication with robotic arm
1. Establish communication between the ur3e and your computer. I recommend an ethernet cable running directly between both computers. On the robot side I used the following network stuff with a __static address__:

<div align="center">
  
| Setting      | Value |
| ----------- | ----------- |
| IP address      | 192.168.1.102       |
| Subnet Mask   | 255.255.255.0        |
| Default gateway | 192.168.1.1 |
| Preferred DNS server | 192.168.1.1 |
| Alternative DNS server | 0.0.0.0 |

 </div>

2. Similarly, your Linux computer (I am running Ubuntu 24.04 with ros2 jazzy) be sure to configure a static address of 192.168.1.101 with the same subnet mask and default gateway.
3. Turn on the robot and verify you can ping the robot's IP address with `ping 192.168.1.102`


### Installing Drivers

1. If you haven't already, but sure [ROS2](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html) is installed on your Linux machine.
2. Install [ros2-control](https://control.ros.org/rolling/index.html). Be sure to replace `DISTRO` with your version of ros `humble, jazzy, rolling, etc.`
```bash
sudo apt-get install ros-DISTRO-ros2-control
sudo apt-get install ros-DISTRO-ros2-controllers
```
3. Install the [ur ros2 drivers](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver).
4. You will also need to put the `External Control` URCap onto your robot with a USB stick. Instructions for putting UR caps onto your robot is well documented online.
5. Configure the `External Control` URCap on the teaching pendant with the following values.

<div align="center">
  
| Setting      | Value |
| ----------- | ----------- |
| Host IP      | 192.168.1.101       |
| Custom Port  | 50002       |
| Host Name | 192.168.1.101 |

 </div>

### Using MoveIT!
Put the robot into `Remote Control` (headless) mode (button is in upper right corner of teaching pendent and will initially say "Local" before you make the switch the "Remote"). 
Then in two separate terminals (both which have been sourced for ROS) run the following:
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.1.102 launch_rviz:=false headless_mode:=true
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true
```

### Other Installations
I am also using the [generate parameter library](https://github.com/PickNikRobotics/generate_parameter_library) in my code which lets me pass parameters from launch and config files into my ros2 controller. I am not sure if you need this installed or not.

### Using the CSV Controller
Create a ros2 workspace and then clone this repository:
```bash
mkdir -p ~/ros2ws/src
cd ~/ros2ws/src
git clone <this repo>
```

Then you can build and source the workspace.
```bash
cd ~/ros2ws
colcon build --symlink-install
source install/setup.bash
```

Then, after proper modification to the launch script to use a ur5e robot (see the [launch](launch) folder) you can make the robot move using the example command at the beginning of this readme file!

### Helpful Aliases for ros development
You might want to put something like this into your `~/.bashrc` file.
```bash
# ros stuff
export ROS_WS=$HOME/ros2ws
alias rosd="cd $ROS_WS"
alias rosds="cd $ROS_WS/src"
alias cb="cd $ROS_WS && colcon build --symlink-install && source $ROS_WS/install/setup.bash"
alias rosclean="rosd && rm -rf build install log"
alias _ws="source $ROS_WS/install/setup.bash"
export ROBOT_IP="192.168.1.102"

source /opt/ros/jazzy/setup.bash
PS1='\[\033[0;32m\]\u@\h\[\033[0m\]:\[\033[0;37m\]\[\033[0m\]\[\033[0;34m\]\w\[\033[0m\]\$ '
```
