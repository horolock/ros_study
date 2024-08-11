# Using `turtlesim`, `ros2`, and `rqt`

Turtlesim is a lightweight simulator for learning ROS 2. It illustrates what ROS2 does at the most basic level to give you an idea of what you will do with a real robot or a robot simulation later on.

## Tasks

### Install turtlesim

```Shell
sudo apt update
sudo apt install ros-humble-turtlesim
```

Check that the package is installed:

```Shell
ros2 pkg executables turtlesim
```

### Start turtlesim

```Shell
ros2 run turtlesim turtlesim_node
```

### Use turtlesim

```Shell
ros2 run turtlesim turtle_teleop_key
```

You can see the nodes, and their associated topics, services, and actions, using the `list` subcommands of the respective commands:

```Shell
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

### Install `rqt`

```Shell
sudo apt update
sudo apt install '~nros-humble-rqt*'
```

To run rqt:

```Shell
rqt
```

### Remapping

You need a second teleop node in order to control `turtle2`. However, if you try to run the same command as before, you will notice that this one also controls `turtle1`.
The way to change this behavior is by **remapping** the `cmd_vel` topic.

```Shell
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```