# Understanding nodes

## The ROS2 graph

The ROS graph is a network of ROS2 elements processing data together at the same time.
It encompasses all executables and the connections between them if you were to map them all out and visualize them.

## Nodes in ROS2

Each node in ROS should be responsible for a single, modular purpose, e.g. controlling the wheel motors or publishing the sensor data from a laser range-finder.
Each node can send and receive data from other nodes via topics, services, actions, or parameters.

A full robotic system is comprised of many nodes working in concert. In ROS2, a single executable (C++ program, Python program, etc.) can contain one or more nodes.

## Tasks

### ros2 run

The command `ros2 run` launches an **executable** from a package.

```Shell
ros2 run <package_name> <executable_name>
```

To run turtlesim, open a new terminal, and enter the following command:

```Shell
ros2 run turtlesim turtlesim_node
```

Here, the package name is `turtlesim` and the executable name is `turtlesim_node`.

### ros2 node list

`ros2 node list` will show you the names of all running nodes.

```Shell
ros2 node list
```

The terminal will return the node name:

```Shell
/turtlesim
```

```Shell
ros2 run turtlesim turtle_teleop_key
```

Here, we are referring to the `turtlesim` package again, but this time we target the executable named `turtle_teleop_key`.

And `ros2 node list` will return:

```Shell
/turtlesim
/teleop_turtle
```

#### Remapping

```Shell
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```

And `ros2 node list` will return:

```Shell
/my_turtle
/turtlesim
/teleop_turtle
```

### ros2 node info

You can access more information about them with:

```Shell
ros2 node info <node_name>

# ex) ros2 node info /my_turtle
```

`ros2 node info` returns a list of subscribers, publishers, services, and actions. i.e. the ROS graph connections that interact with that node.