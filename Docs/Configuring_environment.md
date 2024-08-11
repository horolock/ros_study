# Configuring Environment

## Tasks

### Source the setup files

You will need to run this command on **every new shell** you open to have access to the ROS 2 commands.

```shell
source /opt/ros/humble/setup.bash
```

### Add sourcing to your shell startup script

If you don't want to have to source the setup file every time you open a new shell, then you can add the command to your shell startup script:

```shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Check environment variables

```Shell
printenv | grep -i ROS
```

#### The `ROS_DOMAIN_ID` variable

Once you have determined a unique integer for your group of ROS 2 nodes, you can set the environment variable with the following command:

```Shell
export ROS_DOMAIN_ID=<your_domain_id>
```

or

```Shell
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
```

#### The `ROS_LOCALHOST_ONLY` variable

By default, ROS 2 communication is not limited to localhost.
`ROS_LOCALHOST_ONLY` environment variable allows you to limit ROS2 communication to localhost only.

This means your ROS2 system, and its topics, services, and actions will not be visible to other computers on the local network.

```Shell
export ROS_LOCALHOST_ONLY=1
```

or

```Shell
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
```