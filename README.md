# ROS 2 Frontier-based Exploration

A simple ROS 2 package for autonomous frontier-based exploration, designed to work with Nav2.

This package was developed for the Mobile Robotics course at Institut Polytechnique de Paris.

## Overview

This package provides a single ROS 2 node (`exploration_node`) that implements a basic frontier-based exploration algorithm.

It is intended to be used as a high-level "brain" that coordinates with existing SLAM (e.g., `slam_toolbox`) and Navigation (e.g., `Nav2`) systems.

### Twist Converter

This package also includes a helper node, `twist_converter`, which is launched automatically by `exploration_launch.py`.

This node solves the possible `Twist` vs. `TwistStamped` mismatch between Nav2 and Gazebo.

- Listens for `geometry_msgs/msg/Twist` on `/cmd_vel`.

- Publishes `geometry_msgs/msg/TwistStamped` on `/cmd_vel_stamped` (which `ros_gz_bridge` expects).

## Dependencies

This package requires a full ROS 2 (tested on Humble and Jazzy) setup with the following components:

- `slam_toolbox`: Required to generate the live `nav_msgs/OccupancyGrid` map.

- `nav2_bringup`: Required for the navigation stack that receives goals.

- `turtlebot3_simulations` (or other robot simulation): Required for testing in a simulated environment.

## Installation

This package must be built from source.

1. Navigate to your ROS 2 workspace src directory:
    ```
    cd ~/ros2_ws/src
    ```

2. Clone this repository:
    ```
    git clone https://github.com/yzrobot/ros2_exploration.git
    ```

3. Return to the workspace root and install dependencies:
    ```
    cd ~/ros2_ws
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
    ```

4. Build the workspace:
    ```
    colcon build --symlink-install
    ```

## Usage

This package is designed to be run as part of a larger system. You will need at least 5 terminals. Remember to source your workspace (`source ~/ros2_ws/install/setup.bash`) in each terminal.

**IMPORTANT:** If there is a message type mismatch between your Nav2 and Gazebo, you will have to tell the `ros_gz_bridge` to listen to the `/cmd_vel_stamped` topic that our `twist_converter` will publish to (instead of its default /cmd_vel).

1. Open the file with sudo:
    ```
    sudo nano /opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/launch/spawn_turtlebot3.launch.py
    ```

2. Find the `start_gazebo_ros_bridge_cmd` node.

Add the `remappings` argument to this node:
```python
start_gazebo_ros_bridge_cmd = Node(
    package='ros_gz_bridge',
    # ... other arguments
    output='screen',
    # ADD THESE LINES:
    remappings=[
        ('/cmd_vel', '/cmd_vel_stamped'),
    ]
)
```

3. Save and exit (Ctrl+O, Enter, Ctrl+X).

### Running the exploration

1. Terminal 1: Gazebo
    ```
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ```

2. Terminal 2: SLAM
    ```
    ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
    ```

3. Terminal 3: RViz
    ```
    ros2 launch nav2_bringup rviz_launch.py
    ```

4. Terminal 4: Nav2
    ```
    ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True map_subscribe_transient_local:=True
    ```

5. Terminal 5: Exploration
    ```
    # This command launches both the exploration_node and the twist_converter node.
    ros2 launch ros2_exploration exploration_launch.py use_sim_time:=True
    ```

The robot should now begin exploring the environment autonomously.

## Nodes

`exploration_node`

The main frontier exploration logic.

- **Subscribes:**

    - `/map` (`nav_msgs/OccupancyGrid`): The live map from SLAM.

    - `/odom` (`nav_msgs/Odometry`): Robot's pose (used as a fallback).

    - `/tf`, `/tf_static`: Used to get the robot's pose in the `map` frame.

- **Publishes:**

    - `/exploration_frontiers` (`visualization_msgs/Marker`): Detected frontiers for visualization in RViz.

- **Action Client:**

    - `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`): Sends goals to the Nav2 navigation stack.

`twist_converter`

Helper node to convert velocity command types.

- **Subscribes:**

    - `/cmd_vel` (`geometry_msgs/msg/Twist`): The remapped output from Nav2.

- **Publishes:**

    - `/cmd_vel_stamped` (`geometry_msgs/msg/TwistStamped`): The topic Gazebo expects (after the bridge remapping).
