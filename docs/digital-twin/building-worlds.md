---
sidebar_position: 3
title: Building Worlds
---

# Building Worlds and Environments

A robot's environment is as important as the robot itself. An empty void provides no challenges, no obstacles to avoid, and no objects to manipulate. A **world** in Gazebo is an SDF file that contains all the models, lighting, and physics properties of the simulation environment.

## The World SDF File

A world SDF file has a simple structure:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_world">
    <!-- Scene properties like lighting and background -->
    <scene>
      <ambient>0.4 0.4 0.4 1.0</ambient>
      <background>0.7 0.7 0.7 1.0</background>
      <shadows>true</shadows>
    </scene>

    <!-- Physics engine settings -->
    <physics name="default_physics" default="0" type="dart">
      <gravity>0 0 -9.81</gravity>
    </physics>

    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a sun for lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add other models -->
    <model name="simple_table">
      <!-- ... model definition ... -->
    </model>
    
  </world>
</sdf>
```

### Key Elements:

-   `<scene>`: Defines the visual properties of the world, like ambient light color.
-   `<physics>`: Configures the physics engine, most importantly setting the `gravity` vector.
-   `<include>`: A powerful tag that lets you pull in pre-existing models. Gazebo comes with several built-in models like `ground_plane` and `sun`. You can also include models from your local paths or from the [Gazebo Fuel](https://app.gazebosim.org/fuel/models) online database.
-   `<model>`: Allows you to define a model directly within the world file. This is great for simple, static objects like walls, tables, and shelves.

## Creating a Static Model: A Table

Let's define a simple table for our robot to interact with. This model consists of a top and four legs, all connected by `fixed` joints.

`table.sdf`
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="simple_table">
    <static>true</static> <!-- This model doesn't move -->
    <link name="table_top">
      <pose>0 0 0.8 0 0 0</pose>
      <visual name="visual">
        <geometry><box><size>1.5 0.8 0.04</size></box></geometry>
        <material><ambient>0.6 0.4 0.2 1</ambient><diffuse>0.6 0.4 0.2 1</diffuse></material>
      </visual>
      <collision name="collision">
        <geometry><box><size>1.5 0.8 0.04</size></box></geometry>
      </collision>
    </link>

    <!-- Legs -->
    <link name="leg_1"><pose>0.65 0.3 0.4 0 0 0</pose><visual name="visual"><geometry><box><size>0.04 0.04 0.8</size></box></geometry><material><ambient>0.6 0.4 0.2 1</ambient><diffuse>0.6 0.4 0.2 1</diffuse></material></visual><collision name="collision"><geometry><box><size>0.04 0.04 0.8</size></box></geometry></collision></link>
    <link name="leg_2"><pose>-0.65 0.3 0.4 0 0 0</pose><visual name="visual"><geometry><box><size>0.04 0.04 0.8</size></box></geometry><material><ambient>0.6 0.4 0.2 1</ambient><diffuse>0.6 0.4 0.2 1</diffuse></material></visual><collision name="collision"><geometry><box><size>0.04 0.04 0.8</size></box></geometry></collision></link>
    <link name="leg_3"><pose>0.65 -0.3 0.4 0 0 0</pose><visual name="visual"><geometry><box><size>0.04 0.04 0.8</size></box></geometry><material><ambient>0.6 0.4 0.2 1</ambient><diffuse>0.6 0.4 0.2 1</diffuse></material></visual><collision name="collision"><geometry><box><size>0.04 0.04 0.8</size></box></geometry></collision></link>
    <link name="leg_4"><pose>-0.65 -0.3 0.4 0 0 0</pose><visual name="visual"><geometry><box><size>0.04 0.04 0.8</size></box></geometry><material><ambient>0.6 0.4 0.2 1</ambient><diffuse>0.6 0.4 0.2 1</diffuse></material></visual><collision name="collision"><geometry><box><size>0.04 0.04 0.8</size></box></geometry></collision></link>

    <!-- Fixed joints to hold the table together -->
    <joint name="j_top_leg1" type="fixed"><parent>table_top</parent><child>leg_1</child></joint>
    <joint name="j_top_leg2" type="fixed"><parent>table_top</parent><child>leg_2</child></joint>
    <joint name="j_top_leg3" type="fixed"><parent>table_top</parent><child>leg_3</child></joint>
    <joint name="j_top_leg4" type="fixed"><parent>table_top</parent><child>leg_4</child></joint>
  </model>
</sdf>
```
You can then include this model in your world file like this:
`<include><uri>model://path/to/your/table.sdf</uri></include>`

## Spawning Robots and Objects

While you can define everything in a single world file, it's often more flexible to start a world and then **spawn** your robot and other objects into it. This is done using a ROS 2 service call.

The `ros_gz_sim` package provides a node `create` that offers a `/create` service to spawn entities.

### Spawning a Robot from a Launch File

```python
# In your launch file
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

# ... (Get your robot's SDF description as a string, 'robot_desc') ...

# Use the 'create' node from ros_gz_sim to spawn the robot
spawn_entity = Node(
    package='ros_gz_sim', 
    executable='create',
    arguments=[
        '-string', robot_desc,
        '-name', 'simple_humanoid',
        '-allow_renaming', 'true'
    ],
    output='screen'
)
```
This launch action calls the service to add your robot model into the running simulation. This approach is highly modular, allowing you to build complex scenarios by launching a base world and then dynamically adding robots, objects, and obstacles as needed. This is particularly useful for creating randomized environments for robust AI training.
