---
sidebar_position: 4
title: URDF for Humanoid Structure
---

# URDF: Describing the Robot's Body

Before we can control a robot, we need a way to describe it: its shape, its joints, how its limbs are connected, and its physical properties. The **Unified Robot Description Format (URDF)** is an XML-based standard for this very purpose.

Think of URDF as the robot's blueprint. It defines the robot's **kinematic tree**, which is a collection of rigid bodies (**links**) connected by **joints**.

## Core Components of a URDF

A URDF file is made up of a few key tags:

-   `<robot name="..._name">`: The root element of the file.
-   `<link name="..._name">`: Represents a rigid part of the robot, like a torso, an upper arm, or a finger.
    -   `<visual>`: Defines the appearance of the link (shape, color, mesh).
    -   `<collision>`: Defines the geometry used for physics-based collision detection.
    -   `<inertial>`: Defines the dynamic properties of the link (mass, inertia).
-   `<joint name="..._name" type="...">`: Connects two links together.
    -   `type`: Can be `revolute` (rotating), `prismatic` (sliding), `fixed` (fused together), `continuous`, etc.
    -   `<parent link="..."/>` and `<child link="..."/>`: Defines the kinematic relationship.
    -   `<origin xyz="..." rpy="..."/>`: Specifies the pose of the child link relative to the parent link.
    -   `<axis xyz="..."/>`: The axis of rotation or translation for the joint.

### The Kinematic Tree

The `parent`-`child` relationships in the joints form a tree structure, with one special link acting as the root (usually called `base_link` or `torso`).

```mermaid
graph TD
    A[base_link (torso)] --> B(head_pan_joint);
    B --> C[head_link];
    A --> D(left_shoulder_joint);
    D --> E[left_upper_arm_link];
    E --> F(left_elbow_joint);
    F --> G[left_lower_arm_link];
    A --> H(right_shoulder_joint);
    H --> I[right_upper_arm_link];
```
*A simplified kinematic tree for a humanoid's upper body.*

## Example: A Simple Humanoid URDF

Let's create a URDF for a very simple humanoid containing a torso, a panning head, and one arm.

:::info
**Note:** Real-world humanoid URDFs are much more complex, often composed of dozens of files using `<xacro>` for modularity. This is a simplified example to illustrate the core concepts.
:::

`simple_humanoid.urdf`
```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- The base of our robot: the torso -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.6 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.6 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- The Head -->
  <link name="head_link">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white">
        <color rgba="1.0 1.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Joint connecting torso and head -->
  <joint name="head_pan_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head_link"/>
    <origin xyz="0.0 0.0 0.15" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- The Upper Arm -->
  <link name="upper_arm_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joint connecting torso and upper arm -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0.0 -0.35 0.0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="20" velocity="2.0"/>
  </joint>

</robot>
```

### What this URDF Defines:

1.  **A `base_link` (torso)**, which is a blue box and acts as the root of the kinematic tree.
2.  **A `head_link`**, which is a white sphere.
3.  **A `head_pan_joint`** that connects the `head_link` to the `base_link`. It allows the head to rotate around the Z-axis (panning left and right).
4.  **An `upper_arm_link`**, which is a grey cylinder.
5.  **A `shoulder_joint`** that connects the `upper_arm_link` to the `base_link`. It allows the arm to rotate around the Y-axis (pitching forward and backward).

This file, when loaded by ROS 2 and visualization tools like RViz, provides all the information needed to display the robot's model and calculate the relationship between its parts. This forms the basis for everything from simulation to motion planning.
