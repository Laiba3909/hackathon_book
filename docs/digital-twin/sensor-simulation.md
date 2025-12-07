---
sidebar_position: 4
title: Sensor Simulation
---

# Simulating the Senses: LiDAR, Cameras, and IMUs

A robot is blind without its sensors. A digital twin would be useless if it couldn't replicate the data streams our AI relies on for perception. Gazebo provides a rich set of sensor plugins that can be attached to links in our robot's model to generate realistic sensor data.

This data is then published to Gazebo topics, which can be bridged to ROS 2 topics for our AI nodes to consume.

## How Sensor Plugins Work

A sensor plugin is attached to a link in your URDF/SDF file. The simulator then uses the physics and rendering engines to generate the appropriate data.

-   A **depth camera** sensor uses the rendering engine's depth buffer.
-   A **LiDAR** sensor performs ray-casting into the 3D scene.
-   An **IMU** sensor queries the physics engine for the link's velocity and acceleration.

```mermaid
graph TD
    subgraph "Gazebo Simulation"
        A[Physics & Render Engine]
        B(Sensor Plugin)
        C[/gz/topic]
    end
    subgraph "ROS 2"
        D(ros_gz_bridge)
        E[/ros/topic]
        F[AI Perception Node]
    end

    A -- Data --> B;
    B -- Generates --> C;
    C -- Bridged by --> D;
    D -- Translates to --> E;
    E -- Consumed by --> F;
```
*The flow of data from a simulated sensor to a ROS 2 AI node.*

## Attaching Sensors to our Humanoid

Let's add a few common sensors to our humanoid's head link in its URDF file. We use the `<gazebo>` tag to attach the sensor plugins.

### 1. IMU (Inertial Measurement Unit)

An IMU measures orientation, angular velocity, and linear acceleration. It's the robot's sense of balance and motion.

```xml
<!-- Inside the <gazebo reference="head_link"> tag -->
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <visualize>true</visualize>
  <topic>/imu</topic>
</sensor>
```

-   **`type="imu"`:** Specifies the sensor type.
-   **`<update_rate>`:** The frequency (in Hz) at which the sensor publishes data.
-   **`<topic>`:** The Gazebo topic the data will be published on.

### 2. Depth Camera

A depth camera provides an image where each pixel's value is its distance from the camera. This is crucial for 3D perception and obstacle avoidance.

```xml
<!-- Inside the <gazebo reference="head_link"> tag -->
<sensor name="depth_camera" type="depth_camera">
  <camera name="head_depth_camera">
    <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
  <topic>/depth_camera</topic>
</sensor>
```
-   **`type="depth_camera"`:** Specifies the sensor type.
-   **`<horizontal_fov>`:** The camera's field of view.
-   **`<image>`:** The resolution of the output image.
-   **`<clip>`:** The minimum (`near`) and maximum (`far`) distances the camera can measure.

### 3. 2D LiDAR

A LiDAR (Light Detection and Ranging) sensor measures distances by firing out laser beams. A 2D LiDAR typically does this in a single horizontal plane, providing a 360-degree scan of the surroundings.

```xml
<!-- Inside the <gazebo reference="head_link"> tag -->
<sensor name="lidar" type="gpu_lidar">
  <topic>/lidar_scan</topic>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.15</min>
      <max>12.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <visualize>true</visualize>
</sensor>
```
-   **`type="gpu_lidar"`:** We use the GPU-accelerated version for better performance.
-   **`<scan>`:** Defines the properties of the laser scan, such as the number of samples and the angular range.
-   **`<range>`:** The minimum and maximum detection distances.

### Bridging the Sensor Data to ROS 2

Just like with joint commands, we use the `ros_gz_bridge` to make this simulated sensor data available in ROS 2.

**Launch file snippet:**
```python
# Bridge for the IMU data
imu_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/imu@sensor_msgs/msg/Imu@gz.msgs.IMU']
)

# Bridge for the depth camera
depth_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image']
)

# Bridge for the LiDAR scan
lidar_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/lidar_scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan']
)
```
With these plugins added to the robot's model and the bridges running, our ROS 2 AI nodes can now subscribe to `/imu`, `/depth_camera`, and `/lidar_scan` topics as if they were coming from real hardware. This completes the sensory feedback loop of our digital twin.
