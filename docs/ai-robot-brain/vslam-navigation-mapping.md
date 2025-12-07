---
sidebar_position: 4
title: VSLAM, Navigation, and Mapping
---

# VSLAM, Navigation, and Mapping

For a robot to operate autonomously, it needs to answer three fundamental questions:
1.  **"Where am I?"** (Localization)
2.  **"What does the world around me look like?"** (Mapping)
3.  **"How do I get from here to there?"** (Path Planning)

The process of answering the first two questions simultaneously is called **SLAM (Simultaneous Localization and Mapping)**. When the primary sensor used is a camera, it's called **Visual SLAM (VSLAM)**.

## Isaac ROS for VSLAM

Isaac ROS provides a high-performance, GPU-accelerated package for VSLAM called `isaac_ros_visual_slam`. This package takes in stereo camera images and IMU data and produces two key outputs:
1.  **The robot's pose:** Its position and orientation relative to its starting point (`odom` -> `base_link` transform).
2.  **A map of the environment:** This can be a point cloud or an occupancy grid.

### Why VSLAM?

While LiDAR-based SLAM is very accurate, VSLAM has some key advantages for a humanoid robot:
-   **Passive Sensing:** Cameras are small, cheap, and low-power.
-   **Rich Data:** Cameras provide dense, textured information about the world that can be used for more than just mapping (like object recognition).

### The `isaac_ros_visual_slam` Pipeline

```mermaid
graph TD
    subgraph "Sensors"
        A[Stereo Camera]
        B[IMU]
    end

    subgraph "Isaac ROS VSLAM Node"
        C[isaac_ros_visual_slam]
    end

    subgraph "Outputs"
        D[Pose (TF: odom -> base_link)]
        E[Point Cloud Map]
    end

    A -- Image Data --> C;
    B -- IMU Data --> C;
    C -- Publishes --> D;
    C -- Publishes --> E;
```

This package continuously tracks visual features in the camera images and fuses them with the motion data from the IMU to maintain an accurate and robust estimate of the robot's pose while building a map of its surroundings.

## Navigation with Nav2

Once the robot knows where it is and has a map, it needs to be able to navigate. The standard navigation stack in ROS 2 is **Nav2**.

Nav2 is a highly modular system that takes in a map, the robot's current pose, and a goal pose, and outputs velocity commands to move the robot.

### Core Components of Nav2

-   **Global Planner:** Looks at the entire map and finds an optimal path from the robot's current location to the goal, avoiding known obstacles.
-   **Local Planner (Controller):** Looks at a small window of the map around the robot and generates immediate velocity commands to follow the global plan while avoiding nearby obstacles (which may not have been on the original map).
-   **Costmaps:** Nav2 uses two costmaps—a global one and a local one. These are grids where each cell has a value representing the "cost" of traversing it. Obstacles have a high cost, and open space has a low cost. The planners use these costmaps to find the safest path.

### The Navigation Flow

```mermaid
graph TD
    A[Goal Pose] --> B{Global Planner};
    B -- Global Plan --> C{Local Planner};
    C -- Velocity Commands --> D[Robot Base Controller];
    
    subgraph "Nav2 Stack"
        B; C;
        G[Global Costmap];
        H[Local Costmap];
    end
    
    E[VSLAM (Pose)] --> B;
    E --> C;
    E --> G;
    E --> H;
    
    F[Sensor Data (LiDAR/Depth)] --> G;
    F --> H;
```
1.  A user or an AI node provides a **goal pose**.
2.  The **Global Planner** creates a path across the **Global Costmap**.
3.  The **Local Planner** receives the global plan and, using the **Local Costmap** (which is updated frequently with new sensor data), generates `cmd_vel` (command velocity) messages.
4.  The robot's base controller (e.g., a walking pattern generator for a humanoid) subscribes to `cmd_vel` and moves the robot.
5.  The robot's pose from VSLAM is used to update its position on the map, closing the loop.

By combining a fast VSLAM system like `isaac_ros_visual_slam` with a powerful navigation stack like Nav2, we give our humanoid the ability to autonomously and intelligently move through complex, dynamic environments.
