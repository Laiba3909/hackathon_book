---
sidebar_position: 6
title: 'Capstone Project: Autonomous Fetch'
---

# Final Capstone Project: Autonomous Fetch

This capstone project integrates every concept from the preceding chapters into a single, impressive demonstration. We will task our humanoid robot with understanding a voice command, navigating to a location, finding a specific object, and bringing it back.

## Project Goal

The robot will start in a standby position. A user will issue a voice command: **"Robot, please fetch the red can from the kitchen table."**

The robot must then execute the following sequence autonomously:

1.  **Listen and Understand:** Transcribe the voice command and use an LLM to parse the intent (`action: "fetch"`, `object: "red can"`, `location: "kitchen table"`).
2.  **Generate a Plan:** Use the LLM to create a sub-task plan: `[NAVIGATE, SCAN, GRASP, RETURN]`.
3.  **Navigate:** Use the Nav2 stack, informed by VSLAM, to travel to the predefined "kitchen table" zone.
4.  **Scan and Identify:** Once at the table, use its head camera and a DNN-based object detector (trained on synthetic data from Isaac Sim) to find the "red can".
5.  **Grasp:** Approach the can and use its manipulation stack (e.g., MoveIt) to execute a pre-canned grasp motion.
6.  **Return:** Navigate back to the user's original location.
7.  **Signal Completion:** Announce that the task is complete.

## System Architecture

This diagram shows how all the pieces we've built work together.

```mermaid
graph TD
    subgraph "User Interaction"
        A[Voice Command] --> B(Whisper ASR Node);
    end

    subgraph "VLA Core (Cognitive Control)"
        B -- Text --> C(Orchestrator Node);
        C -- Prompts --> D{LLM Service};
        D -- Plan --> C;
    end

    subgraph "Navigation Stack"
        E[Nav2 Server]
        F[VSLAM Node (isaac_ros_visual_slam)]
        G[Walking Pattern Generator]
        C -- NAVIGATE Goal --> E;
        E -- cmd_vel --> G;
    end
    
    subgraph "Perception Stack"
        H[DNN Inference Node (isaac_ros_dnn)]
        I[Camera Driver / Sim Feed]
        C -- SCAN Request --> H;
        I --> H;
        I --> F;
    end

    subgraph "Manipulation Stack"
        J[MoveIt Servo / Custom Grasp Controller]
        C -- GRASP Goal --> J;
    end

    subgraph "Robot Hardware / Simulation"
        K[Sensors (Camera, IMU)]
        L[Actuators (Joints)]
        K --> I;
        G --> L;
        J --> L;
    end
```

## Step-by-Step Implementation Guide

### 1. Environment Setup (Chapter 2)
-   Create a world in Gazebo or Isaac Sim that contains a room, a table model, and a model of the "red can".
-   Define a named location "kitchen table" in your semantic map configuration.

### 2. Perception Training (Chapter 3)
-   If using Isaac Sim, use the Synthetic Data Recorder to generate a labeled dataset of the red can on the table under various lighting conditions and camera angles (Domain Randomization).
-   Train a simple object detection model (e.g., YOLO) on this synthetic data. Convert the model to ONNX format.

### 3. Node Integration (Chapters 1, 3, 4)
-   **VLA Orchestrator (`vla_orchestrator_node.py`):**
    -   This is the central node you will write.
    -   It subscribes to `/transcribed_text`.
    -   It contains the logic for calling the LLM, parsing the plan, and managing the state machine.
    -   It needs to be a client for the Nav2, perception, and grasping action/service servers.
-   **Whisper Node (`whisper_ros_node.py`):**
    -   Subscribes to your microphone's audio feed.
    -   Publishes transcriptions to `/transcribed_text`.
-   **Perception Node (`dnn_inference_node`):**
    -   Use the `isaac_ros_dnn_inference` node.
    -   Configure it with your trained ONNX model.
    -   It will subscribe to the robot's camera feed and publish detected bounding boxes.

### 4. Launch File (`capstone_launch.py`)
-   Create a master launch file that starts:
    -   The simulation environment (Gazebo/Isaac Sim) with your world.
    -   The `robot_state_publisher` with the humanoid's URDF.
    -   The full Nav2 stack, configured for your humanoid.
    -   The `isaac_ros_visual_slam` node.
    -   The `isaac_ros_dnn_inference` node.
    -   Your custom `vla_orchestrator_node`.
    -   Your custom `whisper_ros_node`.
    -   All necessary `ros_gz_bridge` instances to connect simulation with ROS 2.

### 5. Execution
1.  Launch the `capstone_launch.py` file.
2.  Wait for all nodes to initialize.
3.  Speak the command into the microphone.
4.  Observe the robot as it executes the plan, watching the log output from the Orchestrator node to see the state transitions.

---

This capstone project is the ultimate test of your understanding of modern, AI-driven robotics. By successfully completing it, you will have demonstrated a mastery of simulation, perception, navigation, and cognitive control, building a system that is truly greater than the sum of its parts.
