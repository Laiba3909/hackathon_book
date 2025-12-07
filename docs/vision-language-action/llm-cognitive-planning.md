---
sidebar_position: 4
title: LLM-Based Cognitive Planning
---

# LLM-Based Cognitive Planning: The Robot's Executive Function

With transcribed text from Whisper, the robot now knows *what* the user said. The next step is to figure out *what to do*. This is the role of the **Cognitive Planner**, a high-level reasoning engine that we will build using a Large Language Model (LLM).

The LLM acts as the robot's "executive function," responsible for task decomposition, resource management, and error handling. It transforms a single, high-level command into a sequence of concrete, machine-executable sub-tasks.

## The Orchestrator Node

The heart of this system is an "Orchestrator" node. This ROS 2 node is the central hub of the VLA system.

```mermaid
graph TD
    A[/transcribed_text] --> B(Orchestrator Node);
    
    subgraph "Orchestrator Node"
        C{LLM for Intent Parsing}
        D{LLM for Task Planning}
        E[State Machine]
    end

    B --> C --> D --> E;
    
    E -- NAVIGATE --> F[Nav2 Action Client];
    E -- SCAN_FOR_OBJECT --> G[Perception Service Client];
    E -- GRASP --> H[Manipulation Action Client];
```

The Orchestrator's workflow:
1.  **Receive Text:** Subscribes to `/transcribed_text` from the Whisper node.
2.  **Parse Intent:** Sends the text to an LLM with a prompt designed to extract structured information (the `action`, `target_object`, etc., as seen previously).
3.  **Generate Plan:** Sends the structured intent to another LLM prompt, this time asking for a numbered list of sub-tasks.
4.  **Execute Plan:** Manages a state machine, executing one sub-task at a time. For each sub-task, it calls the appropriate ROS 2 action server or service and waits for the result.
    -   `NAVIGATE` -> Call the `/navigate_to_pose` action server provided by Nav2.
    -   `SCAN_FOR_OBJECT` -> Call a service that triggers the `isaac_ros_dnn_inference` pipeline to find an object.
    -   `GRASP` -> Call a `/grasp_object` action server provided by the robot's manipulation stack (e.g., MoveIt).
5.  **Handle Contingencies:** If a step fails (e.g., Nav2 fails to find a path, or the object is not found), the Orchestrator can report the failure to the LLM and ask for a new plan. ("I could not find a path to the kitchen. What should I do?").

## Example: The Orchestrator's Logic

Let's look at a simplified Python implementation of the Orchestrator's main loop.

```python
# orchestrator_node.py (Simplified)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# ... import action clients for Nav2, etc.

# Assume we have functions to interact with an LLM API
# def parse_intent_with_llm(text: str) -> dict:
# def generate_plan_with_llm(intent: dict) -> list[str]:

class VLAOrchestrator(Node):
    def __init__(self):
        super().__init__('vla_orchestrator')
        self.subscription = self.create_subscription(
            String,
            '/transcribed_text',
            self.text_callback,
            10)
        self.get_logger().info("VLA Orchestrator is ready.")
        self.current_plan = []
        self.is_executing = False

    def text_callback(self, msg):
        if self.is_executing:
            self.get_logger().info("Already executing a plan. Ignoring new command.")
            return

        self.get_logger().info(f"Received command: '{msg.data}'")
        self.is_executing = True

        # 1. Parse Intent
        intent = parse_intent_with_llm(msg.data)
        self.get_logger().info(f"Parsed intent: {intent}")

        # 2. Generate Plan
        self.current_plan = generate_plan_with_llm(intent)
        self.get_logger().info(f"Generated plan: {self.current_plan}")

        # 3. Execute Plan
        self.execute_next_step()

    def execute_next_step(self):
        if not self.current_plan:
            self.get_logger().info("Plan complete!")
            self.is_executing = False
            return

        step = self.current_plan.pop(0)
        self.get_logger().info(f"Executing step: {step}")

        # Parse the step and call the appropriate action/service
        if step.startswith("NAVIGATE"):
            # ... call Nav2 action server ...
            # On completion, call self.execute_next_step() again
            pass
        elif step.startswith("SCAN_FOR_OBJECT"):
            # ... call perception service ...
            # On completion, call self.execute_next_step()
            pass
        elif step.startswith("GRASP"):
            # ... call manipulation action server ...
            # On completion, call self.execute_next_step()
            pass
        else:
            self.get_logger().error(f"Unknown plan step: {step}")
            self.is_executing = False

# ... main function and rclpy boilerplate ...
```

## Prompt Engineering is Key

The success of this entire system hinges on the quality of the prompts given to the LLM.

-   **System Prompt:** The initial instruction that tells the LLM its role ("You are a helpful robot assistant...").
-   **Few-Shot Examples:** Providing 2-3 examples of user commands and the desired JSON/plan output within the prompt dramatically improves the LLM's accuracy and reliability.
-   **Constraining the Output:** Explicitly telling the LLM the exact format for its response (e.g., "Use only these available sub-tasks: ...") prevents it from hallucinating impossible actions.

By using an LLM as a flexible, high-level planner, we create a system that is not brittle. It can adapt to a wide range of commands and can even be extended with new capabilities simply by adding a new sub-task to its list of known actions and describing it in the LLM's prompt.
