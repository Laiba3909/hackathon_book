---
sidebar_position: 2
title: Natural Language to Robotic Actions
---

# From Natural Language to Robotic Actions

The core challenge of a Vision-Language-Action (VLA) system is translating the fluid, often ambiguous nature of human language into the rigid, mathematical world of robot commands. A human might say, "Tidy up the living room," a command that contains a world of unstated assumptions. The robot needs a way to decompose this high-level goal into a sequence of concrete, achievable steps.

This process can be broken down into three main phases: **Understanding**, **Grounding**, and **Planning**.

## 1. Understanding: Parsing the Intent

The first step is to convert the user's utterance into a structured representation of their intent. This is a perfect task for a Large Language Model (LLM). We don't just want a transcript of the words; we want to extract key entities and the core command.

**Input (from Speech-to-Text):** "Hey robot, please bring me the water bottle from the kitchen counter."

We can use an LLM with a carefully crafted prompt to extract a structured output, like JSON.

**Prompt for the LLM:**
```
You are a helpful assistant that extracts robotic commands from natural language.
Given the user's request, identify the 'action', the 'target_object', and the 'target_location'.

User request: "Hey robot, please bring me the water bottle from the kitchen counter."

Output your response in JSON format.
```

**LLM's Structured Output:**
```json
{
  "action": "fetch",
  "target_object": {
    "name": "water bottle",
    "attributes": ["generic"]
  },
  "target_location": {
    "name": "kitchen counter",
    "attributes": ["pre-defined_zone"]
  }
}
```

This structured data is far more useful for a robot than the raw text.

## 2. Grounding: Connecting Words to the World

**Grounding** is the process of linking the concepts from the language model to the robot's physical reality.

-   **`target_location` Grounding:** The planner must connect the string "kitchen counter" to a set of coordinates in its map. This is usually done by having a list of predefined, labeled zones in the environment. The "kitchen counter" might correspond to a bounding box `[x1, y1, x2, y2]` on the global costmap.

-   **`target_object` Grounding:** This is a vision problem. The robot needs to find an object in its camera feed that matches the description "water bottle". This requires a **perception system** (like the DNN-based object detector we discussed in Chapter 3) that has been trained to recognize "water bottles".

The grounding process answers the questions:
-   "Where is the 'kitchen counter'?" -> Looks up coordinates in its semantic map.
-   "What does a 'water bottle' look like?" -> Queries its perception system.

```mermaid
graph TD
    A[LLM Output: {action: "fetch", object: "water bottle"}]
    
    subgraph "Grounding"
        B["Where is the 'kitchen counter'?"] --> C[Semantic Map Lookup];
        C --> D["Coordinates: [x,y,z]"];
        
        E["What is a 'water bottle'?"] --> F[Vision System Query];
        F --> G["Object detector for 'water bottle'"];
    end
    
    A --> B;
    A --> E;
```

## 3. Planning: Creating the Action Sequence

Once the robot has understood the command and grounded the relevant nouns, it must generate a plan. This can be done using another call to an LLM, but this time with a more constrained prompt that asks it to act as a task planner.

**Prompt for the LLM Task Planner:**
```
You are a robotic task planner. Given an action and its grounded parameters, output a sequence of executable sub-tasks.
The available sub-tasks are: 'NAVIGATE(x, y)', 'SCAN_FOR_OBJECT(object_name)', 'GRASP(object_id)', 'RETURN_TO_USER()'.

Goal: {action: "fetch", object: "water bottle", location_coords: "[10.5, 3.2]"}

Output your response as a numbered list.
```

**LLM's Generated Plan:**
```
1. NAVIGATE(10.5, 3.2)
2. SCAN_FOR_OBJECT("water bottle")
3. GRASP("bottle_id_01")
4. RETURN_TO_USER()
```

This numbered list is now a concrete, step-by-step plan that a high-level "Orchestrator" node can execute. The orchestrator would pop the first item from the list, call the appropriate ROS 2 service or action server (e.g., the Nav2 action server for `NAVIGATE`), wait for the result, and then move to the next step.

This three-stage process—**Understand, Ground, Plan**—provides a robust framework for converting abstract human language into a series of executable robotic actions.
