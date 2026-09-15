# 6D Grasping using HCA and PCA Grippers

This repository contains two **6D robotic grasping pipelines for cluttered waste sorting**, implemented using the **Hybrid Compliance Activation (HCA) gripper** and the **Passive Compliance Activation (PCA) gripper**.

The two pipelines provide a common experimental framework for evaluating and comparing the grasping performance of the HCA and PCA grippers in cluttered environments containing heterogeneous inorganic waste objects.

The repository is organized into two main grasping pipelines:

```text
HCA_gripper - 6D grasping pipeline using the HCA gripper
PCA_gripper - 6D grasping pipeline using the PCA gripper
```

Both pipelines are designed for robotic grasp execution in cluttered waste-sorting scenarios while maintaining a consistent experimental framework for comparison between the two gripper mechanisms.

---

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10.12
- CUDA Toolkit 12.1
- PyTorch 2.1.0

---

## Installation

### Set Up the UR10 Robot Arm

Install the Universal Robots ROS 2 Driver:

[Universal Robots ROS 2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)

### Set Up MoveIt 2

Install **MoveIt 2 version 2.5.9**, compatible with ROS 2 Humble.

### Set Up the RealSense Camera

Refer to the official [RealSense ROS 2 Installation Guide](https://github.com/realsenseai/realsense-ros/blob/ros2-development/README.md#installation-on-ubuntu).

Navigate to **Installation on Ubuntu** and complete:

- **Step 2:** Install the latest Intel® RealSense™ SDK 2.0 using **Option 2**
- **Step 3:** Install the ROS Wrapper for Intel® RealSense™ Cameras

---

# HCA Gripper — 6D Grasping Pipeline

The HCA grasping pipeline integrates the **Hybrid Compliance Activation (HCA) gripper** with the robotic manipulation system to perform 6D grasping of inorganic waste objects in cluttered environments using **AnyGrasp**.

## Prerequisites

Ensure that the **AnyGrasp Docker container** is running in the background.

Install the **AnyGrasp SDK** inside a Docker container, as it supports multiple CUDA and Python versions:

[AnyGrasp SDK](https://github.com/graspnet/anygrasp_sdk)

Replace the provided `demo.py` inside:

```text
anygrasp_sdk/anygrasp_sdk/grasp_detection
```

with the `demo.py` provided for this grasping pipeline.

Clone this repository using:

```bash
git clone https://github.com/Moni9612/6D-Grasping-using-HCA-and-PCA-grippers.git
```

---

## Build the HCA Workspace

Navigate to the HCA gripper workspace:

```bash
cd ~/6D-Grasping-using-HCA-and-PCA-grippers/HCA_gripper
```

Build the workspace:

```bash
colcon build
```

---

## Step 1: Source the Environment

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## Step 2: Launch the System

Launch the required system components, including the **UR10 driver, MoveIt 2, and Intel RealSense D415 camera**:

```bash
ros2 launch static_tf combined_launch.py
```

---

## Step 3: Activate the HCA Gripper

Start the Dynamixel motor control node:

```bash
ros2 run dynamixel_motor dynamixel_position_node
```

---

## Step 4: Launch the 6D Grasping Pipeline

After starting all required nodes and scripts, launch the grasping pipeline in a **separate terminal**.

### Operation Modes

Two modes of operation are available.

#### Single Object Execution

Executes one complete pick-and-place cycle for a single object:

```bash
ros2 launch move_robot move_robot_combined.launch.py
```

#### Multiple Object Execution

Executes continuous pick-and-place cycles for multiple objects in the workspace:

```bash
ros2 launch move_robot move_robot_loop.launch.py
```

> **Note:** Ensure that all prerequisite perception and control nodes are running before launching the grasping pipeline.

---

# PCA Gripper — 6D Grasping Pipeline

The PCA grasping pipeline integrates the **Passive Compliance Activation (PCA) gripper** with the same robotic manipulation framework to provide a baseline for comparison with the HCA gripper.

The PCA pipeline follows the **same setup, launch sequence, and operating procedure described in Steps 1–4 of the HCA pipeline**. The primary difference is that the PCA implementation must be built and executed from the `PCA_gripper` workspace instead of the `HCA_gripper` workspace.

## Build the PCA Workspace

Navigate to the PCA gripper workspace:

```bash
cd ~/6D-Grasping-using-HCA-and-PCA-grippers/PCA_gripper
```

Build the workspace:

```bash
colcon build
```

After building the workspace, follow the **same Steps 1–4 described for the HCA gripper**.

Therefore, the overall operating procedure remains consistent between the HCA and PCA experiments. Only the corresponding gripper workspace needs to be selected before building, sourcing, and executing the pipeline:

```text
HCA Gripper → ~/6D-Grasping-using-HCA-and-PCA-grippers/HCA_gripper

PCA Gripper → ~/6D-Grasping-using-HCA-and-PCA-grippers/PCA_gripper
```

Using the same perception, robot-control, and grasp-execution procedure for both grippers provides a consistent experimental framework for evaluating and comparing their 6D grasping performance in cluttered waste-sorting scenarios.
