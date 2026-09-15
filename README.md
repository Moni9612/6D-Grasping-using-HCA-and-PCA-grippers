# 6D Grasping using HCA and PCA Grippers

Two **6D robotic grasping pipelines for cluttered waste sorting**, implemented using the **Hybrid Compliance Activation (HCA) gripper** and the **Passive Compliance Activation (PCA) gripper**.

The pipelines provide a common experimental framework for evaluating and comparing the grasping performance of the HCA and PCA grippers in cluttered environments containing heterogeneous inorganic waste objects.

The project is organized into two grasping pipelines:

```text
HCA_gripper - 6D grasping pipeline using the HCA gripper
PCA_gripper - 6D grasping pipeline using the PCA gripper
```

Both pipelines support robotic grasp execution in cluttered waste-sorting scenarios while maintaining a consistent experimental framework for comparison between the two gripper mechanisms.

---

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10.12
- CUDA Toolkit 12.1
- PyTorch 2.1.0

---

## Installation

### 1. Set Up the UR10 Robot Arm

Install the [Universal Robots ROS 2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble) for ROS 2 Humble.

### 2. Set Up MoveIt 2

Install **MoveIt 2 version 2.5.9**, compatible with ROS 2 Humble.

### 3. Set Up the RealSense Camera

Follow the official [RealSense ROS 2 Installation Guide](https://github.com/realsenseai/realsense-ros/blob/ros2-development/README.md#installation-on-ubuntu).

Under **Installation on Ubuntu**, complete:

- **Step 2:** Install the latest Intel® RealSense™ SDK 2.0 using **Option 2**
- **Step 3:** Install the ROS Wrapper for Intel® RealSense™ Cameras

### 4. Set Up AnyGrasp

Install the [AnyGrasp SDK](https://github.com/graspnet/anygrasp_sdk) inside a Docker container to provide the required CUDA and Python environment.

---

## Prerequisites

Ensure that the **AnyGrasp Docker container** is running before launching the grasping pipeline.

Replace the existing `demo.py` located at:

```text
anygrasp_sdk/anygrasp_sdk/grasp_detection
```

with the provided `demo.py`

Clone the project using:

```bash
git clone https://github.com/Moni9612/6D-Grasping-using-HCA-and-PCA-grippers.git
```

---

# HCA Gripper — 6D Grasping Pipeline

The HCA pipeline integrates the **Hybrid Compliance Activation (HCA) gripper** with the robotic manipulation system to perform AnyGrasp-based 6D grasping of inorganic waste objects in cluttered environments.

## Build the HCA Workspace

Navigate to the HCA gripper workspace:

```bash
cd ~/6D-Grasping-using-HCA-and-PCA-grippers/HCA_gripper
```

Build the workspace:

```bash
colcon build
```

## Step 1: Source the Environment

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Step 2: Launch the System

Launch the required system components, including the **UR10 driver, MoveIt 2, and Intel RealSense D415 camera**:

```bash
ros2 launch static_tf combined_launch.py
```

## Step 3: Start External Control from the UR10 Teach Pendant

On the **UR10 teach pendant**, start the **External Control** program to establish control of the robot through the ROS 2 interface.

Ensure that the External Control program is running before proceeding with gripper activation and grasp execution.

## Step 4: Activate the HCA Gripper

Start the Dynamixel motor-control node:

```bash
ros2 run dynamixel_motor dynamixel_position_node
```

## Step 5: Launch the 6D Grasping Pipeline

After starting the required nodes, External Control, and gripper-control interface, launch the grasping pipeline in a **separate terminal**.

### Operation Modes

Two operating modes are available.

#### Single-Object Execution

Executes one complete pick-and-place cycle for a single object:

```bash
ros2 launch move_robot move_robot_combined.launch.py
```

#### Multiple-Object Execution

Executes continuous pick-and-place cycles for multiple objects in the workspace:

```bash
ros2 launch move_robot move_robot_loop.launch.py
```

> **Note:** Ensure that all required perception and control nodes are running and that External Control is active on the UR10 teach pendant before launching the grasping pipeline.

---

# PCA Gripper — 6D Grasping Pipeline

The PCA pipeline integrates the **Passive Compliance Activation (PCA) gripper** with the same robotic manipulation framework, providing a baseline for comparison with the HCA gripper.

The PCA pipeline follows the **same setup, launch sequence, and operating procedure described in Steps 1–5 of the HCA pipeline**. The primary difference is that the PCA implementation is built and executed from the `PCA_gripper` workspace instead of the `HCA_gripper` workspace.

## Build the PCA Workspace

Navigate to the PCA gripper workspace:

```bash
cd ~/6D-Grasping-using-HCA-and-PCA-grippers/PCA_gripper
```

Build the workspace:

```bash
colcon build
```

After building the workspace, follow **Steps 1–5 of the HCA pipeline** using the PCA workspace.

The corresponding workspace should therefore be selected before building, sourcing, and executing each pipeline:

```text
HCA Gripper → ~/6D-Grasping-using-HCA-and-PCA-grippers/HCA_gripper

PCA Gripper → ~/6D-Grasping-using-HCA-and-PCA-grippers/PCA_gripper
```

Maintaining the same perception, robot-control, and grasp-execution procedure across both implementations provides a consistent experimental framework for evaluating and comparing the 6D grasping performance of the HCA and PCA grippers in cluttered waste-sorting scenarios.
