

# 🚀 Spot Navigation & RL Controller – Quick Start Guide

This guide explains how to launch the **RL locomotion controller** and **Nav2 navigation stack** for the Spot robot inside **Isaac Sim**.

https://github.com/user-attachments/assets/f34d3a30-2202-4b94-832d-f1564d404dcb

---

## ✅ 1. Start Isaac Sim & Load the Environment

1. Open **Isaac Sim**.
2. Load the scene file:

```
spot_space_data.usd
```

3. Press **Play ▶️** to start the simulation.

---

## ✅ 2. Launch the RL-Based Locomotion Controller

Open a **new terminal** (keep it separate from other terminals):

```bash
ros2 launch spot_isaac_driver spot_controller.launch.py
```

This starts the RL policy controller that drives Spot’s legs and publishes `/cmd_vel`-compatible motion.

---

## ✅ 3. Launch the Navigation Stack (Nav2)

Open another **separate terminal**:

```bash
ros2 launch spot_nav spot_nav.launch.py
```

This brings up:

* AMCL localization
* Nav2 planners
* Costmaps
* MPPI controller
* TF tree
* RViz2 configuration (if included)

---

## 4. Rviz (visualization)

```bash
rviz2 
```

>Note: you can also use predefine config file in ``spot_nav`` package
##  Important Step Before Sending Goals

Before issuing any navigation goal:

👉 **Move the robot a little using either keyboard or joystick.**

This “wakes up” the system by making sure:

* `/cmd_vel` is active
* MPPI controller is initialized
* TF tree stabilizes
* Odometry starts flowing

If you skip this, Nav2 may think the robot is stationary or not ready.

---

##  4. Send Navigation Goal (RViz2)

Once the robot is moving:

1. Open **RViz2** (if not auto-opened).
2. Select **2D Goal Pose**.
3. Click a destination on the map.

Spot will:

* Compute a path
* Follow it using MPPI
* Avoid obstacles
* Replan if needed

---
