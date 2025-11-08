````markdown
#  Spot Isaac Driver

A lightweight ROS 2 package to control **Boston Dynamics Spot** inside **NVIDIA Isaac Sim** using a reinforcement-learning controller.

---

##  Setup & Usage

### 1️⃣ Clone the repository
```bash
cd ~/quad_ws/src
git clone https://github.com/siddarth09/spot_isaac_driver.git
````

### 2️⃣ Build the workspace

```bash
cd ~/quad_ws
colcon build --packages-select spot_isaac_driver
```

### 3️⃣ Source the workspace

```bash
source install/setup.bash
```

### 4️⃣ Open the Spot simulation in Isaac Sim

In Isaac Sim, open the following USD file to load Spot in the warehouse environment:

```
/home/ubuntu/quad_ws/src/spot_isaac_driver/spot_space_data.usd
```

Make sure the **ROS 2 Bridge** is enabled in Isaac Sim so topics like `/joint_states`, `/imu`, and `/cmd_vel` are active.

### 5️⃣ Launch the Spot controller

```bash
ros2 launch spot_isaac_driver spot_controller.launch.py
```

This starts the **Spot Full-Body Controller** node, which runs the trained RL policy and publishes joint commands to Isaac Sim.

### 6️⃣ Run teleoperation

In a new terminal (with your workspace sourced):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use the keyboard to send velocity commands to `/cmd_vel` and move Spot in simulation.

---

 **That’s it!**
Spot should now respond to your teleop commands and walk around the warehouse scene inside Isaac Sim.

```
