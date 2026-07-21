# 🤖 ABB IRB 4600 Trajectory Planner (ROS 2)

![ROS 2](https://img.shields.io/badge/ROS_2-Humble%2FFoxy-blue)
![C++17](https://img.shields.io/badge/C%2B%2B-17-blue)
![IKFast](https://img.shields.io/badge/IKFast-Kinematics-orange)
![RViz2](https://img.shields.io/badge/RViz2-Visualization-green)
![Eigen3](https://img.shields.io/badge/Eigen-3-orange)

A ROS 2 application written in **C++17** for simulating the **ABB IRB 4600** industrial robot. The project demonstrates trajectory planning for machining operations, inverse kinematics using **IKFast**, and interactive robot teaching through RViz2 interactive markers.

> 📷 **Screenshot**
> <img width="1102" height="829" alt="Screenshot from 2026-07-21 22-02-33" src="https://github.com/user-attachments/assets/2ca65dbe-db69-4128-88a1-cdc9f712e64c" />
> <img width="1102" height="829" alt="Screenshot from 2026-07-21 22-03-35" src="https://github.com/user-attachments/assets/9e2b387b-9bf6-4a7c-a8b5-65c3fb57f284" />
> <img width="1102" height="829" alt="Screenshot from 2026-07-21 22-02-45" src="https://github.com/user-attachments/assets/babcbc66-3dab-43ef-8183-c8478bbcb06d" />


> *(Add an RViz2 screenshot here by dragging the image into the GitHub editor.)*

---

## 🚀 Key Features

* 🏭 **Trajectory Planning:** Generates smooth robot motion using both **Point-to-Point (PTP)** and **Linear (LIN)** interpolation with velocity and acceleration control.
* 🎯 **IKFast Integration:** Uses a generated IKFast solver for extremely fast inverse and forward kinematics calculations.
* 🖱 **Interactive Pose Teaching:** Provides an interactive RViz2 marker for manually defining robot poses (X, Y, Z, Roll, Pitch, Yaw). Joint angles are automatically computed and displayed in the terminal.
* 📦 **Dynamic Workcell:** Automatically loads a 3D workpiece model (`truck.stl`) into the robot workspace.
* 🎨 **Trajectory Visualization:** Displays the executed tool path in RViz2 using colored markers.
* ⚡ **Service-Based Execution:** Starts machining trajectories on demand through a ROS 2 service.

---

## 📁 Repository Structure

```text
abb_trajectory_planner/
├── my_robot_project/      # Trajectory planner, pose teacher, model loader
├── abb_irb4600_ikfast/    # Generated IKFast kinematics library
└── abb_model/             # URDF and mesh files for ABB IRB 4600
```

---

## 🛠 Requirements

* Ubuntu 22.04 or newer
* ROS 2 (tested on Humble and Foxy)
* C++17
* Eigen3
* RViz2
* IKFast plugin
* `interactive_markers`

---

## ⚙️ Installation & Build

### Install required packages

```bash
sudo apt update

sudo apt install \
    python3-colcon-common-extensions \
    libyaml-cpp-dev
```

### Clone the repository

```bash
mkdir -p ~/abb_ws/src
cd ~/abb_ws/src

git clone https://github.com/YOUR_GITHUB_USERNAME/YOUR_REPOSITORY.git
```

Replace `YOUR_GITHUB_USERNAME` and `YOUR_REPOSITORY` with your GitHub repository.

### Install ROS 2 dependencies

```bash
cd ~/abb_ws

rosdep update

rosdep install \
    -i \
    --from-path src \
    --rosdistro humble \
    -y
```

Replace `humble` with your ROS 2 distribution if necessary.

### Build the workspace

```bash
colcon build

source install/setup.bash
```

---

## 🎮 Usage

### 1. Launch the simulation

Start the complete environment:

* ABB IRB 4600 model
* RViz2 visualization
* Interactive pose teacher
* Trajectory planner

```bash
ros2 launch my_robot_project project_launch.py
```

The robot will appear in its **Home** position and wait for user interaction.

---

### 2. Interactive Pose Teaching

An interactive 3D marker will appear in RViz2.

Move the marker freely to define the desired tool pose:

* X
* Y
* Z
* Roll
* Pitch
* Yaw

The robot follows the marker in real time using the IKFast solver.

When the marker is released, the corresponding joint angles are printed to the terminal. These values can be copied directly into the trajectory planner as target poses (e.g., **T1**, **T3**, or **Tvia**).

---

### 3. Execute the Machining Trajectory

Open a second terminal, source the workspace, and call the execution service:

```bash
source ~/abb_ws/install/setup.bash

ros2 service call /execute_machining std_srvs/srv/Trigger {}
```

The robot will:

1. Execute the programmed machining sequence.
2. Perform both **PTP** and **LIN** motions.
3. Draw the executed tool path in RViz2.
4. Return to the Home position after completion.

---

## 🧠 System Architecture

1. Load the ABB IRB 4600 robot model.
2. Load the machining workpiece into the workspace.
3. Teach robot poses interactively using RViz2.
4. Compute inverse kinematics with IKFast.
5. Generate smooth PTP and LIN trajectories.
6. Execute the trajectory through a ROS 2 service.
7. Visualize the resulting tool path in RViz2.

---

## 📚 Technologies

| Category          | Technology          |
| ----------------- | ------------------- |
| Language          | C++17               |
| Middleware        | ROS 2               |
| Robot Model       | ABB IRB 4600        |
| Kinematics        | IKFast              |
| Mathematics       | Eigen3              |
| Visualization     | RViz2               |
| Interactive Tools | Interactive Markers |
| Build System      | CMake, colcon       |


---

## 📄 License

This project is intended for educational and research purposes.
