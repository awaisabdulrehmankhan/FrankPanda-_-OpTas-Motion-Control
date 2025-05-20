# Franka Emika Panda Motion Planning - MA1 Project

This repository contains two motion planning implementations for the Franka Emika Panda robot in PyBullet, developed for the MA1 project.

## 📁 Contents

- `ik_obstacle_avoidance.py`:  
  Implements inverse kinematics-based reactive obstacle avoidance. The robot end-effector follows a horizontal circular trajectory, and a virtual tree obstacle can be repositioned using sliders. When the obstacle gets close, the robot reacts by lifting its trajectory while maintaining smooth motion.

- `optimization_fk_slsqp.py`:  
  Implements trajectory generation using **SciPy (SLSQP)** and **forward kinematics**. The robot plans a smooth joint-space trajectory that tracks a desired circular path while avoiding a spherical obstacle. Obstacle constraints and joint limits are considered.

- `report.pdf`:  
  A detailed write-up explaining the project, methodology, and observations.

## 🧪 Requirements

- Python 3.x  
- `pybullet`  
- `numpy`  
- `scipy`  

Install dependencies using:

```bash
pip install pybullet numpy scipy
```

## ▶️ How to Run

Clone the repository and ensure you have the URDF and scripts in the correct structure:

```
your_project/
├── MA1/
│   └── panda_arm.urdf
├── ik_obstacle_avoidance.py
├── optimization_fk_slsqp.py
└── report.pdf
```

To run either script:

```bash
python ik_obstacle_avoidance.py
# or
python optimization_fk_slsqp.py
```

## 📝 Features Overview

### ✅ Inverse Kinematics Reactive Avoidance (`ik_obstacle_avoidance.py`)
- Horizontal circular motion of end-effector (like a helicopter blade).
- A virtual obstacle (tree) can be moved interactively.
- Robot avoids the obstacle in real-time by lifting its trajectory vertically.
- Includes a dynamic "proximity meter" showing how close the robot gets.

### ✅ Optimization-Based Motion Planning (`optimization_fk_slsqp.py`)
- Plans smooth joint-space trajectories from a start pose to a goal trajectory.
- Uses forward kinematics in the optimization cost function.
- Penalizes:
  - Deviation from target trajectory
  - Joint motion smoothness (Δq²)
  - Proximity to obstacle
- Uses SciPy's `SLSQP` optimizer with joint limit constraints.
- Visual playback of the optimized motion in PyBullet.


---
Developed by **Awais Abd Ul Rehman**  
**Note:** This project is developed for academic purposes and may not be optimized for real-time robotics applications.
