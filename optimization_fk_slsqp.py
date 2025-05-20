import os
import time
import math
import numpy as np
import pybullet as p
import pybullet_data
from scipy.optimize import minimize

# ——— 1. PyBullet Setup ——————————————————————————————————————
# Connect to PyBullet and load simulation environment
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

# Load the Franka Panda robot URDF from local directory
urdf_path = os.path.join("MA1", "panda_arm.urdf")
robot_id = p.loadURDF(urdf_path, useFixedBase=True)

# Set gravity and simulation timestep
p.setGravity(0, 0, -9.81)
p.setTimeStep(1. / 240.)
p.setRealTimeSimulation(0)

# Identify movable joints (revolute or prismatic)
movable_joints = []
for i in range(p.getNumJoints(robot_id)):
    if p.getJointInfo(robot_id, i)[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
        movable_joints.append(i)

nq = len(movable_joints)           # Number of joints
ee_link = movable_joints[-1]       # End-effector link index


# ——— 2. Obstacle Setup ————————————————————————————————————————
# Create a red spherical obstacle away from the desired trajectory
obs_pos = np.array([0.5, 0.5, 0.5])
obs_radius = 0.15
obs_visual = p.createVisualShape(p.GEOM_SPHERE, radius=obs_radius, rgbaColor=[1, 0, 0, 0.7])
p.createMultiBody(baseMass=0, baseVisualShapeIndex=obs_visual, basePosition=obs_pos.tolist())


# ——— 3. Trajectory Generation —————————————————————————————————
# Define desired Cartesian trajectory as a circle in XY plane
T = 15              # Number of time steps
dt = 0.05           # Time step duration
target_traj = []

for k in range(T):
    theta = 2 * math.pi * k / T
    x = 0.25 * math.cos(theta)
    y = 0.25 * math.sin(theta)
    z = 0.6          # Constant height
    target_traj.append([x, y, z])
target_traj = np.array(target_traj)


# ——— 4. Joint Limits & Initial Configuration ——————————————————
# Get joint limits and current joint positions
q0 = np.array([p.getJointState(robot_id, j)[0] for j in movable_joints])
lower, upper = [], []
for j in movable_joints:
    info = p.getJointInfo(robot_id, j)
    lower.append(info[8])
    upper.append(info[9])

lower = np.array(lower * T)     # Flattened for optimization
upper = np.array(upper * T)


# ——— 5. Forward Kinematics Helper ———————————————————————————
def compute_fk(q_mat):
    """
    Given joint trajectories q_mat (T x nq),
    returns end-effector positions (T x 3).
    """
    ee_positions = []
    # Save current robot state
    saved = [p.getJointState(robot_id, j)[0] for j in movable_joints]

    for q in q_mat:
        # Apply joint angles
        for i, j in enumerate(movable_joints):
            p.resetJointState(robot_id, j, float(q[i]))
        pos = p.getLinkState(robot_id, ee_link)[0]
        ee_positions.append(pos)

    # Restore saved state
    for i, j in enumerate(movable_joints):
        p.resetJointState(robot_id, j, saved[i])

    return np.array(ee_positions)


# ——— 6. Cost Function ————————————————————————————————————————
def cost(x):
    """
    Objective function for trajectory optimization:
    - Tracks the desired Cartesian path
    - Penalizes non-smooth joint motion
    - Adds penalty near the obstacle
    """
    x = x.reshape(T, nq)
    total = 0.0
    ee_positions = compute_fk(x)

    for k in range(T):
        pe = ee_positions[k]

        # Term 1: Path tracking error (position)
        total += 20.0 * np.linalg.norm(pe - target_traj[k])**2

        # Term 2: Joint smoothness penalty
        if k > 0:
            dq = x[k] - x[k-1]
            total += 1.0 * np.sum(dq**2)

        # Term 3: Obstacle penalty
        dist = np.linalg.norm(pe - obs_pos)
        if dist < obs_radius + 0.15:
            total += 300.0 / (dist + 1e-3)

    return total


# ——— 7. Optimization ———————————————————————————————————————
# Solve for joint trajectories minimizing the cost function
x0 = np.tile(q0, T)    # Initial guess (repeat initial pose)
res = minimize(cost,
               x0,
               method='SLSQP',
               bounds=list(zip(lower, upper)),
               options={'disp': True, 'maxiter': 300})

# Check for success
if not res.success:
    raise RuntimeError("Optimization failed: " + res.message)

traj = res.x.reshape(T, nq)


# ——— 8. Playback in PyBullet ———————————————————————————————
print("Executing smoothed trajectory...")

for q_t in traj:
    for i, j in enumerate(movable_joints):
        p.setJointMotorControl2(robot_id, j,
                                p.POSITION_CONTROL,
                                targetPosition=q_t[i],
                                force=300)
    p.stepSimulation()
    time.sleep(dt)

p.disconnect()
