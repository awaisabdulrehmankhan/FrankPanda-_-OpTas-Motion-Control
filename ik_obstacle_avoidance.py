import pybullet as p
import pybullet_data
import os
import time
import math
import numpy as np

# ——— 1. Setup Simulation Environment ——————————————————————
# Connect to PyBullet GUI
p.connect(p.GUI)

# Add default data path (for plane.urdf, textures, etc.)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load a ground plane
p.loadURDF("plane.urdf")

# Load the Franka Emika Panda robot from URDF
urdf_path = os.path.join("MA1", "panda_arm.urdf")  # Adjust path if needed
robot_id = p.loadURDF(urdf_path, useFixedBase=True)

# Setup camera for better visualization
p.resetDebugVisualizerCamera(cameraDistance=1.2,
                             cameraYaw=60,
                             cameraPitch=-30,
                             cameraTargetPosition=[0, 0, 0.5])

# Set physics parameters
p.setGravity(0, 0, -9.81)
p.setPhysicsEngineParameter(numSolverIterations=50)
time_step = 1. / 240.
p.setTimeStep(time_step)
p.setRealTimeSimulation(0)

# Add joint damping for smooth and stable motion
for jid in range(p.getNumJoints(robot_id)):
    p.changeDynamics(robot_id, jid, jointDamping=0.1)


# ——— 2. Add Obstacle (Tree) & GUI Sliders ——————————————————
# Get bounding box of the robot to compute approximate height
aabb_min, aabb_max = p.getAABB(robot_id)
robot_h = aabb_max[2] - aabb_min[2]
trunk_h = robot_h / 2.0  # Tree trunk height as half of robot's height

# Create tree trunk (cylinder)
trunk_r = 0.1
trunk_col = p.createCollisionShape(p.GEOM_CYLINDER,
                                   radius=trunk_r,
                                   height=trunk_h)
trunk_vis = p.createVisualShape(p.GEOM_CYLINDER,
                                radius=trunk_r,
                                length=trunk_h,
                                rgbaColor=[0.55, 0.27, 0.07, 1])  # Brown color
trunk_pos = [0.5, 0, trunk_h * 0.5]
p.createMultiBody(baseMass=0,
                  baseCollisionShapeIndex=trunk_col,
                  baseVisualShapeIndex=trunk_vis,
                  basePosition=trunk_pos)

# Create foliage (sphere on top of trunk)
fol_r = trunk_h * 2  # Bigger than trunk for visibility
fol_col = p.createCollisionShape(p.GEOM_SPHERE, radius=fol_r)
fol_vis = p.createVisualShape(p.GEOM_SPHERE,
                              radius=fol_r,
                              rgbaColor=[0, 1, 0, 1])  # Green
fol_pos = [trunk_pos[0], trunk_pos[1], trunk_pos[2] + fol_r]
tree_id = p.createMultiBody(baseMass=0,
                            baseCollisionShapeIndex=fol_col,
                            baseVisualShapeIndex=fol_vis,
                            basePosition=fol_pos)

# Add GUI sliders to move the tree during runtime
slider_x = p.addUserDebugParameter("Obs X", -0.5, 0.5, fol_pos[0])
slider_y = p.addUserDebugParameter("Obs Y", -0.5, 0.5, fol_pos[1])
slider_z = p.addUserDebugParameter("Obs Z", 0.0, 1.0, fol_pos[2])


# ——— 3. Robot Motion Parameters ————————————————————————
ee_link = p.getNumJoints(robot_id) - 1  # End-effector link index

# Define a circular trajectory (like a helicopter blade)
radius_circle = 1
omega = 10                  # Angular velocity for the circle
center = np.array([0.0, 0.0, 0.5])  # Circle center in 3D

# Exponential smoothing parameters
beta = 0.9
filtered_tgt = center + np.array([radius_circle, 0, 0])  # Initial target

# Obstacle avoidance parameters
avoid_dist = 0.25          # Desired minimum distance from obstacle
lift_strength = 0.02       # Gain for lifting robot to avoid collision

# Proximity meter parameters (visual indicator)
meter_start = np.array([0.0, -0.7, 1.0])
meter_end   = meter_start + np.array([0.6, 0.0, 0.0])  # Line length = 0.6

# Time variable for simulation
t = 0.0

print("Running: helicopter sweep. Slide the sphere into path to see avoidance + meter.")

try:
    while True:
        # — 3.1 Generate circular target motion
        t += time_step
        x = center[0] + radius_circle * math.cos(omega * t)
        y = center[1] + radius_circle * math.sin(omega * t)
        raw = np.array([x, y, center[2]])

        # — 3.2 Update obstacle position from sliders
        obs = [p.readUserDebugParameter(slider_x),
               p.readUserDebugParameter(slider_y),
               p.readUserDebugParameter(slider_z)]
        p.resetBasePositionAndOrientation(tree_id, obs, [0,0,0,1])

        # — 3.3 Obstacle proximity check and avoidance
        threshold = fol_r + avoid_dist  # Trigger distance for reaction
        pts = p.getClosestPoints(robot_id, tree_id, threshold)

        if pts:
            # Get minimum distance to obstacle
            dmin = min(pt[8] for pt in pts)
            # Compute vertical lift (Z direction only)
            lift = lift_strength / (dmin**2 + 1e-6)
            corrected = raw + np.array([0, 0, lift])
        else:
            # No obstacle near → no correction
            dmin = threshold
            corrected = raw

        # — 3.4 Smooth the motion target and compute IK
        filtered_tgt = beta * filtered_tgt + (1 - beta) * corrected
        target = filtered_tgt.tolist()
        joints = p.calculateInverseKinematics(robot_id,
                                              ee_link,
                                              targetPosition=target)
        # Apply joint positions using position control
        for jid, angle in enumerate(joints):
            p.setJointMotorControl2(robot_id, jid,
                                    p.POSITION_CONTROL,
                                    targetPosition=angle,
                                    force=500)

        # — 3.5 Draw proximity meter (visual feedback)
        ratio = max(0.0, min(1.0, (threshold - dmin) / threshold))
        
        # Background (gray bar)
        p.addUserDebugLine(meter_start.tolist(),
                           meter_end.tolist(),
                           lineColorRGB=[0.2, 0.2, 0.2],
                           lifeTime=time_step,
                           lineWidth=4)

        # Foreground (dynamic color: green to red)
        fg_end = meter_start + (meter_end - meter_start) * ratio
        color = [1 - ratio, ratio, 0]  # Red when close, green when far
        p.addUserDebugLine(meter_start.tolist(),
                           fg_end.tolist(),
                           lineColorRGB=color,
                           lifeTime=time_step,
                           lineWidth=6)

        # — 3.6 Step simulation and sleep
        p.stepSimulation()
        time.sleep(time_step)

except KeyboardInterrupt:
    # Graceful exit
    p.disconnect()
