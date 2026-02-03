import pybullet as p
import pybullet_data
import numpy as np
import os
import time
import json
from robot import Panda
from teleop import KeyboardController


# parameters
control_dt = 1. / 240.

# create simulation and place camera
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
# disable keyboard shortcuts so they do not interfere with keyboard control
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.resetDebugVisualizerCamera(cameraDistance=1.0, 
                                cameraYaw=40.0,
                                cameraPitch=-30.0, 
                                cameraTargetPosition=[0.5, 0.0, 0.2])

# load the objects
urdfRootPath = pybullet_data.getDataPath()
plane = p.loadURDF(os.path.join(urdfRootPath, "plane.urdf"), basePosition=[0, 0, -0.625])
table = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"), basePosition=[0.5, 0, -0.625])
cube1 = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"), basePosition=[0.6, -0.2, 0.05])
cube2 = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"),
                    basePosition=[0.7, 0.2, 0.05], baseOrientation=p.getQuaternionFromEuler([0, 0, 0.7]))

# load the robot
jointStartPositions = [0.0, 0.0, 0.0, -2*np.pi/4, 0.0, np.pi/2, np.pi/4, 0.0, 0.0, 0.04, 0.04]
panda = Panda(basePosition=[0, 0, 0],
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                jointStartPositions=jointStartPositions)

# teleoperation interface
teleop = KeyboardController()

# import trajectory
with open("sample.json", "r") as f:
    demo = json.load(f)

for waypoint in demo:
    waypoint = np.array(waypoint)
    target_position = waypoint[0:3]
    target_quaternion = waypoint[3:7]
    gripper_open = waypoint[7]

    for idx in range(800):
        panda.move_to_pose(ee_position=target_position, ee_quaternion=target_quaternion, positionGain=0.01)
        p.stepSimulation()
        time.sleep(control_dt)

    if gripper_open:
        for idx in range(300):
            panda.open_gripper()
            p.stepSimulation()
            time.sleep(control_dt)
    else:
        for idx in range(300):
            panda.close_gripper()
            p.stepSimulation()
            time.sleep(control_dt)