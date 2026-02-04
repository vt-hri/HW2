import pybullet as p
import pybullet_data
import numpy as np
import os
import time
import json
from robot import Panda
from teleop import KeyboardController
from cameras import ExternalCamera
import matplotlib.pyplot as plt
import argparse
import os

# create the parser
parser = argparse.ArgumentParser()
parser.add_argument('--number', type=int, default=0, help='An integer input')
args = parser.parse_args()
savename = "demos/demo" + str(args.number) + ".json"
foldername = "demos/images/demo" + str(args.number) + "/"
os.makedirs(foldername, exist_ok=True)

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
cube_x = np.random.uniform(0.4, 0.6)
cube_y = np.random.uniform(-0.3, +0.3)
plane = p.loadURDF(os.path.join(urdfRootPath, "plane.urdf"), basePosition=[0, 0, -0.625])
table = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"), basePosition=[0.5, 0, -0.625])
cube = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"), basePosition=[cube_x, cube_y, 0.025])

# load the robot
jointStartPositions = [0.0, 0.0, 0.0, -2*np.pi/4, 0.0, np.pi/2, np.pi/4, 0.0, 0.0, 0.04, 0.04]
panda = Panda(basePosition=[0, 0, 0],
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                jointStartPositions=jointStartPositions)

# teleoperation interface
teleop = KeyboardController()

# camera for logging
external_camera = ExternalCamera(cameraDistance=1.0, 
                                    cameraYaw=40.0, 
                                    cameraPitch=-30.0,
                                    cameraTargetPosition=[0.5, 0.0, 0.2],
                                    cameraWidth=512, cameraHeight=512)

# run simulation
# you can teleoperate the robot using the keyboard;
# see "teleop.py" for the mapping between keys and motions
state = panda.get_state()
target_position = state["ee-position"]
target_quaternion = state['ee-quaternion']
last_press = time.time()
gripper_open = 1.0
demo = []
score = 0
while True:
    # update the target pose
    action = teleop.get_action()
    target_position = target_position + action[0:3]
    target_quaternion = p.multiplyTransforms([0, 0, 0], p.getQuaternionFromEuler(action[3:6]),
                                                [0, 0, 0], target_quaternion)[1]
    # move to the target pose
    panda.move_to_pose(ee_position=target_position, ee_quaternion=target_quaternion)

    # open or close the gripper
    if action[6] == +1:
        panda.open_gripper()
        gripper_open = +1.0
    elif action[6] == -1:
        panda.close_gripper()
        gripper_open = -1.0

    # print when "." is pressed
    if action[7] == +1 and time.time() - last_press > 0.5:
        image = external_camera.get_image()
        imagename = foldername + str(len(demo)) + '.png'
        plt.imsave(imagename, image)
        print("button pressed")
        last_press = time.time()
        state = panda.get_state()
        robot_position = np.array(state["ee-position"])
        cube_position = p.getBasePositionAndOrientation(cube)[0]
        state_score = -np.linalg.norm(robot_position - cube_position) + (cube_position[2] - 0.025)
        score = score + state_score
        record_state = list(robot_position) + list(cube_position) + [gripper_open]
        demo.append(record_state)
        with open(savename, "w") as f:
            json.dump({"trajectory": demo, "score": score}, f)

    # step simulation
    p.stepSimulation()
    time.sleep(control_dt)