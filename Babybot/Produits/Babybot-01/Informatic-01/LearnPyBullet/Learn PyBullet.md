****

PyBullet: https://pybullet.org/wordpress/
- [x] Guide: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3

- [ ] Robotic tutorial: https://github.com/adityasagi/robotics_tutorial?tab=readme-ov-file
- [x] Quick intro: https://alexanderfabisch.github.io/pybullet.html
- [x] Tuto in colab: https://www.sagargv.com/blog/pybullet-colab-tutorials/
- [ ] RL: https://github.com/bulletphysics/bullet3/releases/tag/2.87
- [x] ==Guide== : [[PyBullet Quickstart Guide.pdf]]

Reproduce and understand all the examples of pybullet.
- [ ] PyBullet Examples: https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/examples
****

pip install pybullet 
pip install numpy\==1.23.5
pip install --upgrade pip setuptools wheel




Code memo

```Python
import pybullet
import pybullet_data

pybullet.connect(pybullet.GUI) # p.DIRECT
pybullet.resetSimulation()
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

plane = pybullet.loadURDF("plane")

pybullet.setGravity(0,0,-9.81)
pybullet.setTimeStep(0.0001)
pybullet.setRealTimeSimulation(0) #1 = realtime

pybullet.getNumJoints(robot_id)

# Parameters
angularDampingSlider = p.addUserDebugParameter("angularDamping", 0, 1, 0)
angularDamping = p.readUserDebugParameter(angularDampingSlider)

# Processing
pybullet.calculateInverseKinematics(robot_id, kuka_end_effector_idx, target_pos)
pybullet.getQuaternionFromEuler([0,1.01*math.pi, 0])

# Control joints
pybullet.resetJointState(kuka_id, jointIndex, jointPositions[jointIndex])
pybullet.setJointMotorControl2(bodyIndex=robot_id, jointIndex=j, controlMode=pybullet.POSITION_CONTROL, targetPosition=joint_poses[j])
		
pybullet.setJointMotorControlArray(robot, range(6), p.POSITION_CONTROL, targetPositions= [0.1]*6)

for i in range(1e5):
	pybullet.stepSimulation()

pybullet.disconnect()
```

Camera
```Python
# Set the camera parameters
camera_target_position = [0, 0, 0] # Position to look at
camera_distance = 5 # Distance from the target
camera_yaw = 50 # Yaw angle
camera_pitch = -30 # Pitch angle

# Calculate the camera position
camera_position = [
camera_target_position[0] + camera_distance * math.sin(math.radians(camera_yaw)) * math.cos(math.radians(camera_pitch)),
camera_target_position[1] + camera_distance * math.sin(math.radians(camera_pitch)),
camera_target_position[2] + camera_distance * math.cos(math.radians(camera_yaw)) * math.cos(math.radians(camera_pitch))
]
# Set the camera
p.resetDebugVisualizerCamera(cameraDistance=camera_distance,
cameraYaw=camera_yaw,
cameraPitch=camera_pitch,
cameraTargetPosition=camera_target_position)
```



