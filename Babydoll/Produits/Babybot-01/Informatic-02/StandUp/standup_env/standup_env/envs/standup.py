from enum import Enum
import gymnasium as gym
from gymnasium import spaces
import pygame
import numpy as np
import pybullet as p
import pybullet_data
import math
import importlib.resources
import time

'''
Joints range [-pi, pi], 0 is straight. positive goes u:left, v:down , w:down
Normalize from [-1,1] to [-pi/2, pi/2], give joints w and offset of pi/2

Task:
Start on the floor: u=0, v=-1, w=0
Goal: Stand up (pos z = 0.2)
Fail: orientation > 0.1 rad
Reward: -1 for each step, +100 for success
max timestep: 500

observation: joints position, pos z, orientation
action: continuousincrement [-1,1] for each joint (increment rate of 0.1 is applyed)

'''

max_angle = 0.1
z_goal = 0.2
start_height = 0.15

class StandUpEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array",None], "render_fps": 4}

    def __init__(
            self, 
            render_mode='human',
            sim_freq = 60,
            skip_frame = 10,
            debug_mode=False
            ):
        super().__init__()
        self.render_mode = render_mode
        self.sim_freq = sim_freq
        self.skip_frame = skip_frame
        self.debug_mode = debug_mode

        self.time_step = 1.0 / sim_freq
        self.step_ = 0

        # PyBullet
        print("render", self.render_mode)
        p.connect(p.GUI if self.render_mode == "human" else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)  # Use step simulation for better control
        p.setTimeStep(self.time_step)  # Set a higher time step for smoother simulation
        p.setPhysicsEngineParameter(numSolverIterations=50) 
        camera_target_position = [0, 0, 0] # Position to look at
        camera_distance = 1 # Distance from the target
        camera_yaw = 165 # Yaw angle
        camera_pitch = -40 # Pitch angle

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

        # Load the plane and the mill URDF model
        plane_id = p.loadURDF("plane.urdf", useFixedBase=1)
        #self.sphere_id = p.loadURDF("sphere_small.urdf", useFixedBase=1)
        urdf_path = importlib.resources.files('standup_env.data') / 'quadruped.urdf'
        self._robot_id = p.loadURDF(str(urdf_path), useFixedBase=0, basePosition=[0, 0, start_height], baseOrientation=[0, 0, 0, 1],)

        # debug
        #self._button_id = p.addUserDebugParameter("value",-1.0,1.0,0.0)

        # Joints info
        joint_index = {}
        self._revolute_joint_index = {}
        self.end_effector_index = {}
        numJoints = p.getNumJoints(self._robot_id)
        for i in range(numJoints):
            jointInfo = p.getJointInfo(self._robot_id, i)
            key = jointInfo[1].decode('utf-8')
            joint_index[key] = i
            if key[0] == 'u' or key[0] == 'v' or key[0] == 'w':
                self._revolute_joint_index[key] = i
            elif key[0] == 'x':
                self.end_effector_index[key] = i
        num_revolute_joint = len(self._revolute_joint_index)

        # Change dynamics
        for idx in self._revolute_joint_index.values():
            p.changeDynamics(
                bodyUniqueId=self._robot_id,
                linkIndex=idx,
                jointDamping=0.1             # Damping specifically for joints
            )
        for idx in self.end_effector_index.values():
            p.changeDynamics(
                bodyUniqueId=self._robot_id,
                linkIndex=idx,
                lateralFriction=1,         # Lateral friction coefficient
                contactStiffness=10.0,     # Stiffness of contact constraints
                contactDamping=0.5,          # Damping of contact constraints
            )

        # Debug
        if debug_mode == True:
            self.cursors = []
            for key, value in self._revolute_joint_index.items():
                self.cursors.append(p.addUserDebugParameter(f"Revolute joint {key}", -np.pi , np.pi, 0))
        
        # observation and action space
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(num_revolute_joint + 2,), dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(num_revolute_joint,), dtype=np.float32)

        self.revolute_joint_position = {}

        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode

    def joints_position_env2bullet(self, revolute_joint_position):
        return {key: value * (np.pi / 2) - (np.pi / 2 if revolute_joint_position == 'w' else 0) for key, value in revolute_joint_position.items()}     

    def angle_between_z_axis(self, quaternion):

        qx, qy, qz, qw = quaternion
        z_axis_robot = np.array([
            2 * (qx * qz + qw * qy),
            2 * (qy * qz - qw * qx),
            1 - 2 * (qx**2 + qy**2)
        ])
        
        # World z-axis
        z_axis_world = np.array([0, 0, 1])
        
        # Compute the dot product and magnitudes of the vectors
        dot_product = np.dot(z_axis_robot, z_axis_world)
        magnitude_robot = np.linalg.norm(z_axis_robot)
        magnitude_world = np.linalg.norm(z_axis_world)
        
        # Compute the angle between the vectors
        angle = np.arccos(dot_product / (magnitude_robot * magnitude_world))
    
        return angle / np.pi

    def _get_obs(self):
        robot_state = p.getLinkState(self._robot_id,0)
        robot_position = list(robot_state[0])
        robot_orientation = list(robot_state[1])
        return  np.array(list(self.revolute_joint_position.values()) + [robot_position[2], self.angle_between_z_axis(robot_orientation)], dtype=np.float32)

    def _get_info(self):

        return {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_ = 0

        # Robot
        
        p.resetBasePositionAndOrientation(self._robot_id,[0,0,start_height],[0,0,0,1])
        for key,value in self._revolute_joint_index.items():
            self.revolute_joint_position[key] = -1 if key[0] == 'w' else 0
            p.resetJointState(self._robot_id, value, self.revolute_joint_position[key])

        observation = self._get_obs()
        info = self._get_info()


        return observation, info

    def step(self, action):

        terminated = False
        reward = 1 if terminated else 0  # Binary sparse rewards
        observation = self._get_obs()
        info = self._get_info()

        #Debug
        if self.debug_mode == True:
            for i, key in enumerate(self._revolute_joint_index.keys()):
                value = p.readUserDebugParameter(self.cursors[i])
                p.setJointMotorControl2(self._robot_id, self._revolute_joint_index[key], p.POSITION_CONTROL, targetPosition=value)

        else:
 

            self.revolute_joint_position = {key: np.clip(value + action[i] * 0.1, -1, 1) for i, (key, value) in enumerate(self.revolute_joint_position.items())}

            bullet_revolute_joints_poistions = self.joints_position_env2bullet(self.revolute_joint_position)
            p.setJointMotorControlArray(self._robot_id, jointIndices=list(self._revolute_joint_index.values()), controlMode=p.POSITION_CONTROL, targetPositions=list(bullet_revolute_joints_poistions.values()), forces=[100]*len(self._revolute_joint_index))

        

        # Get robot position and orientation
        robot_state = p.getLinkState(self._robot_id,0)
        robot_position = list(robot_state[0])
        robot_orientation = list(robot_state[1])
        angle = self.angle_between_z_axis(robot_orientation)

        terminated = True if robot_position[2] >= z_goal else False

        angle_reward = ((max_angle - angle) / max_angle) * 0.5
        z_position_reward = (1 - abs(robot_position[2] - z_goal) / z_goal) * 0.5
        step_penality = -1.0
        
        reward = angle_reward + z_position_reward + step_penality

        # Step simulation
        for _ in range(self.skip_frame):
            p.stepSimulation()

            

            if self.render_mode == 'human':
                # Update camera
                robot_state = p.getLinkState(self._robot_id,0)
                robot_position = list(robot_state[0])
                cam = p.getDebugVisualizerCamera()
                p.resetDebugVisualizerCamera(
                cameraDistance=cam[10],
                cameraYaw=cam[8],
                cameraPitch=cam[9],
                cameraTargetPosition=robot_position)

                # Sleep
                time.sleep(self.time_step)

        self.step_ = self.step_ + 1

        return observation, reward, terminated, False, {**info,"reward:": reward, "angle_reward": angle_reward, "z_position_reward": z_position_reward}


    

    def close(self):
        p.disconnect()
