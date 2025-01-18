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




class StandUpEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array",None], "render_fps": 4}

    def __init__(
            self, 
            render_mode='human',
            sim_freq = 60,
            skip_frame = 10,
            ):
        super().__init__()
        self.render_mode = render_mode
        self.sim_freq = sim_freq
        self.skip_frame = skip_frame

        self.observation_space = spaces.Box(low=0, high=1.0, shape=(2,), dtype=int)

        self.action_space = spaces.Discrete(4)

        self.time_step = 1.0 / sim_freq

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
        self.sphere_id = p.loadURDF("sphere_small.urdf", useFixedBase=1)
        urdf_path = importlib.resources.files('standup_env.data') / 'hexapod.urdf'
        self._robot_id = p.loadURDF(str(urdf_path), useFixedBase=0, basePosition=[0, 0, 0.2], baseOrientation=[0, 0, 0, 1],)

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
        self._num_revolute_joint = len(self._revolute_joint_index)

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


        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode

    def _get_obs(self):
        return [0,0]

    def _get_info(self):
        return {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # Robot
        
        p.resetBasePositionAndOrientation(self._robot_id,[0,0,0.2],[0,0,0,1])
        for i in self._revolute_joint_index.values():
            p.resetJointState(self._robot_id, i, 0)

        observation = self._get_obs()
        info = self._get_info()


        return observation, info

    def step(self, action):

        terminated = False
        reward = 1 if terminated else 0  # Binary sparse rewards
        observation = self._get_obs()
        info = self._get_info()

        # Get robot position and orientation
        robot_state = p.getLinkState(self._robot_id,0)
        robot_position = list(robot_state[0])
        robot_orientation = list(robot_state[1])

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


        return observation, reward, terminated, False, info


    

    def close(self):
        p.disconnect()
