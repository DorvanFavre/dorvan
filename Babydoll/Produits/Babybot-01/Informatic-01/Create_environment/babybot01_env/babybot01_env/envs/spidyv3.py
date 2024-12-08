from enum import Enum
import gymnasium as gym
from gymnasium import spaces
import pygame
import numpy as np
import time
import pybullet as p
import pybullet_data
import math
import importlib.resources


SIM_F = 60.0
TIME_STEP = 1.0/SIM_F
OBSERVATION_TYPE=np.float32
ACTION_TYPE = np.float32




class SpidyEnvV3(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array", None]}

    def __init__(self, render_mode=None):
        super().__init__()
        
        # self.observation_space = spaces.Dict(
        #     {
        #     "joints_position": spaces.Box(low=POSITION_MIN,high= POSITION_MAX, shape=(18,), dtype=OBSERVATION_TYPE)
        #     }
        # )
        self.observation_space = spaces.Box(low=-1.0,high= 1.0, shape=(18,), dtype=OBSERVATION_TYPE)
        #self._joints_position = np.zeros(18, dtype=OBSERVATION_TYPE)

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(18,), dtype=ACTION_TYPE)

        self._last_observation = np.zeros(18)
        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode

        self._distance = 0.0

        # PyBullet
        p.connect(p.GUI if self.render_mode == "human" else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)  # Use step simulation for better control
        p.setTimeStep(TIME_STEP)  # Set a higher time step for smoother simulation
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
        plane_id = p.loadURDF("plane.urdf")
        urdf_path = importlib.resources.files('babybot01_env.data') / 'hexapod.urdf'
        self._robot_id = p.loadURDF(str(urdf_path), useFixedBase=0, basePosition=[0, 0, 0.2], baseOrientation=[0, 0, 0, 1],)

        # debug
        #self._button_id = p.addUserDebugParameter("value",-1.0,1.0,0.0)

        # Joints info
        joint_index = {}
        self._revolute_joint_index = {}
        end_effector_index = {}
        numJoints = p.getNumJoints(self._robot_id)
        for i in range(numJoints):
            jointInfo = p.getJointInfo(self._robot_id, i)
            key = jointInfo[1].decode('utf-8')
            joint_index[key] = i
            if key[0] == 'u' or key[0] == 'v' or key[0] == 'w':
                self._revolute_joint_index[key] = i
            elif key[0] == 'x':
                end_effector_index[key] = i
        self._num_revolute_joint = len(self._revolute_joint_index)

        # Change dynamics
        for idx in self._revolute_joint_index.values():
            p.changeDynamics(
                bodyUniqueId=self._robot_id,
                linkIndex=idx,
                jointDamping=0.1             # Damping specifically for joints
            )
        for idx in end_effector_index.values():
            p.changeDynamics(
                bodyUniqueId=self._robot_id,
                linkIndex=idx,
                lateralFriction=1,         # Lateral friction coefficient
                contactStiffness=10.0,     # Stiffness of contact constraints
                contactDamping=0.5,          # Damping of contact constraints
            )


    def _get_obs(self):

        joints_position = np.array([p.getJointState(self._robot_id, i)[0] for i in self._revolute_joint_index.values()])
        mask = (np.arange(len(joints_position))+1) % 3 == 0
        joints_position[mask] -= np.pi / 2.0
        joints_position = joints_position / (np.pi / 2.0)
        joints_position = np.clip(joints_position, self.observation_space.low, self.observation_space.high)
        return joints_position.astype(OBSERVATION_TYPE)
    
    def _get_info(self):
        return {
            "distance": self._distance
        }
    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        p.resetBasePositionAndOrientation(self._robot_id,[0,0,0.2],[0,0,0,1])
        for i in self._revolute_joint_index.values():
            p.resetJointState(self._robot_id, i, 0)

        observation = self._get_obs()
        self._last_observation = observation
        info = self._get_info()
        self._steps = 0

        
        #print(f"max episode steps: {self.spec.max_episode_steps}")

        return observation, info

    def step(self, action):

        # Action
        #value = p.readUserDebugParameter(self._button_id)
        
        joints_position = action * (np.pi / 2.0)
        mask = (np.arange(len(joints_position))+1) % 3 == 0
        joints_position[mask] += np.pi / 2


        # Compute with PyBullet
        p.setJointMotorControlArray(self._robot_id,
                                    self._revolute_joint_index.values(),
                                    p.POSITION_CONTROL,
                                    joints_position, 
                                    forces=[5]*self._num_revolute_joint, 
                                    positionGains=[0.2]*self._num_revolute_joint
                                    )
        
        p.stepSimulation()
        if self.render_mode == 'human':
            time.sleep(TIME_STEP)

        observation = self._get_obs()

        # Velocity reward
        velocities, _ = p.getBaseVelocity(self._robot_id)
        xVelocity = velocities[0]
        x_velocity_reward = xVelocity * 1.0

        yVelocity = velocities[1]

        robot_state = p.getLinkState(self._robot_id,0)
        robot_position = list(robot_state[0])

        # Orientation reward
        quaternion = np.array(robot_state[1])
        quaternion /= np.linalg.norm(quaternion)
        x, y, z, w = quaternion
        up_z = 1 - 2 * (x**2 + y**2)  # This is the z-component of the up vector
        orientation_reward = up_z  # P
    
    # Reward is 1 when z_component is close to 0 (horizontal) and 0 whe

        
        y_velocity_unreward = - abs(yVelocity)*0.1 

        # Energy reward
        energy = sum(((action - observation) / 2) ** 2 ) / self._num_revolute_joint
        energy_reward = min(1.0, 0.001/energy )

    # Total reward
        reward = energy_reward + orientation_reward + x_velocity_reward

        terminated = False if robot_position[2] >= 0.04 and robot_position[2] <= 0.5 else True
        truncated = (self._steps >= 1200)
        
        reward_info = {"energy_reward":energy_reward, "orientation_reward":orientation_reward, "x_velocity_reward":x_velocity_reward}
        info = {**self._get_info(), **reward_info}

        self._steps += 1
        self._last_observation = observation
        return observation, reward, terminated, truncated, info
    
    def close(self):
        p.disconnect()
        




