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


SIM_F = 240
TIME_STEP = 1/SIM_F
POSITION_MIN = -np.pi/2
POSITION_MAX = np.pi/2
OBSERVATION_TYPE=np.float32
ACTION_TYPE = np.int64
POSITION_INCREMENT = np.pi / 180 # 1 degre per action

class Actions(Enum):
    INCREASE = 0
    HOLD = 1
    DECREASE = 2

class SpidyEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"]}

    def __init__(self,render_mode=None):

        # self.observation_space = spaces.Dict(
        #     {
        #     "joints_position": spaces.Box(low=POSITION_MIN,high= POSITION_MAX, shape=(18,), dtype=OBSERVATION_TYPE)
        #     }
        # )
        self.observation_space = spaces.Box(low=POSITION_MIN,high= POSITION_MAX, shape=(18,), dtype=OBSERVATION_TYPE)
        self._joints_position = np.zeros(18, dtype=OBSERVATION_TYPE)

        self.action_space = spaces.MultiDiscrete(np.full(18, 3), dtype=ACTION_TYPE)

        self._action_to_position_increment = {
            Actions.INCREASE.value: POSITION_INCREMENT,
            Actions.HOLD.value: 0.0,
            Actions.DECREASE.value: -POSITION_INCREMENT
        }

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
        #return {"joints_position": self._joints_position}
        return self._joints_position
    
    def _get_info(self):
        return {
            "distance": self._distance
        }
    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        p.resetBasePositionAndOrientation(self._robot_id,[0,0,0.2],[0,0,0,1])
        self._joints_position = np.random.uniform(POSITION_MIN,POSITION_MAX, 18).astype(OBSERVATION_TYPE)

        observation = self._get_obs()
        info = self._get_info()

        return observation, info

    def step(self, action):

        # Action
        increments = np.vectorize(self._action_to_position_increment.get)(action)
        self._joints_position = np.clip(self._joints_position + increments, POSITION_MIN, POSITION_MAX).astype(OBSERVATION_TYPE)

        # Compute with PyBullet
        p.setJointMotorControlArray(self._robot_id,
                                    self._revolute_joint_index.values(),
                                    p.POSITION_CONTROL,
                                    self._adjust_joints_positions(), 
                                    forces=[5]*self._num_revolute_joint, 
                                    positionGains=[0.5]*self._num_revolute_joint
                                    )
        
        p.stepSimulation()
        if self.render_mode == 'human':
            time.sleep(TIME_STEP)

        # Return
        robot_state = p.getLinkState(self._robot_id,0)
        robot_position = list(robot_state[0])
        self._distance = math.sqrt(robot_position[0]**2 + robot_position[1]**2)

        terminated = self._distance >= 2
        reward = self._distance
        observation = self._get_obs()
        info = self._get_info()

        return observation, reward, terminated, False, info
    
    def close(self):
        p.disconnect()

            
    def _adjust_joints_positions(self):
        new_joints_position = self._joints_position.copy()
        mask = (np.arange(len(new_joints_position))+1) % 3 == 0
        new_joints_position[mask] += np.pi / 2
        return new_joints_position




