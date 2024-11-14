from enum import Enum
import gymnasium as gym
from gymnasium import spaces
import pygame
import numpy as np
import time

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

    def __init__(self):

        self.observation_space = spaces.Dict(
            {
            "joints_position": spaces.Box(low=POSITION_MIN,high= POSITION_MAX, shape=(18,), dtype=OBSERVATION_TYPE)
            }
        )
        self._joints_position = np.zeros(18, dtype=OBSERVATION_TYPE)

        self.action_space = spaces.MultiDiscrete(np.full(18, 3), dtype=ACTION_TYPE)

        self._action_to_position_increment = {
            Actions.INCREASE.value: POSITION_INCREMENT,
            Actions.HOLD.value: 0.0,
            Actions.DECREASE.value: -POSITION_INCREMENT
        }

        self._height = 0.0

    def _get_obs(self):
        return {"joints_position": self._joints_position}
    
    def _get_info(self):
        return {
            "height": self._height
        }
    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        
        self._joints_position = np.random.uniform(POSITION_MIN,POSITION_MAX, 18).astype(OBSERVATION_TYPE)

        observation = self._get_obs()
        info = self._get_info()

        return observation, info

    def step(self, action):

        # Action
        increments = np.vectorize(self._action_to_position_increment.get)(action)
        self._joints_position = np.clip(self._joints_position + increments, POSITION_MIN, POSITION_MAX) 

        # Compute with PyBullet
        time.sleep(0.01)

        # Return
        terminated = self._height >= 0.2
        reward = 1 if terminated else 0
        observation = self._get_obs()
        info = self._get_info()

        return observation, reward, terminated, False, info
    
    def close(self):
        print("close")

            
       




