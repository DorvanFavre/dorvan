from enum import Enum
import gymnasium as gym
from gymnasium import spaces
import pygame
import numpy as np


class Actions(Enum):
    right = 0
    up = 1
    left = 2
    down = 3


class ContinuousWorldEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"],"action_mode": ["angle"], "render_fps": 5}

    def __init__(self, render_mode=None, size=200, step_size=10, action_mode = None):
        self.size = size  # The size of the square grid
        self.window_size = self.size  # The size of the PyGame window
        self.step_size = step_size
        self.observation_space = spaces.Box(shape=(3,), low=-1., high=1, dtype=np.float32)
        self.action_mode = action_mode
        # We have 4 actions, corresponding to "right", "up", "left", "down", "right"
        if action_mode == "angle":
            self.action_space = spaces.Box(shape=(1,), low=0., high=1., dtype= np.float32)
        else:
            self.action_space = spaces.Discrete(4)

        """
        The following dictionary maps abstract actions from `self.action_space` to 
        the direction we will walk in if that action is taken.
        i.e. 0 corresponds to "right", 1 to "up" etc.
        """
        self._action_to_direction = {
            Actions.right.value: [1, 0],
            Actions.up.value: [0, 1],
            Actions.left.value: [-1, 0],
            Actions.down.value: [0, -1],
        }
        assert action_mode is None or action_mode in self.metadata["action_mode"]
        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode

        """
        If human-rendering is used, `self.window` will be a reference
        to the window that we draw to. `self.clock` will be a clock that is used
        to ensure that the environment is rendered at the correct framerate in
        human-mode. They will remain `None` until human-mode is used for the
        first time.
        """
        self.window = None
        self.clock = None

    def _get_obs(self):
        proximity =  {"agent": self._agent_location, "target": self._target_location}
        # relative position
        proximity = proximity["target"] - proximity["agent"]
        # Normalized proximity
        proximity = proximity / self.size
        
        # angle 
        angle = np.arctan2(proximity[0], proximity[1]) / np.pi
        
        obs = np.array([*proximity, angle])
        return obs.astype(np.float32)

    def _get_info(self):
        return {
            "distance": np.linalg.norm(
                self._agent_location - self._target_location, ord=1
            )
        }

    def reset(self, seed=None, options=None):
        # We need the following line to seed self.np_random
        super().reset(seed=seed)
        

        # Choose the agent's location uniformly at random
        self._agent_location = self.np_random.integers(0, self.size, size=2, dtype=int)

        # We will sample the target's location randomly until it does not
        # coincide with the agent's location
        self._target_location = self._agent_location
        while np.array_equal(self._target_location, self._agent_location):
            self._target_location = self.np_random.integers(
                0, self.size, size=2, dtype=int
            )

        observation = self._get_obs()
        info = self._get_info()

        if self.render_mode == "human":
            self._render_frame()

        return observation, info

    def step(self, action):
        last_distance = np.linalg.norm(self._agent_location - self._target_location, ord=1)
        
        # Action
        if self.action_mode == 'angle':
            action = action[0]
            action = action * 2 * np.pi
            direction = [np.cos(action), np.sin(action)]
            
        else:
            direction = self._action_to_direction[action] 
            
        # Update position
        direction = np.array(direction) * self.step_size
        self._agent_location = np.clip(
            self._agent_location + direction, 0, self.size - 1
        )
        
        
        distance = np.linalg.norm(self._agent_location - self._target_location, ord=1)
        
        terminated = distance < 15
        
        improvment = last_distance - distance
        proximity_reward = (improvment / self.size) 
        
        terminated_reward = 1 if terminated else 0  # Binary sparse rewards
        
        observation = self._get_obs()
        reward = terminated_reward + proximity_reward
        info = self._get_info()

        if self.render_mode == "human":
            self._render_frame()


        return observation, reward, terminated, False, info

    def render(self):
        if self.render_mode == "rgb_array":
            return self._render_frame()

    def _render_frame(self):
        if self.window is None and self.render_mode == "human":
            pygame.init()
            pygame.display.init()
            self.window = pygame.display.set_mode((self.window_size, self.window_size))
        if self.clock is None and self.render_mode == "human":
            self.clock = pygame.time.Clock()

        canvas = pygame.Surface((self.window_size, self.window_size))
        canvas.fill((255, 255, 255))
         # The size of a single grid square in pixels

        # First we draw the target
        pygame.draw.rect(
            canvas,
            (255, 0, 0),
            pygame.Rect(
                (self._target_location),
                (5, 5),
            ),
        )
        # Now we draw the agent
        pygame.draw.circle(
            canvas,
            (0, 0, 255),
            (self._agent_location),
            5,
        )


        if self.render_mode == "human":
            # The following line copies our drawings from `canvas` to the visible window
            self.window.blit(canvas, canvas.get_rect())
            pygame.event.pump()
            pygame.display.update()

            # We need to ensure that human-rendering occurs at the predefined framerate.
            # The following line will automatically add a delay to
            # keep the framerate stable.
            self.clock.tick(self.metadata["render_fps"])
        else:  # rgb_array
            return np.transpose(
                np.array(pygame.surfarray.pixels3d(canvas)), axes=(1, 0, 2)
            )

    def close(self):
        if self.window is not None:
            pygame.display.quit()
            pygame.quit()

