from enum import Enum
import gymnasium as gym
from gymnasium import spaces
import pygame
import numpy as np




class SteeringWorldEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 10}

    def __init__(self, render_mode=None, size=200, speed=10, freq = 10, max_episode_steps=500):
        self.size = size  # The size of the square grid
        self.window_size = self.size  # The size of the PyGame window
        self.speed = speed # pix / s
        self.freq = freq
        self.max_episode_steps = max_episode_steps

        self.observation_space = spaces.Box(shape=(4,), low=-1., high=1, dtype=np.float32)

        self.action_space = spaces.MultiDiscrete([3,3])


        """
        The following dictionary maps abstract actions from `self.action_space` to 
        the direction we will walk in if that action is taken.
        i.e. 0 corresponds to "right", 1 to "up" etc.
        """
        self._action_to_increment = {
            0: -1,
            1: 0,
            2: 1,
        }
        self.max_steering = 2.

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


        time_obs = self._time_step / self.max_episode_steps

        target_steering = self._target_steering
        actual_steering = self._steering
        noise = np.random.uniform(-1,1)
        
        # [*proximity, steering, angle, direction]
        obs = np.array([target_steering, actual_steering, self._param1, self._param2])
        return obs.astype(np.float32)

    def _get_info(self):
        return {
            "steering": self._steering
        }

    def reset(self, seed=None, options=None):
        # We need the following line to seed self.np_random
        super().reset(seed=seed)
        
        self._time_step = 0

        # Choose the agent's location uniformly at random
        self._agent_location = self.np_random.integers(0, self.size, size=2, dtype=int)
        
        self._target_steering = np.random.uniform(-1.,1.)

        self._direction = np.random.uniform(-np.pi,np.pi)
        self._steering = 0
        self._param1 = np.random.uniform(-1.,1.)
        self._param2 = np.random.uniform(-1.,1.)

        observation = self._get_obs()
        info = self._get_info()

        if self.render_mode == "human":
            self._render_frame()

        return observation, info

    def step(self, action):

        # if len(action) == 1:
        #     action = action[0]
        # Action

        self._param1 = min(1.0, max(-1.0, self._param1 + self._action_to_increment[action[0]] * 0.1))
        self._param2 = min(1.0, max(-1.0, self._param2 + self._action_to_increment[action[1]] * 0.1))
        
        # calculate steering according to params
        if 0.2 < self._param1 < 0.3 and -0.6 < self._param2 < -0.5:
            #self._steering = (self._param1 - 0.2) * 10 # steering [0,1]
            if self._target_steering < 0 :
                self._steering = self._target_steering
        if -0.7 < self._param1 < -0.6 and 0.3 < self._param2 < 0.4:
            #self._steering = (self._param1 + 0.6) * 10 # steering [-1,0]
            if self._target_steering >= 0 :
                self._steering = self._target_steering
        else:
            self._steering = 0
            

        # Update direction according to steering
        steering = self._steering * self.max_steering / self.freq
        self._direction = self._direction + steering
        if self._direction < -np.pi:
            self._direction += 2 * np.pi
        elif self._direction > np.pi:
            self._direction -= 2*np.pi
            
        # Update position according to direction
        xy_direction = [np.cos(self._direction), np.sin(self._direction)]
        xy_direction = np.array(xy_direction) * self.speed / self.freq
        self._agent_location = np.clip(
            self._agent_location + xy_direction, 0, self.size - 1
        )
        
    
        # Calculate rewards, obs, info...
        terminated = abs(self._target_steering - self._steering) < 0.1 or self._time_step >= self.max_episode_steps
        observation = self._get_obs()
        info = self._get_info()
        
        
        #proximity_reward = (2. - abs(self._target_steering - self._steering)) / 4.
        #proximity_reward =  max(0, 1 - abs(self._target_steering - self._steering))
        #do_somthing_reward = 0.5
        #steering_reward = proximity_reward + do_somthing_reward if self._steering != 0 else 0
        

        reward = 1 if terminated else 0
        
        

        # Update target steering
        #self._target_steering = np.clip(self._target_steering + np.random.randint(-1,2) * 0.01, -1., 1.)
        

        if self.render_mode == "human":
            self._render_frame()

        self._time_step += 1
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
            (0, 0, 0),
            pygame.Rect(
                (50, 0),
                (100, 5),
            ),
        )
        pygame.draw.rect(
            canvas,
            (255, 0, 0),
            pygame.Rect(
                (self._target_steering * 50 + 100, 0),
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

