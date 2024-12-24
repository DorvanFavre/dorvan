import gymnasium as gym


class NormalizedObservation(gym.ObservationWrapper):
    def __init__(self, env, factor):
        super().__init__(env)
        self.factor = factor
        self.env = env
        
    def observation(self, obs):
        obs = obs / self.factor
        self.env.last_obs = obs
        return obs
