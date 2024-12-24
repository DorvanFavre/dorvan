import gymnasium as gym

class ProximityReward(gym.RewardWrapper):
    def __init__(self, env):
        super().__init__(env)
        
    def step(self, action):
        
        observation, reward, terminated, truncated, info = self.env.step(action)
        return observation, self.reward(reward), terminated, truncated, info
    
    def reward(self, reward):
        return reward