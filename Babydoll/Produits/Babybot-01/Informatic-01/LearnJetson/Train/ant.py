import argparse
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import BaseCallback
import numpy as np



def main():
    # Create the argument parser
    parser = argparse.ArgumentParser(description="A script to create, train, or visualize.")
    
    # Add arguments
    parser.add_argument("action", choices=["create", "train", "visualize"], 
                        help="Specify the action to perform: create, train, or visualize.")
    
    # Parse the arguments
    args = parser.parse_args()
    
    # SETUP
    

    name = 'ppo_ant_v1'
    env_id = "Ant-v5"
    n_envs = 4

    policy = 'MlpPolicy'
    tensorboard_log = f"./{name}/t_logs/"
    path = f"./{name}/model/"
    device = 'cpu'

    def make_env(render_mode:str='rgb_array'):
        e = gym.make(env_id, render_mode=render_mode)
        return e

    env = make_vec_env(make_env, n_envs)

    class SaveOnStep(BaseCallback):
        def __init__(self, steps: int, path: str, verbose: int = 0):
            super().__init__(verbose)
            self.steps = steps
            self.save_path = path

        def _on_step(self) -> bool:
            # Check if the current step matches the saving frequency
            if self.n_calls % self.steps == 0:
                # Save model with the current timestep in the filename
                if self.verbose > 0:
                    print(f"Saving model at step {self.n_calls} to {self.save_path}")
                self.model.save(self.save_path)
            return True
    
    callbacks = [SaveOnStep(2.5e4, path, verbose=1)]
    
    
    # Call the appropriate function based on the action argument
    if args.action == "create":
        print("Creating a new model...")
        model = PPO(
            policy,
            env,
            verbose=0,
            tensorboard_log=tensorboard_log,
            device=device
        )
        model.save(path)

    elif args.action == "train":
        print("Train the model...")
        model = PPO.load(path,env)
        total_timesteps = 20e6
        model.learn(
            total_timesteps, 
            reset_num_timesteps=False, 
            progress_bar=True, 
            callback=callbacks)
        model.save(path)
        
    elif args.action == "visualize":
        print("Visualize...")
        model = PPO.load(path,env)
        display_env = make_env(render_mode='human')

        for e in range(1):
            obs,_ = display_env.reset()
            while True:
                action, _ = model.predict(obs)
                obs, reward, terminated, truncated, info = display_env.step(action)

                if terminated or truncated:
                    break

if __name__ == "__main__":
    main()

