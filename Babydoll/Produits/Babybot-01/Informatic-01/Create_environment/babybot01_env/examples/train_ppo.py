import argparse
import gymnasium as gym
import babybot01_env
from tqdm import tqdm
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import BaseCallback, EvalCallback, CallbackList
import numpy as np
from stable_baselines3.common.monitor import Monitor


def main():
    # Create the argument parser
    parser = argparse.ArgumentParser(description="A script to create, train, or display.")
    
    # Add arguments
    parser.add_argument("action", choices=["create", "train", "display"], 
                        help="Specify the action to perform: create, train, or display.")
    
    # Parse the arguments
    args = parser.parse_args()
    
    # SETUP
    

    name = 'ppo_spidy_v1'
    env_id = "Spidy-v0"
    n_steps = 6000
    n_envs = 1

    policy = 'MlpPolicy'
    tensorboard_log = f"./{name}/t_logs/"
    path = f"./{name}/model/{name}"
    log_path = f"./{name}/logs/"
    device = 'cpu'

    def make_env(render_mode:str=None):
        e = gym.make(env_id, max_episode_steps=n_steps, render_mode=render_mode)
        return e

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
    
    callbacks = [SaveOnStep(2.5e4, path)]
    
    
    # Call the appropriate function based on the action argument
    if args.action == "create":
        print("Creating a new model...")
        train_env = gym.make("Spidy-v0", render_mode="rgb_array")
        model = PPO(policy,  train_env, batch_size=60, tensorboard_log=tensorboard_log, verbose=0, n_steps=n_steps)
        model.save(path)

    elif args.action == "train":
        print("Train the model...")
        #train_env = make_vec_env(make_env, n_envs)
        train_env = make_env()
        eval_env = Monitor(make_env())

        eval_callback = EvalCallback(eval_env,
                                    log_path=log_path, eval_freq=1e4,
                                    deterministic=True, render=False)

        class SaveOnStep(BaseCallback):
            def __init__(self, steps: int, path: str):
                super().__init__()
                self.steps = steps
                self.save_path = path

            def _on_step(self) -> bool:
                # Check if the current step matches the saving frequency
                if self.n_calls % self.steps == 0:
                    # Save model with the current timestep in the filename

                    print(f"Saving model at step {self.n_calls} to {self.save_path}")
                    self.model.save(self.save_path)
                return True
            
        callbacks = [SaveOnStep(1e4, path), eval_callback]
        model = PPO.load(path,train_env ,device=device)
        model.learn(total_timesteps=10e6,
                    progress_bar=True, 
                    callback=callbacks, 
                    reset_num_timesteps=False)
        model.save(path)
        train_env.close()

        
    elif args.action == "display":
        print("Display...")
        display_env = gym.make(env_id,max_episode_steps=n_steps, render_mode="human", )
        model = PPO.load(path)

        for episode in range(1):

            episode_reward = 0
            t = 0
            done = False
            obs, info = display_env.reset()
            while True:
            #for t in tqdm(range(n_steps)):

                action = model.predict(obs)[0]
                
                obs, reward, terminate, trunc, info = display_env.step(action)
                episode_reward += reward
                # if t%10 ==0:
                #     print(action)
                t+=1

                if terminate or trunc:
                    mean_reward = episode_reward / t
                    print(f"Episode total reward: {episode_reward}")
                    break

                

        display_env.close()

if __name__ == "__main__":
    main()
