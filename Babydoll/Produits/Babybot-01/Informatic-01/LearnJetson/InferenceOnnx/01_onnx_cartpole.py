import gymnasium as gym
import torch
import time
import onnx
import onnxruntime as ort
from typing import Tuple
import numpy as np

onnx_model_path = '/home/dorvan/Documents/dorvan/Babydoll/Produits/Babybot-01/Informatic-01/LearnJetson/InferenceOnnx/dqn_cartpole_v3.onnx'
observation_size = (4,)

onnx_model = onnx.load(onnx_model_path)
onnx.checker.check_model(onnx_model)

observation = np.zeros((1, *observation_size)).astype(np.float32)
ort_sess = ort.InferenceSession(onnx_model_path)

onnx_env = gym.make('CartPole-v1', render_mode='human')

for e in range(5):

    obs, info = onnx_env.reset()

    done = False
    total_reward = 0

    while not done:
        # Preprocess observation to match ONNX input shape
        # Ensure it's a batch (e.g., [1, 4] for CartPole)
        #obs_input = np.expand_dims(obs, axis=0).astype(np.float32)
        

        obs = np.expand_dims(obs, axis=0).astype(np.float32)

        # Get action probabilities from the ONNX model
        action = ort_sess.run(None, {"input": obs})
        action = action[0].item()
       
        #action = np.expand_dims(action, axis=0)
        
        # Step in the environment
        obs, reward, done, truncated, info = onnx_env.step(action)
        total_reward += reward

    print(f"Total reward: {total_reward}")