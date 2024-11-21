Let's see if I can use colab to train agents.

## Tasks

- [x] Visualize gymnasium environment  
      Live is not possible. Instead make a video: [Colab](https://colab.research.google.com/drive/17iGU9aKz38hkwGTo-Esos-2QI2MEYdXX?usp=drive_link)
- [ ] Visualize PyBullet environment
- [ ] Use SpidyEnv
- [ ] Train agent on Gymnasium env
	- [x] 07_unit6_A2C (Not learning)
	- [ ] Train on cartpool
- [ ] Train agent on PyBullet env
- [ ] Train agent on SpidyEnv

## Ressources

SB3 Unit6 A2C [colab](https://colab.research.google.com/drive/17iGU9aKz38hkwGTo-Esos-2QI2MEYdXX?usp=drive_link)



## Other

_dense reward function_
sparse reward function

Make video
```Python
import gymnasium as gym
import cv2

# Create the environment
env = gym.make("CartPole-v1", render_mode="rgb_array")
obs = env.reset()

# Set up video writer
video_filename = "cartpole_video.mp4"
fps = 30
frame_size = (env.render().shape[1], env.render().shape[0])
video_writer = cv2.VideoWriter(video_filename, cv2.VideoWriter_fourcc(*"mp4v"), fps, frame_size)

# Record frames
for _ in range(200):  # Adjust the number of frames as needed
    action = env.action_space.sample()  # Take random actions
    obs, reward, done, truncated, info = env.step(action)
    frame = env.render()  # Get the frame as an RGB array
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)  # Convert RGB to BGR for OpenCV
    video_writer.write(frame_bgr)  # Write frame to video
    if done or truncated:
        obs = env.reset()

# Cleanup
video_writer.release()
env.close()

```