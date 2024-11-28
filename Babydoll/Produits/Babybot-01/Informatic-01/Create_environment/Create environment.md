
Get inspired by how environment are codded and replicate.
## Ressources:

Environement for the crawling robot : https://github.com/krcarter/Reinforcement-Learning-Crawling-Robot

PyBullet Environement, agent, etc... : https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet

CartPoolEnv: https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_envs/bullet/cartpole_bullet.py

https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_envs/baselines/enjoy_pybullet_cartpole.py

==Follow the guide of Gymnasium==: https://gymnasium.farama.org/
It explains how to create a custom environement and everything

API : https://gymnasium.farama.org/api/env/

**Install the environment**

Be sure the version of the environment is increased.
Open console in the bin folder of the python virtual environment:
```
./pip install -e /home/dorvan/Documents/dorvan/Babydoll/Produits/Babybot-01/Informatic-01/Create_environment/environment
```


## Spidy-v0

Resources: https://gymnasium.farama.org/environments/mujoco/ant/

**Terminated**
Distance > 2 m

**Truncated** 
10 s = 60 fps * 10 = 600 timesteps

**Action space:**
Value of the 18 joints. From min (-1) to max (1). Angle are converted in the environment.
Box(-1, 1, (18,), float32)


**Observation space**:
Position of the 18 joints. From min (-1) to max (1).

**Rewards**
Rewards = distance reward * healthy

distance reward = distance (m) / step
terminated reward = 1000
healthy : 0.02 < z < 0.1


#### Test host

render_mode = None / 'rgb_array' : No difference