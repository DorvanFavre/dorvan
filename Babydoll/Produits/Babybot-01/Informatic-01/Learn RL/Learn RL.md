Learn RL by following tutos and codding

## Tasks

**11_toys** : Create a template code 
- Run local/colab
- Multiple environments
- Multiple agents
- Train monitoring
- Evaluate and compare agent/env


## Ressources

StableBaseline3:
- https://stable-baselines3.readthedocs.io/en/master/
- https://github.com/DLR-RM/stable-baselines3
- 

HuggingFace : https://huggingface.co/learn/deep-rl-course/unit1/hands-on

SpinninAI:


Gymnasium: https://gymnasium.farama.org/environments/box2d/lunar_lander/

Panda-gym: https://panda-gym.readthedocs.io/en/latest/index.html

Stb3 Zoo: https://github.com/DLR-RM/rl-baselines3-zoo

PandaGym: https://github.com/qgallouedec/panda-gym



**Exercices**

2_karpool_stb3: https://colab.research.google.com/drive/1255ugOV1iNyNV25FmHmzuwIhx2GZc3u3#scrollTo=mIrtj89lco2B


Find a tuto to learn a robot to walk and adapt it to my hexapod.



## Results


#### DQN

dqn_cartpole_v1
local, cpu, 1 env: 600 it/s

![[Pasted image 20241118151543.png]]

host, cpu, 4 env: 1800 it/s
Normalization is useless

![[Pasted image 20241122143210.png]]

explorate_fraction = 0.5
![[Pasted image 20241122150932.png]]

#### A2C
a2c_cartpole_v1
local, cpu, 1env : 300 it/s
colab(cpu) 440 it/s
![[Pasted image 20241118154440.png]]



a2c_cartpole_v2 (max 500)
local, cpu, 4env : 1200 it/s

![[Pasted image 20241118185245.png]]

a2c_lunar_v1 (max 100)
local, cpu, 4env 700 it/s
8 env : 400 it/s
![[Pasted image 20241118190923.png]]

a2c_panda_reach_v1
local, cpu, 4env, 150 it/s

#### PPO

ppo_lunar_v1
loca, cpu, 4env : 700 - 200 decreasing
result: Bad

![[Pasted image 20241120112606.png]]
