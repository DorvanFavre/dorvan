
Project is in "Create environement/Train/train_ppo"

## Spidy-v0

host, 1 env : 350 it/s
4env : take like 3min to start. The speed is so bad

**ppo spidy v1**
First training (1.5e6 steps):
Robot stopped moving at all because I un-reward too much energy and joints velocity

**ppo spidy v2**
reward x, unreward y
1e6 steps
Nothing work

**ppo spidy v2_2**
reward x, unreward abs y, unreward energy
2M step: no improvment

**ppo spidy v2_3**
Try to stabilize the robot. No movements.

energy = sum(((action - observation) / 2) ** 2 ) / self._num_revolute_joint
energy_reward = min(1.0, 0.001/energy )

n joints not moving : mean reward
18 : 25 -> 1.0
17 : 0.43 ,
16 : 0.20 ,
10: 0.028
0: 0.011

reach max reward after 300k steps.
issues: robot not standing. Legs not moving but in a creepy position.

**ppo spidy v2_4**

Add a healthy position reward
reach max reward after 450k steps
OKAY... we've got some improvements.
![[Pasted image 20241127165724.png]]


**ppo spidy v2_5**

Add some reward for x displacment.
