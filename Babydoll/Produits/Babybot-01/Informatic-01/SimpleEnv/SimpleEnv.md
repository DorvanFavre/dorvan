Create a very simple environment that works and gradually add complexity and eventually reach the Spidy env configuration.

## Ressources

Read gymnasium env tutorials: https://gymnasium.farama.org/tutorials/gymnasium_basics/load_quadruped_model/

Read Env class: https://github.com/openai/gym/blob/master/gym/core.py


Get inspiration from ant-v5 env: https://gymnasium.farama.org/main/environments/mujoco/ant/#arguments
Review DQN algorithm: https://huggingface.co/learn/deep-rl-course/unit3/introduction
## Tasks

### 1. Grid World - DQN

normal configuration: Overfit is possible.

![[Pasted image 20241224125716.png]]

Add the eval callback and save on best result only.
Can take ages to find a solution since it is sparse and the grid is big 10x10 and the exploration rate is low very quickly.
![[Pasted image 20241224131316.png]]

try to make the exploration rate higher: exploration fraction = 0.5
num_training_steps = 1e5
Better than before:
![[Pasted image 20241224131743.png]]

maybe the ep_length = 100 is too low. The probability of finding randomly a solution in only 100 steps is low.
try 500.

does not change that much
![[Pasted image 20241224132248.png]]

Try with a smaller grid: 5x5 and max_ep:length = 100 again.
Find a solution very quickly but then fuck up again as if it overfits something.
![[Pasted image 20241224132704.png]]

go back to a 8x8 grid, max ep length = 100.
Learned nothing.
try with max_ep_length = 200
When i display i can see that the agent stays stuck in certain state.
![[Pasted image 20241224135814.png]]

Try to normalize observation to range [-1,1] 
Okay.. normalization has a huge impact.
the agent is still stuck when the target is far away. 
![[Pasted image 20241224141825.png]]

Add a timestep observation to give avoid being stuck i a certain state.
Even batter. but looks like there is something fishy

![[Pasted image 20241224161600.png]]

See that model after 80k training is the best.
This model is really nice to see playing.
![[Pasted image 20241224163055.png]]

Try learning rate of 0.001 instead of 0.0001
Quickly find optimal solution but i should stop training,
![[Pasted image 20241224164540.png]]

Learn in half time : 50k timesteps
Not ideal. Kepp to 100k for now
and try linear decay of learning rate from 0.001 to 0
I still remain in a sub-optimal policy. The agent could be improved. An average of 10 timestep to reach the goal can be improved to 5 or 6.

![[Pasted image 20241224170055.png]]



With sparse reward = 10 instead of 1
![[Pasted image 20241224173917.png]]
With sparse reward of 100: even worse:
![[Pasted image 20241224174328.png]]
Sparse reward of 0.1: Struggle too much
![[Pasted image 20241224174723.png]]

Sparse reward of 1 is the best. is it total of 1 per episode or per step that we should aim for ?
set the learning rate as : learning_rate=linear_schedule(0.0001)
learning rate should be sufficiently small.
That is very good ! we can use this configuration as baseline now.
![[Pasted image 20241224175427.png]]

Try to improve the reward function. instead of a sparse reward. give reward each time the relative distance gets closer. Try proximity reward (-1,0,1) per timestep

Not as efficient... bu progress look stable until 60k
![[Pasted image 20241224180207.png]]
try with a reward that normalize as 1.0 per epoch -> (-0.2, 0, 0.2) per step
same same
![[Pasted image 20241224180556.png]]

try with 0.1 per step: Same

Try to combine sparse reward and dense reward (sparse = 1, dense = ~0.5) per episode.
it is a little better.
![[Pasted image 20241224181543.png]]

But I thing the big difference coul be on a 10x10 grid. Lets try it:
yes. that would have been impossible with sparse reward.
![[Pasted image 20241224182026.png]]

lets try with max_episode_steps = 10, since it sho8uld be able to converte under 10:
takes a bit more time to converge but that's very good:
![[Pasted image 20241224182450.png]]

**Use a combination of dense and sparse reward**
**Make total reward per episode reach 1.0**
**Try to keep learning rate low : under 0.0001**

### 2. Other agent on Grid world

Try with A2C.
It is very efficient. I will need a harder env to see a real difference.
![[Pasted image 20241225090412.png]]

Try with PPO:
Even better ! Use PPO from nowon.
![[Pasted image 20241225090912.png]]

### 3. Vectorize env

Run on CPU : use SubprocVecEnv
https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html

Check cartpole.py to see how implementing vectorized env.
https://github.com/Farama-Foundation/Gymnasium/blob/main/gymnasium/envs/classic_control/cartpole.py

compare differences:

Class name change.
Inherits from VectorEnv
no 'human' mode in metadata

init:
	num_envs
	action space: batch space

try another time.


### 4. ActionSpace: Direction (GridWorld_v2)

Instead of giving the next move (up,right,down,left), it will change the direction (keep going, increase 90deg, decrease 90deg)

Change env:
Intern variable: direction (0,1,2,3)
Each timestep go in the direction.
Action space as (do noting, steer right, steer left)

Indeed, it struggle a little bit more than the ohter manner (PPO) (need 3x more steps to converge)
![[Pasted image 20241225100752.png]]

Try with DQN: HAHAHA DQN is better than PPO this time !
![[Pasted image 20241225101247.png]]
Try with A2C: worse than PPO
![[Pasted image 20241225103508.png]]

### 5 Continuous action space (gridworld_v3)

It is in direct again.
Action space [0,1]
0- 0.25 go up, 0.25-0.5 go right, 0.5-0.75 go down and 0.75 - 1 go left

Try with PPO: should give same result as env_v1
Definetly struggle to converge
![[Pasted image 20241225105548.png]]

![[Pasted image 20241225110157.png]]


Means that discretizing continuous action space may be good.

### 6 ContinuousWorld

Direct action

Easy (PPO)
![[Pasted image 20241225113140.png]]


Change action space
Continuous, 1 value : direction angle
Take 10x more time
![[Pasted image 20241225134849.png]]
What if i also give the angle as observation ? risk that it maps directly the observed angle as the optimal action. so be it. let's try: Not at all. result is the same: optimum at 200k steps

![[Pasted image 20241225140728.png]]

Try another model: DDPG
The training rate is 10x slower (50it/s)
But so efficient ! reach optimum after 2k steps equivalent to 20k steps for DQN
the display result is very smooth.
![[Pasted image 20241225141033.png]]![[Pasted image 20241225141510.png]]

Try PPO with discretised actions
discretize 360 deg in 10 actions
Reaches optimum after 20k steps
![[Pasted image 20241225142433.png]]

Try with DQN
Takes more time (40k steps)
![[Pasted image 20241225142801.png]]

Try to increase action space (20 actions) to see if DQN is doing bad according to action space size.
DQN: Same performance as before
Try with 100 actions: SAME
PPo: reach optimum at 25k steps. same as with 10 actions only
The size of the action space does not influence so much the performences.


### 7 Continuous world v2

Steering actions ! 

variable : direction -pi to pi
action descrete : increase angle , decrease angle, do nothing
each steps move in the direction
obs: proximity, angle, direction ([-1,1], time

increment is 0.25 / timestep (total is 2PI)

Try with PPO: reach optimum after 150k steps: 6x more than with direct actions.
Result is realllly nice to see. it takes more time to make revers turn obviously. That can explain the complexity
![[Pasted image 20241225151211.png]]

try DQN
Same result faset (60k steps)
![[Pasted image 20241225151533.png]]


try now adjust steering value

actions: more left, do nothing, more right
action adjust steering [-pi/2, pi/2]
steering adjust direction proportionaly
add steering to observation

The model is realistic now. It hase to do with " how much i want to adjust the steering of my car"

Let's see what DQN is capable of:
Needs a little more time but eventually reaches an optimal solution:
![[Pasted image 20241225152951.png]]

Try PPO.
a bit more difficult. Display result is descent.

![[Pasted image 20241225155601.png]]

**Try to reduce observation space now.**

Remove proximity obs.
converge even faser.
![[Pasted image 20241225163010.png]]

remove time step: does not change much
I remain with the 3  necessary value: steering, direction, angle
PPO:
![[Pasted image 20241225163712.png]]

DQN:
![[Pasted image 20241225165000.png]]


try to remove direction. should be able to learn something but less efficiently.
Indeed it take too long. put it back again
![[Pasted image 20241225164302.png]]



#### **Steering with continues value.** 
action [-1,1] directly set steering value.
Very good ! converge 2x faster
![[Pasted image 20241225165736.png]]

try DDPG (it/s goes 50x slower)
result are better, but i retried and get bad results.. it may be a bit unstable.
![[Pasted image 20241225170134.png]]
![[Pasted image 20241225170513.png]]

discreet actions with PPO (20 actions)
Not so much changes. go back to continuous values.

![[Pasted image 20241225171710.png]]

steering obs is now useless. try to remove it.
Work still very well.
![[Pasted image 20241225172642.png]]
Try with continuous action from [0,1] instead of [-1,1]
result is worst. so keep range [-1,1] but not so big difference

![[Pasted image 20241225173454.png]]
### 8.  Simulate spidy ( continuous_world_v3)

now, action set directly param1 and param2[-1,1]
if param1 is in range 0.2 - 0.3 and param2 [-0.3, -0.2]]
steering according to param1 value
do the other way around

add steering to observation

**Results**
It is relatively rare that botn param1 and param2 are in the target range. Thus the environment is too sparse. Let see with PPO 1M timesteps

If not i should first teach the model how to tweak parm1 and parma2 in order to produce the desired steering. -> observation (desired steering, actual steering)
reward, if the actual steering matches the desired steering at epsilon close.
when this model is trained

Use another model that has as observation the target and angle and action is steering. (like the model I strained before)

**Learn hierarchical modeling.**

## 9. SteeringEnv

New strategie: As the value of param1 and param2 will affect directly the steering of the agent, reward the steering instead of some far away goal.
Create an environment dedicated to learn how to turn.

Actions: continuous, set directly param1 and parma2.
Observation : target steering value [ -1, 1], timestep
reward: (2 - abs(target steering - actual steering) )/2
physics : target steering changes incremetaly and randomly.