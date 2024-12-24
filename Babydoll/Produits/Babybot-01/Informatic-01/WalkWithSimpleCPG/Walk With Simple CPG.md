
Try to make the simulated hexapod walk with the simple CPG implementation from NeuroMechFly. There are different option to try. See which one gives the bests results. 

- CPG on joints value
- CPG on leg coordinated using invert kinematics
- Hard coded CPG parameters to produce tripod gait
- Use RL agent to learn CPG parameters

Going further:
Implement a more complexe CPG network to make Fourier signals.


## Task 1 

- Use invert kinematic so i can use cpg for X and Z axis and leave Y to unchanged. The result is also more comprehensive.
- use a predefined gait. No use of RL.

**Results**

DONE : 01_walk_ik_cpg
It's working good.

## Task 2

**Use RL to find a gait pattern.**
-> Speedy v3
-> 02_walk with_ik_cpg_ppo

#### Preparation 

- Addapt the spidy env to incorporate CPG and IK
- Train PPO

**Actions**
- coupling weights
- phase_biases
self.action_space = spaces.Box(low=0.,high= 1., shape=(2,12,12), dtype=ACTION_TYPE)

**Observation**
No observation : Constant
Reward-Driven Training

**Reward**
Speed
Position

Remarks:
It is good to use action value in the range of -1 to 1 or 0 to 1.
It is not good to use high dimensional action space such ax 12x12 actions.

#### Realization

Create Spidyv3 env from spidyv2.
Adapt the env to add CPG and IK.
Walk with CPG configuration from "01_walk_ik_cpg"
Display CPG configuration as a plot.

#### Results

**ppo_spidy_v3_1**
Before training
![[Pasted image 20241210105448.png]]
After 1M training: no results:
![[Pasted image 20241210132949.png]]

It show this but actually the robot isn't moving

Let's get inspired by the gymnasium Walker environment.
Let's try to add observation space: **Dummy observation space**
It need to add some random noise otherwasie PPO can nerver learn.
Or use other algorithm for direct optimization problem. (CMA-ES)

**ppo_spidy_v3_2**
observation = np.random.uniform(-0.1, 0.1, size=(10,))
Try to terminated after 990 steps

try with observation = coupling_weights and phase_biases.
Did not learn anything after 2M steps.


**ppo_spidy_v4_1

Make some adjustments:
Action: 
- Continuous value [-1,1] that to be added to some pertinent CPG parameters (pertinent phases_biases)
- Add a update rate to 0.1

observation: 
- pertinent parameter of phases_biases

Modification to the environment:
- Map pertinent CPG phases_biases parameters to 12x12 phases_biases matrix.

Reward:
- distance from origin

Training:
- With pre-initiate walking gait
- Without pre-trained walking gait

Comments:
- Watch actions evolution (pyplot)

Results:
Not working :'(
1M step: learn nothing

**Spidyv4_1**
Try to simplify again by only exposing currently used phases biases.
Add a debug mode to the env which gives acces to cursor to change these phases biases manually and see how it respond.

Result.
I can easily control the phase_biases.
Keep this project as a demo.

**Spidyv4_2**
04_walk.ipynb
spidy-v4_2

See if he can recover from learning only two phase biases (0 and 5)
observation: 2 phases biases, try to add some random noise too in a third value.

Action space: 2 continuous value. increment phase biases.

Result isn't good.
![[Pasted image 20241214112048.png]]

Try to reset phases_biases on reset.
Monitor action:
ppo_2_env_4_2:
Not so much difference.

**Spidy-v4_3**

Try reach target by making turns 

spidy-4_3
05_reach.ipynb

Not convincing

![[Pasted image 20241214154630.png]]



**Spidy-v4_4**
try with discrete action space
Better than continuous.
But still bad

Mais il trouve un minimum local: c'est mieux de rester sur place que de d'éloigner de la cible.
![[Pasted image 20241214183353.png]]

Try to change the reward function again


**06_reach_DQN**

It seems that there is no reset after one epoch.
There is reset but investigate why phase_biases does not reset (log every 1 step to see)