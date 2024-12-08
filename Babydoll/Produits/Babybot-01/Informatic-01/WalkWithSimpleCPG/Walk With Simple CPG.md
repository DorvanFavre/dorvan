
Try to make the simulated hexapod walk with the simple CPG implementation from NeuroMechFly. There are different option to try. See which one gives the bests results. 

- CPG on joints value
- CPG on leg coordinated using invert kinematics
- Hard coded CPG parameters to produce tripod gait
- Use RL agent to learn CPG parameters

Going further:
Implement a more complexe CPG network to make Fourier signals.


### Task 1 

- Use invert kinematic so i can use cpg for X and Z axis and leave Y to unchanged. The result is also more comprehensive.
- use a predefined gait. No use of RL.

**Results**

DONE : 01_walk_ik_cpg
It's working good.


### Task 2

Use RL to find a gait pattern.

- Addapt the spidy env to incorporate CPG and IK
- Train PPO

**Actions**
- coupling weights
- phase_biases

**Observation**
No observation : Constant
Reward-Driven Training

**Reward**
Speed
Position

Remarks:
It is good to use action value in the range of -1 to 1 or 0 to 1.
It is not good to use high dimensional action space such ax 12x12 actions.

**Results**


