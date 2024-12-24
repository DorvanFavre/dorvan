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



