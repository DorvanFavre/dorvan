

Video: https://www.youtube.com/watch?v=VwTd5cWJx2M&list=PLomN94wShLICFTL-wyW1Sa2QqSPm_KvMh&index=7&ab_channel=KrisTemmerman

CHatGPT:
```Python
import numpy as np

# Parameters
step_length = 20  # forward distance each leg will move (mm)
step_height = 15  # height each leg lifts off the ground (mm)
cycle_duration = 1.0  # duration of a full walking cycle (seconds)
time_steps = 100  # number of steps in the cycle

# Hexapod leg positions for resting stance (leg positions when not moving)
# Adjust according to hexapod body dimensions and IK function requirements
rest_positions = {
    'leg_1': [x1, y1, z1],
    'leg_2': [x2, y2, z2],
    'leg_3': [x3, y3, z3],
    'leg_4': [x4, y4, z4],
    'leg_5': [x5, y5, z5],
    'leg_6': [x6, y6, z6]
}

# Gait groupings for tripod gait (groups of legs that move together)
tripod_1 = ['leg_1', 'leg_4', 'leg_5']
tripod_2 = ['leg_2', 'leg_3', 'leg_6']

def generate_leg_trajectory(start_pos, step_length, step_height, phase):
    """Generate the trajectory for a single leg."""
    # Forward step in x-direction
    trajectory = []
    for t in np.linspace(0, 1, time_steps // 2):
        x = start_pos[0] + step_length * (2 * t - 1) * (-1)**phase
        z = start_pos[2] + step_height * np.sin(np.pi * t) if phase == 0 else start_pos[2]
        trajectory.append([x, start_pos[1], z])

    return trajectory

# Walking cycle function
def walking_cycle(time_step):
    positions = {}
    phase = (time_step // (time_steps // 2)) % 2
    for leg in rest_positions.keys():
        if leg in tripod_1:
            positions[leg] = generate_leg_trajectory(rest_positions[leg], step_length, step_height, phase)[time_step % (time_steps // 2)]
        elif leg in tripod_2:
            positions[leg] = generate_leg_trajectory(rest_positions[leg], step_length, step_height, 1 - phase)[time_step % (time_steps // 2)]
    return positions

# Main loop to execute walking cycle
for t in range(time_steps):
    leg_positions = walking_cycle(t)
    for leg, pos in leg_positions.items():
        ik_solution = inverse_kinematics(leg, pos)  # Call your provided IK function
        # Send ik_solution to motors for each joint

```


Quadruped: https://github.com/miguelasd688/4-legged-robot-model


