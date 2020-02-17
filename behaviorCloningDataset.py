'''
This program is used to generate an "expert" trajectory and create a dataset.
The sequence give is far from being good, that is my point, making easy to the robot to walk
but without too much preparation and human work.

For more information https://stable-baselines.readthedocs.io/en/master/guide/pretrain.html
'''

import gym

from stable_baselines.gail import generate_expert_traj

import numpy as np

def degToRad(deg):
    return deg * 0.0175

state = 0

# There are 2 different patterns to make it go forward.
# 0.2 equals to 11.46 degrees
mov0 = [degToRad(30), degToRad(-30), degToRad(30), degToRad(-30), -0.2, 0.0, 0.0, 0.0, degToRad(-90), degToRad(-90), degToRad(-90), degToRad(-90)] # after reset
mov1 = [0.0, degToRad(-30), degToRad(30), degToRad(-30), -0.2, 0.0, 0.0, 0.0, degToRad(-90), degToRad(-90), degToRad(-90), degToRad(-90)] 
mov2 = [0.0, 0.0, degToRad(30), degToRad(-30), 0.0, -0.2, 0.0, 0.0, degToRad(-90), degToRad(-90), degToRad(-90), degToRad(-90)]
mov3 = [0.0, 0.0, degToRad(90), degToRad(-30), 0.0, 0.0, -0.2, 0.0, degToRad(-90), degToRad(-90), degToRad(-90), degToRad(-90)]
mov4 = [degToRad(30), degToRad(-30), degToRad(30), degToRad(30), 0.2, 0.2, 0.2, 0.0, degToRad(-90), degToRad(-90), degToRad(-90), degToRad(-90)] # next ups all and move back
mov5 = [degToRad(30), degToRad(-30), degToRad(30), degToRad(30), 0.0, 0.0, 0.0, 0.0, degToRad(-90), degToRad(-90), degToRad(-90), degToRad(-90)]

mov6 = [degToRad(30), 0.0, degToRad(30), degToRad(-30), 0.0, -0.2, 0.0, 0.0, degToRad(-90), degToRad(-90), degToRad(-90), degToRad(-90)]
mov7 = [degToRad(30), 0.0, degToRad(30), degToRad(-30), 0.0, -0.2, 0.0, 0.0, degToRad(-90), degToRad(-90), degToRad(-90), degToRad(-90)] 
mov8 = [0.0, 0.0, degToRad(30), degToRad(-30), -0.2, 0.0, 0.0, 0.0, degToRad(-90), degToRad(-90), degToRad(-90), degToRad(-90)]
mov9 = [0.0, 0.0, degToRad(30), degToRad(-30), 0.0, 0.0, -0.2, 0.0, degToRad(-90), degToRad(-90), degToRad(-90), degToRad(-90)]
mov10 = [degToRad(30), degToRad(-30), degToRad(30), degToRad(30), 0.2, 0.2, 0.0, 0.2, degToRad(-90), degToRad(-90), degToRad(-90), degToRad(-90)] # next ups all and move back
mov11 = [degToRad(30), degToRad(-30), degToRad(30), degToRad(30), 0.0, 0.0, 0.0, 0.0, degToRad(-90), degToRad(-90), degToRad(-90), degToRad(-90)]

actions = [mov0, mov1, mov2, mov3, mov4, mov5, mov6, mov7, mov8, mov9, mov10, mov11]

actions = [np.array(mov) for mov in actions]

def dummy_expert(_obs):
    global state
    global actions
    state += 1
    if state == 11:
        state = 0
    # TODO add noise to each angle independently
    return actions[state] + round(0.2*np.random.random_sample()-0.1, 2) # +-0.1 rad as noise for all angles

env = gym.make('gym_quadruped:quadruped-v0', visualize=False)

# Data will be saved in a numpy archive named `dummy_quadruped.npz`
# when using something different than an RL expert,
# you must pass the environment object explicitly
generate_expert_traj(dummy_expert, './pretrain/dummy_quadruped', env, n_episodes=200)