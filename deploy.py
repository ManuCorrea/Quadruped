"""
This script is to deploy and see the model performance on the virtual environment.
Choose the model you used with --model and the saved params with --modelFile
"""
import gym

from stable_baselines.common.policies import MlpPolicy
from stable_baselines import TRPO

import argparse

import os
def check_dir(path):
    if not os.path.exists(path):
        os.makedirs(path)

parser = argparse.ArgumentParser(description='Script for deploy and visualize trained models')
parser.add_argument('--modelFile', type=str,
                    help='Model to deploy')
parser.add_argument('--model', default='TRPO', type=str,
                    help='Choose what model to use. Available TRPO, DDPG, PPO_2.')

args = vars(parser.parse_args())

choosenModel = args['model']

if choosenModel == 'TRPO':
    from stable_baselines import TRPO
    from stable_baselines.common.policies import MlpPolicy

    env = gym.make('gym_quadruped:quadruped-v0', visualize=args['v'])
    check_dir('./pretrain/TRPO/')
    model = TRPO(MlpPolicy, env, verbose=1,
            tensorboard_log='./pretrain/TRPO/',
            timesteps_per_batch=args['bs'])

elif choosenModel == 'DDPG':
    from stable_baselines import DDPG
    from stable_baselines.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise, AdaptiveParamNoiseSpec
    from stable_baselines.ddpg.policies import MlpPolicy
    import numpy as np

    env = gym.make('gym_quadruped:quadruped-v0', visualize=args['v'])

    # the noise objects for DDPG
    n_actions = env.action_space.shape[-1]
    param_noise = None
    action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=float(0.5) * np.ones(n_actions))

    check_dir('./pretrain/DDPG/')
    model = DDPG(MlpPolicy, env, verbose=1, param_noise=param_noise, action_noise=action_noise, tensorboard_log='./pretrain/DDPG/')

elif choosenModel == 'PPO_2':
    from stable_baselines.common import make_vec_env
    from stable_baselines.common.policies import MlpPolicy
    from stable_baselines import PPO2

    env = make_vec_env('gym_quadruped:quadruped-v0', n_envs=4)

    check_dir('./pretrain/PPO/')
    model = PPO2(MlpPolicy, env, verbose=1,
                tensorboard_log='./pretrain/PPO/')
    
else:
    print('Model choosen not available, check spelling or if it is supported')

model.load(args['modelFile'])

obs = env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, done, info = env.step(action)
    env.render()
    if done:
        env.reset()