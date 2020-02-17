"""
Simple train in the environment, no pretrain which could take much longer
"""
import gym

import argparse

import os

def check_dir(path):
    if not os.path.exists(path):
        os.makedirs(path)

parser = argparse.ArgumentParser(description='Pretrain with dummy dataset and then perform real RL training')
parser.add_argument('--timesteps', default=100000, type=int,
                    help='The total number of samples to train on')
parser.add_argument('-bs', default=1024, type=int,
                    help='the number of timesteps to run per batch (horizon)')
                    
parser.add_argument('-v', default=0, type=bool,
                    help='render or not render the env')

parser.add_argument('--model', default='TRPO', type=str,
                    help='Choose what model to use. Available TRPO, DDPG, PPO_2.')

parser.add_argument('--baseModel', default=None, type=str,
                    help='Keep training a trained model. Select the directory where it is')

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
    from stable_baselines.ddpg.policies import MlpPolicy # Special MLP policy for DPPG
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

    # make_vec_env() is used for multiprocess enviroment
    env = make_vec_env('gym_quadruped:quadruped-v0', n_envs=4)

    check_dir('./pretrain/PPO/')
    model = PPO2(MlpPolicy, env, verbose=1,
                tensorboard_log='./pretrain/PPO/')
    
else:
    print('Model choosen not available, check spelling or if it is supported')

if args['baseModel'] is not None:
    print("Using trained model {}". format(args['baseModel']))
    model.load(args['baseModel'])
else:
    print("Training model from scratch")

# This loop is used to save models and keep training to avoid losses of models and being able to choose quadruped
# abilities at different stages
for i in range(5):
    model.learn(total_timesteps=args['timesteps'])
    model.save("./TRPO/millon/largo3/trpo_{}_{} timesteps".format(i, args['timesteps']))
