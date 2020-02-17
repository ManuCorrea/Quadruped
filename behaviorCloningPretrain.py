"""
This script is used to pretrain an agent with behavior cloning and then train on virtual environment.
"""
import gym

from stable_baselines.gail import ExpertDataset

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

# in my case it performed well with epochs between 600 to 7000 but I have to create a more diverse dataset to make it work properly. 
parser.add_argument('-pt', default=800, type=int,
                    help='the number of pretrain epochs') 
parser.add_argument('-v', default=0, type=bool,
                    help='render or not render the env')

parser.add_argument('--model', default='TRPO', type=str,
                    help='Choose what model to use. Available TRPO, DDPG, PPO_2.')

parser.add_argument('--pretrainVisualization', default=0, type=bool,
                    help='See how your model performs after pretraining with expert dataset')

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

# Using only one expert trajectory
# you can specify `traj_limitation=-1` for using the whole dataset
dataset = ExpertDataset(expert_path='./pretrain/dummy_quadruped.npz',
                        traj_limitation=-1, batch_size=128)


model.pretrain(dataset, n_epochs=args['pt'])

if args['pretrainVisualization']:
    # Test the pre-trained model
    env = model.get_env()
    obs = env.reset()

    reward_sum = 0.0
    for _ in range(1000):
            action, _ = model.predict(obs)
            obs, reward, done, _ = env.step(action)
            reward_sum += reward
            env.render()
            if done:
                    print(reward_sum)
                    reward_sum = 0.0
                    obs = env.reset()

# As an option, you can train the RL agent
model.learn(total_timesteps=args['timesteps'])
model.save('./pretrain/Preentrenado_{} bs, {} timesteps'.format(args['bs'], args['timesteps']))
