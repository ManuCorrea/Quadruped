import gym

from stable_baselines.common.policies import MlpPolicy
from stable_baselines import TRPO

import argparse

parser = argparse.ArgumentParser(description='Script for deploy and visualize trained models')
parser.add_argument('--model', type=str,
                    help='The total number of samples to train on')

args = vars(parser.parse_args())

env = gym.make('gym_quadruped:quadruped-v0', visualize=True)


model = TRPO(MlpPolicy, env, verbose=1)

model.load(args['model'])

obs = env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, done, info = env.step(action)
    env.render()
    if done:
        env.reset()