import gym

from stable_baselines.common.policies import MlpPolicy
from stable_baselines import TRPO

env = gym.make('gym_quadruped:quadruped-v0', visualize=True)
model = TRPO(MlpPolicy, env, verbose=1,
            tensorboard_log='./TRPO/')
model.learn(total_timesteps=20000)
model.save("./TRPO/trpo_20000")

del model # remove to demonstrate saving and loading

model = TRPO.load("./TRPO/trpo_20000")

obs = env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render()