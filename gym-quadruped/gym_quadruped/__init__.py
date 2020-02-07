from gym.envs.registration import register

register(
    id='quadruped-v0',
    entry_point='gym_quadruped.envs:QuadrupedEnv',
)