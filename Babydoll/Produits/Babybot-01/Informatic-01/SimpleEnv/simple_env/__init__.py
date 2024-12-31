from gymnasium.envs.registration import register

register(
    id="simple_env/GridWorld-v0",
    entry_point="simple_env.envs:GridWorldEnv",
)
register(
    id="simple_env/GridWorld-v2",
    entry_point="simple_env.envs:GridWorldEnv2",
)
register(
    id="simple_env/GridWorld-v3",
    entry_point="simple_env.envs:GridWorldEnv3",
)
register(
    id="simple_env/ContinuousWorld-v1",
    entry_point="simple_env.envs:ContinuousWorldEnv",
)
register(
    id="simple_env/ContinuousWorld-v2",
    entry_point="simple_env.envs:ContinuousWorldEnv2",
)
register(
    id="simple_env/ContinuousWorld-v3",
    entry_point="simple_env.envs:ContinuousWorldEnv3",
)







