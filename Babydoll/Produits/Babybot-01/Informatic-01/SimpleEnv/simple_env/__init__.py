from gymnasium.envs.registration import register

register(
    id="simple_env/GridWorld-v0",
    entry_point="simple_env.envs:GridWorldEnv",
)
