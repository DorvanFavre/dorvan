from gymnasium.envs.registration import register

register(
    id="GridWorld-v0",
    entry_point="babybot01_env.envs:GridWorldEnv",
)
register(
    id="Spidy-v0",
    entry_point="babybot01_env.envs:SpidyEnv",
)
