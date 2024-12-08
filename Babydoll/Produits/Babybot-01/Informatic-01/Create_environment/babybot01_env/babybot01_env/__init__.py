from gymnasium.envs.registration import register

register(
    id="GridWorld-v0",
    entry_point="babybot01_env.envs:GridWorldEnv",
)
register(
    id="Spidy-v0",
    entry_point="babybot01_env.envs:SpidyEnv",
)
register(
    id="Spidy-v2",
    entry_point="babybot01_env.envs:SpidyEnvV2",
)
register(
    id="Spidy-v3",
    entry_point="babybot01_env.envs:SpidyEnvV3",
)