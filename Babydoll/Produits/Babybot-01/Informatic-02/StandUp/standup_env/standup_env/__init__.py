from gymnasium.envs.registration import register

register(
    id="standup_env/GridWorld-v0",
    entry_point="standup_env.envs:GridWorldEnv",
)
register(
    id="standup_env/StandUp-v1",
    entry_point="standup_env.envs:StandUpEnv",
)
