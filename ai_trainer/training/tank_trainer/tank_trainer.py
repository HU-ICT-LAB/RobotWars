from stable_baselines3 import PPO
from ai_trainer.envs.tank_simulation.environment import TankEnv

env = TankEnv()

model = PPO("MlpPolicy", env, verbose=1)
try:
    model.learn(total_timesteps=30)
except Exception as e:
    print(e)
finally:
    model.save("../../trained_policies/tank_policy")
