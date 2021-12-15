from stable_baselines3 import PPO
import supersuit as ss
from ai_trainer.envs.tank_simulation.environment import TankEnv

env = TankEnv()
env = ss.frame_stack_v1(env, 3)

model = PPO("MlpPolicy", env, verbose=1)
try:
    model.learn(total_timesteps=5000000)
except Exception as e:
    print(e)
finally:
    model.save("../../trained_policies/tank_policy")
