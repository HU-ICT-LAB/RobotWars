"""Visualise the model trained for multi agent tank trainer."""
from cv2 import cv2
from stable_baselines3 import PPO
import supersuit as ss

import ai_trainer.envs.tank_simulation.environment as tank_simulation

policy_file = "../../trained_policies/1vq6y4sp/model.zip"
env = tank_simulation.TankEnv(step_size=1 / 30)
env = ss.frame_stack_v1(env, 5)
model = PPO.load(policy_file)

window_name = "Tank Environment"
cv2.namedWindow(window_name)
env.reset()
for agent in env.agent_iter():
    obs, reward, done, info = env.last()
    act = model.predict(obs, deterministic=True)[0] if not done else None
    env.step(act)
    cv2.imshow(window_name, env.render(mode="rgb_array"))
    cv2.waitKey(round(1000 / 60))
    if not cv2.getWindowProperty(window_name, 0) >= 0:
        break
    if all(env.dones.values()):
        env.reset()
        model = PPO.load(policy_file)
env.close()
