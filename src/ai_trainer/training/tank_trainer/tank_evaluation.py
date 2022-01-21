import numpy as np
from cv2 import cv2
from stable_baselines3 import PPO
import supersuit as ss

from ai_trainer.envs.tank_simulation.environment import TankEnv

env = TankEnv()
env = ss.frame_stack_v1(env, 3)
model = PPO.load("../../trained_policies/28xhr4md/model.zip", env=env)

window_name = "Tank Environment"
cv2.namedWindow(window_name)
obs = env.reset()
while cv2.getWindowProperty(window_name, 0) >= 0:
    # action = env.action_space.sample()
    action = model.predict(obs)[0]
    # action = np.array([0., 0., 0.1, 0., 0.1, 1.])
    obs, reward, done, info = env.step(action)
    cv2.imshow(window_name, env.render())
    cv2.waitKey(round(env.step_size * 1000))
    if done:
        obs = env.reset()
env.close()
