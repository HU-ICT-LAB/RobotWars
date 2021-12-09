import cv2
import numpy as np
from tank_env import TankEnv

window_name = "Tank Environment"
cv2.namedWindow(window_name)
env = TankEnv()
obs = env.reset()
while cv2.getWindowProperty(window_name, 0) >= 0:
    action = env.action_space.sample()
    obs, reward, done, info = env.step(action)
    cv2.imshow(window_name, env.render())
    cv2.waitKey(round(env.step_size * 1000))
    if done:
        obs = env.reset()
env.close()
