import numpy as np
import cv2
from stable_baselines3 import PPO
from tank_env import TankEnv

env = TankEnv()

model = PPO("MlpPolicy", env, verbose=1)
try:
    model.learn(total_timesteps=30)
except Exception as e:
    print(e)
finally:
    model.save("tank_policy")

model = PPO.load("tank_policy", env=env)
window_name = "Tank Environment"
cv2.namedWindow(window_name)
obs = env.reset()
while cv2.getWindowProperty(window_name, 0) >= 0:
    # action = env.action_space.sample()
    action = model.predict(obs)[0]
    #action = np.array([0., 0., 0.1, 0., 0.1, 1.])
    obs, reward, done, info = env.step(action)
    cv2.imshow(window_name, env.render())
    cv2.waitKey(round(env.step_size * 1000))
    if done:
        obs = env.reset()
env.close()
