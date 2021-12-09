import cv2
from stable_baselines3 import PPO
from tank_env import TankEnv

env = TankEnv()

# model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="runs")
# model.learn(total_timesteps=3000000)
# model.save("tank_policy")

model = PPO.load("tank_policy")
window_name = "Tank Environment"
cv2.namedWindow(window_name)
obs = env.reset()
while cv2.getWindowProperty(window_name, 0) >= 0:
    action = model.predict(obs)
    obs, reward, done, info = env.step(action[0])
    cv2.imshow(window_name, env.render())
    cv2.waitKey(round(env.step_size * 1000))
    if done:
        obs = env.reset()
env.close()
