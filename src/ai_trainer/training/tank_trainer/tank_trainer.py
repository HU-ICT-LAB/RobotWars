from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecVideoRecorder
import supersuit as ss
import wandb
from wandb.integration.sb3 import WandbCallback
from ai_trainer.envs.tank_simulation.environment import TankEnv

config = {
    'policy_type': 'MlpPolicy',
    'total_timesteps': 5000000,
}
run = wandb.init(
    project='Tank',
    entity="robotwarshu",
    config=config,
    sync_tensorboard=True,
    save_code=True,
)


def make_env():
    env = TankEnv()
    env = ss.frame_stack_v1(env, 3)
    return env


env = DummyVecEnv([make_env])
#env = VecVideoRecorder(env, f"../../videos/{run.id}", record_video_trigger=lambda x: x % 2000 == 0, video_length=200)

model = PPO(config['policy_type'], env, verbose=1, tensorboard_log=f"../../runs/{run.id}")
model.learn(
    total_timesteps=config['total_timesteps'],
    callback=WandbCallback(
        gradient_save_freq=100,
        model_save_path=f"../../trained_policies/{run.id}",
        verbose=2
    ))
run.finish()