import os
from stable_baselines3 import PPO
import supersuit as ss
import wandb
from wandb.integration.sb3 import WandbCallback
import ai_trainer.envs.tank_simulation.environment as tank_simulation

config = {
    'policy_type': 'MlpPolicy',
    'total_timesteps': 999999999999999999,
}
run = wandb.init(
    project='Tank',
    entity="robotwarshu",
    config=config,
    sync_tensorboard=True,
    save_code=True,
)

env = tank_simulation.parallel_env()
env = ss.frame_stack_v1(env, 5)
env = ss.pettingzoo_env_to_vec_env_v1(env)
if os.name != 'nt':
    env = ss.concat_vec_envs_v1(env, 8, num_cpus=4, base_class='stable_baselines3')


#env = VecVideoRecorder(env, f"../../videos/{run.id}", record_video_trigger=lambda x: x % 2000 == 0, video_length=200)

model = PPO(config['policy_type'], env, verbose=3, tensorboard_log=f"../../runs/{run.id}")
model.learn(
    total_timesteps=config['total_timesteps'],
    callback=WandbCallback(
        gradient_save_freq=100,
        model_save_freq=100,
        model_save_path=f"../../trained_policies/{run.id}",
        verbose=2
    ))
run.finish()
