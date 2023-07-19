import config
import numpy as np
import torch
import argparse
import os
from FeatureExtractionWrapper import FeatureExtractionWrapper
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.envs.unity_gym_env import UnityToGymWrapper
from mlagents_envs.side_channel.engine_configuration_channel import \
    EngineConfigurationChannel

from common import make_unity_env, SaveOnBestTrainingRewardCallback, linear_schedule

from PIL import Image
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from tensorboard import program

parser = argparse.ArgumentParser(description='Train an RL agent. Can be used to train a new agent or to continue training an existing one.')
parser.add_argument('-v', '--visualize', action='store_true', help='Whether to visualize the simulation')
parser.add_argument('-tb', '--tensorboard', action='store_true', help='Whether to start Tensorboard')
parser.add_argument('-c', '--cuda', action='store_true', help='Whether to start use Cuda or cpu')
parser.add_argument('-n', '--num_envs', type=int, default=1, help='Number of parallel environments to use for training (max. ~number of CPU cores)')
parser.add_argument('-st', '--sim_timescale', type=float, default=1.0, help='Timescale of the simulation')
parser.add_argument('-ex', '--executable', type=str, default="build/ml-racing-project", help='Executable to train on')
argus = parser.parse_args()

if __name__ == '__main__':

	os.makedirs(config.log_dir, exist_ok=True)
	os.makedirs(config.tb_logs, exist_ok=True)
	os.makedirs(config.models_dir, exist_ok=True)
	# This is a non-blocking call that only loads the environment.
	channel = EngineConfigurationChannel()
	channel.set_configuration_parameters(time_scale=5.0)


	#tensorboard
	if argus.tensorboard:
		tb = program.TensorBoard()
		tb.configure(argv=[None, '--logdir', config.tb_logs])
		url = tb.launch()
		print(f"Tensorflow listening on {url}")

	# model = PPO('MlpPolicy', env, verbose=1)
	env = make_unity_env(argus.executable, argus.num_envs, visual=argus.visualize, sim_timescale=argus.sim_timescale, log_dir=config.log_dir)
	model = PPO('MlpPolicy', env, verbose=1, use_sde=False, tensorboard_log=config.tb_logs, n_steps=config.n_steps, learning_rate=linear_schedule(config.lr), gamma=config.gamma, policy_kwargs=config.policy_kwargs, device="cuda" if argus.cuda else "cpu")
	# model = PPO.load(config.models_dir + "image_racing_02.zip", env=env, device="cuda")

	callback = SaveOnBestTrainingRewardCallback(check_freq=(config.n_steps*2)+5, log_dir=config.log_dir, save_path=config.models_dir)
	model.learn(total_timesteps=5000000, callback=callback)
	print("Training complete.")

	#save to disk
	model.save(config.models_dir + "")
	print("Model saved to " + config.models_dir + "my_trained_model.")

