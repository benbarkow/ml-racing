from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
from mlagents_envs.envs.unity_gym_env import UnityToGymWrapper

from stable_baselines3.common.monitor import Monitor
from stable_baselines3 import PPO

import config

if __name__ == '__main__':
	# This is a non-blocking call that only loads the environment.
	channel = EngineConfigurationChannel()
	channel.set_configuration_parameters(time_scale=50.0)
	unity_env = UnityEnvironment(file_name="build/ml-racing-project", seed=1, side_channels=[channel])
	env = UnityToGymWrapper(unity_env)
	env.reset()
	print("sample action: ", env.action_space.sample())
	print("observation space shape: ", env.observation_space.shape)
	env = Monitor(env, config.log_dir)
	# Start interacting with the environment.

	model = PPO('MlpPolicy', env, verbose=1)
	#begin learning
	print("Training model...")
	model.learn(total_timesteps=50000)
	print("Training complete.")

	#save to disk
	model.save(config.models_dir + "my_trained_model")
	print("Model saved to " + config.models_dir + "my_trained_model.")

