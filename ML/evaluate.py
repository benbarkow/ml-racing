import argparse

from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
from mlagents_envs.envs.unity_gym_env import UnityToGymWrapper

from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3 import PPO

import config

if __name__ == '__main__':

    #init and start simulation
	channel = EngineConfigurationChannel()
	channel.set_configuration_parameters(time_scale=5.0)
	unity_env = UnityEnvironment(file_name="build/ml-racing-project", seed=1, side_channels=[channel])
	env = UnityToGymWrapper(unity_env)
	env.reset()
	print("sample action: ", env.action_space.sample())
	print("observation space shape: ", env.observation_space.shape)
	env = Monitor(env, config.log_dir)

	model = PPO.load("models/betweenModel0.02.zip", env)

	reward, _ = evaluate_policy(model, env, n_eval_episodes=5)
	print(f"Reward: {reward}")


