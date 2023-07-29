import argparse

from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
from mlagents_envs.envs.unity_gym_env import UnityToGymWrapper

from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3 import PPO
from ImageWrapper import ImageWrapper

from FeatureExtractionWrapper import FeatureExtractionWrapper

import config

from time import sleep

if __name__ == '__main__':

    #init and start simulation
	channel = EngineConfigurationChannel()
	channel.set_configuration_parameters(time_scale=1.0)
	unity_env = UnityEnvironment(file_name="image_only_build_linux/ml-racing", seed=1, side_channels=[channel])
	env = UnityToGymWrapper(unity_env, allow_multiple_obs=True)
	env = ImageWrapper(env)
	env = Monitor(env, config.log_dir)

	reward_list = []

	model = PPO.load(config.models_dir + "best_model.zip", env, device="cuda")
	commulative_reward = 0
	
	observation = env.reset()
	for i in range(5):
		while True:
			action, _states = model.predict(observation)
			observation, reward, done, info = env.step(action)	
			commulative_reward += reward
			# print("observation: ", observation)
			# print("reward: ", reward)
			if done:
				print("Episode {} reward: ".format(i), commulative_reward)
				observation = env.reset()
				reward_list.append(commulative_reward)
				commulative_reward = 0
				break

	env.close()
	print("Average reward: ", sum(reward_list)/len(reward_list))


