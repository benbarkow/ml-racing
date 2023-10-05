import argparse

from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
from mlagents_envs.envs.unity_gym_env import UnityToGymWrapper

from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3 import PPO
import argparse
from StackCnnWrapper import StackCnnWrapper

import config

from time import sleep

parser = argparse.ArgumentParser(description='Train an RL agent. Can be used to train a new agent or to continue training an existing one.')
parser.add_argument('-ex', '--executable', type=str, default="build/ml-racing-project", help='Executable to train on')
parser.add_argument('-m', '--model', type=str, default="best_model.zip", help='Model to use')
#timescale
parser.add_argument('-ts', '--timescale', type=float, default=1.0, help='Time scale of the simulation')
argus = parser.parse_args()

if __name__ == '__main__':

    #init and start simulation
	channel = EngineConfigurationChannel()
	channel.set_configuration_parameters(time_scale=argus.timescale)
	unity_env = UnityEnvironment(file_name=argus.executable, seed=1, side_channels=[channel])
	env = UnityToGymWrapper(unity_env, allow_multiple_obs=True)
	# env = CnnWrapper(env)
	env = StackCnnWrapper(env)
	env = Monitor(env, config.log_dir)

	reward_list = []

	model = PPO.load(config.models_dir + argus.model, env, device="cuda")

	# Evaluate the agent
	# mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=1)
	# print("mean_reward: ", mean_reward, "std_reward: ", std_reward)


	commulative_reward = 0
	
	observation = env.reset()
	for i in range(30):
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


