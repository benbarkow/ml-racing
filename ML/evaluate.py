import argparse
import numpy as np

from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
from mlagents_envs.envs.unity_gym_env import UnityToGymWrapper

from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3 import PPO
import argparse
from MixedWrapper import MixedWrapper

import config

from time import sleep

parser = argparse.ArgumentParser(description='Train an RL agent. Can be used to train a new agent or to continue training an existing one.')
parser.add_argument('-ex', '--executable', type=str, default="build/ml-racing-project", help='Executable to train on')
parser.add_argument('-m', '--model', type=str, default="best_model.zip", help='Model to use')
parser.add_argument('--test', action='store_true', help='Test model')
parser.add_argument('-n', '--num_envs', type=int, default=1, help='Number of environments to use')
parser.add_argument('--no_image', action='store_true', help='Disable image input')
#timescale
parser.add_argument('-ts', '--timescale', type=float, default=1.0, help='Time scale of the simulation')
argus = parser.parse_args()

def make_env(env_path, channel):
    def _init():
        unity_env = UnityEnvironment(file_name=env_path, side_channels=[channel])
        env = UnityToGymWrapper(unity_env, allow_multiple_obs=True)
        env = MixedWrapper(env, disable_image=argus.no_image)
        env = Monitor(env, config.log_dir)
        return env
    return _init

if __name__ == '__main__':
    channel = EngineConfigurationChannel()
    channel.set_configuration_parameters(time_scale=1)
    env = DummyVecEnv([make_env(argus.executable, channel) for _ in range(argus.num_envs)])

    if not argus.test:
        model = PPO.load(argus.model, env, device="cuda")

    reward_list = []
    obs = env.reset()
    commulative_rewards = np.zeros(argus.num_envs)

    for i in range(100):
        while True:
            if not argus.test:
                actions, _states = model.predict(obs)
            else:
                actions = np.array([[3, 3]] * argus.num_envs)

            obs, rewards, dones, info = env.step(actions)
            commulative_rewards += rewards

            if all(dones):
                print("Episode {} reward: {}".format(i, commulative_rewards))
                obs = env.reset()
                reward_list.extend(commulative_rewards)
                commulative_rewards = np.zeros(argus.num_envs)
                break

    env.close()
    print("Average reward: ", np.mean(reward_list))