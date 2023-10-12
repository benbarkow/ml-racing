import argparse
import numpy as np
from stable_baselines3.common.vec_env import SubprocVecEnv
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
from mlagents_envs.envs.unity_gym_env import UnityToGymWrapper
from stable_baselines3.common.monitor import Monitor
from stable_baselines3 import PPO
from MixedWrapper import MixedWrapper
import config

def make_env(env_path, rank, channel):
    def _init():
        unity_env = UnityEnvironment(file_name=env_path, worker_id=rank, side_channels=[channel])
        env = UnityToGymWrapper(unity_env, allow_multiple_obs=True)
        env = MixedWrapper(env, disable_image=argus.no_image)
        env = Monitor(env, config.log_dir + f"_env_{rank}")
        return env
    return _init

parser = argparse.ArgumentParser(description='Evaluate an RL agent.')
parser.add_argument('-ex', '--executable', type=str, default="build/ml-racing-project", help='Executable to evaluate on')
parser.add_argument('-m', '--model', type=str, default="best_model.zip", help='Model to use')
parser.add_argument('--test', action='store_true', help='Test model')
parser.add_argument('-ts', '--timescale', type=float, default=1.0, help='Time scale of the simulation')
parser.add_argument('-n', '--num_envs', type=int, default=1, help='Number of environments to use')
parser.add_argument('--no_image', action='store_true', help='Disable image input')
argus = parser.parse_args()

if __name__ == '__main__':
    channel = EngineConfigurationChannel()
    channel.set_configuration_parameters(time_scale=argus.timescale)
    env = SubprocVecEnv([make_env(argus.executable, i, channel) for i in range(argus.num_envs)])

    if not argus.test:
        model = PPO.load(argus.model, env, device="cuda")

    reward_list = []

    obs = env.reset()
    commulative_rewards = np.zeros(argus.num_envs)

    for i in range(30):
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
