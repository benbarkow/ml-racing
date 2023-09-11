import os
import numpy as np
import argparse
from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy
from imitation.algorithms.adversarial.gail import GAIL
from imitation.data import rollout
from imitation.data.types import Trajectory
from imitation.rewards.reward_nets import BasicShapedRewardNet
from imitation.util.networks import RunningNorm
from common import make_unity_env, SaveOnBestTrainingRewardCallback
from mlagents_envs.side_channel.engine_configuration_channel import \
	EngineConfigurationChannel
import config
from common import make_unity_env, SaveOnBestTrainingRewardCallback, linear_schedule
from tensorboard import program

parser = argparse.ArgumentParser(description='Train an RL agent using GAIL.')
parser.add_argument('-e', '--expert_data', type=str, default=None, help='Path to the expert dataset (.npz file)')
parser.add_argument('-ex', '--executable', type=str, default="build/ml-racing-project", help='Executable to train on')
parser.add_argument('-n', '--num_envs', type=int, default=1, help='Number of parallel environments to use for training')
parser.add_argument('-st', '--sim_timescale', type=float, default=1.0, help='Timescale of the simulation')
parser.add_argument('-v', '--visualize', action='store_true', help='Whether to visualize the simulation')
parser.add_argument('-tb', '--tensorboard', action='store_true', help='Whether to start Tensorboard')
parser.add_argument('-c', '--cuda', action='store_true', help='Whether to start use Cuda or cpu')
argus = parser.parse_args()

SEED = 42

if __name__ == '__main__':
	os.makedirs(config.log_dir, exist_ok=True)
	os.makedirs(config.tb_logs, exist_ok=True)
	os.makedirs(config.models_dir, exist_ok=True)
	# This is a non-blocking call that only loads the environment.
	channel = EngineConfigurationChannel()
	channel.set_configuration_parameters(time_scale=1.0)

	#tensorboard
	if argus.tensorboard:
		tb = program.TensorBoard()
		tb.configure(argv=[None, '--logdir', config.tb_logs])
		url = tb.launch()
		print(f"Tensorflow listening on {url}")

	# model = PPO('MlpPolicy', env, verbose=1)
	env = make_unity_env(argus.executable, argus.num_envs, visual=argus.visualize, sim_timescale=argus.sim_timescale, log_dir=config.log_dir)

	# Load expert data
	expert_data = np.load(argus.expert_data, allow_pickle=True)

	expert_trajectories = expert_data['trajectories']
	rollouts = [Trajectory(obs=traj['observations'], acts=traj['actions'][:-1], infos=None, terminal=None) for traj in expert_trajectories]


	# Initialize the PPO learner
	learner = PPO('MlpPolicy', env, tensorboard_log=config.tb_logs, n_epochs=1 , learning_rate=linear_schedule(config.lr), gamma=config.gamma, policy_kwargs=config.policy_kwargs, device="cuda" if argus.cuda else "cpu")

	# Initialize the reward network for GAIL
	reward_net = BasicShapedRewardNet(
		observation_space=env.observation_space,
		action_space=env.action_space,
		normalize_input_layer=RunningNorm
	)

	# Initialize the GAIL trainer
	gail_trainer = GAIL(
		demonstrations=rollouts,
		demo_batch_size=512,
		gen_replay_buffer_capacity=1024,
		n_disc_updates_per_round=4,
		venv=env,
		gen_algo=learner,
		reward_net=reward_net
	)

	# Callback for saving best model
	# callback = SaveOnBestTrainingRewardCallback(check_freq=1000, log_dir="logs/", save_path="models/")

	# Train using GAIL
	gail_trainer.train(20000)

	# Evaluate the policy
	mean_reward, _ = evaluate_policy(learner, env, n_eval_episodes=10)
	print(f"Mean reward after GAIL training: {mean_reward}")

	# Save the trained model
	learner.save("models/gail_trained_model")
	print("Model saved!")