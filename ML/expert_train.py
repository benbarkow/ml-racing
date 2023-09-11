import pygame
import numpy as np
import argparse
from mlagents_envs.envs.unity_gym_env import UnityToGymWrapper
from CnnWrapper import CnnWrapper
from StackCnnWrapper import StackCnnWrapper
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
import os

parser = argparse.ArgumentParser(description='Train an RL agent. Can be used to train a new agent or to continue training an existing one.')
parser.add_argument('-ex', '--executable', type=str, default="build/ml-racing-project", help='Executable to train on')
parser.add_argument('-df', '--datafile', type=str, default=None, help='Path to an existing dataset file to extend')
argus = parser.parse_args()

# Initialize pygame
pygame.init()

# Simulator settings
screen_width = 800
screen_height = 600
screen = pygame.display.set_mode((screen_width, screen_height))

joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
	# No joysticks!
	print("Error, I didn't find any joysticks.")
else:
	# Use the first joystick
	joystick = pygame.joystick.Joystick(0)
	joystick.init()

def human_interface():
	"""Map pygame inputs to actions."""
	action = np.zeros(2)  # Initialize a zero array of shape (2,)
	action[0] = 5
	keys = pygame.key.get_pressed()  # Get the state of all keys

	if keys[pygame.K_w]:
		action[1] = 5.0  # Forward
	if keys[pygame.K_a]:
		action[0] = 0.0  # Left
	if keys[pygame.K_d]:
		action[0] = 11.0  # Right

	# Still process other events to prevent the event queue from filling up
	for event in pygame.event.get():
		pass

	return action

def joystick_interface():
	"""Map PS4 controller inputs to actions."""
	action = np.zeros(2)  # Initialize a zero array of shape (2,)
	action[0] = 5

	# Get the state of the joystick axes
	left_x_axis = joystick.get_axis(0)  # Left stick horizontal (left is negative, right is positive)
	left_y_axis = joystick.get_axis(1)  # Left stick vertical (up is negative, down is positive)
	right_trigger = joystick.get_axis(5)  # Right trigger (positive is fully pressed, negative is not pressed)

	#remat left_x_axis [-1, 1] to discrete range [0, 11] integer values and store in action[0]
	action[0] = (left_x_axis + 1) * 5.5
	#remap right_trigger [-1, 1] to discrete range [0, 5] integer values and store in action[1]
	action[1] = (right_trigger + 1) * 2.5

	# convert to discrete values
	action[0] = int(action[0])
	action[1] = int(action[1])


	# Still process other events to prevent the event queue from filling up
	for event in pygame.event.get():
		pass

	return action

def collect_human_data(env, num_episodes=100, datafile=None):
	if datafile and os.path.exists(datafile):
		# Load existing dataset if provided
		data = np.load(datafile, allow_pickle=True)
		expert_trajectories = list(data['trajectories'])
	else:
		expert_trajectories = []

	for _ in range(num_episodes):
		step_count = 0
		obs = env.reset()
		done = False
		trajectory = {"observations": [], "actions": []}

		while not done:
			joy_action = joystick_interface()
			action = joy_action

			if action is not None:
				trajectory["observations"].append(obs.tolist())  # Convert numpy array to list
				trajectory["actions"].append(action.tolist())  # Convert numpy array to list
				obs, _, done, _ = env.step(action)
				step_count += 1

		# Check if the user wants to use the recorded data
		print("Recorded {} steps.".format(step_count))
		print("Press 'X' to use the recorded data or 'Circle' to discard.")
		if wait_for_decision():
			print(trajectory.shape)
			expert_trajectories.append(trajectory)
			if datafile:
				np.savez(datafile, trajectories=expert_trajectories)
			else:
				np.savez('expert_datasets/new_dataset.npz', trajectories=expert_trajectories)

	pygame.quit()

	# Save the updated dataset

	return expert_trajectories

def wait_for_decision():
	while True:
		for event in pygame.event.get():
			if event.type == pygame.JOYBUTTONDOWN:
				if joystick.get_button(0):  # 'X' button
					print("Using recorded data.")
					return True
				elif joystick.get_button(1):  # 'Circle' button
					print("Discarding recorded data.")
					return False
		pygame.time.wait(100)  # Wait for a short time to reduce CPU usage


channel = EngineConfigurationChannel()
channel.set_configuration_parameters(time_scale=1.0)
unity_env = UnityEnvironment(file_name=argus.executable, side_channels=[channel])
env = UnityToGymWrapper(unity_env, allow_multiple_obs=True)
env = StackCnnWrapper(env)

expert_data = collect_human_data(env, datafile=argus.datafile)
