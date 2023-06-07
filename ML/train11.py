import os
import argparse
import warnings

from stable_baselines3 import PPO

from tensorboard import program

from common import make_unity_env, SaveOnBestTrainingRewardCallback, linear_schedule
import config

from stable_baselines3.common.torch_layers import CombinedExtractor



# arguments
parser = argparse.ArgumentParser(description='Train an RL agent. Can be used to train a new agent or to continue training an existing one.')
parser.add_argument('-s', '--sim_exec', type=str, default='basic_test', help='Path to the Unity executable\'s build folder and the name of the executable, i.e. "path/to/build_folder/executable_name"')
parser.add_argument('-t', '--timesteps', type=int, default=10000, help='Number of timesteps for training')
parser.add_argument('-m', '--model', type=str, default=None, help='Path to the model to continue training')
parser.add_argument('-n', '--num_envs', type=int, default=1, help='Number of parallel environments to use for training (max. ~number of CPU cores)')
parser.add_argument('-v', '--visualize', action='store_true', help='Whether to visualize the simulation')
parser.add_argument('-st', '--sim_timescale', type=float, default=1.0, help='Timescale of the simulation')
parser.add_argument('-tb', '--tensorboard', action='store_true', help='Whether to start Tensorboard')
argus = parser.parse_args()


warnings.filterwarnings('ignore', '.*truncated to dtype int32.*')

    
if __name__ == '__main__':

    # Create log dirs
    os.makedirs(config.log_dir, exist_ok=True)
    os.makedirs(config.tb_logs, exist_ok=True)
    os.makedirs(config.models_dir, exist_ok=True)

    if argus.tensorboard == True:
        # Start Tensorboard
        tb = program.TensorBoard()
        tb.configure(argv=[None, '--logdir', config.tb_logs])
        url = tb.launch()
        print(f"TensorBoard server listening on {url}")

    #init and start simulation
    env = make_unity_env(argus.sim_exec, argus.num_envs, render=argus.visualize, sim_timescale=argus.sim_timescale, log_dir=config.log_dir)

    #create the model
    if argus.model != None:
        # load existing model to continue training
        custom_objs = {'observation_space': env.observation_space, 'action_space': env.action_space}
        model = PPO.load(argus.model, env=env, custom_objects=custom_objs)
        #model.set_env(env)
    else:
        # create new model from scratch
        policy_kwargs = dict(
            features_extractor_class=CombinedExtractor, 
            features_extractor_kwargs=dict(
                cnn_output_dim=env.observation_space["image"].shape[0]), # cnn (feature extractor) feature size same as image side length (width or height)
            net_arch=[128, 128, 128], # MLP network hidden layers and their sizes
        )
        model = PPO('MultiInputPolicy', env, verbose=1, use_sde=False, tensorboard_log=config.tb_logs, n_steps=config.n_steps, learning_rate=linear_schedule(config.lr), gamma=config.gamma, policy_kwargs=policy_kwargs)


    #begin learning
    callback = SaveOnBestTrainingRewardCallback(check_freq=(config.n_steps*2)+5, log_dir=config.log_dir, save_path=config.models_dir)
    model.learn(total_timesteps=argus.timesteps, callback=callback)
    print("Training complete.")

    #save to disk
    model.save(config.models_dir + "my_trained_model")
    print("Model saved to " + config.models_dir + "my_trained_model.")

