


# training parameters
n_steps = 1024 # number of steps to run environment for before updating NN params
lr = 0.0002 # learning rate
gamma = 0.9 # discount factor
policy_kwargs = policy_kwargs=dict(net_arch=[256, 256]) # network architecture (hidden layers and their sizes)
log_dir = "logs/reward-logs/"
tb_logs = "logs/tb-logs/" #tensorboard logs
models_dir = "models/"