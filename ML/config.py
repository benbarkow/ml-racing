


# training parameters
n_steps = 512 # number of steps to run environment for before updating NN params
lr = 0.0001 # learning rate
gamma = 0.999 # discount factor
policy_kwargs = policy_kwargs=dict(net_arch=[256, 256, 256, 128]) # network architecture (hidden layers and their sizes)
log_dir = "logs/reward-logs/"
tb_logs = "logs/tb-logs/" #tensorboard logs
models_dir = "models/"