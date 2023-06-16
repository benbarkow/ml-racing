import config
from tensorboard import program

tb = program.TensorBoard()
tb.configure(argv=[None, '--logdir', config.tb_logs])
url = tb.launch()
print(f"Tensorflow listening on {url}")

# keep the program running
while True:
    pass
