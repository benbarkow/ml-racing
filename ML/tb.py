from tensorboard import program
import config

tb = program.TensorBoard()
tb.configure(argv=[None, '--logdir', config.tb_logs])
url = tb.launch()
print(f"Tensorflow listening on {url}")

while True:
    pass

