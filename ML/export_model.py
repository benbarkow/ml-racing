import torch as th
import argparse
import config

from stable_baselines3 import PPO

parser = argparse.ArgumentParser(description='Export a model to ONNX.')
parser.add_argument('-m', '--model', type=str, default="best_model.zip", help='Executable to train on')
parser.add_argument('-o', '--output', type=str, default="new_onnx_export.onnx", help='Model to use')
argus = parser.parse_args()

class OnnxablePolicy(th.nn.Module):
    def __init__(self, extractor, action_net, value_net):
        super().__init__()
        self.extractor = extractor
        self.action_net = action_net
        self.value_net = value_net

    def forward(self, observation):
        # NOTE: You may have to process (normalize) observation in the correct
        #       way before using this. See `common.preprocessing.preprocess_obs`
        action_hidden, value_hidden = self.extractor(observation)
        return self.action_net(action_hidden), self.value_net(value_hidden)


# Example: model = PPO("MlpPolicy", "Pendulum-v1")
model = PPO.load(config.models_dir + argus.model, device="cpu")
onnxable_model = OnnxablePolicy(
    model.policy.mlp_extractor, model.policy.action_net, model.policy.value_net
)

observation_size = model.observation_space.shape
print(observation_size)
dummy_input = th.randn(1, *observation_size)
th.onnx.export(
    onnxable_model,
    dummy_input,
    argus.output,
    opset_version=9,
    input_names=["input"],
)