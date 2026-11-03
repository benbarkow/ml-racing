import torch as th

from stable_baselines3 import PPO
from typing import Dict


class OnnxablePolicy(th.nn.Module):
	def __init__(self, extractor, action_net, value_net):
		super().__init__()
		self.extractor = extractor
		self.action_net = action_net
		self.value_net = value_net

	def forward(self, observation: Dict[str, th.Tensor]) -> Dict[str, th.Tensor]:
		# NOTE: You may have to process (normalize) observation in the correct
		#       way before using this. See `common.preprocessing.preprocess_obs`
		action_hidden, value_hidden = self.extractor(observation)
		return self.action_net(action_hidden), self.value_net(value_hidden)

	# def forward(self, observation):
	#     # NOTE: You may have to process (normalize) observation in the correct
	#     #       way before using this. See `common.preprocessing.preprocess_obs`
	#     action_hidden, value_hidden = self.extractor(observation)
	#     return self.action_net(action_hidden), self.value_net(value_hidden)


# Example: model = PPO("MlpPolicy", "Pendulum-v1")
model = PPO.load("models/archive/working_drive_circle.zip", device="cpu")
onnxable_model = OnnxablePolicy(
	model.policy.mlp_extractor, model.policy.action_net, model.policy.value_net
)
print(onnxable_model.eval())
print(model.observation_space["image"].shape)
print(model.observation_space)

image_obs_size = model.observation_space["image"].shape
dummy_img_input = th.randn(1, 4800)
dict_input_tensor = {"image": dummy_img_input}
th.onnx.export(
	onnxable_model,
	dummy_img_input,
	"working_drive_circle.onnx",
	opset_version=9,
	input_names=["input"],
)