import gym
import os
from gym.spaces import Box, Tuple
import numpy as np
from torchvision.models import resnet18, ResNet18_Weights
from torchvision import transforms
import torch
from PIL import Image
from torch import nn


model = resnet18(weights=ResNet18_Weights.IMAGENET1K_V1)
model = nn.Sequential(*list(model.children())[:-1])
#change shape from (1, 512, 1, 1) to (512,)
# Add a new layer to the model that will reshape the output tensor
model.add_module("reshape", nn.Flatten())

model.eval()
model.cuda()

#export model to onnx
dummy_input = torch.randn(1, 3, 224, 224, device='cuda')
torch.onnx.export(model, dummy_input, "models/weights/resnet18-avgpool.onnx", verbose=True, opset_version=8, input_names=["input"])
