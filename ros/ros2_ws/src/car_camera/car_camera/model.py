import torch
import torch.nn as nn
import torch.nn.functional as F

class ResidualBlock(nn.Module):
	def __init__(self, in_channels, out_channels, stride=1):
		super(ResidualBlock, self).__init__()
		self.conv1 = nn.Conv2d(in_channels, out_channels, kernel_size=3, stride=stride, padding=1, bias=False)
		self.bn1 = nn.BatchNorm2d(out_channels)
		self.conv2 = nn.Conv2d(out_channels, out_channels, kernel_size=3, stride=1, padding=1, bias=False)
		self.bn2 = nn.BatchNorm2d(out_channels)
		
		self.shortcut = nn.Sequential()
		if stride != 1 or in_channels != out_channels:
			self.shortcut = nn.Sequential(
				nn.Conv2d(in_channels, out_channels, kernel_size=1, stride=stride, bias=False),
				nn.BatchNorm2d(out_channels)
			)
			
	def forward(self, x):
		out = F.relu(self.bn1(self.conv1(x)))
		out = self.bn2(self.conv2(out))
		out += self.shortcut(x)
		out = F.relu(out)
		return out

class CNN(nn.Module):
	def __init__(self):
		super(CNN, self).__init__()
		
		# Initial convolution
		self.conv1 = nn.Conv2d(9, 32, kernel_size=3, stride=1, padding=1)
		self.bn1 = nn.BatchNorm2d(32)
		
		# Residual blocks
		self.res1 = ResidualBlock(32, 64, stride=1)
		self.res2 = ResidualBlock(64, 128, stride=2)
		
		# Fully connected layers
		self.fc1 = nn.Linear(8960, 256)  # Adjusted for 80x60 images after pooling operations
		self.fc2 = nn.Linear(256, 128)
		self.fc3 = nn.Linear(128, 8)
		
	def forward(self, x):
		x = F.relu(self.bn1(self.conv1(x)))
		x = F.max_pool2d(x, 2)
		x = self.res1(x)
		x = self.res2(x)
		x = F.max_pool2d(x, 2)
		x = x.view(x.size(0), -1)
		x = F.relu(self.fc1(x))
		x = F.relu(self.fc2(x))
		x = torch.sigmoid(self.fc3(x))  # Reintroduced sigmoid activation for the output
		return x

