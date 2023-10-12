import cv2
import matplotlib.pyplot as plt
import numpy as np
import torch
import h5py
from torch.utils.data import DataLoader, Dataset
from torchvision import transforms
import argparse

from cnn_torch.model import CNN
from cnn_torch.cnn_model_2conv import CNN_2Layers
from cnn_torch.cnn_model_3conv_2 import CNN_3Layers
from cnn_torch.cnn_model_2conv_4 import CNN_2LayersDrop
from cnn_torch.cnn_model_3conv_5 import CNN_3LayersDrop

cnns = {
	0: CNN,
	1: CNN_2Layers,
	2: CNN_3Layers,
	4: CNN_2LayersDrop,
	5: CNN_3LayersDrop,
}

parser = argparse.ArgumentParser()
parser.add_argument("-d", "--data", help="Path to data file", default="data/data.h5")
parser.add_argument("-m", "--model", help="Path to model file", default="trained_model.pth")
parser.add_argument("-c", "--cnn", help="CNN to use", default="cnn_model_2conv_1.py", type=int)
args = parser.parse_args()

print(args.cnn)
			
model_save_path = args.model

# Load data.h5 file
with h5py.File(args.data, 'r') as hf:
	# Access and print dataset shapes
	images = np.array(hf['images'])
	feature_vectors = np.array(hf['feature_vectors'])

#combine the two datasets into one
data = [
	(images[i], feature_vectors[i]) for i in range(len(images))
]
#split the data into training and testing and validation
training_data = data[:int(len(data)*0.8)]
testing_data = data[int(len(data)*0.8):int(len(data)*0.9)]
validation_data = data[int(len(data)*0.9):]


class CustomDataset(Dataset):
	def __init__(self, data, transform=None):
		self.data = data
		self.transform = transform

	def __len__(self):
		return len(self.data)

	def __getitem__(self, idx):
		image, vector = self.data[idx]
		if self.transform:
			image = self.transform(image)
		return image, torch.tensor(vector)

transform = transforms.Compose([
	transforms.ToTensor(),	
	transforms.Normalize(mean=[0.2780, 0.2813, 0.2733, 0.2788, 0.2823, 0.2744, 0.2801, 0.2840, 0.276],
					 std=[0.1518, 0.1828, 0.1830, 0.1524, 0.1838, 0.1840, 0.1534, 0.1855, 0.1858])
])

training_dataset = CustomDataset(training_data, transform=transform)
training_dataloader = DataLoader(training_dataset, batch_size=1, shuffle=True)
testing_dataset = CustomDataset(testing_data, transform=transform)
testing_dataloader = DataLoader(testing_dataset, batch_size=1, shuffle=True)
validation_dataset = CustomDataset(validation_data, transform=transform)
validation_dataloader = DataLoader(validation_dataset, batch_size=1, shuffle=True)


def generate_cam(model, image, target_class):
	# Use the forward hook to capture the intermediate features
	features = []
	def hook_fn(module, input, output):
		features.append(output)
		
	# Assuming the layer name where you want to attach the hook is `features_layer`
	# This name needs to be changed based on your model's architecture
	if args.cnn == 1 or args.cnn == 4:
		hook = model.conv2.register_forward_hook(hook_fn)
	elif args.cnn == 2 or args.cnn == 5 or args.cnn == 0:
		hook = model.conv3.register_forward_hook(hook_fn)
	elif args.cnn == 3 or args.cnn == 6:
		hook = model.conv6.register_forward_hook(hook_fn)

	output = model(image)
	hook.remove()

	# Ensure we captured the features
	if len(features) == 0:
		print("Failed to capture feature maps. Check the hook placement.")
		return None
	
	feature_map = features[0]
	fc_weights = model.fc3.weight.data.to(feature_map.device)

	print("Feature map shape:", feature_map.shape)
	print("FC weights shape:", fc_weights[target_class].shape)

	cam = torch.zeros(feature_map.shape[2], feature_map.shape[3], device=feature_map.device)
	for i, w in enumerate(fc_weights[target_class]):
		cam += w * feature_map[0, i, :, :]

	cam = torch.relu(cam)
	cam = cam - cam.min()
	cam = cam / cam.max()

	print("CAM shape:", cam.shape)

	return cam

def visualize_cam_on_image(image, cam, subplot):
	# Resize the CAM to match the image size
	cam_resized = cv2.resize(cam, (image.shape[2], image.shape[1]))
	heatmap = plt.get_cmap("viridis")(cam_resized)[:, :, :3]
	heatmap = plt.get_cmap("jet")(cam_resized)[:, :, :3]

	image_np = image.cpu().numpy().transpose(1, 2, 0)
	superimposed_img = heatmap * 0.5 + image_np * 0.5
	superimposed_img = heatmap * 0.7 + image_np * 0.3

	plt.subplot(1, 3, subplot)
	plt.imshow(superimposed_img)
	plt.axis('off')

# Your existing code to load the model...
model = cnns[args.cnn]()
checkpoint = torch.load(args.model)
checkpoint = {k.replace('module.', ''): v for k, v in checkpoint.items()}
model.load_state_dict(checkpoint)
model.eval()

with torch.no_grad():
	for images, vectors in testing_dataloader:
		images = images

		# Extracting 3 RGB images from 9-channel tensor
		for i in range(3):
			# Calculate CAM for given image and target class
			# Assuming you're targeting the first class for demonstration purposes


			cam = generate_cam(model, images, 0)

			plt.figure(figsize=(15, 5))
			for j in range(images.size(0)):

				image_single = images[j, :3]  # just use the first 3 channels for visualization
				visualize_cam_on_image(image_single, cam.cpu().numpy(), j+1)
				
			plt.show()

