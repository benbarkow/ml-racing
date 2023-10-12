import h5py
import torch
import cv2
import matplotlib.pyplot as plt
from models.cnn_model_4conv_d import CNN_4LayersDrop
import numpy as np
from torchvision import transforms

def generate_cam(model, image, target_class):
	# Use the forward hook to capture the intermediate features
	features = []
	def hook_fn(module, input, output):
		features.append(output)
		
	# Assuming the layer name where you want to attach the hook is `features_layer`
	# This name needs to be changed based on your model's architecture
	hook = model.conv4.register_forward_hook(hook_fn)

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

	return superimposed_img


def main(model_path, h5_file_path):
	# Load model
	model = CNN_4LayersDrop()  # Replace with your actual model class
	checkpoint = torch.load(model_path)
	checkpoint = {k.replace('module.', ''): v for k, v in checkpoint.items()}
	model.load_state_dict(checkpoint)
	model.eval()

	# Load the h5 file
	with h5py.File(h5_file_path, 'r') as hf:
		images_data = hf["images"][:]
		vector_data = hf["data"][:]  # Assuming this is where the vector information is stored

	# Set up transformations for images
	transform = transforms.Compose([
		transforms.ToTensor(),
		# Add any additional transformations if needed
	])

	# Loop through the images and generate CAMs
	for i, (img_data, vector) in enumerate(zip(images_data, vector_data)):
		if(i % 10 != 0):
			continue
		# Transform image data to tensor
		img = transform(img_data)
		img = img.unsqueeze(0)  # Add batch dimension

		# Assume target_class_index is the position in the vector where the target class/feature index is stored
		target_class_index = 0  # This needs to be changed based on your actual data structure
		target_class = int(vector[target_class_index])  # Convert to int in case it's stored as float

		# Generate CAM
		cam = generate_cam(model, img, target_class)

		# Display CAM on every image
		img1 = img[0,0:3]
		img2 = img[0,3:6]
		img3 = img[0,6:9]

		# Visualize CAM on image
		superimposed_img1 = visualize_cam_on_image(img1, cam.cpu().detach().numpy(), 1)
		superimposed_img2 = visualize_cam_on_image(img2, cam.cpu().detach().numpy(), 2)
		superimposed_img3 = visualize_cam_on_image(img3, cam.cpu().detach().numpy(), 3)

		# Display the images
		plt.subplot(1, 3, 1)
		plt.imshow(superimposed_img1)
		plt.axis('off')
		plt.subplot(1, 3, 2)
		plt.imshow(superimposed_img2)
		plt.axis('off')
		plt.subplot(1, 3, 3)
		plt.imshow(superimposed_img3)
		plt.axis('off')

		plt.show()

if __name__ == '__main__':
	data_path = "drive_logs/" + "drive_logs_german_test.h5"
	model_path = "models/" + "cnn_green_4conv_d.pth"
	main(model_path, data_path)