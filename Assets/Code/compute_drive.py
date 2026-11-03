import socket
import numpy as np
import cv2
import argparse
import signal
import time
from stack_model import CNN
from models.cnn_model_4conv_d import CNN_4LayersDrop
from torchvision import transforms
import torch
import torch.nn as nn
import json
import h5py

parser = argparse.ArgumentParser()
parser.add_argument("-o", "--out", help="Name of dataset", default="cumpute_logs.json")
args = parser.parse_args()

file_name = "compute_logs/" + args.out
# Global variables

class Compute():
	def __init__(self, model_path, drive_logs_path) -> None:
		self.plot_object = {
			"path": [],
			"mae": None,
			"datapoints": [],
			"align": ()
		}

		self.model = CNN_4LayersDrop().to('cuda' if torch.cuda.is_available() else 'cpu')  # move model to GPU if available
		# Load the trained model
		checkpoint = torch.load(model_path)
		checkpoint = {k.replace('module.', ''): v for k, v in checkpoint.items()}
		self.model.load_state_dict(checkpoint)
		self.model.eval()

		self.transform = transforms.Compose([
			transforms.ToTensor(),	
			# transforms.Normalize(mean=[0.485, 0.456, 0.406, 0.485, 0.456, 0.406, 0.485, 0.456, 0.406],
			#                      std=[0.229, 0.224, 0.225, 0.229, 0.224, 0.225, 0.229, 0.224, 0.225])
			transforms.Normalize(mean=[0.2780, 0.2813, 0.2733, 0.2788, 0.2823, 0.2744, 0.2801, 0.2840, 0.276],
							std=[0.1518, 0.1828, 0.1830, 0.1524, 0.1838, 0.1840, 0.1534, 0.1855, 0.1858])
		])

		#load drive logs
		self.drive_logs = {
			"images": [],
			"data": [],
			"path_data": [],
			"align": (),
			"timestamps": []
		}
		with h5py.File(drive_logs_path, 'r') as hf:
			self.drive_logs["images"] = hf["images"][:]
			self.drive_logs["data"] = hf["data"][:]
			self.drive_logs["path_data"] = hf["path_data"][:]
			self.drive_logs["align"] = hf["align"][:]
			self.drive_logs["timestamps"] = hf["timestamps"][:]

	def get_model_features(self, img):
		img = self.transform(img)
		img = img.unsqueeze(0)
		img = img.to('cuda:0')
		features = self.model(img)
		return features
	
	def save_as_datapoint(self, feature_vector, img, time_stamp):

		predicted_features = self.get_model_features(img)
		predicted_features = [float(x) for x in predicted_features.cpu().detach().numpy().tolist()[0]]
		feature_vector = feature_vector.tolist()
		self.plot_object["datapoints"].append({
			"target_features": feature_vector[0:8],
			"cnn_features": predicted_features,
			"car_pos": (feature_vector[8], feature_vector[9]),
			"car_direction": (feature_vector[10], feature_vector[11]),
			"actions": (feature_vector[12], feature_vector[13]),
			"time_stamp": time_stamp,
		})

	def compute_mae(self):
		target_features = np.array([x["target_features"] for x in self.plot_object["datapoints"]])
		cnn_features = np.array([x["cnn_features"] for x in self.plot_object["datapoints"]])

		mae = np.mean(np.abs(target_features - cnn_features), axis=0)
		mae_scalar = np.mean(mae)
		return mae_scalar

	def save_data_to_file(self):
		print("Saving data to file")

		global file_name
		with open(file_name, 'w') as outfile:
			json.dump(self.plot_object, outfile)

	def compute_data(self):
		print(self.drive_logs.keys())
		self.plot_object["path"] = self.drive_logs["path_data"].tolist()
		position_align = self.drive_logs["align"][0:2].tolist()
		direction_align = self.drive_logs["align"][2:].tolist()
		self.plot_object["align"] = {
			"position": position_align,
			"direction": direction_align
		}

		for i in range(len(self.drive_logs["images"])):
			img = self.drive_logs["images"][i]
			feature_vector = self.drive_logs["data"][i]
			time_stamp = self.drive_logs["timestamps"][i]
			self.save_as_datapoint(feature_vector, img, time_stamp)

		mae = self.compute_mae()
		self.plot_object["mae"] = mae
		print("MAE: ", mae)

def main():
	global count

	compute = Compute("models/cnn_green_4conv_d.pth", "drive_logs/almost5000.h5")

	compute.compute_data()

	compute.save_data_to_file()

if __name__ == '__main__':
	main()
