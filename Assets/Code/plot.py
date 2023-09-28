import socket
import numpy as np
import cv2
import argparse
import signal
import time
from stack_model import CNN
from torchvision import transforms
import torch
import torch.nn as nn
import json

parser = argparse.ArgumentParser()
parser.add_argument("-o", "--out", help="Name of dataset", default="tracked.json")
args = parser.parse_args()

file_name = 'data/' + args.out
# Global variables

def save_to_dataset(feature_vector, img):
	global count
	# within your data collection loop:

class Plot():
	def __init__(self) -> None:
		self.plot_object = {
			"path": [],
			"datapoints": []
		}

		self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.s.connect(('127.0.0.1', 8080))

		self.model = CNN().to('cuda' if torch.cuda.is_available() else 'cpu')  # move model to GPU if available
		# Load the trained model
		checkpoint = torch.load("stack_cnn.pth")
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

	def get_model_features(self, img):
		img = self.transform(img)
		img = img.unsqueeze(0)
		img = img.to('cuda:0')
		features = self.model(img)
		return features
	
	def save_as_datapoint(self, feature_vector, img):

		predicted_features = self.get_model_features(img)
		predicted_features = [float(x) for x in predicted_features.cpu().detach().numpy().tolist()[0]]
		feature_vector = feature_vector.tolist()
		self.plot_object["datapoints"].append({
			"target_features": feature_vector[0:8],
			"cnn_features": predicted_features,
			"car_pos": (feature_vector[8], feature_vector[9]),
			"time_stamp": time.time()
		})

	def compute_mae(self):
		target_features = np.array([x["target_features"] for x in self.plot_object["datapoints"]])
		cnn_features = np.array([x["cnn_features"] for x in self.plot_object["datapoints"]])

		mae = np.mean(np.abs(target_features - cnn_features), axis=0)
		mae_scalar = np.mean(mae)
		print("MAE: ", mae_scalar)
		

	def save_data_to_file(self):
		print("Saving data to file")
		print(self.plot_object)

		global file_name
		with open(file_name, 'w') as outfile:
			json.dump(self.plot_object, outfile)


	def collect_data(self):
		delimiter_start = b'<START>'
		delimiter_end = b'<END>'
		BUFFER_SIZE = 4096
		incoming_buffer = b""

		while True:
			chunk = self.s.recv(BUFFER_SIZE)
			if not chunk:
				break
			incoming_buffer += chunk
		
			# Try to find the delimiters
			start_idx = incoming_buffer.find(delimiter_start)
			end_idx = incoming_buffer.find(delimiter_end, start_idx + len(delimiter_start))

			# If both delimiters are found, process data
			if start_idx != -1 and end_idx != -1:
				data_chunk = incoming_buffer[start_idx + len(delimiter_start):end_idx]

				vector_size = int.from_bytes(data_chunk[:4], byteorder='little')
				feature_vector_data = data_chunk[4:4+vector_size]
				feature_vector = np.frombuffer(feature_vector_data, dtype=np.float32)


				image_sets = []

				current_idx = 4+vector_size
				#read image count 
				image_count = int.from_bytes(data_chunk[current_idx:4+vector_size+4], byteorder='little')
				current_idx += 4

				rand_darken = np.random.uniform(0.5, 1.0)

				for i in range(image_count):
					image_size = int.from_bytes(data_chunk[current_idx:current_idx+4], byteorder='little')
					current_idx += 4
					image_data = data_chunk[current_idx:current_idx+image_size]
					current_idx += image_size
					image = cv2.imdecode(np.frombuffer(image_data, dtype=np.uint8), cv2.IMREAD_COLOR)
					#apply random darkening
					# cv2.imshow('image', image)
					# cv2.waitKey(1)

					#show image 
					image_sets.append(image)

				#create a single image from the image sets so shape is (60, 80, 3*image_count)
				image = np.concatenate(image_sets, axis=2)

				# image_size = int.from_bytes(data_chunk[4+vector_size:4+vector_size+4], byteorder='little')
				# image_data = data_chunk[4+vector_size+4:]

				# image = cv2.imdecode(np.frombuffer(image_data, dtype=np.uint8), cv2.IMREAD_COLOR)		

				# show image

				#save to dataset
				self.save_as_datapoint(feature_vector, image)

				# Remove the processed data from the buffer
				incoming_buffer = incoming_buffer[end_idx + len(delimiter_end):]
		self.s.close()

	def collect_path_data(self):
		global path_data
		delimiter_start = b'<START_PATH>'
		delimiter_end = b'<END_PATH>'

		BUFFER_SIZE = 4096
		incoming_buffer = b""
		found = False
		while not found:
			chunk = self.s.recv(BUFFER_SIZE)
			if not chunk:
				break
			incoming_buffer += chunk
		
			# Try to find the delimiters
			start_idx = incoming_buffer.find(delimiter_start)
			end_idx = incoming_buffer.find(delimiter_end, start_idx + len(delimiter_start))

			# If both delimiters are found, process data
			if start_idx != -1 and end_idx != -1:
				data_chunk = incoming_buffer[start_idx + len(delimiter_start):end_idx]
				# Parse the number of points
				# Parse the number of points
				vector_size = int.from_bytes(data_chunk[:4], byteorder='little')
				print(vector_size)
				
				# Get the Vector2 data and convert it to a numpy array of floats
				vector_data = data_chunk[4:4+vector_size*8]  # 8 bytes per Vector2 (2 floats)
				vectors = np.frombuffer(vector_data, dtype=np.float32).reshape(-1, 2)  # Reshape to Nx2 array
				
				# Store vectors in path_data
				self.plot_object["path"] = [(float(row[0]), float(row[1])) for row in vectors]
				found = True
				
				# Remove the processed data from the buffer
				incoming_buffer = incoming_buffer[end_idx + len(delimiter_end):]


def main():
	global count

	plot = Plot()

	plot.collect_path_data()
	try:
		plot.collect_data()
	except Exception as e:
		print(e)

	plot.save_data_to_file()
	plot.compute_mae()



if __name__ == '__main__':
	main()
