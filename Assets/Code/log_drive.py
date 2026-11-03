import socket
import numpy as np
import cv2
import argparse
import signal
import time
import torch.nn as nn
import json
import h5py
import os

parser = argparse.ArgumentParser()
parser.add_argument("-o", "--out", help="Name of dataset", default="tracked.h5")
parser.add_argument("-th", "--threshold", help="Threshold for the episode count to save model", default=200, type=float)
parser.add_argument("--auto", help="Automatically save the model", action="store_true")
args = parser.parse_args()

file_name = 'drive_logs/' + args.out
# Global variables

def save_to_dataset(feature_vector, img):
	global count
	# within your data collection loop:

class Logging():
	def __init__(self) -> None:
		self.images_list = []
		self.data_list = []
		self.path_data = []
		self.timestamps = []
		self.align = []

	def connect(self):
		self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.s.connect(('127.0.0.1', 8080))
		self.images_list = []
		self.data_list = []
		self.path_data = []
		self.timestamps = []
		self.align = []

	def reset(self):
		self.images_list = []
		self.data_list = []
		self.path_data = []
		self.timestamps = []
		self.align = []

	def disconnect(self):
		self.s.close()

	def save_as_datapoint(self, feature_vector, img):
		self.images_list.append(img)
		self.data_list.append(feature_vector)
		self.timestamps.append(time.time())  # Save the current time

	def save_path_data(self, path_data):
		self.path_data = path_data

	def save_data_to_file(self, episodeCount):
		global file_name
		#if file exists, add add or increment number at the end of the file name
		if os.path.isfile(file_name):
			count = 1
			while os.path.isfile(file_name):
				file_name = file_name[:-3] + "_" + str(count) + ".h5"
				count += 1
		print("Saving data to file: ", file_name, " with ep:", episodeCount)
		with h5py.File(file_name, 'a') as hf:
			# Save the new data to the extended datasets
			hf['images'] = self.images_list
			hf['data'] = self.data_list
			hf['path_data'] = self.path_data
			hf['align'] = self.align
			hf['timestamps'] = self.timestamps

	def collect_data(self):
		delimiter_start = b'<START>'
		delimiter_end = b'<END>'
		delimiter_abort = b'<Abort>'
		BUFFER_SIZE = 4096
		incoming_buffer = b""

		while True:
			chunk = self.s.recv(BUFFER_SIZE)
			if not chunk:
				break
			incoming_buffer += chunk

			#check for abort
			if delimiter_abort in incoming_buffer:
				print("Aborting")
				#find the abort int value sent after the abort delimiter it is 4 bytes long
				abort_idx = incoming_buffer.find(delimiter_abort)
				abort_value_index = abort_idx + len(delimiter_abort)
				abort_value = int.from_bytes(incoming_buffer[abort_value_index:abort_value_index+4], byteorder='little')	
				return abort_value
		
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
				image_count = int.from_bytes(data_chunk[current_idx:current_idx+4], byteorder='little')

				current_idx += 4

				if(image_count == 0):
					print("No images found")
					continue

				for i in range(image_count):
					image_size = int.from_bytes(data_chunk[current_idx:current_idx+4], byteorder='little')
					current_idx += 4
					image_data = data_chunk[current_idx:current_idx+image_size]
					current_idx += image_size
					image = cv2.imdecode(np.frombuffer(image_data, dtype=np.uint8), cv2.IMREAD_COLOR)

					#images are in BGR format, convert to RGB
					image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

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

	def collect_path_data(self):
		print("waiting for path data")
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
				
				# Get the Vector2 data and convert it to a numpy array of floats
				vector_data = data_chunk[4:4+vector_size*8]  # 8 bytes per Vector2 (2 floats)
				vectors = np.frombuffer(vector_data, dtype=np.float32).reshape(-1, 2)  # Reshape to Nx2 array
				
				# Store vectors in path_data
				self.save_path_data([(float(row[0]), float(row[1])) for row in vectors])
				found = True

				current_idx = 4+vector_size*8

				#read two pairs of floats for two points to sync with the path
				car_pos = np.frombuffer(data_chunk[current_idx:current_idx+8], dtype=np.float32)
				current_idx += 8
				path_init_pos = np.frombuffer(data_chunk[current_idx:current_idx+8], dtype=np.float32)
				current_idx += 8
				car_direction = np.frombuffer(data_chunk[current_idx:current_idx+8], dtype=np.float32)
				current_idx += 8
				path_init_direction = np.frombuffer(data_chunk[current_idx:current_idx+8], dtype=np.float32)
				current_idx += 8

				self.align = [(float(car_pos[0]), float(car_pos[1])), (float(path_init_pos[0]), float(path_init_pos[1])),
					(float(car_direction[0]), float(car_direction[1])), (float(path_init_direction[0]), float(path_init_direction[1]))]
				
				# Remove the processed data from the buffer
				incoming_buffer = incoming_buffer[end_idx + len(delimiter_end):]
				print("Found path data")


def main():
	global count
	
	gotRun = False
	logging = Logging()
	logging.connect()
	while not gotRun:
		logging.reset()
		logging.collect_path_data()
		abort_val = logging.collect_data()
		print("Abort value: ", abort_val)
		if args.auto and type(abort_val) == int:
			if abort_val > args.threshold:
				logging.save_data_to_file(abort_val)
				continue
			else:
				print("Not saving data")
				continue
		got_input = input("Would you like to save this run? (y/n/x): ")
		if got_input == 'x':
			logging.disconnect()
			return
		if got_input == 'y':
			logging.save_data_to_file(abort_val)




if __name__ == '__main__':
	main()
