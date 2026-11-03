import json
import matplotlib.pyplot as plt
import numpy as np
from create_drive_plots import plot_drive
import argparse

parser = argparse.ArgumentParser(description='Plot features from a drive log.')
parser.add_argument('-i', '--input', help='Input file name', required=True, type=str)
args = parser.parse_args()


def plot_features_from_file(filename):
    # Load data from the specified file
    with open(filename, "r") as file:
        data = json.load(file)

    # Extract target features, cnn features, and timestamps
    target_features = [datapoint['target_features'] for datapoint in data['datapoints']]
    cnn_features = [datapoint['cnn_features'] for datapoint in data['datapoints']]
    # Original timestamps
    original_timestamps = [datapoint['time_stamp'] for datapoint in data['datapoints']]
    # Find the first timestamp
    first_timestamp = original_timestamps[0] if original_timestamps else 0
    # Adjust timestamps to start at 0
    timestamps = [timestamp - first_timestamp for timestamp in original_timestamps]

    # Feature names
    feature_names = [
        r"$c_1$",
        r"$c_2$",
        r"$c_3$",
        r"$o$",
        r"$\theta$",
        r"$\omega$",
        r"$v_x$",
        r"$v_y$",
    ]


    # Plot each feature
    fig, axs = plt.subplots(len(feature_names), 1, figsize=(10, 20))
    for i in range(len(feature_names)):
        cnn_features_i = [feature[i] for feature in cnn_features]
        if feature_names[i] == r"$\theta$":
             cnn_features_i = [1-feature[i] for feature in cnn_features]
        axs[i].plot(timestamps, [feature[i] for feature in target_features], label='Target')
        axs[i].plot(timestamps, cnn_features_i, label='CNN', linestyle='--')
        axs[i].set_title(feature_names[i])
        axs[i].legend()
        axs[i].set_ylim(0, 1)

    #space between subplots
    plt.tight_layout()
    plt.show()

# To use the function:
plot_features_from_file("compute_logs/" + args.input + ".json")
plot_drive("compute_logs/" + args.input + ".json")

