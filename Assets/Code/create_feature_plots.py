import json
import matplotlib.pyplot as plt
import numpy as np
from create_drive_plots import plot_drive_from_file

def plot_features_from_file(filename):
    # Load data from the specified file
    with open(filename, "r") as file:
        data = json.load(file)

    # Extract target features, cnn features, and timestamps
    target_features = [datapoint['target_features'] for datapoint in data['datapoints']]
    cnn_features = [datapoint['cnn_features'] for datapoint in data['datapoints']]
    timestamps = [datapoint['time_stamp'] for datapoint in data['datapoints']]

    # Feature names
    feature_names = [
        "normalCurvatures[0]",
        "normalCurvatures[1]",
        "normalCurvatures[2]",
        "normalPosition",
        "normalRotation",
        "normalAngularVelocity",
        "normalVel",
        "normalVelPerpendicular",
    ]

    # Plot each feature
    fig, axs = plt.subplots(len(feature_names), 1, figsize=(10, 20))
    for i in range(len(feature_names)):
        axs[i].plot(timestamps, [feature[i] for feature in target_features], label='Target')
        axs[i].plot(timestamps, [feature[i] for feature in cnn_features], label='CNN', linestyle='--')
        axs[i].set_title(feature_names[i])
        axs[i].legend()
        axs[i].set_ylim(0, 1)

    plt.tight_layout()
    plt.show()

# To use the function:
plot_features_from_file("data/tracked.json")
plot_drive_from_file("data/tracked.json", 4)

