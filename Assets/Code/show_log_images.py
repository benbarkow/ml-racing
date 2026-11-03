import h5py
import numpy as np
import matplotlib.pyplot as plt

def display_images(image_triple):
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    
    for i in range(3):
        ax = axes[i]
        img_slice = image_triple[:,:, i*3:(i+1)*3]
        ax.imshow(img_slice)
        ax.set_title(f"Image {i+1}")
        ax.axis('off')
    
    plt.show()

def main():
    file_name = 'drive_logs/drive_logs_german_test.h5'  # Replace with your actual file name

    with h5py.File(file_name, 'r') as hf:
        images_dataset = hf['images']
        for i in range(len(images_dataset)):
            image_triple = np.array(images_dataset[i])
            display_images(image_triple)

if __name__ == '__main__':
    main()