import numpy as np
import matplotlib.pyplot as plt

def plot_mag_csv(filename):
    """
    Load and plot 3D magnetometer data from a CSV file.

    Parameters:
        filename (str): Path to CSV file with header: x,y,z
    """
    data = np.genfromtxt(filename, delimiter=',', skip_header=1)
    
    if data.shape[1] < 3:
        raise ValueError("CSV file must contain at least 3 columns for x, y, z")

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(data[:, 0], data[:, 1], data[:, 2], s=5, color='blue', alpha=0.6)

    ax.set_title(f'Magnetometer Data: {filename}')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.axis('equal')
    plt.tight_layout()


plot_mag_csv("mag_data/mag_data.csv")

plt.show()