import numpy as np
import matplotlib.pyplot as plt

# Function to load and visualize radar data
def visualize_radar_data():
    # Load radar data from a saved NumPy array
    data = np.load(f'/home/apg/manideep/carla/out/radar/7689.925766221364data.npy', allow_pickle=True)

    # Access individual fields of the data
    x = data[:, 0]  # X coordinates
    y = data[:, 1]  # Y coordinates
    z = data[:, 2]  # Z coordinates
    intensity = data[:, 3]  # Intensity values

    # Create a scatter plot of the radar data
    plt.figure(figsize=(8, 6))
    plt.scatter(x, y, c=intensity, cmap='viridis', s=10)
    plt.colorbar(label='Intensity')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Radar Data Scatter Plot')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    # Replace {timestamp} with the actual timestamp you want to visualize
    visualize_radar_data()

