import matplotlib.pyplot as plt
import matplotlib.patches as patches
import json
from PIL import Image

# Load the JSON data
with open('./10.json', 'r') as json_file:
    data = json.load(json_file)

# Get the image information
image_info = data['images'][0]
image_width = image_info['width']
image_height = image_info['height']
image_id = image_info['id']
image_filename = image_info['file_name']

image = Image.open(image_filename)
# Create a function to visualize bounding boxes


def visualize_bounding_boxes(image_filename, annotations):
    # Load the image using your preferred method (e.g., OpenCV, PIL).
    # Here, we assume you've loaded it into an 'image' variable.

    # Create a figure and axis for plotting
    print(image_filename)
    print("image_filename")
    fig, ax = plt.subplots(1)
    ax.imshow(image)  # Display the image.

    # Iterate through annotations and draw bounding boxes
    for annotation in annotations:
        bbox = annotation['bbox']
        x, y, width, height = bbox
        rect = patches.Rectangle(
            (x, y), width, height, linewidth=1, edgecolor='r', facecolor='none'
        )
        ax.add_patch(rect)

    # Set axis limits and display the image
    ax.set_xlim(0, image_width)
    # Invert the y-axis to match image coordinates
    ax.set_ylim(image_height, 0)
    plt.axis('off')  # Turn off axis labels and ticks
    plt.show()


# Extract annotations for the specified image_id
annotations = [ann for ann in data['annotations']
               if ann['image_id'] == image_id]

# Call the visualization function
visualize_bounding_boxes(image_filename, annotations)
