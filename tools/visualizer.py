import sys
from PIL import Image

def create_image_from_colors(filename, imagesize):
    # Open the file
    with open(filename, 'r') as file:
        # Read lines from the file
        lines = file.readlines()

        # Initialize an empty list to store the values
        hex_values = []

        # Iterate over each line
        for line in lines:
            # Split the line by commas
            values = line.strip().split(',')

            # Convert each hex value to an integer and append to the list
            for hex_str in values:
                decimal_int = int(hex_str.strip(), 16)
                hex_values.append(decimal_int)

    width = height = int(imagesize)
    num_colors = width * height

    # Create a new image with the calculated dimensions
    image = Image.new("RGB", (width, height))

    # Set each pixel in the image to the corresponding color from the list
    for y in range(height):
        for x in range(width):
            color_index = y * width + x
            if color_index < num_colors:
                color = hex_values[color_index]
                image.putpixel((x, y), ((color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF))

    # Save or display the image
    image.show()

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python visualizer.py <filename> <imagesize>")
        sys.exit(1)

    filename = sys.argv[1]
    imagesize = sys.argv[2]
    create_image_from_colors(filename, imagesize)