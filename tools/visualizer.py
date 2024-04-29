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
                color = rgb565_to_rgb888(hex_values[color_index])
                image.putpixel((x, y), ((color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF))

    # Save or display the image
    image.show()

def rgb565_to_rgb888(rgb565):
    # Extract individual color components from RGB565
    red = (rgb565 >> 11) & 0x1F
    green = (rgb565 >> 5) & 0x3F
    blue = rgb565 & 0x1F

    # Scale color components to fit into 8-bit range (0-255)
    red = (red * 255) // 31
    green = (green * 255) // 63
    blue = (blue * 255) // 31

    # Combine color components into RGB888 format
    rgb888 = (red << 16) | (green << 8) | blue
    return rgb888

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python visualizer.py <filename> <imagesize>")
        sys.exit(1)

    filename = sys.argv[1]
    imagesize = sys.argv[2]
    create_image_from_colors(filename, imagesize)