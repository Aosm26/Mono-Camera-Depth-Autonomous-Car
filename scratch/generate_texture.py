import numpy as np
from PIL import Image, ImageDraw, ImageFilter
import os

# Create base dark gray image
height, width = 1024, 1024
pixels = np.full((height, width, 3), 45, dtype=np.uint8)

# Add noise for texture
noise = np.random.normal(0, 10, (height, width, 3)).astype(np.int16)
pixels = np.clip(pixels.astype(np.int16) + noise, 0, 255).astype(np.uint8)

# Convert to PIL Image
image = Image.fromarray(pixels)

# Apply a tiny blur to soften the noise
image = image.filter(ImageFilter.GaussianBlur(1))

# Create drawing context
draw = ImageDraw.Draw(image)

# Colors
white = (240, 240, 240)
line_width = 16

# Left solid line
draw.line([(80, 0), (80, height)], fill=white, width=line_width)

# Right solid line
draw.line([(width - 80, 0), (width - 80, height)], fill=white, width=line_width)

# Center dashed line
# Dashes are from:
# 0 to 128, 384 to 640, 896 to 1024
dash_ranges = [(0, 128), (384, 640), (896, 1024)]
center_x = width // 2

for start_y, end_y in dash_ranges:
    draw.line([(center_x, start_y), (center_x, end_y)], fill=white, width=line_width)

# Ensure the destination directory exists
dest_dir = "/home/aosm/Mono Camera Depth Autonomuos Car/src/autonomous_car/media/materials/textures"
os.makedirs(dest_dir, exist_ok=True)
dest_path = os.path.join(dest_dir, "road_asphalt.png")
image.save(dest_path)
print(f"Generated texture successfully and saved to {dest_path}")
