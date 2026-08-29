from PIL import Image

# 1. Configuration ( Verilog parameters)
WIDTH = 735
HEIGHT = 511
INPUT_IMAGE = "test_image.jpeg"
OUTPUT_HEX = "image_rgb.hex"

# 2. Open and resize the image
img = Image.open(INPUT_IMAGE).convert('RGB')
img = img.resize((WIDTH, HEIGHT))

# 3. Extract pixels and write to hex file
with open(OUTPUT_HEX, "w") as f:
    for y in range(HEIGHT):
        for x in range(WIDTH):
            r, g, b = img.getpixel((x, y))
            # Format as a 6-character hex string 
            f.write(f"{r:02x}{g:02x}{b:02x}\n")

print(f"Successfully created {OUTPUT_HEX} with {WIDTH * HEIGHT} pixels.")