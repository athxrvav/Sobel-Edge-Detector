


# Sobel Edge Detector (Verilog Hardware Implementation)

## Project Overview

This project implements a Sobel Edge Detection algorithm natively in Verilog. Edge detection is a fundamental computer vision concept used in object detection, autonomous cars, and medical imaging.



In software (like Python or C++), image processing is straightforward because memory is cheap and floating-point math is handled by the CPU. In hardware design (ASIC/FPGA), we must manually manage memory addressing, avoid floating-point arithmetic, size our registers down to the exact bit, and optimize multipliers to save silicon area and meet timing requirements.

### The Hardware Pipeline:

**RGB Hex Image** -> **Load Memory** -> **Grayscale Conversion** -> **3x3 Kernel Convolution** -> **Magnitude Thresholding** -> **Binary Edge Map Output**

---

## How to Run the Simulation

To test this hardware logic, we use Python to bridge the gap between standard image files and Verilog memory files.

### Prerequisites

* **Icarus Verilog** (or any standard Verilog simulator)
* **Python 3**
* **Pillow** (Python imaging library): Install via `pip install Pillow`

### Step 1: Configure Your Dimensions

Ensure your image dimensions match across all files. Open the following files and set your `WIDTH` and `HEIGHT` parameters (e.g., 735x511):

1. The Verilog testbench (`tb.v`)
2. The image conversion script (`imgtohex.py`)
3. The binary output script (`out_bin_to_img.py`)

### Step 2: Convert the Input Image to Hex

Place your test image (e.g., `test_image.jpeg`) in the project folder. Run the Python script to extract the RGB data and format it into a Verilog-readable hex file.

```bash
python3 imgtohex.py

```

*This generates `image_rgb.hex`.*

### Step 3: Run the RTL Simulation

Compile and run the Verilog files using Icarus Verilog. Depending on your image size, the pixel-by-pixel hardware simulation may take a few minutes.

```bash
iverilog -o sim_sobel tb.v main_design.v
vvp sim_sobel

```

*This runs the state machine and generates `edge_output.bin`.*

### Step 4: Convert the Binary Output Back to an Image

Parse the 1-bit memory output from the simulator and reconstruct it into a standard grayscale image file.

```bash
python3 out_bin_to_img.py

```

*This generates `edge_result.png`, revealing the hardware-accelerated edge detection.*

---

## Bit-Width Engineering (Why are the registers sized this way?)

In hardware, every single bit costs silicon area and power. We cannot just use standard 32-bit `int` variables like in C++. We must calculate the maximum possible theoretical value for every operation and size the registers exactly.

### 1. Why is gray_full 18 bits ([17:0])?

During the Grayscale conversion, we avoid floating-point math by multiplying the RGB channels by scaled integer weights:
`gray_full = (306 * R) + (601 * G) + (117 * B)`

* The maximum value for an 8-bit RGB color channel is 255.
* If a pixel is pure white (R=255, G=255, B=255), the math evaluates to:
`(306 * 255) + (601 * 255) + (117 * 255) = 78,030 + 153,255 + 29,835 = 261,120`
* To store the number 261,120 in binary, we need to find the nearest power of 2.
2^17 = 131,072 (too small)
2^18 = 262,144 (fits perfectly!)
* Therefore, `gray_full` requires exactly 18 bits.

### 2. Why are the Pixel Registers P00 to P22 9 bits ([8:0])?

The grayscale memory (`gray_mem`) outputs 8-bit values (0 to 255). However, we immediately pad them with a 0 at the Most Significant Bit (MSB) to make them 9-bit positive signed integers (e.g., `{1'b0, pixel_value}`).
We do this because in the next step, we are going to subtract them. By making them 9-bit signed numbers upfront, we prevent binary underflow/overflow errors when the hardware performs subtraction.

### 3. Why are the Gradients Gx and Gy 11 bits ([10:0])?

The Sobel gradient calculates the difference between pixels.
`Gx = (Right_Pixels) - (Left_Pixels)`

* **Maximum Positive Value:** The right pixels are all maximum white (255) and left pixels are all pure black (0).
`Gx_max = (255 + (255 * 2) + 255) - (0 + 0 + 0) = 1020`
* **Maximum Negative Value:** The right pixels are all black (0) and left pixels are all white (255).
`Gx_min = (0 + 0 + 0) - (255 + (255 * 2) + 255) = -1020`
* Our theoretical range is -1020 to +1020.
* To represent this range in Two's Complement (signed binary), we need 10 bits for the number (2^10 = 1024) plus 1 bit for the sign. Total = 11 bits.

### 4. Why is magnitude 12 bits ([11:0])?

`magnitude = abs_Gx + abs_Gy`
The maximum absolute value for `Gx` is 1020, and for `Gy` is 1020.
Maximum Magnitude = 1020 + 1020 = 2040.
An 11-bit unsigned integer holds up to 2047, which would technically fit 2040. However, standard RTL design dictates that adding two N-bit numbers yields an (N+1)-bit result. Making it 12 bits provides a safe overflow margin for the synthesizer.

---

## Deep Dive 1: The load_window Task

In Verilog, a `task` is used to group reusable blocks of combinatorial or sequential logic. Here, the `load_window` task is responsible for fetching a 3x3 grid of pixels around our current pixel so we can perform mathematical convolution.

### 1D Array Addressing for a 2D Image

Hardware memory is physically a 1D linear array. We cannot ask for `gray_mem[row][col]`. We must mathematically calculate the 1D index using this formula:
**Address = (Row * Width) + Column**

If our image width is 8, and we want the pixel at Row 2, Column 3:
`Address = (2 * 8) + 3 = 19`

### Handling Image Boundaries (Zero-Padding)

When calculating the 3x3 window for a pixel on the absolute edge of an image (e.g., Row 0, Column 0), looking "left" or "up" will result in a negative address, crashing the system. We solve this using Zero-Padding via ternary operators `(condition) ? true : false`.

```verilog
// Top-Left Pixel (P00) Extraction
P00 = (c_row > 0 && c_col > 0) ? {1'b0, gray_mem[(c_row-1)*IMG_WIDTH + (c_col-1)]} : 9'd0;

```

* **The Condition (c_row > 0 && c_col > 0):** Checks if we have a valid row above us and a valid column to our left.
* **If True:** Calculates the address of the top-left pixel.
* **If False:** Assigns `9'd0` (a 9-bit zero). This safely pads the image borders with black pixels.

---

## Deep Dive 2: The SOBEL State (Convolution & Math)

Once the 3x3 window is loaded into registers `P00` through `P22`, the state machine transitions to computing the edge gradient.

### 1. The Sobel Kernels

The algorithm uses two distinct 3x3 matrices to find edges in the X (vertical edges) and Y (horizontal edges) directions.

**Horizontal Kernel (Gx):**

```text
[-1  0 +1]
[-2  0 +2]
[-1  0 +1]

```

### 2. Hardware Convolution (Shift vs. Multiply)

To apply the Gx kernel, we multiply the kernel values by our 3x3 pixel window.
`Gx = (P02 + 2*P12 + P22) - (P00 + 2*P10 + P20)`

**The Hardware Trick:** Multipliers consume massive logic area. Multiplying a binary number by 2 is the exact same as shifting the bits left by 1 position.

```verilog
// Notice the << 1 which replaces multiplication by 2
Gx = (P02 + (P12 << 1) + P22) - (P00 + (P10 << 1) + P20);

```

### 3. Absolute Values (Two's Complement)

The gradients are signed integers, meaning they can be negative. We need their absolute values. If the MSB (`Gx[10]`) is `1`, the number is negative. We invert the bits and add 1 to get the positive absolute value.

```verilog
abs_Gx = (Gx[10]) ? (~Gx + 1) : Gx;

```

### 4. Magnitude and Thresholding

Standard software calculates total magnitude using Euclidean distance: `sqrt(Gx^2 + Gy^2)`.

**The Hardware Trick:** Square roots and continuous squaring are catastrophic for hardware timing. Instead, we use the Manhattan Distance approximation, which is extremely cheap in hardware (just one adder) and provides near-identical thresholding accuracy:
`Magnitude = |Gx| + |Gy|`

Finally, we compare this magnitude against a `THRESHOLD`. If the gradient is steep enough (magnitude > 30), we record a 1 (edge). Otherwise, we record a 0 (background).

```verilog
edge_mem[row_cnt * IMG_WIDTH + col_cnt] = (magnitude > THRESHOLD) ? 1'b1 : 1'b0;

```

---

## Summary of Hardware Optimizations

* **Division by Bit-Slicing:** Divided by 1024 implicitly by slicing bits `[17:10]`.
* **Multiplication by Shifting:** Substituted multiply-by-2 operations with `<< 1` logical shifts.
* **Manhattan over Euclidean:** Replaced square-root magnitude with absolute sum to save DSP slices.
* **Exact Bit-Sizing:** Reduced routing congestion by bounding registers to their theoretical mathematical limits (18-bit, 11-bit, 9-bit).
