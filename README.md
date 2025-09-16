# UTokyo-Pipeline

# Term Translation

base2gripper = optical origin to cluster

gripper2cam = clubster to stereoscope camera

## 1. Collection

- Collect 10-20 images with the stereoscope of a checkerboard with known square size
- Recommend to use a digital screen to display a checkerboard. This way you can percisely create a checkerboard.

## 2. Split Images

- A single image capture with the stereoscope encodes both left and right images together
- Split a single image by even and odd rows and determine which are left and right images

## 3. Resizing

- After splitting an image to left and right images, the original number of rows will be cut in half
- Need to upscale or downscale left and right images to desired resolution
- Tested nearest neighbour, bilinear interpolation, bicubic interpolation, Lanczos

## 4. Calibrate Camera

- Use images collected and calibrate the camera
- Check reprojection error to determine if calibration was good

## 5. Hand-eye Calibrate

- Need to collect 10-20 images and motion capture data
- Each image contains the checkerboard, and must collect motion caputre data at the same time
- This creates a paired data set
- Must extract a 4x4 transformation from motion capute data, describing the position of the steroscope relative to motion capture world space
- Make sure to move the camera in different poses when capturing images and motion caputre data
