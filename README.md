# UTokyo-Pipeline

# Term Translation

base2gripper = optical origin to cluster

gripper2cam = clubster to stereoscope camera

base2world = optical origin to checkerboard world space

world2cam = checker world space to stereoscop camera

## 1. Collection

- Collected 10-20 images with the stereoscope of a checkerboard with known square size. Hand-eye calibration can use the same image for calibration, so motion capture data was collected at the same time to create a paired dataset.
- Make sure to move the camera in different poses when capturing images and motion caputre data.
- Recommend to use a digital screen to display a checkerboard. This way you can percisely create a checkerboard.
- Square size used in this experiment was 10.0006 mm.

## 2. Split Images

- A single image capture with the stereoscope encodes both left and right images together.
- Split a single image by even and odd rows and determine which are left and right images.
- splitimages.ipynb was used to split scope images and organize mocap data.

## 3. Resizing

- After splitting an image to left and right images, the original number of rows will be cut in half.
- Need to upscale or downscale left and right images to desired resolution.
- Tested nearest neighbour (NN), bilinear interpolation (BL), bicubic interpolation (BC), Lanczos (LZ).

## 4. Calibrate Camera

- Use images collected and calibrate the camera.
- Check reprojection error to determine if calibration was good.
- Compared different calibration reprojection errors with different image sizes.
- workflow1.m in MATLAB was used to perform stereo calibration. Output was formatted to feed into OpenCV.

NN 1280x720: 0.2544 px
NN 1920x1080: 0.3014 px
BL 1280x720: 0.1898 px
BL 1920x1080: 0.3003 px
BC 1280x720: 0.1895 px
BC 1920x1080: 0.3017 px
LZ 1280x720: 0.1895 px
LZ 1920x1080: 0.3019 px

- No significant difference, so nearest neighbour at 1280x720 resolution was chosen for the rest of the experiment.

## 5. Hand-eye Calibrate

- Used the same calibration image/mocap dataset as camera calibration.
- workflow2.ipynb processes the calibration images by rectifying, and finding camera extrinsics.
- Using stereoParams_handeye2.mat, we get a mean reprojection error from solvePnP of **0.15 pixels**. This indicates the estimated camera extrinsics were excellent.
- The next section of workflow2.ipynb process the mocap data. A 4x4 transformation matrix is extracted for every pose of the cluster on the stereo camera. The transformation matrix is gripper2base.
- **Main error came from creating the transformation matrix. Important to read in Motive data as INTRISIC rotations.**
- The next section of workflow2.ipynb performs hand-eye calibration.The Shah method for hand-eye calibration was used. The average RMS reprojection error was **8.95 pixels**. This was calculated by reprojecting the checkerboard points using the estimated hand-eye transformation and camera intrinsics/extrinsics.
- Another validation step was to check the loop closure error. The average RMS rotation error was **0.005 degrees** and the average RMS translation error was **1.31 mm**.
- Attached is a visualization of the detected corners and reprojected corners using the estimated hand-eye transformation.
