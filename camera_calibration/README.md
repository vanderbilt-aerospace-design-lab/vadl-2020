# Camera Calibration

Camera calibration is an essential process that must be completed before any image processing is done. Understanding your camera's intrinsic parameters is how you map an image to any meaningful output. Camera calibration consists of two steps: 

1) Collect a good set of calibration images.
2) Obtain K and D parameters by calibrating the camera.

## Calibration Images

The most common type of calibration images are of a black and white chessboard. This is a great object since the black and white borders are easy to detect with image processing and the checkers are of a known length. A set of ~30 images must be taken using your camera of the chessboard from different angles and distances. It is important to take time on this step to ensure good pictures are taken: have sufficient lighting, make sure the chessboard is completely flat, and take images with the checkerboard in all sections of the camera. Here is a [great guide](https://pgaleone.eu/computer-vision/2018/03/04/camera-calibration-guidelines/) on camera calibration.

## What are the K and D parameters?

K is the camera matrix and D are the distortion coefficients. These parameters are used to undistort the image, essentially *making straight lines in the real world look straight in the 2D image.* There are plenty more details on this if you’re interested, but you don’t *need* to know much more than that. Here are some resources from [OpenCV](https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html) and [Medium](https://medium.com/analytics-vidhya/camera-calibration-with-opencv-f324679c6eb7).

## Pinhole Camera Calibration

Pinhole models can be used for most normal cameras, such as the [Raspberry PiCam](https://www.raspberrypi.org/products/camera-module-v2/). To obtain K and D paremeters, run [calibrate_pinhole.py](/camera_calibration/calibrate_pinhole.py) after you've obtained your calibration images. You can use [capture_image.py](/camera_calibration/capture_image.py) to take these images. This is a simple script that captures images on the Pi with the press of a key. 


## Fisheye Camera Calibration

Fisheye camera calibration requires a different method than pinhole due to the significantly larger distortion around the camera corners. Images should be collected the same way, with extra care taken to covering the entire field of view and getting primarily very distorted images in your database. 

Run [calibrate_fisheye.py](/camera_calibration/calibrate_fisheye.py) to obtain K and D parameters for your fisheye camera. Then, verify that it works by running [undistort.py](/camera_calibration/undistort.py). You should see the curved chessboard become straight lines. 

A major problem with undistorting fisheye images is that a lot of the image is *cropped out.* This is unfortunately just a limitation of fisheye cameras. If you want to see how much is being cropped out, run [undistort_balance.py](/camera_calibration/undistort_balance.py) and mess around with the “balance” param until you get satisfactory images. The higher the balance, the greater the FOV shown, but there is also a lot more distortion.

