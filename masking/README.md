# Masking

This docker image contains a yolov8 container with a ROS wrapper. The yolov8 ros node subscribes to the `/camera/image_color` topic from the rgb camera. It then masks and crops the largest person in the frame. If there is no person detected in the image then nothing is publsihed. The masked images are published on the `/image/masked` topic. The images are not compressed.

If you want to run this image standalone follow these steps:
```
./run.bash #start the image
roslaunch yolov8_ros mask.launch
```
