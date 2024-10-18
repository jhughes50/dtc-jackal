## Jackal Base

This how to setup the Jackal docker Image to run the DTC image.

### Usage
Start the docker image with `./run.bash` or just type in this directory, or just type `start` in the terminal, you will know you're in the docker image by the yellow color in the terminal. Start the jackal with `roslaunch jackal_launch jackal_hw.launch`, This will start the jackal software, the rgb camera, event camera and gps (make sure the gps is turned on) are started by default. You can configure what cameras/sensors are on with the following arguments `roslaunch jackal_launch jackal_hw rgb:=true/false ir:=true/false event:=true/false ouster:=true/false gps:=true/false`. 

### Initial Setup

Run `sudo bash setup.bash` to install the udev rules on the jackals base OS. There are seven udev rules, one for spinnaker, two for the boson (one for the trigger, one for the image stream), two for the event camera, one for the jackal hardware and one for the ublox gps. It will then build the docker image. Now you can run `bash run.bash` to start the container, do make sure the cameras are plugged in before starting the image so the devices can be found within the image.

### What's Inside

The device drivers are as followed:
- [Spinnaker SDK 3.2.0](https://www.flir.com/products/spinnaker-sdk/?vertical=machine+vision&segment=iis), for the Chamelion3 RGB camera.
- Modified [Boson SDK](https://github.com/jhughes50/boson-sdk), for the IR camera.
- [Metavision SDK](https://docs.prophesee.ai/stable/index.html), for the event camera.
- [FLIR ROS Driver](https://github.com/ros-drivers/flir_camera_driver), a ROS wrapper for the spinnaker camera.
- [eeyoreROS](https://github.com/jhughes50/eeyoreROS) a ROS wrapper for the boson camera.
- [Metavision Driver](https://github.com/ros-event-camera/metavision_driver) a ROS wrapper for the event camera.
- [Ublox Driver](https://github.com/KumarRobotics/ublox) a ROS driver for the RTK Facet gps module. This is configured in `config/zed_f9p.yaml`.
- [jackal](https://github.com/jhughes50/jackal) ROS drivers for the jackal hardware and teleop control.

### Other
Use `build.bash` if you need to rebuild the image without installing the udev rules. Use `join.bash` to join a running image, or simply type `join` in the trerminal. We use `dtc-jackal` as the image name. To setup a spektrum controller, bind the controller and reciever. Then plug the reciever into the jackal and run `jstest-gtk` and follow the steps to calibrate the controller. Then save the calibration config with `sudo jscal-store /dev/input/js0`.
