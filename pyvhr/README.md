# PYVHR
This module does heart rate detection through red channel amplification. It uses remote photoplethysmography(rPPG) method called `cpu_CHROM` by default. 

Credit: [@luyingz06](https://github.com/luyingz06)

### How it works
The pyvhr node listens to the `/jackal_teleop/trigger` topic, if the trigger > 0, it takes in 300 frames of masked images from rostopic `/image/masked` published by masking module. Once 300 frames of masked images are received, they are acquired to pass through the algorithm. In the following steps, ROI (skin area) is extracted in HSV color space and then passed on to BVP and BPM calculation. Finally, the heart rate number is publish on the rosropic `/heart_reate/pyvhr`.

### Running Standalone
 - Start the docker image: `./run.bash`
 - Start tmux: `tmux`
 - Start a roscore (if one is not already running elsewhere): `roscore`
 - Split and run: `rosrun pyvhr pyvhr_node.py` to start the node

### TODOS
 - GPU acceleration with pytorch.
