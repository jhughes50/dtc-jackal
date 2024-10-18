# PYVHR
This module does heart rate detection through red channel amplification. Perhaps Luying can add more about how it actually works. When a trigger is recieved it saves 10 seconds of masked images from the masking module. Once 10 seoncds are acquired the frames are passed through the algorithm. The heart rate is then published on the `heart_rate/pyvhr` topic. 

### Running Standalone
Start the docker image: `./run.bash`
Start tmux: `tmux`
Start a roscore (if one is not already running elsewhere): `roscore`
Split and run: `rosrun pyvhr pyvhr_node.py` to start the node

### TODOS
 - GPU acceleration with pytorch.
