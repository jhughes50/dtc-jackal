## DTC Scripts ##

This repo was created for the purpose of holding the scripts that deal with DARPAs requirements. 

### E-Stop ###
There are three scripts for the DARPA mandated xbee e-stop.
 - `xbee_reciever.py`: recieves signal and stops the jackal by killing ROS
 - `xbee_reciever_mavros_node.py`: recieves signal and changes the PX4 flight mode to `AUTO.LOITER` via mavros.
 - `xbee_sender.py`: a hacky script to send an e-stop command on the xbee radio. 
