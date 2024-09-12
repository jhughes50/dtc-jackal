Steps:
git clone https://github.com/acconeer/acconeer-python-exploration.git
python -m pip install --upgrade acconeer-exptool[app]
python -m acconeer.exptool.setup
#choose option 0 Linux for Ubuntu 20
#REBOOT THE MACHINE to get the uart permissions to take effect
# I had to do the following to get the acconeer-python-explorer to work
sudo apt-get install libxcb-cursor0
# Verify that you can use the demo explorer and connect to the sensor
python -m acconeer.exptool.app
# Optional verification of command line based usage
cd acconeer-python-exploration/examples/a121/
python basic.py --serial-port /dev/ttyUSB0
cd acconeer-python-exploration/examples/a121/algo/breathing
python breathing_with_gui.py --serial-port /dev/ttyUSB0
# I had to make sure that the ROS python path contained the acconeer module
$ echo $PYTHONPATH
    /opt/ros/noetic/lib/python3/dist-packages
$ export PYTHONPATH=$(python -c 'import site; print(site.getsitepackages()[0])'):$PYTHONPATH
$ echo $PYTHONPATH
    /home/ros/miniconda3/lib/python3.12/site-packages:/opt/ros/noetic/lib/python3/dist-packages
##############################
#      TO INSTALL FOR ROS
##############################
# Figure out which executable python ROS is using. Mine was /usr/bin/python3
$ rosrun respiration_acconeer_radar_sensor.py check_python_executable.py
    /usr/bin/python3
    
# Install acconeer-exptool without the [app]
$ /usr/bin/python3 -m pip install --upgrade acconeer-exptool
$ /usr/bin/python3 -m pip install --upgrade scipy
# Verify that it can find the acconeer package
$ /usr/bin/python3 -c "import acconeer.exptool; print('Acconeer import successful')"
    Acconeer import successful
# run catkin_make
# add the package to PYTHONPATH
export PYTHONPATH=$PYTHONPATH:~/catkin_ws/devel/lib/python3/dist-packages
#Run with the following command; might have to update USB#
$ rosrun acconeer_radar_sensor respiration_radar_sensor.py --serial-port /dev/ttyUSB0
# For the visualization
# Install plotting tools
$ /usr/bin/python3 -m pip install pyqtgraph PySide6
