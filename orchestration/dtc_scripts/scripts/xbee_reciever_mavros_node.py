"""
    Jason Hughes
    April 2024
    ROS-Node to interface with xbee radio via serial
    and change flight mode to land.
"""

import serial
import rospy
from mavros_msgs.srv import SetMode, SetModeRequest

DEV = "/dev/ttyUSB1"
BAUDRATE = 9600

NORMAL = "7e000b8801495300010002000000d7"
ESTOP  = "7e000b8801495300010002000002d5"

QUERY = "7E0004080149535A"
RESET = "7E000508014431047D"
RESET_SUCCESS = "7E0005880144310001"


def main():

    rospy.init_node("xbee_reciever")
    print("[XBEE] ROS node initialized")
    rospy.wait_for_service("/mavros/set_mode")
    mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    set_mode = SetModeRequest()
    set_mode.custom_mode = "AUTO.LAND"

    try:
        xbee_ser = serial.Serial(DEV, BAUDRATE)
        print("[XBEE] Connected to radio at %s with baud %s" %(DEV, BAUDRATE))
    except Exception as e:
        print("[XBEE] couldn't connect to radio on %s with baud %s. %s" %(DEV, BAUDRATE, e))
        exit()

    while (not rospy.is_shutdown()):
        # set to query mode
        
        res = xbee_ser.write(bytes.fromhex(QUERY))
        # read the 15 bytes
        raw_b = xbee_ser.read(15)
        if raw_b.hex() != NORMAL:
            #change flight mode to land
            print("[XBEE] Setting to AUTO.LAND mode")
            if (mode_client.call(set_mode).mode_sent == True):
                print("[XBEE] AUTO.LAND set")
            else:
                print("[XBEE] unable to change flight mode, retrying")
                continue
            exit() # REPLACE this with option to RESET            
        print(raw_b.hex())
        
    xbee_ser.close()

if __name__ == "__main__":
    main()
