"""
    Jason Hughes
    April 2024
    Code to interface with xbee radio vis serial
"""

import serial
import psutil
import rospy
import subprocess


DEV = "/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DN0403T1-if00-port0"
BAUDRATE = 9600

NORMAL = "7e000b8801495300010002000000d7"
ESTOP  = "7e000b8801495300010002000002d5"

QUERY = "7E0004080149535A"
RESET = "7E000508014431047D"
RESET_SUCCESS = "7E0005880144310001"


def main():

    try:
        xbee_ser = serial.Serial(DEV, BAUDRATE)
        print("[XBEE] Connected to radio at %s with baud %s" %(DEV, BAUDRATE))
    except Exception as e:
        print("[XBEE] couldn't connect to radio on %s with baud %s. %s" %(DEV, BAUDRATE, e))
        exit()

    while True:
        # set to query mode
        res = xbee_ser.write(bytes.fromhex(QUERY))
        # read the 15 bytes
        raw_b = xbee_ser.read(15)
        if raw_b.hex() != NORMAL:
            #kill the process
            subprocess.run(["rosnode", "kill", "/jackal_node"]) 
            #for proc in psutil.process_iter():
            #    for c in proc.cmdline():
            #        if "jackal_node" in c:
            #            proc.kill()
            print("[XBEE] STOPPING")
            exit() # REPLACE this with option to RESET            
        #print(raw_b.hex())
        
    xbee_ser.close()

if __name__ == "__main__":
    rospy.init_node("xbee_reciever")
    main()
