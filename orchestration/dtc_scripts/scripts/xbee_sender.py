"""
    Jason Hughes
    April 2024
    Send ESTOP command via xbee radio
"""

import serial

DEV = "/dev/ttyUSB0"
BAUDRATE = 9600

#TRANSMIT_HIGH = "7E/0014/10/00/000000000000FFFF/FFFE/00/00/DATA/FF"
# Tranmit a random packet and hope it changes something.
TRANSMIT_HIGH = "7E00171000000000000000FFFFFFFE010042726F61646361737460"

def main():
    
    try:
        xbee_ser = serial.Serial(DEV, BAUDRATE)
        print("[XBEE] Connected to xbee sender")
    except Exception as e:
        print("[XBEE] could not connect to radio at %s with baud %s" %(DEV, BAUDRATE))
        exit()

    i = 0
    while True:
        xbee_ser.write(bytes.fromhex(TRANSMIT_HIGH))
        if i == 100:
            break
        i += 1
    print("[XBEE] Sent 100 msgs")
    xbee_ser.close()


if __name__ == "__main__":
    main()
