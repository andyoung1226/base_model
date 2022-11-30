#!/usr/bin/python
import serial
import rospy
import threading
import sys, select, termios, tty
from std_msgs.msg import UInt8MultiArray

serial_port = serial.Serial(
    port="/dev/uart",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)
quit = False

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def check_sum(mode,lw,lws,rw,rws,brk):
    ck=0X00
    datalist=[mode,lw,lws,rw,rws,brk]
    for i in range(len(datalist)):
        ck+=datalist[i]
        ck & 0xFF
    ck=~(ck)+1
    return ck & 0xFF

def data_send(mode):
    serial_port.write(chr(0x48))	#start
    serial_port.write(chr(mode))	#mode
    serial_port.write(chr(83))	#left,w => 0x53, 0x43, 0x57
    serial_port.write(chr(33))	#lw speed => 0x21 ~ 0xFF
    serial_port.write(chr(83))	#rw => 0x53, 0x43, 0x57
    serial_port.write(chr(33))	#rw speed => 0x21 ~ 0xFF
    serial_port.write(chr(79))
    serial_port.write(chr(check_sum(mode,83,33,83,33,79)))          #checksum
    serial_port.write(chr(0x0D))	#finish
    serial_port.write(chr(0x0A))	#finish
	
def UartCallback(data):
    global quit
    key = getKey(0.1)

    if key == "a":
        print("auto mode")
        data_send(65)
    elif key == "m":
        print("manual mode")
        data_send(77)
    elif key == "q":
        print("The end")
        quit = True

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node("change_automode")
    datalist = []
    try:
        while not rospy.is_shutdown():
            if quit == True:
                break

            rospy.Subscriber("uart_pub", UInt8MultiArray, UartCallback)
            rospy.sleep(100)


    except KeyboardInterrupt:
        print("keyboard interrupt")

    finally:
        serial_port.close()
        pass



