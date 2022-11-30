#!/usr/bin/python
import serial
import rospy
import threading
import sys, select, termios, tty
from std_msgs.msg import UInt8MultiArray

auto = True
cmd = [65,83,33,83,33]

serial_port = serial.Serial(
    port="/dev/uart",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)

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

def data_send():
    serial_port.write(chr(0x48))	#start
    serial_port.write(chr(cmd[0]))	#mode
    serial_port.write(chr(cmd[1]))	#left,w => 0x53, 0x43, 0x57
    serial_port.write(chr(cmd[2]))	#lw speed => 0x21 ~ 0xFF
    serial_port.write(chr(cmd[3]))	#rw => 0x53, 0x43, 0x57
    serial_port.write(chr(cmd[4]))	#rw speed => 0x21 ~ 0xFF
    serial_port.write(chr(79))
    serial_port.write(chr(check_sum(cmd[0],cmd[1],cmd[2],cmd[3],cmd[4],79)))          #checksum
    serial_port.write(chr(0x0D))	#finish
    serial_port.write(chr(0x0A))	#finish
	
def UartCallback(data):
    global cmd, auto
    if ord(data.data[1]) == 77:
        #cmd = [ord(data.data[1]),83,33,83,33]
        auto = False

    key = getKey(0.1)

    if key == "a":
        auto = True
    elif key == "m":
        auto = False

    if auto == True:
        if key == "l":
            print("left_rotation")
            cmd = [65,87,40,67,40]
        elif key == "r":
            print("right_rotation")
            cmd = [65,67,40,87,40]
        elif key == "s":
            print("stop")
            cmd = [65,83,33,83,33]

        print(cmd)
        data_send()

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node("rotation_mode")
    datalist = []
    try:
        while not rospy.is_shutdown():
            rospy.Subscriber("uart_pub", UInt8MultiArray, UartCallback)
            rospy.sleep(1)

    except KeyboardInterrupt:
        print("keyboard interrupt")

    finally:
        serial_port.close()
        pass



