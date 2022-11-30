#!/usr/bin/python
import time, serial
import rospy
from std_msgs.msg import UInt8MultiArray

class uart_pbibub():
    def __init__(self):
        self.uart_pub = rospy.Publisher("uart_pub", Int8MultiArray, queue_size=10)
        self.serial_port = serial.Serial(
        port="/dev/uart",
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        )
            
def check_sum(mode,lw,lws,rw,rws,brk,bat,sig):
	ck=0X00
	datalist=[mode,lw,lws,rw,rws,brk,bat,sig]
	for i in range(len(datalist)):
		ck+=datalist[i]
                ck & 0xFF
	ck=~(ck)+1
	return ck & 0xFF

if __name__=="__main__":
	rospy.init_node("uart_pub")
        uart_pub = rospy.Publisher("uart_pub", UInt8MultiArray, queue_size=10)
        serial_port = serial.Serial(
                port="/dev/uart",
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                )
	datalist = []
	try:
		while not rospy.is_shutdown():
                        datalist.append(ord(serial_port.read()) & 0xFF)
                        if datalist[-1] == 10 and datalist[-2] == 13:
				chk=check_sum(datalist[-11], datalist[-10], datalist[-9],datalist[-8],datalist[-7],datalist[-6],datalist[-5],datalist[-4])
				if(datalist[-3]==chk):
                                    msg = UInt8MultiArray()
				    msg.data = datalist
                                    print(msg.data)
				    uart_pub.publish(msg)
				    datalist =[]
				    #rospy.sleep(0.01)
			    
	except KeyboardInterrupt:
		print("keyboard interrupt")

	finally:
		serial_port.close()
		pass
