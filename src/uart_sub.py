#!/usr/bin/python
import time, serial
import rospy
from std_msgs.msg import UInt8MultiArray, Int16, Bool

class uartsub():
    def __init__(self):
        self.serial_port = serial.Serial(
                        port="/dev/uart",
                        baudrate=115200,
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        )
        self.is_obs = False
        self.uart_sub = rospy.Subscriber("/uart_pub", UInt8MultiArray, self.UartCallback)
        self.obs_sub = rospy.Subscriber('/is_obs', Bool, self.obsCB)
        self.is_clicked_pub = rospy.Publisher("/is_clicked", Bool, queue_size = 1)
    
    def check_sum(self, mode,lw,lws,rw,rws,brk,chk):
        ck=0X00
        datalist=[mode,lw,lws,rw,rws,brk,chk]
        for i in range(len(datalist)):
            ck+=datalist[i]
            ck & 0xFF
        ck=~(ck)+1
        return ck & 0xFF

    def obsCB(self, msg):
        self.is_obs = msg.data

    def data_send(self, mode, chk):
        serial_port.write(chr(0x48))              #start
        serial_port.write(chr(mode))              #mode
        serial_port.write(chr(0x53))                #left,w => 0x53, 0x43, 0x57
        serial_port.write(chr(0x21))               #lw speed => 0x21 ~ 0xFF
        serial_port.write(chr(0x53))                #rw => 0x53, 0x43, 0x57
        serial_port.write(chr(0x21))               #rw speed => 0x21 ~ 0xFF
        serial_port.write(chr(0x4F))              #break
        serial_port.write(chr(chk))
        ck=self.check_sum(mode,0x53, 0x21,0x53,0x21,0x4F,chk)
        serial_port.write(chr(ck))                #checksum
        serial_port.write(chr(0x0D))              #finish
        serial_port.write(chr(0x0A))              #finish
        #print('***')
        #print('lws = ', lws)
        #print('rws = ', rws)
        #print([0x48,mode,lw,lws,rw,rws,brk,chk,ck,0x0D,0x0A])
        #print('--------------------')

    def UartCallback(self, data):
        if self.is_obs == True:
            mode = 65
        else:
            mode = 77
        if ord(data.data[8]) == 67:
            chk = 67
            is_clicked = True
            self.data_send(mode, chk)
            self.is_clicked_pub.publish(is_clicked)
        else:
            is_clicked = False
            chk = 77
            self.data_send(mode, chk)
            self.is_clicked_pub.publish(is_clicked)

if __name__=="__main__":
    rospy.init_node("uart_sub")
    serial_port = serial.Serial(
                        port="/dev/uart",
                        baudrate=115200,
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        )
    try:
        uarts = uartsub()
        rospy.spin()

    finally:
        serial_port.close()
        pass



