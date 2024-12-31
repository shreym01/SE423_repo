# https://wiki.python.org/moin/UdpCommunication

import socket
import struct
import time

# UDP_IP = "10.192.48.94" #need to be the ip address of current device
UDP_IP = "192.168.0.99" #need to be the ip address of current device
UDP_PORT = 3500 #random number

# send_to = "172.16.115.229" #ip address of pi
send_to = "192.168.0.71" #ip address of pi
send_to_port= 3500 
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
    MESSAGE = struct.pack('fffff', 0.15,0.26,0.38,0.44,0.52) # MESSAGE contains of the reversed order of the receiving float
    sock.sendto((MESSAGE), (send_to,send_to_port)) #send MESSAGE to pi
    print('done sending')
    time.sleep(0.01)