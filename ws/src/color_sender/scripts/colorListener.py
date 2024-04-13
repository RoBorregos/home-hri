#!/usr/bin/env python3
import socket
import rospy
from std_msgs.msg import String

UDP_IP = "192.168.31.221"  # IP address of the ESP32
UDP_PORT = 1234            # Port number
MESSAGE = ""               # Initialize message

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def callback(data):
    instruction = data.data.strip().upper()
    rospy.loginfo(data.data)
    if instruction in ["RED", "GREEN", "BLUE", "ON", "OFF", "WHITE", "RGB"]:
        MESSAGE = instruction
        sock.sendto(MESSAGE.encode(), (UDP_IP, UDP_PORT))
        print("Instruction sent successfully.")
    else:
        print("Invalid instruction. Please enter a valid instruction.")

def listener():
    rospy.init_node('colorListener', anonymous=False)
    rospy.Subscriber('/colorInstruction', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
