#!/usr/bin/env python3
import socket
import rospy
from std_msgs.msg import String

UDP_IP = "192.168.31.10"  # IP address of the ESP32
UDP_PORT = 1234            # Port number
MESSAGE = ""               # Initialize message

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def callback(data):
    instruction = data.data.strip().upper()
    
    if DEBUG:
        rospy.loginfo(data.data)
    
    if instruction in ["RED", "GREEN", "BLUE", "ON", "OFF", "WHITE", "RGB"]:
        MESSAGE = instruction
        sock.sendto(MESSAGE.encode(), (UDP_IP, UDP_PORT))
        if DEBUG:
            rospy.loginfo("Instruction sent successfully.")
    else:
        rospy.loginfo("Invalid instruction '" + instruction + "'. Please enter a valid instruction.")

def main():
    rospy.init_node('ColorListener', anonymous=True)
    
    global DEBUG
    DEBUG = rospy.get_param('~debug', False)
    
    global UDP_IP
    UDP_IP = rospy.get_param('~ip', "192.168.31.10")
    
    global UDP_PORT
    UDP_PORT = rospy.get_param('~port', 1234)
    
    rospy.loginfo("*Color Listener node initialized*")

    rospy.Subscriber('/colorInstruction', String, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
