#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
import pyttsx3
from gtts import gTTS
from time import sleep
import socket
from WavUtils import WavUtils
import os

from frida_hri_interfaces.srv import Speak

#SPEAK_TOPIC = "/robot_text"
SPEAK_TOPIC = "/speech/speak"

# See TestSpeaker.py for more information about speaker device selection
OUTPUT_DEVICE_INDEX = os.getenv("OUTPUT_DEVICE_INDEX", default=None)

if OUTPUT_DEVICE_INDEX is not None:
    OUTPUT_DEVICE_INDEX = int(OUTPUT_DEVICE_INDEX)

class Say(object):
    DEBUG = True

    def __init__(self):
        self.engine = pyttsx3.init()
        self.engine.setProperty('voice', 'english_rp+f3')
        self.connected = self.is_connected()

        rospy.Service(SPEAK_TOPIC, Speak, self.speak_handler)
        #self.text_suscriber = rospy.Subscriber("/robot_text", String, self.callback)
        self.hear_publisher = rospy.Publisher("saying", Bool, queue_size=20)
    
    @staticmethod
    def is_connected():
        '''
        Try to connect the fastest possible to a stablished server to see if
        there is internet connection. It connects to one of the Google's 
        dns servers (port 53) (https://developers.google.com/speed/public-dns/docs/using),
        this to avoid timeouts in DNS servers via a hostname. 
        https://stackoverflow.com/a/33117579
        '''
        try:
            # Connect to the host -- tells us if the host is actually reachable.
            socket.setdefaulttimeout(0.80)
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect(("8.8.8.8", 53))
            sock.shutdown(socket.SHUT_RDWR)
            sock.close()
            return True
        except socket.error:
            pass
        return False
    
    """
    New implementation of speak as a service
    """
    def speak_handler(self, req):
        self.debug("I will say: " + req.text)
        return self.trySay(req.text)

    def debug(self, text):
        if(self.DEBUG):
            rospy.loginfo(text)

    def callback(self, msg):
        self.debug("I will say: " + msg.data)
        self.trySay(msg.data)

    def disconnectedVoice(self, text):
        save_path = "play_offline.mp3"
        self.engine.save_to_file(text, save_path)
        self.engine.runAndWait()
        
        WavUtils.play_mp3(save_path, device_index=OUTPUT_DEVICE_INDEX)

    def connectedVoice(self, text): 
        tts = gTTS(text=text, lang='en')
        save_path = "play.mp3"
        tts.save(save_path)
        self.debug("Saying...")
        WavUtils.play_mp3(save_path, device_index=OUTPUT_DEVICE_INDEX)
        self.debug("Stopped")

    def trySay(self, text):
        self.hear_publisher.publish(Bool(True))
        rospy.logwarn("Published: False ")
        self.connectedVoice(text)
        success = True
        try:
            pass
        except  Exception as e:
            print(e)
            self.disconnectedVoice(text)
            success = False
        sleep(1)
        self.hear_publisher.publish(Bool(False))
        rospy.logwarn("Published: True ")
        return success


def main():
    rospy.init_node('say', anonymous=True)
    say = Say()
    say.debug('Say Module Initialized.')
    rospy.spin()

if __name__ == '__main__':
    main()