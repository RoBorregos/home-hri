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
from SpeechApiUtils import SpeechApiUtils

# Get device index using environment variables
SPEAKER_DEVICE_NAME = os.getenv("SPEAKER_DEVICE_NAME", default=None)
SPEAKER_INPUT_CHANNELS = int(os.getenv("SPEAKER_INPUT_CHANNELS", default=2))
SPEAKER_OUT_CHANNELS = int(os.getenv("SPEAKER_OUT_CHANNELS", default=0))

OUTPUT_DEVICE_INDEX = SpeechApiUtils.getIndexByNameAndChannels(SPEAKER_DEVICE_NAME, SPEAKER_INPUT_CHANNELS, SPEAKER_OUT_CHANNELS)

if OUTPUT_DEVICE_INDEX is None:
    print("Warning: output device index not found, using system default.")

DEBUG = True

class Say(object):

    def __init__(self):
        self.engine = pyttsx3.init()
        self.engine.setProperty('voice', 'english_rp+f3')
        self.connected = self.is_connected()
        self.text_suscriber = rospy.Subscriber("robot_text", String, self.callback)
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

    def debug(self, text):
        if(DEBUG):
            self.log(text)
            
    def log(self, text):
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
        self.debug("Finished speaking.")

    def trySay(self, text):
        self.hear_publisher.publish(Bool(True))
        self.connectedVoice(text)
        try:
            pass
        except  Exception as e:
            print(e)
            self.disconnectedVoice(text)
        sleep(1)
        self.hear_publisher.publish(Bool(False))


def main():
    rospy.init_node('say', anonymous=True)
    global DEBUG
    DEBUG = rospy.get_param('~debug', False)
    
    say = Say()
    say.log('Say Module Initialized.')
    
    rospy.spin()

if __name__ == '__main__':
    main()