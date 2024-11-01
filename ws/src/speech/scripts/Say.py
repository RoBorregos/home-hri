#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from gtts import gTTS
from time import sleep
import socket
from WavUtils import WavUtils
import os
from SpeechApiUtils import SpeechApiUtils

from pathlib import Path
from pygame import mixer
from frida_hri_interfaces.srv import Speak
import subprocess

SPEAK_TOPIC = "/speech/speak"
SPEAK_NOW_TOPIC = "/speech/speak_now"

# Offline voice
MODEL = "en_US-amy-medium.onnx"
CURRENT_FILE_PATH = os.path.abspath(__file__)
CURRENT_DIRECTORY = os.path.dirname(CURRENT_FILE_PATH)
TEXT_FILE = os.path.join(CURRENT_DIRECTORY, 'offline_voice', "offline_say.txt")


# Get device index using environment variables
SPEAKER_DEVICE_NAME = os.getenv("SPEAKER_DEVICE_NAME", default=None)
SPEAKER_INPUT_CHANNELS = int(os.getenv("SPEAKER_INPUT_CHANNELS", default=2))
SPEAKER_OUT_CHANNELS = int(os.getenv("SPEAKER_OUT_CHANNELS", default=0))

OUTPUT_DEVICE_INDEX = SpeechApiUtils.getIndexByNameAndChannels(
    SPEAKER_DEVICE_NAME, SPEAKER_INPUT_CHANNELS, SPEAKER_OUT_CHANNELS)

if OUTPUT_DEVICE_INDEX is None:
    print("Warning: output device index not found, using system default.")

DEBUG = True
OFFLINE = True


class Say(object):

    def __init__(self):
        self.connected = False
        if not OFFLINE:
            self.connected = self.is_connected()

        rospy.Service(SPEAK_TOPIC, Speak, self.speak_handler)

        self.text_suscriber = rospy.Subscriber(
            SPEAK_NOW_TOPIC, String, self.callback)
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
        rospy.logerr("No internet connection.")
        return False

    def play_audio(self, file_path):
        mixer.pre_init(frequency=48000, buffer=2048)
        mixer.init()
        mixer.music.load(file_path)
        mixer.music.play()
        while mixer.music.get_busy():
            pass

    """
    New implementation of speak as a service
    """

    def speak_handler(self, req):
        self.debug("I will say: " + req.text)
        return self.trySay(req.text)

    def debug(self, text):
        if (DEBUG):
            self.log(text)

    def log(self, text):
        rospy.loginfo(text)

    def callback(self, msg):
        self.debug("I will say: " + msg.data)
        self.trySay(msg.data)

    def connectedVoice(self, text):
        tts = gTTS(text=text, lang='en')
        save_path = "play.mp3"
        tts.save(save_path)
        self.debug("Saying...")
        # WavUtils.play_mp3(save_path, device_index=OUTPUT_DEVICE_INDEX)
        # .play_mp3(save_path, device_index=OUTPUT_DEVICE_INDEX)
        self.play_audio(save_path)
        self.debug("Finished speaking.")

    def trySay(self, text):
        self.hear_publisher.publish(Bool(True))
        success = False
        try:
            if OFFLINE or not self.connected:
                self.offlineVoice(text)
            else:
                self.connectedVoice(text)
            success = True
        except Exception as e:
            print(e)
            if not OFFLINE:
                print("Retrying with offline mode")
                self.offlineVoice(text)

        sleep(1)
        self.hear_publisher.publish(Bool(False))
        rospy.logwarn("Published: True ")
        return success

    def offlineVoice(self, text):
        output_base_path = os.path.join(CURRENT_DIRECTORY, "offline_voice")

        text_chunks = self.split_text(text, 4000)
        counter = 0

        # Generate all wav files for each chunk
        for chunk in text_chunks:
            counter += 1
            output_path = os.path.join(output_base_path, f"{counter}.wav")
            self.synthesize_voice_offline(output_path, chunk)

        print(f"Generated {counter} wav files.")

        # Play and remove all mp3 files
        for i in range(1, counter+1):
            save_path = os.path.join(output_base_path, f"{i}.wav")
            # WavUtils.play_wav(save_path, device_index=OUTPUT_DEVICE_INDEX)
            self.play_audio(save_path)
            sleep(0.5)
            WavUtils.discard_wav(save_path)

    @staticmethod
    def split_text(text: str, max_len, split_sentences=False):
        """Split text into chunks of max_len characters. The model may not be able to synthesize long texts."""
        text_chunks = text.split(".") if split_sentences else [text]
        limited_chunks = []
        for chunk in text_chunks:
            while len(chunk) > 0:
                limited_chunks.append(chunk[:max_len])
                chunk = chunk[max_len:]

        return limited_chunks

    @staticmethod
    def synthesize_voice_offline(output_path, text):
        """Synthesize text using the offline voice model."""
        model_path = os.path.join(CURRENT_DIRECTORY, 'offline_voice', MODEL)

        command = [
            'echo', f'"{text}"', '|',
            'python3.10', '-m', 'piper', '--model', model_path, '--output_file', output_path
        ]

        subprocess.run(' '.join(command), shell=True)


def main():
    rospy.init_node('say', anonymous=True)
    global DEBUG
    DEBUG = rospy.get_param('~debug', False)

    global OFFLINE
    OFFLINE = rospy.get_param('~offline', False)

    say = Say()
    say.log('Say Module Initialized.')

    rospy.spin()


if __name__ == '__main__':
    main()
