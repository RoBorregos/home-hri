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

import wave
from pathlib import Path
from typing import Any, Dict
from piper import PiperVoice
from download import ensure_voice_exists, find_voice, get_voices

from frida_hri_interfaces.srv import Speak

#SPEAK_TOPIC = "/robot_text"
SPEAK_TOPIC = "/speech/speak"
SPEAK_NOW_TOPIC = "/speech/speak_now"

# Offline voice
MODEL = "en_US-amy-medium"
DOWNLOAD = Path.cwd()

# Get device index using environment variables
SPEAKER_DEVICE_NAME = os.getenv("SPEAKER_DEVICE_NAME", default=None)
SPEAKER_INPUT_CHANNELS = int(os.getenv("SPEAKER_INPUT_CHANNELS", default=2))
SPEAKER_OUT_CHANNELS = int(os.getenv("SPEAKER_OUT_CHANNELS", default=0))

OUTPUT_DEVICE_INDEX = SpeechApiUtils.getIndexByNameAndChannels(SPEAKER_DEVICE_NAME, SPEAKER_INPUT_CHANNELS, SPEAKER_OUT_CHANNELS)

if OUTPUT_DEVICE_INDEX is None:
    print("Warning: output device index not found, using system default.")

DEBUG = True
OFFLINE = True

class Say(object):

    def __init__(self):
        self.connected = self.is_connected()

        if OFFLINE:
            self.offline_engine = self.initialize_offline_engine()
            self.synthesize_args = {
                "speaker_id": None,
                "length_scale": None,
                "noise_scale": None,
                "noise_w": None,
                "sentence_silence": 0.0,
            }
        
        rospy.Service(SPEAK_TOPIC, Speak, self.speak_handler)
        self.text_suscriber = rospy.Subscriber(SPEAK_NOW_TOPIC, String, self.callback)
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
        if(DEBUG):
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
        WavUtils.play_mp3(save_path, device_index=OUTPUT_DEVICE_INDEX)
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
        output_base_path = "./offline_voice"
        
        text_chunks = self.split_text(text, 50)
        
        counter = 0
        
        # Generate all wav files for each chunk
        for chunk in text_chunks:
            counter += 1
            output_path = os.path.join(output_base_path, f"{counter}.wav")
            with wave.open(output_path, "wb") as wav_file:
                self.voice.synthesize(chunk, wav_file, **self.synthesize_args)
        
        # Play and remove all mp3 files
        for i in range(1, counter):
            save_path = os.path.join(output_base_path, f"{i}.wav")
            WavUtils.play_wav(save_path, device_index=OUTPUT_DEVICE_INDEX)
            WavUtils.discard_wav(save_path)


    def split_text(text: str, max_len):
        """Split text into chunks of max_len characters. The model may not be able to synthesize long texts."""
        
        text_chunks = text.split(".")
        limited_chunks = []
        for chunk in text_chunks:
            while len(chunk) > 0:
                limited_chunks.append(chunk[:max_len])
                chunk = chunk[max_len:]

        return limited_chunks
    
    def initialize_offline_engine():
        model_path = Path(MODEL)
        if not model_path.exists():
            # Load voice info
            voices_info = get_voices(DOWNLOAD, update_voices=True)

            # Resolve aliases for backwards compatibility with old voice names
            aliases_info: Dict[str, Any] = {}
            for voice_info in voices_info.values():
                for voice_alias in voice_info.get("aliases", []):
                    aliases_info[voice_alias] = {"_is_alias": True, **voice_info}

            voices_info.update(aliases_info)
            ensure_voice_exists(MODEL, [DOWNLOAD], DOWNLOAD, voices_info)
            model_path, model_config = find_voice(MODEL, [DOWNLOAD])

        voice = PiperVoice.load(model_path, config_path=model_config, use_cuda=False)
        return voice
    


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