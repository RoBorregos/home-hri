#!/usr/bin/env python3

'''
This script creates the node `Whisper` that taking voice audio from topic
`UsefulAudioWhisper`, does speech-to-text and publishes the resulting text.

'''

# TODO: 
# Reduce audio messages in pipeline, while maintaining decoupling 
# Check which whisper model performs better

import rospy
import whisper
import torch

from WavUtils import WavUtils

from audio_common_msgs.msg import AudioData
from speech.msg import RawInput
from std_msgs.msg import String


#SPEECH_COMMAND_TOPIC = "RawInput"
SPEECH_COMMAND_TOPIC = "/speech/raw_command"
DEBUG = True

class Timer():
    def __init__(self):
        self.timer = rospy.Time.now()

    def startTime(self):
        if DEBUG:
            self.timer = rospy.Time.now()
    
    def endTimer(self, message):
        if DEBUG:
            time_delta = rospy.Time.now() - self.timer
            time_delta_second = time_delta.to_sec()
            rospy.logdebug(f"{message}: {time_delta_second} seconds")

class Whisper():
    def __init__(self):
        # Discard parameters
        self.min_time = 2 # seconds
        self.max_time = 30 # seconds

        # Set the parameters for the temporary WAV file
        # Ensure parameters match UsefulAudio's settings (or Hear's settings, if it modifies the input)
        self.sample_rate = 16000 
        self.n_channels = 1
        self.sample_width = 2  # in bytes
        self.load_model()

    # Select model to load. This only needs to be done once.
    def load_model(self):
        # Note: when using a model for the first time, the program will access the internet to download the model.
        # choices=["tiny.en", "base.en", "small.en", "medium.en", "large.en"]
        # model = "tiny.en"
        model = "small.en"
        timer = Timer()
        self.audio_model = whisper.load_model(model)
        timer.endTimer(f"Finished loading whisper model [{model}]")
    
    # Audio interpretation
    def interpret(self, data):
        timer = Timer()
        temp_file = WavUtils.generate_temp_wav(self.n_channels, self.sample_width, self.sample_rate, data)
        timer.endTimer("Finished generating temp wav file")
        empty = True

        if WavUtils.within_time_frame(temp_file, self.min_time, self.max_time):
            # WavUtils.play_wav_file(temp_file) # Debug if file created sounds good, check when varying parameters 
            timer.startTime()
            result = self.audio_model.transcribe(temp_file, fp16=torch.cuda.is_available())
            timer.endTimer("Finished transcribing wav file")
            empty = False
        else:
            rospy.loginfo("Discarded audio as it didn't match expected length")

        # Remove temporary file, after using
        WavUtils.discard_wav(temp_file)

        if not empty:
            return result["text"] 
    

def on_audio_callback(data):
    rospy.loginfo("Whisper computing...")
    
    text = whisperModel.interpret(data.data)

    if text is None or len(text) == 0 or text.isspace():
       rospy.loginfo("Audio is empty")
       return

    # Remove white spaces of resulting text
    text = text.strip()

    rospy.loginfo("Voice audio said (whisper): \"{0}\".".format(text))

    msg = String()
    msg.data = text
    publisher.publish(msg)
    rospy.loginfo("Published whisper result.")

def main():
    
    rospy.init_node('Whisper')
    rospy.loginfo("*Starting Whisper Node*")

    global DEBUG
    DEBUG = rospy.get_param('~debug', False)

    global publisher
    publisher = rospy.Publisher(SPEECH_COMMAND_TOPIC, String, queue_size=10)
    
    global whisperModel
    
    whisperModel = Whisper()

    rospy.Subscriber("UsefulAudioWhisper", AudioData, on_audio_callback, queue_size=10)        
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.loginfo("*Ready to callback whisper.*")
    rospy.spin()

    rospy.loginfo("*Node finished*")

if __name__ == '__main__':
    main()
