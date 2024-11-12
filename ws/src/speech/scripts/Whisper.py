#!/usr/bin/env python3

'''
This script creates the node `Whisper` that taking voice audio from topic
`UsefulAudioWhisper`, does speech-to-text and publishes the resulting text.

'''


import rospy
import whisper
import torch
import os
from WavUtils import WavUtils

from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
from frida_hri_interfaces.srv import AudioText, AudioTextResponse

SPEECH_COMMAND_TOPIC = "/speech/raw_command"
SPEECH_SERVICE_TOPIC = "/speech/service/raw_command"
DEBUG = True

CURRENT_FILE_PATH = os.path.abspath(__file__)
CURRENT_DIRECTORY = os.path.dirname(CURRENT_FILE_PATH)
MODEL_DIRECTORY = os.path.join(CURRENT_DIRECTORY, 'models')


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
            rospy.loginfo(f"{message}: {time_delta_second} seconds")


class Whisper():
    def __init__(self):
        # Discard parameters
        self.min_time = 2  # seconds
        self.max_time = 30  # seconds

        # Set the parameters for the temporary WAV file
        # Ensure parameters match UsefulAudio's settings (or Hear's settings, if it modifies the input)
        self.sample_rate = 16000
        self.n_channels = 1
        self.sample_width = 2  # in bytes
        self.load_model()

    # Select model to load. This only needs to be done once.
    def load_model(self):
        # Note: when using a model for the first time, the program will access the internet to download the model.
        # choices=['tiny.en', 'tiny', 'base.en', 'base', 'small.en', 'small', 'medium.en', 'medium', 'large-v1', 'large-v2', 'large-v3', 'large', 'large-v3-turbo', 'turbo']
        model_size = "small"
        # Only download the model if it is not already downloaded
        whisper._download(whisper._MODELS[model_size], MODEL_DIRECTORY, False)

        self.audio_model = whisper.load_model(
            model_size, download_root=MODEL_DIRECTORY)

    def infer(self, data):
        temp_file = WavUtils.generate_temp_wav(
            self.n_channels, self.sample_width, self.sample_rate, data)
        empty = True

        if WavUtils.within_time_frame(temp_file, self.min_time, self.max_time):
            # WavUtils.play_wav_file(temp_file) # Debug if file created sounds good, check when varying parameters
            result = self.infer_wav(temp_file)
            empty = False
        else:
            rospy.loginfo("Discarded audio as it didn't match expected length")

        # Remove temporary file, after using
        WavUtils.discard_wav(temp_file)

        if not empty:
            return result

    def infer_wav(self, wav_path):
        result = self.audio_model.transcribe(
            wav_path, fp16=torch.cuda.is_available())
        return result["text"]


def audio_text_handler(req):

    rospy.loginfo("Whisper service computing...")
    timer = Timer()

    text = whisperModel.infer(req.audio.data)
    timer.endTimer("Whisper finished audio inference")

    if text is None or len(text) == 0 or text.isspace():
        rospy.loginfo("Audio is empty")
        return ""

    # Remove white spaces of resulting text
    text = text.strip()

    rospy.loginfo("Voice audio said (whisper): \"{0}\".".format(text))
    return text


def on_audio_callback(data):
    rospy.loginfo("Whisper computing...")

    text = whisperModel.infer(data.data)

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

    rospy.init_node('Whisper', anonymous=True)
    global DEBUG
    DEBUG = rospy.get_param('~debug', False)
    rospy.loginfo("*Starting Whisper Node*")

    global publisher
    publisher = rospy.Publisher(SPEECH_COMMAND_TOPIC, String, queue_size=10)

    global whisperModel

    timer = Timer()
    whisperModel = Whisper()
    timer.endTimer("Finished loading whisper model")

    rospy.Subscriber("UsefulAudioWhisper", AudioData,
                     on_audio_callback, queue_size=10)
    rospy.Service(SPEECH_SERVICE_TOPIC, AudioText, audio_text_handler)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.loginfo("*Ready to callback whisper.*")
    rospy.spin()

    rospy.loginfo("*Node finished*")


if __name__ == '__main__':
    main()
