#!/usr/bin/env python3
import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import Bool, String
from frida_hri_interfaces.srv import STT, AudioText

import webrtcvad
import pyaudio
import collections

import numpy as np
import os
import onnxruntime

# Constants
FORMAT = pyaudio.paInt16
CHANNELS = 1
CHUNK_SIZE = 512
RATE = 16000
CHUNK_DURATION = CHUNK_SIZE / RATE
TIME_FOR_CHANGE = 0.25
COUNT_FOR_CHANGE = TIME_FOR_CHANGE / CHUNK_DURATION
MIN_AUDIO_LENGTH = 0.50
MIN_CHUNKS_AUDIO_LENGTH = MIN_AUDIO_LENGTH / CHUNK_DURATION

PADDING_DURATION = 0.50
NUM_PADDING_CHUNKS = int(PADDING_DURATION / CHUNK_DURATION)

# Modififed at main()
DEBUG = False
USE_SILERO_VAD = True
DISABLE_KWS = False
MAX_AUDIO_DURATION = 10
STT_SERVICE = False

STT_SERVICE_TOPIC = "/speech/STT"
WHISPER_SERVICE_TOPIC = "/speech/service/raw_command"

current_dir = os.path.dirname(os.path.abspath(__file__))


class Timer():
    def __init__(self):
        self.timer = rospy.Time.now()

    def startTime(self):
        self.timer = rospy.Time.now()

    def endTimer(self):
        time_delta = rospy.Time.now() - self.timer
        time_delta_second = time_delta.to_sec()
        return time_delta_second


class UsefulAudio(object):
    triggered = False
    chunk_count = 0
    voiced_frames = []
    ring_buffer = collections.deque(maxlen=NUM_PADDING_CHUNKS)

    audioCollection = []  # Temporary storage for audio data, to be used in the VAD
    chunkCollection = None  # Temporary storage for chunk data, to be sent to the ASR model

    def __init__(self, threshold: float = 0.1):
        if not USE_SILERO_VAD:
            self.vad = webrtcvad.Vad()
            self.vad.set_mode(3)
        else:
            model_path = os.path.join(
                current_dir, "../assets", "silero_vad.onnx")
            options = onnxruntime.SessionOptions()
            options.log_severity_level = 4

            self.inference_session = onnxruntime.InferenceSession(
                model_path, sess_options=options
            )
            self.SAMPLING_RATE = 16000
            self.threshold = threshold
            self.h = np.zeros((2, 1, 64), dtype=np.float32)
            self.c = np.zeros((2, 1, 64), dtype=np.float32)

        self.timer = None
        self.isSaying = False
        self.audioState = "None"  # ['idle', 'saying', 'listening']
        self.service_active = False

        self.publisher = rospy.Publisher(
            "UsefulAudio", AudioData, queue_size=20)
        self.audioStatePublisher = rospy.Publisher(
            "AudioState", String, queue_size=10)
        self.lampPublisher = rospy.Publisher(
            "colorInstruction", String, queue_size=10)
        self.respeakerLightPublisher = rospy.Publisher(
            "/ReSpeaker/light", String, queue_size=10)

        if STT_SERVICE:
            rospy.wait_for_service(WHISPER_SERVICE_TOPIC)
            self.whisper_client = rospy.ServiceProxy(
                WHISPER_SERVICE_TOPIC, AudioText)
            rospy.Service(STT_SERVICE_TOPIC, STT, self.stt_handler)

        rospy.Subscriber("rawAudioChunk", AudioData, self.callbackRawAudio)
        rospy.Subscriber("saying", Bool, self.callbackSaying)
        rospy.Subscriber("keyword_detected", Bool, self.callbackKeyword)

    def debug(self, text):
        if (DEBUG):
            self.log(text)

    def log(self, text):
        rospy.loginfo(text)

    def buildAudio(self, data):
        if self.voiced_frames == None:
            self.voiced_frames = data
        else:
            self.voiced_frames += data
        self.chunk_count += 1

    def discardAudio(self):
        if not self.service_active:
            self.ring_buffer.clear()
            self.voiced_frames = None
            self.chunk_count = 0

    def publishAudio(self):
        if self.chunk_count > MIN_CHUNKS_AUDIO_LENGTH:
            self.publisher.publish(AudioData(self.voiced_frames))
        self.discardAudio()

    def callbackActive(self, msg):
        self.inputAudioActive = msg.data

    def saying_callback(self, msg):
        self.saying = msg.data

    def int2float(self, sound):
        abs_max = np.abs(sound).max()
        sound = sound.astype('float32')
        if abs_max > 0:
            sound *= 1/32768
        sound = sound.squeeze()  # depends on the use case
        return sound

    def vad_collector(self, chunk):
        is_speech = False

        if USE_SILERO_VAD:
            chunk_vad = np.frombuffer(chunk, dtype=np.int16)
            audio_float32 = self.int2float(chunk_vad)

            if self.chunkCollection == None:
                self.chunkCollection = chunk
            else:
                self.chunkCollection += chunk

            if (len(self.audioCollection) < 3):
                self.audioCollection.append(audio_float32)
                return

            audio = np.concatenate(self.audioCollection)
            chunk = self.chunkCollection
            self.audioCollection = []
            self.chunkCollection = None
            is_speech = self.is_speech_silero_vad(audio)

        else:
            is_speech = self.vad.is_speech(chunk, RATE)

        # if is_speech:
        #     print("Speech detected")
        # else:
        #     print("Speech not detected")

        if not self.triggered:
            self.ring_buffer.append((chunk, is_speech))
            num_voiced = len([f for f, speech in self.ring_buffer if speech])

            # If we're NOTTRIGGERED and more than 90% of the frames in
            # the ring buffer are voiced frames, then enter the
            # TRIGGERED state.
            if num_voiced > 0.75 * self.ring_buffer.maxlen:
                self.triggered = True
                self.computeAudioState()
                self.debug("Moving to triggered state")
                # We want to publish all the audio we see from now until
                # we are NOTTRIGGERED, but we have to start with the
                # audio that's already in the ring buffer.
                for f, _ in self.ring_buffer:
                    self.buildAudio(f)
                self.ring_buffer.clear()
        else:
            # Start timer when audio collection starts
            if self.timer is None:
                self.timer = Timer()

            # We're in the TRIGGERED state, so collect the audio data
            # and add it to the ring buffer.
            self.buildAudio(chunk)
            self.ring_buffer.append((chunk, is_speech))
            num_unvoiced = len(
                [f for f, speech in self.ring_buffer if not speech])
            # If more than 75% of the frames in the ring buffer are
            # unvoiced, then enter NOTTRIGGERED and publish whatever
            # audio we've collected.
            if num_unvoiced > 0.75 * self.ring_buffer.maxlen or self.timer.endTimer() > MAX_AUDIO_DURATION:
                self.triggered = False
                self.computeAudioState()
                if not self.service_active:
                    self.publishAudio()
                self.timer = None

    def is_speech_silero_vad(self, audio_data: np.ndarray) -> bool:
        input_data = {
            "input": audio_data.reshape(1, -1),
            "sr": np.array([self.SAMPLING_RATE], dtype=np.int64),
            "h": self.h,
            "c": self.c,
        }
        out, h, c = self.inference_session.run(None, input_data)
        self.h, self.c = h, c
        return out > self.threshold

    def callbackRawAudio(self, data):
        if self.audioState == 'saying':
            self.discardAudio()
        #  Make it possible to collect audio if KWS is disabled
        elif self.audioState == 'listening' or DISABLE_KWS:
            # print("Listening")
            self.vad_collector(data.data)
        else:
            self.discardAudio()

    def callbackSaying(self, data):
        self.isSaying = data.data
        self.computeAudioState()

    def callbackKeyword(self, data):
        self.triggered = True

        if not self.service_active:
            self.discardAudio()
            self.computeAudioState()

    def computeAudioState(self):
        new_state = None
        if self.isSaying:
            new_state = 'saying'
        elif self.triggered:
            new_state = 'listening'
        else:
            new_state = 'idle'

        if self.audioState != new_state:
            self.debug("Audio state changed from: " +
                       self.audioState + " to: " + new_state)
            self.timer = None
            self.audioState = new_state
            self.audioStatePublisher.publish(String(self.audioState))
            self.publish_lamp(self.audioState)
            self.publish_respeaker_light(self.audioState)

    def publish_lamp(self, state):
        if state == 'saying':
            self.lampPublisher.publish(String("GREEN"))
        elif state == 'listening':
            self.lampPublisher.publish(String("BLUE"))
        else:
            self.lampPublisher.publish(String("RED"))

    def publish_respeaker_light(self, state):
        if state == 'saying':
            self.respeakerLightPublisher.publish(String("speak"))
        elif state == 'listening':
            self.respeakerLightPublisher.publish(String("think"))
        else:
            self.respeakerLightPublisher.publish(String("off"))

    def stt_handler(self, req):
        self.discardAudio()
        self.service_active = True
        self.triggered = True
        self.computeAudioState()
        timeout = 15
        counter = 0

        while self.triggered and counter < timeout:
            rospy.sleep(1)
            counter += 1

        raw_text = self.whisper_client(
            AudioData(self.voiced_frames)).text_heard

        self.service_active = False
        self.discardAudio()

        return raw_text


def main():
    rospy.init_node('UsefulAudio', anonymous=True)

    global DEBUG
    DEBUG = rospy.get_param('~debug', False)

    global USE_SILERO_VAD
    USE_SILERO_VAD = rospy.get_param('~USE_SILERO_VAD', True)

    THRESHOLD = rospy.get_param('~threshold', 0.1)

    global DISABLE_KWS
    DISABLE_KWS = rospy.get_param('~DISABLE_KWS', False)

    global MAX_AUDIO_DURATION
    MAX_AUDIO_DURATION = rospy.get_param('~MAX_AUDIO_DURATION', 10)

    global STT_SERVICE
    STT_SERVICE = rospy.get_param('~STT_SERVICE', 10)

    usefulAudio = UsefulAudio(threshold=THRESHOLD)

    usefulAudio.log('UsefulAudio Initialized.')
    rospy.spin()


if __name__ == '__main__':
    main()
