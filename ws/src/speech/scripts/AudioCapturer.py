#!/usr/bin/env python3
import rospy
from audio_common_msgs.msg import AudioData
import pyaudio
import os


# Format for the recorded audio by PyAudio. It is exactly as RNNoise
# (devices/InputAudio) needs it.
CHUNK_SIZE = 480
# Signed 2 bytes.
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 48000

INPUT_DEVICE_INDEX = os.getenv("INPUT_DEVICE_INDEX", default=None)

def main():
    rospy.init_node('AudioCapturer', anonymous=True)
    publisher = rospy.Publisher("rawAudioChunk", AudioData, queue_size=20)

    p = pyaudio.PyAudio()
    stream = p.open(input_device_index=INPUT_DEVICE_INDEX, # See list_audio_devices() or set it to None for default
                    format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK_SIZE)
    print("*Recording*")

    # Loop while node is not closed and audio is working. Note that
    # here is "not needed" something like `spin()` or `loop_sleep()`
    # because (presumably) PyAudio manages the blocked times waiting for 
    # IO and puts the process/thread to sleep, also there are no callbacks
    # by ROS.
    while stream.is_active() and not rospy.is_shutdown():
        try:
            in_data = stream.read(CHUNK_SIZE, exception_on_overflow = False)
            msg = in_data
            publisher.publish(AudioData(data=msg))
        except IOError as e:
            print("I/O error({0}): {1}".format(e.errno, e.strerror))
            # break
 
    if not stream.is_active():
        print("Stream was not active.")

    stream.stop_stream()
    stream.close()
    p.terminate()

# Get available devices
def list_audio_devices():
    p = pyaudio.PyAudio()
    num_devices = p.get_device_count()
    print("Available audio devices:")
    for i in range(num_devices):
        device_info = p.get_device_info_by_index(i)
        print(f"Device {i}: {device_info['name']}, {device_info['maxInputChannels']} input channels, {device_info['maxOutputChannels']} output channels")
    p.terminate()

if __name__ == "__main__":
    # list_audio_devices()
    main()
