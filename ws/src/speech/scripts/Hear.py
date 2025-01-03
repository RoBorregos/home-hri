#!/usr/bin/env python3
import rospy
import grpc
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
import speech_pb2
import speech_pb2_grpc
from SpeechApiUtils import SpeechApiUtils

class WhisperClient:
    def __init__(self, address):
        self.channel = grpc.insecure_channel(address)
        self.stub = speech_pb2_grpc.SpeechServiceStub(self.channel)

    def transcribe(self, audio_data):
        try:
            request = speech_pb2.AudioRequest(audio_data=audio_data)
            response = self.stub.Transcribe(request)
            return response.text
        except grpc.RpcError as e:
            rospy.logerr(f"Detailed gRPC error: {str(e.details())}")
            rospy.logerr(f"Code: {e.code()}")
            raise

def callback_audio(data, args):
    client, publisher = args
    rospy.loginfo("Received audio data, sending to Whisper gRPC server...")

    # Resample the audio to 16 kHz (if needed)
    resampled = SpeechApiUtils.resample_ratecv(data.data, 16000, 16000)
    all_samples = SpeechApiUtils.get_all_samples(resampled[0])
    
    # Convert samples to bytes
    audio_bytes = bytes(all_samples)  # Convert the list to bytes
    
    # Send audio to Whisper gRPC server
    try:
        transcription = client.transcribe(audio_bytes)
        rospy.loginfo(f"Transcription received: {transcription}")

        # Publish the transcription to a ROS topic
        msg = String()
        msg.data = transcription
        publisher.publish(msg)
        rospy.loginfo("Transcription published to ROS topic.")
    except grpc.RpcError as e:
        rospy.logerr(f"gRPC error: {e}")

def main():
    rospy.init_node('hear', anonymous=True)
    rospy.loginfo("*Starting Hear Node*")

    # Create the gRPC client
    whisper_client = WhisperClient(rospy.get_param("~STT_SERVER_IP"))

    # ROS Publisher for transcription
    publisher = rospy.Publisher("/speech/raw_command", String, queue_size=10)

    # ROS Subscriber for audio data
    rospy.Subscriber("UsefulAudio", AudioData, callback_audio, (whisper_client, publisher))

    rospy.loginfo("*Hear Node is ready*")
    rospy.spin()

if __name__ == '__main__':
    main()
