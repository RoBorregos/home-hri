import grpc
from concurrent import futures
import speech_pb2
import speech_pb2_grpc
from faster_whisper import WhisperModel
import os
import torch
from WavUtils import WavUtils

class WhisperServicer(speech_pb2_grpc.SpeechServiceServicer):
    def __init__(self):
        self.audio_model = self.load_model()

    def load_model(self):
        model_size = "base" # .en?
        model_directory = os.path.join(os.path.dirname(__file__), 'models')
        device = "cuda" if torch.cuda.is_available() else "cpu"
        return WhisperModel(model_size, download_root=model_directory, device=device, compute_type="float32")

    def Transcribe(self, request, context):
        # Generate a temporary WAV file from received audio data
        temp_file = WavUtils.generate_temp_wav(1, 2, 16000, request.audio_data)

        # Perform transcription
        result = self.audio_model.transcribe(
            temp_file,
            language="en",
            hotwords="Frida kitchen attendance",
            condition_on_previous_text=True,
        )

        WavUtils.discard_wav(temp_file)

        # Return the transcribed text
        return speech_pb2.TextResponse(text=result["text"].strip())

def serve():
    # Create the gRPC server
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    speech_pb2_grpc.add_SpeechServiceServicer_to_server(WhisperServicer(), server)

    # Bind to a port
    server.add_insecure_port('0.0.0.0:50051')
    print("Whisper gRPC server is running on port 50051...")
    server.start()
    server.wait_for_termination()

if __name__ == '__main__':
    serve()
