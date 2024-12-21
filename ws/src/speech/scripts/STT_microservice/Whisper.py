import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import grpc
from concurrent import futures
import speech_pb2
import speech_pb2_grpc
import whisper
import torch
from WavUtils import WavUtils

class WhisperServicer(speech_pb2_grpc.SpeechServiceServicer):
    def __init__(self):
        self.audio_model = self.load_model()

    def load_model(self):
        model_size = "base.en"
        model_directory = os.path.join(os.path.dirname(__file__), '../models')
        whisper._download(whisper._MODELS[model_size], model_directory, False)
        return whisper.load_model(model_size, download_root=model_directory)

    def Transcribe(self, request, context):
        # Generate a temporary WAV file from received audio data
        temp_file = WavUtils.generate_temp_wav(1, 2, 16000, request.audio_data)

        # Perform transcription
        result = self.audio_model.transcribe(temp_file, fp16=torch.cuda.is_available())
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
