import os
from dotenv import load_dotenv
from datetime import datetime

import pvporcupine
import pyaudio
import struct

load_dotenv()

ACCESS_KEY = os.getenv("ACCESS_KEY")
KEYWORD_DIR = os.getenv("KEYWORD_DIR")

def list_files_with_extension(directory, extension):
    if not os.path.exists(directory):
        print(f"The directory '{directory}' does not exist.")
        return
    
    file_list = []

    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(extension):
                file_list.append(os.path.join(root, file))

    return file_list

def main():

    # Sensitivities for detecting keywords. Each value should be a number within [0, 1]. 
    # A higher sensitivity results in fewer misses at the cost of increasing the false alarm rate. If not set 0.5 will be used.
    keyword_paths = list_files_with_extension(KEYWORD_DIR, '.ppn')
    sensitivities = [0.3, 0.3, 0.3] 
    
    access_key = ACCESS_KEY

    try:
        porcupine = pvporcupine.create(
            access_key=access_key,
            keyword_paths=keyword_paths,
            sensitivities=sensitivities)
    except pvporcupine.PorcupineActivationError as e:
        print("AccessKey activation error")
        raise e
    except pvporcupine.PorcupineActivationLimitError as e:
        print("AccessKey '%s' has reached it's temporary device limit" % access_key)
        raise e
    except pvporcupine.PorcupineActivationRefusedError as e:
        print("AccessKey '%s' refused" % access_key)
        raise e
    except pvporcupine.PorcupineActivationThrottledError as e:
        print("AccessKey '%s' has been throttled" % access_key)
        raise e
    except pvporcupine.PorcupineError as e:
        print("Failed to initialize Porcupine")
        raise e
    
    CHUNK_SIZE = porcupine.frame_length
    # Signed 2 bytes.
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = porcupine.sample_rate
    
    p = pyaudio.PyAudio()
    stream = p.open(
        rate=RATE,
        channels=CHANNELS,
        format=FORMAT,
        input=True,
        frames_per_buffer=CHUNK_SIZE,
        input_device_index=None) # See list_audio_devices() or set it to None for default

    def get_next_audio_frame():
        pcm = stream.read(CHUNK_SIZE)
        pcm = struct.unpack_from("h" * CHUNK_SIZE, pcm)
        return pcm


    keywords = list()
    for x in keyword_paths:
        keyword_phrase_part = os.path.basename(x).replace('.ppn', '').split('_')
        if len(keyword_phrase_part) > 6:
            keywords.append(' '.join(keyword_phrase_part[0:-6]))
        else:
            keywords.append(keyword_phrase_part[0])

    print('Listening ... (press Ctrl+C to exit)')
    result = -1
    try:          
        while True:
            audio_frame = get_next_audio_frame()
            result = porcupine.process(audio_frame)
            
            if result >= 0:
                print('[%s] Detected %s' % (str(datetime.now()), keywords[result]))
                    
    except KeyboardInterrupt:
        print('Stopping ...')
    finally:
        porcupine.delete()


if __name__ == '__main__':
    main()