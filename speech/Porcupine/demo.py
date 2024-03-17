#
# Copyright 2018-2023 Picovoice Inc.
#
# You may not use this file except in compliance with the license. A copy of the license is located in the "LICENSE"
# file accompanying this source.
#
# Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on
# an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.
#

import argparse
import os
from dotenv import load_dotenv
import struct
import wave
from datetime import datetime

import pvporcupine
from pvrecorder import PvRecorder

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

    keywords = list()
    for x in keyword_paths:
        keyword_phrase_part = os.path.basename(x).replace('.ppn', '').split('_')
        if len(keyword_phrase_part) > 6:
            keywords.append(' '.join(keyword_phrase_part[0:-6]))
        else:
            keywords.append(keyword_phrase_part[0])


    recorder = PvRecorder(
        frame_length=porcupine.frame_length,
        device_index=-1)
    recorder.start()

    print('Listening ... (press Ctrl+C to exit)')

    try:
        while True:
            pcm = recorder.read()
            result = porcupine.process(pcm)

            # if wav_file is not None:
            #     wav_file.writeframes(struct.pack("h" * len(pcm), *pcm))

            if result >= 0:
                print('[%s] Detected %s' % (str(datetime.now()), keywords[result]))
    except KeyboardInterrupt:
        print('Stopping ...')
    finally:
        recorder.delete()
        porcupine.delete()


if __name__ == '__main__':
    main()