#!/usr/bin/env python3

import os
import sys
import time

current_dir = os.path.dirname(os.path.abspath(__file__))

sys.path.append(os.path.join(current_dir, '..'))


from Whisper import Whisper

if __name__ == "__main__":
    whisper = Whisper()
    
    test_files = ['audio/output.wav']
    
    for file in test_files:
        start_time = time.time()
        text = whisper.infer_wav(file)
        print(text)
        print("Inference taken: ", time.time() - start_time)
    