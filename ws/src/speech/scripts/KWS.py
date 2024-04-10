#!/usr/bin/env python3

import os
import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import Bool
from datetime import datetime

import pvporcupine
import struct

ACCESS_KEY = os.getenv("ACCESS_KEY")
KEYWORD_DIR = os.getenv("KEYWORD_DIR")
DEBUG = False

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

class KWS(object):
    def __init__(self):
        # Set ROS publishers and subscribers
        self.subscriber = rospy.Subscriber("rawAudioChunk", AudioData, self.detect_keyword)
        self.publisher = rospy.Publisher("inputAudioActive", Bool, queue_size=20)
        
        # Sensitivities for detecting keywords. Each value should be a number within [0, 1]. 
        # A higher sensitivity results in fewer misses at the cost of increasing the false alarm rate. If not set 0.5 will be used.
        keyword_paths = list_files_with_extension(KEYWORD_DIR, '.ppn')
        sensitivities = [0.3, 0.3, 0.3] 
        
        access_key = ACCESS_KEY
        try:
            self.porcupine = pvporcupine.create(
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
        
        self.keywords = list()
        for x in keyword_paths:
            keyword_phrase_part = os.path.basename(x).replace('.ppn', '').split('_')
            if len(keyword_phrase_part) > 6:
                self.keywords.append(' '.join(keyword_phrase_part[0:-6]))
            else:
                self.keywords.append(keyword_phrase_part[0])
        
    def get_next_audio_frame(self, msg):
        pcm = msg.data
        pcm = struct.unpack_from("h" * self.porcupine.frame_length, pcm)
        return pcm
        
    def detect_keyword(self, msg):
        audio_frame = self.get_next_audio_frame(msg)
        result = self.porcupine.process(audio_frame)
            
        if result >= 0:
            print('[%s] Detected %s' % (str(datetime.now()), self.keywords[result]))
            self.publisher.publish(Bool(True))
    
    def debug(self, text):
        if(DEBUG):
            rospy.loginfo(text)
def main():
    rospy.init_node('KWS', anonymous=True)
    
    # global DEBUG
    # DEBUG = rospy.get_param('~debug', False)
    
    usefulAudio = KWS()
    usefulAudio.debug('KWS Initialized.')
    rospy.spin()

if __name__ == '__main__':
    main()
