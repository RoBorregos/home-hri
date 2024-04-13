#!/usr/bin/env python3

# Rospy dependencies
import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import Bool, String, Int16

# DOA dependencies
from Tuning import Tuning
import usb.core
import usb.util

# LED dependencies
from pixel_ring import pixel_ring

# Modififed at main()
DEBUG = False

class Timer():
    def __init__(self):
        self.timer = rospy.Time.now()

    def startTime(self):
        self.timer = rospy.Time.now()
    
    def endTimer(self):
        time_delta = rospy.Time.now() - self.timer
        time_delta_second = time_delta.to_sec()
        return time_delta_second

class MovingAverage:
    def __init__(self, window_size):
        self.window_size = window_size
        self.data = [0] * window_size  # Circular buffer
        self.sum = 0
        self.size = 0
        self.index = 0

    def next(self, val):
        if self.size < self.window_size:
            self.size += 1

        self.sum -= self.data[self.index]
        self.data[self.index] = val
        self.sum += val
        self.index = (self.index + 1) % self.window_size

        return self.sum / self.size

class ReSpeaker(object):

    def __init__(self):
        self.WAIT_TIME = 0.5 # Time to wait before publishing DOA
        self.timer = Timer()
        self.dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        self.moving_average = MovingAverage(10)
        
        self.publisher = rospy.Publisher("DOA", Int16, queue_size=20)
        rospy.Subscriber("ReSpeaker/light", String, self.callback_light)
        
        if self.dev:
            self.tuning = Tuning(self.dev)
        else:
            self.log("Warning: ReSpeaker not found")
                
        self.timer = Timer()

    def debug(self, text):
        if(DEBUG):
            self.log(text)
            
    def log(self, text):
        rospy.loginfo(text)

    def run(self):
        while not rospy.is_shutdown():
            if self.timer.endTimer() > self.WAIT_TIME:
                self.publish_DOA()
                self.timer.startTime()
    
    def publish_DOA(self):
        if self.tuning:
            next_angle = self.moving_average(self.tuning.direction)
            self.publisher.publish(Int16(next_angle))
    
    def callback_light(self, data):
        command = data.data
        
        if command == 'off':
            pixel_ring.off()
        elif command == 'think':
            pixel_ring.think()
        elif command == 'speak':
            pixel_ring.speak()
        elif command == 'listen':
            pixel_ring.listen()
        else:
            self.log("Command: " + command + " not supported")
        
                  

def main():
    rospy.init_node('ReSpeaker', anonymous=True)
    
    global DEBUG
    DEBUG = rospy.get_param('~debug', False)
    
    respeaker = ReSpeaker()
    
    respeaker.log('ReSpeaker Initialized.')
    respeaker.run()

if __name__ == '__main__':
    main()
    