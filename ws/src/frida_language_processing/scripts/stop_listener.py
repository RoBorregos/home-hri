#!/usr/bin/env python3

"""
This ROS node interprets the commands received from the Speech processing,
using the fine-tuned model to send the actions to the Task Manager
"""

# Libraries
import rospy
import os
from openai import OpenAI

from sentence_transformers import SentenceTransformer, util
import pickle
            
# Messages
from std_msgs.msg import String, Bool

# ROS topics
SPEECH_COMMAND_TOPIC = "/speech/raw_command"
OUT_COMMAND_TOPIC = "/stop_following"


# OpenAI constants
MODEL = "gpt-3.5-turbo"
SYSTEM_PROMPT = "You are a service robot for domestic applications. The following text was recieved from an audio command. Determine if the command is a stop or pause command. If the command is a stop or pause command send yes. Otherwise, send no."

class StopListener:
    def __init__(self) -> None:
        """Initialize the ROS node, subscribers and publishers"""
        self._node = rospy.init_node("stop_listener", anonymous=True)
        self.use_embeddings = rospy.get_param('~use_embeddings', True)
        self.DEBUG = rospy.get_param('~debug', True)

        ### Objects and variables
        if self.use_embeddings:            
            current_dir = os.path.dirname(os.path.abspath(__file__))
            embeddings_path = os.path.join(current_dir, "assets", "stop_commands.pkl")
            self.description_embeddings = pickle.load(open(embeddings_path, "rb"))
            self.model = SentenceTransformer("all-MiniLM-L12-v2")
        else:
            self.openai_client = OpenAI(
                api_key=os.getenv("OPENAI_API_KEY")
            )
            
        self._rate = rospy.Rate(10)
        self._sub = rospy.Subscriber(SPEECH_COMMAND_TOPIC, String, self._callback)
        self._pub = rospy.Publisher(OUT_COMMAND_TOPIC, Bool, queue_size=10)
        
        rospy.loginfo("Stop listener initialized")
        rospy.spin()

    def _callback(self, data: String) -> None:
        """Callback for the speech command subscriber"""
        if data.data == "" or len(data.data) < 4:
            self.debug("Noise detected, ignoring")
            return
        
        lower_command = data.data.lower()
        
        if "stop" in lower_command or 'pause' in lower_command or 'cease' in lower_command:
            rospy.loginfo("Stop command detected")
            self._pub.publish(Bool(True))
            return
        self.run(data.data)

    def run(self, raw_command: String) -> None:
        if self.use_embeddings:
            new_command_embedding = self.model.encode(raw_command, convert_to_tensor=True)
            for index, description in enumerate(self.description_embeddings):
                similarity = util.pytorch_cos_sim(description, new_command_embedding).item()
                
                if similarity > 0.6:
                    self._pub.publish(Bool(True))
                    return
        else:
            """Method for running the interpretation of the commands"""
            chat_completion = self.openai_client.chat.completions.create(
                model = MODEL,
                messages=[
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user", "content": raw_command}
                ]
            )
            self.debug("Completion received: " + chat_completion.choices[0].message.content)
            interpreted_command = chat_completion.choices[0].message.content
            if "yes" in interpreted_command.lower():
                self._pub.publish(Bool(True))
    
    def debug(self, text):
        if self.DEBUG:
            rospy.loginfo(text)

if __name__ == "__main__":
    try:
        stop_listener = StopListener()
    except rospy.ROSInterruptException as e:
        rospy.logerr("Error: {}".format(e))
        