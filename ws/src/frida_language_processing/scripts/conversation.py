#!/usr/bin/env python3

"""
Node to store context from the environment, previous prompt and user interactions to provide accurate and conversational responses 
"""

# Libraries
import rospy
import os
from openai import OpenAI

# Messages
from std_msgs.msg import String

# ROS topics
SPEECH_COMMAND_TOPIC = "/speech/raw_command"
CONVERSATION_TOPIC = "/conversation/interact"
SPEAK_TOPIC = "/speech/speak"

# Environment static context
ORIGINS_CONTEXT = "You are a service robot for domestic applications. You were developed by RoBorregos team from Tec de Monterrey, from Mexico."
DATE_CONTEXT = "Today is Sunday, April 7th, 2024. It's 6:00 AM"
LOCATION_CONTEXT = "You are in the RoBorregos lab."
ENVIRONMENT_CONTEXT = f"{ORIGINS_CONTEXT} {DATE_CONTEXT} {LOCATION_CONTEXT}"

class Conversation:
    def __init__(self) -> None:
        """Initialize the ROS node"""
        self._node = rospy.init_node("conversation")
        self._rate = rospy.Rate(10)
        self._sub_speech = rospy.Subscriber(SPEECH_COMMAND_TOPIC, String, self.speech_callback)
        self._sub_conversation = rospy.Subscriber(CONVERSATION_TOPIC, String, self.conversation_callback)
        self._pub_speak = rospy.Publisher(SPEAK_TOPIC, String, queue_size=10)

        ### Objects and variables
        self.openai_client = OpenAI(
            api_key=os.getenv("OPENAI_API_KEY")
        )
        self.prompts_context = ""
        self.interactions_context = ""

        self.interaction_guide = {
            "feedback": f"{ORIGINS_CONTEXT} Provide conversational feedback to the user on the current task being performed, avoid pleasantries such as 'Sure', 'Lets go' etc: ",
            "interact": f"{ENVIRONMENT_CONTEXT} Share information with the user in a conversational way, based on the context given about your environment, the required information, and the perceived data. Don't be redundant on repeating previous interactions, ",
            "ask": f"{ENVIRONMENT_CONTEXT} Ask the user for information it requested or that is needed for you to proceed, Don't be redundant on repeating previous prompts or interactions, "
        }

        rospy.spin()

    def speech_callback(self, data: String) -> None:
        """Callback for the speech command subscriber"""
        self.prompts_context += f"{data.data} "
    
    def conversation_callback(self, data: String) -> None:
        """Callback for the conversation subscriber"""
        string_msg = String()
        string_msg.data = self.process(data.data)
        self._pub_speak.publish(string_msg)
        
    def process(self, request: str) -> str:
        """Process the request using the LMM model
        Args:
            request (str): The request from the user"""
        chat_prompt = ""
        if "feedback" in request:
            chat_prompt = self.interaction_guide["feedback"]
        elif "interact" in request:
            chat_prompt = self.interaction_guide["interact"] + "Prev prompts: " + self.prompts_context + "Prev interactions: " + self.interactions_context
        elif "ask" in request:
            chat_prompt = self.interaction_guide["ask"] + "Prev prompts: " + self.prompts_context + "Prev interactions: " + self.interactions_context

        chat_completion = self.openai_client.chat.completions.create(
            model = "gpt-4",
            messages=[
                {"role": "system", "content": chat_prompt},
                {"role": "user", "content": request}
            ] 
        )

        response = chat_completion.choices[0].message.content
        self.interactions_context += f"{response} "
        rospy.loginfo(f"Response: {response}")
        return response

if __name__ == "__main__":
    try:
        Conversation()
    except rospy.ROSInterruptException as e:
        rospy.logerr("Error: {}".format(e))
        pass