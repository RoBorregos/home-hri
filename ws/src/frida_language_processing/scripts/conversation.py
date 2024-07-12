#!/usr/bin/env python3

"""
Node to store context from the environment, previous prompt and user interactions to provide accurate and conversational responses 
"""

# Libraries
import rospy
import os
from openai import OpenAI
import actionlib
import time

# Messages
from std_msgs.msg import String
from frida_hri_interfaces.msg import ConversateAction, ConversateResult, ConversateFeedback, ConversateGoal
from frida_hri_interfaces.srv import Speak

# ROS topics
SPEECH_COMMAND_TOPIC = "/speech/raw_command"
SPEAK_TOPIC = "/speech/speak"
CONVERSATION_TOPIC = "/conversation_as"

# Environment static context
ORIGINS_CONTEXT = "You are a service robot for domestic applications called Frida. You were developed by RoBorregos team from Tec de Monterrey, from Mexico."
DATE_CONTEXT = "Today is Saturday, April 20th, 2024. It's 10:00 am"
LOCATION_CONTEXT = "You are in the Faculty of mechanical and electrical engineering of Nuevo Leon University."
ENVIRONMENT_CONTEXT = f"{ORIGINS_CONTEXT} {DATE_CONTEXT} {LOCATION_CONTEXT}"

class Conversation:
    def __init__(self) -> None:
        """Initialize the ROS node"""
        self._node = rospy.init_node("conversation")
        self._rate = rospy.Rate(10)
        self._sub_speech = rospy.Subscriber(SPEECH_COMMAND_TOPIC, String, self.speech_callback)
        #self._pub_speak = rospy.Publisher(SPEAK_TOPIC, String, queue_size=10) 
        rospy.wait_for_service(SPEAK_TOPIC)
        self.speak_client = rospy.ServiceProxy(SPEAK_TOPIC, Speak)


        ### Action server 
        self._conversation_as = actionlib.SimpleActionServer(
            CONVERSATION_TOPIC, ConversateAction, execute_cb=self.conversation_callback, auto_start=False
        )
        self._conversation_as.start()

        ### Objects and variables
        self.openai_client = OpenAI(
            api_key=os.getenv("OPENAI_API_KEY")
        )
        self.prompts_context = ""
        self.interactions_context = ""

        self.interaction_guide = {
            "feedback": f"{ORIGINS_CONTEXT} Provide feedback to the user on the current task being performed, avoid pleasantries such as 'Sure', 'Lets go' etc. Don't be verbose on your response. For example, if you receive as the user input 'go: kitchen', you should answer 'I'm going to the kitchen', or if you receive 'pick: apple', you should answer 'I'm searching the apple for picking it'.",
            "interact": f"{ENVIRONMENT_CONTEXT} Share information with the user in a briefly conversational way, based mainly on required information, and if needed the perceived data you received but you could also use the context given about your environment, previous prompts and interactions and your own relevant knowledge if relevant. Don't be redundant on repeating previous interactions. Don't ask any questions here. For example, if you are prompted with 'interact: salute Charlie and inform which day is tomorrow', you should answer 'Hi Charlie, tomorrow will be Monday, April 8th  of 2024', or if prompted 'interact: blue shirt person count, perceived info: go kitche, 3 blue shirt', you should answer 'I saw 3 persons with blue shirts in the kitchen'.",
            "ask": f"{ENVIRONMENT_CONTEXT} Ask the user for information it requested or that is needed for you to proceed, based mainly on the perceived data you received and the required information, but you could also use the context given about your environment, previous prompts and interactions and tour own relevant knowledge if relevant. Don't be redundant on repeating previous prompts or interactions. For example, if you are prompted with 'ask: its name to the person' you should answer 'Hi, I'm Frida, what's your name?', or if prompted 'ask: quiz question', you should answer 'What questions do you wish to ask?'"
        }

        rospy.spin()

    def speech_callback(self, data: String) -> None:
        """Callback for the speech command subscriber"""
        self.prompts_context += f"{data.data} "
    
    def conversation_callback(self, goal: ConversateGoal) -> None:
        """Callback used for the conversation action server
        Args:
            goal (ConversateGoal): The request to be processed and responded to"""
        result = ConversateResult()
        #string_msg = String()
        #string_msg.data = self.process(goal.request)
        #self._pub_speak.publish(string_msg)
        speak_msg = Speak()
        speak_msg.text = self.process(goal.request)
        self.speak_client(speak_msg.text)
        time.sleep(1)

        result.success = 1
        self._conversation_as.set_succeeded(result)
        
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