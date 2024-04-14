#!/usr/bin/env python3
"""
Python ROS node to analyze and extract information from the guest.
For the Receptionist task  of Stage 1
"""

# Libraries
import rospy
import os
from openai import OpenAI
import actionlib
import time

# Messages
from std_msgs.msg import String
from frida_hri_interfaces.msg import GuestAnalysisAction, GuestAnalysisFeedback, GuestAnalysisGoal, GuestAnalysisResult
from frida_hri_interfaces.srv import GuestInfo, GuestInfoResponse

# ROS topics
SPEECH_COMMAND_TOPIC = "/speech/raw_command"
SPEAK_TOPIC = "/speech/speak"
SPEAK_NOW_TOPIC = "/speech/speak_now"
GUEST_INFO_SERVICE = "/guest_info"
GUEST_ANALYSIS_SERVER = "/guest_analysis_as"

# Environment static context
ORIGINS_CONTEXT = "You are a service robot for domestic applications called Frida. You were developed by RoBorregos team from Tec de Monterrey, from Mexico."
DATE_CONTEXT = "Today is Thursday, April 11th, 2024. It's 02:20"
LOCATION_CONTEXT = "You are in the RoBorregos lab."
ENVIRONMENT_CONTEXT = f"{ORIGINS_CONTEXT} {DATE_CONTEXT} {LOCATION_CONTEXT}"

class GuestAnalyzer:
    """Class to encapsulate the guest analysis node"""
    def __init__(self) -> None:
        """Initialize the ROS node"""
        self._node = rospy.init_node("guest_analyzer")
        self._rate = rospy.Rate(10)
        self._sub_speech = rospy.Subscriber(SPEECH_COMMAND_TOPIC, String, self.speech_callback)

        ## Service to extract information from the guest
        rospy.Service(GUEST_INFO_SERVICE, GuestInfo, self.guest_info_requested)

        ### Action server for the image analysis
        self.analysis_as = actionlib.SimpleActionServer(
            GUEST_ANALYSIS_SERVER, GuestAnalysisAction, execute_cb=self.analysis_callback, auto_start=False
        )
        self.analysis_as.start()

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

        self.detected_speech = ""

        rospy.spin()

    def speech_callback(self, data: String) -> None:
        """Callback for the speech command subscriber"""
        self.detected_speech = data.data

    def guest_info_requested(self, request: GuestInfo) -> GuestInfoResponse:
        """Service to extract information from the guest"""
        rospy.loginfo(f"Guest #{request.guest_id} info requested")
        while self.detected_speech == "":
            rospy.loginfo("Waiting for speech input")
            time.sleep(0.5)

        instruction = "You are a service robot for domestic applications, you are currently attending guests and you need their name and favorite drink information. Provide the guest's name and favorite drink in the format 'name, drink', all lowercase letters. For example if you receive 'My name is Charlie and I like coffee', you should answer 'charlie, coffee'. Or if you receive 'I'm Adan and my favorite drink is water', you should answer 'adan, water'. If you are unable to detect the guest's name or favorite drink, you should answer 'error'." 
        text_completion = self.openai_client.chat.completions.create(
            model = "gpt-4",
            messages=[
                {"role": "system", "content": instruction},
                {"role": "user", "content": self.detected_speech}
            ]
        ).choices[0].message.content

        response = GuestInfoResponse()
        if "error" in text_completion.lower():
            response.success = False
            return response

        guest_name = text_completion.split(",")[0].strip().capitalize() # Extract the guest name
        guest_drink = text_completion.split(",")[1].strip(' .').lower() # Extract the guest favorite drink
        response.name = guest_name
        response.favorite_drink = guest_drink
        response.success = True

        self.detected_speech = ""

        return response
    
    def analysis_callback(self, goal: GuestAnalysisGoal) -> None:
        """Callback for the image analysis action server"""
        rospy.loginfo("Guest analysis action server")
        self.prompts_context = goal.prompts_context
        self.interactions_context = goal.interactions_context

        if goal.command not in self.interaction_guide:
            rospy.logerr("Command not found")
            self.analysis_as.set_aborted()
            return

        text_completion = self.openai_client.chat.completions.create(
            model = "gpt-4",
            messages=[
                {"role": "system", "content": self.interaction_guide[goal.command]},
                {"role": "user", "content": self.interactions_context}
            ]
        ).choices[0].message.content

        result = GuestAnalysisResult()
        result.interactions_context = text_completion
        self.analysis_as.set_succeeded(result)

if __name__ == "__main__":
    try:
        GuestAnalyzer()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f"Error: {e}")
