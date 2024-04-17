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
from cv_bridge import CvBridge
import rospkg
import base64
import cv2

# Messages
from sensor_msgs.msg import Image
from std_msgs.msg import String
from frida_hri_interfaces.msg import GuestAnalysisAction, GuestAnalysisFeedback, GuestAnalysisGoal, GuestAnalysisResult
from frida_hri_interfaces.srv import GuestInfo, GuestInfoResponse, STT

# ROS topics
SPEECH_COMMAND_TOPIC = "/speech/raw_command"
SPEAK_TOPIC = "/speech/speak"
SPEAK_NOW_TOPIC = "/speech/speak_now"
GUEST_INFO_SERVICE = "/guest_info"
GUEST_ANALYSIS_SERVER = "/guest_analysis_as"
CAMERA_TOPIC = "/zed2/zed_node/rgb/image_rect_color"
STT_SERVICE_TOPIC = "/speech/STT"

class GuestAnalyzer:
    """Class to encapsulate the guest analysis node"""
    def __init__(self) -> None:
        """Initialize the ROS node"""
        self._node = rospy.init_node("guest_analyzer")
        self._rate = rospy.Rate(10)
        #self._sub_speech = rospy.Subscriber(SPEECH_COMMAND_TOPIC, String, self.speech_callback)
        self.bridge = CvBridge()

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
        self.detected_speech = ""

        rospy.spin()

    def speech_callback(self, data: String) -> None:
        """Callback for the speech command subscriber"""
        self.detected_speech = data.data

    def guest_info_requested(self, request: GuestInfo) -> GuestInfoResponse:
        """Service to extract information from the guest"""
        rospy.loginfo(f"Guest #{request.guest_id} info requested")

        try:
            stt_client = rospy.ServiceProxy(STT_SERVICE_TOPIC, STT)
            response = stt_client()
            self.detected_speech = response.text_heard
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return GuestInfoResponse(success=False)

        """
        while self.detected_speech == "":
            rospy.loginfo("Waiting for speech input")
            time.sleep(0.5)
        """

        instruction = "You are a service robot for domestic applications, you are currently attending guests and you need their name and favorite drink information. Provide the guest's name and favorite drink in the format 'name, drink', all lowercase letters. For example if you receive 'My name is Charlie and I like coffee', you should answer 'charlie, coffee'. Or if you receive 'I'm Adan and my favorite drink is water', you should answer 'adan, water'. If you are unable to detect the guest's name or favorite drink, you should answer 'error'." 
        text_completion = self.openai_client.chat.completions.create(
            model = "gpt-4",
            messages=[
                {"role": "system", "content": instruction},
                {"role": "user", "content": self.detected_speech}
            ],
            temperature=0.2
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
        rospy.loginfo(f"Visual analysis for guest {goal.guest_id}")
        picture = rospy.wait_for_message(CAMERA_TOPIC, Image, timeout=10.0) # Wait for the picture
        rospy.loginfo("Picture taken for guest analysis")

        ### Image encoding for the input
        cv_image = self.bridge.imgmsg_to_cv2(picture, "bgr8")
        buffer = cv2.imencode('.jpg', cv_image)[1]
        encoded_image = base64.b64encode(buffer).decode('utf-8')
        
        ### Image-to-text model format
        instruction = "You are a service robot for domestic applications, you are currently attending guests and you have to introduce the person in the image to the other guests in the room. You have to name visible physical characteristics of the person in the image. The characteristics you could find are: color of clothes, color of hair and characteristic features. Write the characteristics in a short way and don't be verbose, only create a small paragraph telling each of them. For example 'It's a young adult, with blonde hair, wearing a light blue t-shirt and white pants. Wears a silver collar', or 'It has gray hair, is wearing a black suit and has a beard'. Don't include information about the environment or mention the image, only the main person."
        prompt = {
            "role": "user",
            "content": [
                {"type": "text", "text": f"{instruction}"},
                {
                    "type": "image_url",
                    "image_url": f"data:image/jpeg;base64,{encoded_image}"
                },
            ],
        }

        # Send request to OpenAI API
        guest_description = self.openai_client.chat.completions.create(
            model="gpt-4-vision-preview",
            messages=[prompt]
        ).choices[0].message.content

        result = GuestAnalysisResult()
        result.guest_id = goal.guest_id
        result.description = guest_description
        self.analysis_as.set_succeeded(result)

if __name__ == "__main__":
    try:
        GuestAnalyzer()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f"Error: {e}")
