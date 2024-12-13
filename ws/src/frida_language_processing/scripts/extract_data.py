#!/usr/bin/env /usr/bin/python3.10

"""
Python ROS node to extract information from text
"""

# Libraries
from frida_hri_interfaces.srv import ExtractInfo, ExtractInfoResponse
from frida_hri_interfaces.msg import GuestAnalysisAction, GuestAnalysisFeedback, GuestAnalysisGoal, GuestAnalysisResult
from std_msgs.msg import String
from typing import Optional
import rospy
import os
from openai import OpenAI
from pydantic import BaseModel
import json

# Messages

# ROS topics
EXTRACT_DATA_SERVICE = "/extract_data"


class ExtractedData(BaseModel):
    data: Optional[str] = None


class DataExtractor:
    """Class to encapsulate the guest analysis node"""

    def __init__(self) -> None:
        """Initialize the ROS node"""
        self._node = rospy.init_node("data_extractor")
        rospy.loginfo("Starting data extractor node")

        base_url = rospy.get_param("~base_url", None)
        self.model = rospy.get_param("~model", "gpt-4o-2024-08-06")

        if base_url == "None":
            base_url = None

        self.client = OpenAI(api_key=os.getenv(
            "OPENAI_API_KEY", "ollama"), base_url=base_url)

        # Service to extract information from the guest
        rospy.Service(EXTRACT_DATA_SERVICE, ExtractInfo,
                      self.extract_info_requested)

        rospy.loginfo("Data extractor node started")
        self._rate = rospy.Rate(10)
        rospy.spin()

    def extract_info_requested(self, request: ExtractInfo) -> ExtractInfoResponse:
        """Service to extract information from text."""

        rospy.loginfo("Extracting information from text")

        instruction = "You will be presented with some text and data to extract. Please provide the requested information or leave empty if it isn't available."
        response = self.client.beta.chat.completions.parse(
            model=self.model,
            temperature=0,
            messages=[
                {"role": "system", "content": instruction},
                {"role": "user", "content": str(request.full_text) +
                    "Data to extract: " + str(request.data)}
            ],
            response_format=ExtractedData
        ).choices[0].message.content

        rospy.loginfo(f"Extracted data: {response}")

        try:
            response_data = json.loads(response)
            result = ExtractedData(**response_data)
        except Exception as e:
            rospy.logerr(f"Service error: {e}")
            raise rospy.ServiceException(str(e))  # Propagate error to client

        return ExtractInfoResponse(result=result.data)


if __name__ == "__main__":
    try:
        DataExtractor()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f"Error: {e}")
