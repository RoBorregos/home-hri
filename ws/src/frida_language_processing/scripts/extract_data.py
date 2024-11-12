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
import openai
from pydantic import BaseModel
import json

# Messages

# ROS topics
EXTRACT_DATA_SERVICE = "/extract_data"


class ExtractedData(BaseModel):
    data: Optional[str] = None


def extract_information():
    return {
        "name": "extract_information",
        "description": "Extract structured data including summary, keywords, and important points.",
        "parameters": ExtractedData.schema()
    }


class DataExtractor:
    """Class to encapsulate the guest analysis node"""

    def __init__(self) -> None:
        """Initialize the ROS node"""
        self._node = rospy.init_node("data_extractor")
        rospy.loginfo("Starting data extractor node")
        self._rate = rospy.Rate(10)

        # Service to extract information from the guest
        rospy.Service(EXTRACT_DATA_SERVICE, ExtractInfo,
                      self.extract_info_requested)

        # Objects and variables
        openai.api_key = os.getenv("OPENAI_API_KEY")
        self.openai_client = openai
        rospy.loginfo("Data extractor node started")
        rospy.spin()

    def extract_info_requested(self, request: ExtractInfo) -> ExtractInfoResponse:
        """Service to extract information from text."""

        rospy.loginfo("Extracting information from text")

        instruction = "You will be presented with some text and data to extract. Please provide the requested information or leave empty if it isn't available."
        response = openai.beta.chat.completions.parse(
            model="gpt-4o-2024-08-06",
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
