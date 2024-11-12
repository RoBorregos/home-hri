#!/usr/bin/env python3

"""
This ROS node interprets the commands received from the Speech processing,
using the fine-tuned model to send the actions to the Task Manager
"""

# Libraries
from typing import Optional, List
import rospy
import os
import openai
from pydantic import BaseModel, Field
import json

# Messages
from std_msgs.msg import String
from frida_hri_interfaces.msg import Command, CommandList
from assets.prompts import get_system_prompt_ci_v2

# ROS topics
SPEECH_COMMAND_TOPIC = "/speech/raw_command"
OUT_COMMAND_TOPIC = "/task_manager/commands"


class CommandShape(BaseModel):
    action: str = Field(description="The action to be performed")
    characteristic: Optional[str] = Field(
        description="A characteristic related to the action")
    complement: Optional[str] = Field(
        description="A complement related to the action")


class CommandListShape(BaseModel):
    commands: List[CommandShape]


class CommandInterpreter:
    def __init__(self) -> None:
        """Initialize the ROS node, subscribers and publishers"""
        self._node = rospy.init_node("command_interpreter")
        self._rate = rospy.Rate(10)
        self._sub = rospy.Subscriber(
            SPEECH_COMMAND_TOPIC, String, self._callback)
        self._pub = rospy.Publisher(
            OUT_COMMAND_TOPIC, CommandList, queue_size=10)

        openai.api_key = os.getenv("OPENAI_API_KEY")
        rospy.loginfo("Initialized Command interpreter v2")

        rospy.spin()

    def _callback(self, data: String) -> None:
        """Callback for the speech command subscriber"""
        self.run(data.data)

    def run(self, raw_command: String) -> None:
        """Method for running the interpretation of the commands"""
        response = openai.beta.chat.completions.parse(
            model="gpt-4o-2024-08-06",
            messages=[
                {"role": "system", "content": get_system_prompt_ci_v2()},
                {"role": "user", "content": raw_command}
            ],
            response_format=CommandListShape
        ).choices[0].message.content

        try:
            response_data = json.loads(response)
            result = CommandListShape(**response_data)
        except Exception as e:
            rospy.logerr(f"Service error: {e}")
            raise rospy.ServiceException(str(e))  # Propagate error to client

        rospy.logwarn(f"Commands interpreted: {result.commands}")

        command_list = CommandList()
        command_list.commands = [Command(action=command.action, characteristic=command.characteristic,
                                         complement=command.complement) for command in result.commands]
        self._pub.publish(command_list)


if __name__ == "__main__":
    try:
        command_interpreter = CommandInterpreter()
    except rospy.ROSInterruptException as e:
        rospy.logerr("Error: {}".format(e))
        pass
