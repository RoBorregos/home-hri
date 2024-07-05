#!/usr/bin/env python3

"""
This ROS node interprets the commands received from the Speech processing,
using the fine-tuned model to send the actions to the Task Manager
"""

# Libraries
import rospy
import os
from openai import OpenAI

# Messages
from std_msgs.msg import String
from frida_hri_interfaces.msg import Command, CommandList

# ROS topics
SPEECH_COMMAND_TOPIC = "/speech/raw_command"
OUT_COMMAND_TOPIC = "/task_manager/commands"

# OpenAI constants
FINE_TUNED_MODEL = "ft:gpt-3.5-turbo-0125:personal:roborregos:9fX2cKIt"
SYSTEM_PROMPT = "You are a service robot for domestic applications. You were developed by a robotics team called RoBorregos, from Tec de Monterrey university. Now you are located in a house environment and we will give you general purpose tasks in the form of natural language. You have in your architecture the modules of: navigation, manipulation, person recognition, object detection and human-robot interaction. Your job is to understand the task and divide it into smaller actions proper to your modules, considering a logical flow of the actions along the time. In order to write it, you have to divide each subtask with a semicolon and separate the action and the complements with a coma. The result should be in the form of: 'action, complement; action, complement;', like 'do, x; do, y; do, z'. For example, for the prompt 'Locate a dish in the kitchen then get it and give it to Angel in the living room', the result would be: 'go, kitchen; find, dish; grab, dish; go, living room; find, Angel; approach, Angel; give, dish.'. Another example is, for the prompt: 'Tell me what is the biggest object on the tv stand' and its result will be 'remember, location; go, tv stand; identify, biggest + object; go, past location; interact, biggest object information.'. Don't add single quotes. Another important thing is that when we give you the general task, some complements are grouped in categories. For example: apple, banana and lemon are all of them in the fruits category; cola, milk and red wine are all of them in the drinks category. If we give you a task talking about an item category, do not change the category word. It is very important don't made up information not given explicitly. If you add new words, we will be disqualified. For example:  'navigate to the bedroom then locate a food'. The result will be: 'remember, location; go, bedroom; find, food; pick, food; go, past location; give, food.'. The last important thing is that you have to rememeber the name of the person, in case we are talking about someone specifically. An example for the prompt can be: 'Get a snack from the side tables and deliver it to Adel in the bedroom'. And de result will be: 'remember, location; go, side tables; find, snack; pick, snack; go, bedroom; find, Adel; approach Adel; give, snack.'. You can ask for clarification if the task is not clear enough."

class CommandInterpreter:
    def __init__(self) -> None:
        """Initialize the ROS node, subscribers and publishers"""
        self._node = rospy.init_node("command_interpreter")
        self._rate = rospy.Rate(10)
        self._sub = rospy.Subscriber(SPEECH_COMMAND_TOPIC, String, self._callback)
        self._pub = rospy.Publisher(OUT_COMMAND_TOPIC, CommandList, queue_size=10)

        ### Objects and variables
        self.openai_client = OpenAI(
            api_key=os.getenv("OPENAI_API_KEY")
        )

        rospy.spin()

    def _callback(self, data: String) -> None:
        """Callback for the speech command subscriber"""
        self.run(data.data)

    def parse_command(self, interpreted_command: String) -> CommandList:
        """Method for parsing the interpreted command into a list of commands"""
        command_list = CommandList()
        command_strings = interpreted_command.split(";") # Split the commands by semicolon
        for tmp_command in command_strings:
            command = Command()
            command.action = tmp_command.split(",")[0].strip() # Get the action
            command.characteristic = ""
            command.complement = tmp_command.split(",")[1].strip(" .") # Get the complement
            if "+" in command.complement:
                command.characteristic = command.complement.split("+")[0].strip() # Get the characteristic
                command.complement = command.complement.split("+")[1].strip() # Update the complement
            
            command_list.commands.append(command)
            rospy.loginfo(f"- {command.action}-> {command.complement} : {command.characteristic}")

        # @TODO: Implement embedding extraction?
        return command_list

    def run(self, raw_command: String) -> None:
        """Method for running the interpretation of the commands"""
        chat_completion = self.openai_client.chat.completions.create(
            model = FINE_TUNED_MODEL,
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": raw_command}
            ]
        )
        interpreted_command = chat_completion.choices[0].message.content
        rospy.logwarn(f"Command interpreted: {interpreted_command}")
        self._pub.publish(self.parse_command(interpreted_command))

if __name__ == "__main__":
    try:
        command_interpreter = CommandInterpreter()
    except rospy.ROSInterruptException as e:
        rospy.logerr("Error: {}".format(e))
        pass