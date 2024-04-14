#!/usr/bin/env python3

"""
This ROS node interprets the commands received from the Speech processing,
using the fine-tuned model to send the actions to the Task Manager
"""

# Libraries
import rospy
import os
from openai import OpenAI
import pandas as pd
import numpy as np
import sys
import pathlib
import rospkg

# Messages
from std_msgs.msg import String
from frida_hri_interfaces.msg import Command, CommandList

# ROS topics
SPEECH_COMMAND_TOPIC = "/speech/raw_command"
OUT_COMMAND_TOPIC = "/task_manager/commands"

# OpenAI constants
FINE_TUNED_MODEL = "ft:gpt-3.5-turbo-0125:ixmatix:roborregos:9ArVJ9Nf"
SYSTEM_PROMPT = "You are a service robot for domestic applications. You were developed by RoBorregos team from Tec de Monterrey, from Mexico. You are given general purpose tasks in the form of natural language inside a house environment. You have in your architecture the modules of: navigation, manipulation, person recognition, object detection and human-robot interaction. Your job is to understand the task and divide it to actions proper to your modules, considering a logical flow of the actions. You can ask for clarification if the task is not clear enough. Try to abstract the verbs as much as possible. Divide each action with a semicolon. The actions should be in the form of: 'do x; do y; do z'. For example, for the prompt 'Locate a dish in the kitchen then get it and give it to Angel in the living room', the actions would be: 'go, kitchen; find, dish; grab, dish; go, living room; find, Angel; approach, Angel; give, dish.'. Another example is, for the prompt: 'Tell me what is the biggest object on the tv stand' and its actions are 'remember, location; go, tv stand; identify, biggest + object; go, past location; interact, biggest object information.'. Don't add single quotes"

# Embeddings constants
#DATAFRAMES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "/dataframes")
#DATAFRAMES_DIR = ""
rp = rospkg.RosPack()
DATAFRAMES_DIR = rp.get_path('frida_language_processing') + "/scripts/dataframes/" # Path to the dataframes ESTO HAY QUE CAMBIARLO!!!
ITEMS = "items"
LOCATIONS = "locations"
NAMES = "names"
ACTIONS = "actions"

#sys.path.append(str(pathlib.Path(__file__).parent) + '/dataframes')
class CommandInterpreter:
    """Class to interpret the commands received from the speech processing"""
    embeddings_data = {
        ACTIONS: pd.read_pickle(DATAFRAMES_DIR + "embeddings_actions.pkl"),
        LOCATIONS: pd.read_pickle(DATAFRAMES_DIR + "embeddings_locations.pkl"),
        ITEMS: pd.read_pickle(DATAFRAMES_DIR + "embeddings_items.pkl"),
        NAMES: pd.read_pickle(DATAFRAMES_DIR + "embeddings_names.pkl")
    }

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

        rospy.loginfo("Command Interpreter node initialized")
        rospy.spin()

    def _callback(self, data: String) -> None:
        """Callback for the speech command subscriber"""
        #if data.data == "" or not ("frida" in data.data or "Frida" in data.data):
        #   rospy.logwarn("Noise detected, ignoring")
        #   return
        self.run(data.data)

    def action_embeddings(self, action: str) -> str:
        """Method to analyze the embeddings of the action with cosine similarity"""
        target_action_embedding = self.openai_client.embeddings.create(
            input = [action],
            model = "text-embedding-3-small",
        ).data[0].embedding

        dataframe = CommandInterpreter.embeddings_data[ACTIONS]
        dataframe['similarity'] = dataframe['action_embedding'].apply( lambda x: np.dot(x, target_action_embedding) )
        sorted_dataframe = dataframe.sort_values(by='similarity', ascending=False)
        ## @TODO: Implement a threshold for the similarity
        return sorted_dataframe.iloc[0]['action']

    def complement_embeddings(self, action: str, complement: str) -> str:
        """Method to analyze the embeddings of the complements"""
        target_complement_embedding = complement
        if action in ("pick", "place", "grasp", "give", "pour"):
            target_complement_embedding = self.openai_client.embeddings.create(
                input = [complement],
                model = "text-embedding-3-small",
            ).data[0].embedding

            dataframe = CommandInterpreter.embeddings_data[ITEMS]
            dataframe['similarity'] = dataframe['name_embedding'].apply( lambda x: np.dot(x, target_complement_embedding) )
            sorted_dataframe = dataframe.sort_values(by='similarity', ascending=False)
            return sorted_dataframe.iloc[0]['name']
        if action == "go":
            target_complement_embedding = self.openai_client.embeddings.create(
                input = [complement],
                model = "text-embedding-3-small",
            ).data[0].embedding

            dataframe = CommandInterpreter.embeddings_data[LOCATIONS]
            dataframe['similarity'] = dataframe['name_embedding'].apply( lambda x: np.dot(x, target_complement_embedding) )
            sorted_dataframe = dataframe.sort_values(by='similarity', ascending=False)
            return sorted_dataframe.iloc[0]['name']
        return complement

    def parse_command(self, interpreted_command: String) -> CommandList:
        """Method for parsing the interpreted command into a list of commands"""
        command_list = CommandList()
        command_strings = interpreted_command.split(";") # Split the commands by semicolon
        for tmp_command in command_strings:
            command = Command()
            command.action = self.action_embeddings(
                tmp_command.split(",")[0].strip()
            ) # Get the action to execute
            command.characteristic = ""
            command.complement = tmp_command.split(",")[1].strip(" .") # Get the complement
            if "+" in command.complement:
                command.characteristic = command.complement.split("+")[0].strip() # Get the characteristic
                command.complement = command.complement.split("+")[1].strip() # Update the complement

            command.complement = self.complement_embeddings(command.action, command.complement)
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
        rospy.logerr(f'Error: {e}')
