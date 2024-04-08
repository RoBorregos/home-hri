#!/usr/bin/env python3
from openai import OpenAI
import rospy
import pandas as pd
import numpy as np
from dotenv import load_dotenv
import os
from std_msgs.msg import String
from prueba_hri.msg import command, list_of_commands # ESTO HAY QUE CAMBIARLO (nombre del paquete)

''' IMPORTANTE
        Find + person
        Grab + item
        Go/Put + location
        Introduce (va solo)
'''

# Load the environment variables
load_dotenv()

# Client setup
client = OpenAI(
    api_key=os.getenv("OPENAI_API_KEY")
)

# INICIALIZATION PARAMETERS
USER = 'Marina'

KNOWN_ROOM = False # in the first iteration maybe the robot doesn't know where it is
ACTUAL_ROOM = None # Options: living_room, bedroom, kitchen, batroom

# Constants
SIMILARITY_THRESHOLD = 0.67 # Minimum similarity percentage in which 2 words are considered the same
CONFIDENCE_THRESHOLD = 0.5 # ask for user confirmation 

DATAFRAMES_DIR = "/home/marina/catkin_ws/src/prueba_hri/dataframes/" # Path to the dataframes ESTO HAY QUE CAMBIARLO!!!
ITEMS = "items"
LOCATIONS = "locations"
NAMES = "names"
ACTIONS = "actions"

# Topics
RAW_TEXT_INPUT_TOPIC = "/RawInput"
COMMAND_TOPIC = "/speech/processed_commands"

# Load embeddings dataframes
embeddings_data = {
    ACTIONS: pd.read_pickle(os.path.join(DATAFRAMES_DIR, "embeddings_actions.pkl")),
    LOCATIONS: pd.read_pickle(os.path.join(DATAFRAMES_DIR, "embeddings_locations.pkl")),
    ITEMS: pd.read_pickle(os.path.join(DATAFRAMES_DIR, "embeddings_items.pkl")),
    NAMES: pd.read_pickle(os.path.join(DATAFRAMES_DIR, "embeddings_names.pkl"))
}

def fineTunning(petition):
    petition = petition.lower().strip().capitalize()

    context = {"role": "system",
                "content": "Command System parser. Filter the command from the input and Parse different commands in the text, separate them in comma separated commands that contain a main verb (go, grab, find, introduce, put) followed by the object or place or person"}
    messages = [context]
    messages.append({"role": "user", "content": petition})
    
    response = client.chat.completions.create(
        model="ft:gpt-3.5-turbo-0613:personal:teus-bot-bueno-2:8P5sfSyZ",
        messages=messages
    )
    response_content = response.choices[0].message.content

    messages.append({"role": "assistant", "content": response_content})

    print("Fine tuned sentence: ", response_content)

    return response_content

# Embedding function
def get_embedding(text, model="text-embedding-3-small"):
   emb = client.embeddings.create(input = [text], model=model).data[0].embedding
   
   return emb

def get_user_confirmation(doubt):
    print(f"** WARNING! Did you mean {doubt}?? ** ")
    user_response = input("Please, confirm with 'yes' or 'no': ")
    
    return user_response

# Calculate similarity between embeddings_input and embeddings in data_frame
def calculate_similarity(embeddings_input, df, column_name): 

    df['similarity'] = df[column_name].apply(lambda x: np.dot(x, embeddings_input))
    df_sorted = df.sort_values('similarity', ascending=False)
    max_similarity = df_sorted.iloc[0]['similarity']
    
    return df_sorted, max_similarity

def get_action_similarity(embeddings_input):
    best_action = list()

    action_df_sorted, action_similarity = calculate_similarity(embeddings_input, embeddings_data[ACTIONS], 'action_embedding')

    # Check confidence
    if action_similarity >= SIMILARITY_THRESHOLD:
        best_action.append(action_df_sorted.iloc[0]['action'])

    elif action_similarity >= CONFIDENCE_THRESHOLD: # Si la confianza es baja, se solicita una confirmación al usuario
        user_response = get_user_confirmation(action_df_sorted.iloc[0]['action'])
        if user_response.lower() != 'yes':
            print("** User didn't confirm the command. Cancelling the action **")
            return
        else:
            print("** Command confirmed **")
            best_action.append(action_df_sorted.iloc[0]['action'])

    else:
        print("** ERROR: No similar action found **")

    #print(best_action)

    return best_action

def move_to_another_room(rooms):
    global KNOWN_ROOM, ACTUAL_ROOM

    rospy.loginfo("Room options: %s", rooms)
    for r in rooms:
        rospy.loginfo("Do you want to move to the %s (yes/no)?: ", r)
        decision = input().lower()
        if decision == "yes":
            new_room = r
            
            comands = list()
            go_command = command()
            go_command.action = "go"
            go_command.complements = [new_room]

            comands.append(go_command)

            KNOWN_ROOM = True
            ACTUAL_ROOM = new_room

            rospy.loginfo("Actual room: %s", ACTUAL_ROOM)
            publisher_commands = rospy.Publisher(COMMAND_TOPIC, list_of_commands, queue_size=10)
            publisher_commands.publish(comands)
            
            return
        else:
            rospy.loginfo("Ok, I will not move to the %s", r)
        

def find_name_in_room(name, df, room):
    name_in_actual_room = (ACTUAL_ROOM == room) and (name in df[df['room'] == ACTUAL_ROOM]['name'].values)
    name_found = list()

    if KNOWN_ROOM: # Robot location is known
        if name_in_actual_room:
            name_found.append(name)
        else:
            rospy.loginfo("I have not found any %s in this room, but I remember it in another room", name)
            move_to_another_room([room])
            name_found.append(name)

    else: # Robot location is unknown -> We ask where to go
        rospy.loginfo("I know I can find %s in another room", name)
        move_to_another_room([room])
        name_found.append(name)

    return name_found

def find_category_in_room(category, df, rooms):
    category_in_actual_room = (ACTUAL_ROOM in rooms) and (category in df[df['room'] == ACTUAL_ROOM]['category'].values)
    best_complement = list()

    if KNOWN_ROOM: # Robot location is known
        if category_in_actual_room:
            df_filtered_by_category_and_room = df[df['room'] == ACTUAL_ROOM]["name"].tolist()
            best_complement = df_filtered_by_category_and_room
        else:
            rospy.loginfo("I have not found any %s in this room, but I remember it another room", category)
            move_to_another_room(rooms)
            df_filtered_by_category_and_room = df[df['room'] == ACTUAL_ROOM]["name"].tolist() # Con la room actualizada
            best_complement = df_filtered_by_category_and_room
        
    else: # Robot location is unknown -> We ask where to go
        rospy.loginfo("I know I can find %s in another room", category )
        move_to_another_room(rooms)
        df_filtered_by_category_and_room = df[df['room'] == ACTUAL_ROOM]["name"].tolist() # Con la room actualizada
        best_complement = df_filtered_by_category_and_room

    return best_complement


def get_complement_similarities(embeddings_input, df):
    best_complement = list()

    df_namesorted, name_similarity = calculate_similarity(embeddings_input, df, 'name_embedding')
    df_categorysorted, category_similarity = calculate_similarity(embeddings_input, df, 'category_embedding')

    if (name_similarity > category_similarity):
        room = df_namesorted.iloc[0]['room'] # Detected room
        name = df_namesorted.iloc[0]['name'] # Detected name

        # Check confidence
        if name_similarity >= SIMILARITY_THRESHOLD: 
            best_complement = find_name_in_room(name, df_namesorted, room)

        elif name_similarity >= CONFIDENCE_THRESHOLD:
            user_response = get_user_confirmation(name)
            if user_response.lower() == 'yes':
                print("** Command confirmed **")
                best_complement = find_name_in_room(name, df_namesorted, room)
            else:
                print("** User didn't confirm the command. Cancelling the action. Please try again **")
                return []
        else: 
            print("** SORRY I can not understand you **")
            return []    
                         
    else: # (category_similarity > name_similarity) -> devuelve una lista con todos los elementos de esa categoria
        category = df_categorysorted.iloc[0]['category']    # Detected category
        df_category = df[df['category'] == category]        # Dataframe with only the elements of the detected category
        rooms = df_category['room'].values                  # List of rooms with elements of the detected category
        rooms = list(set(rooms))                            # Eliminate duplicates

        # Check confidence
        if category_similarity >= SIMILARITY_THRESHOLD: 
            best_complement = find_category_in_room(category, df_category, rooms)

        elif category_similarity >= CONFIDENCE_THRESHOLD: # Si la confianza es baja, se solicita una confirmación al usuario
            user_response = get_user_confirmation(category)
            if user_response.lower() == 'yes':
                print("** Command confirmed **")
                best_complement = find_category_in_room(category, df_category, rooms)
            else:
                print("** User didn't confirm the command. Cancelling the action. Please try again**")
                return []
            
        else: 
            print("** SORRY I can not understand you **")
            return []
   
    return best_complement

def create_embedding(item):
    response = get_embedding(item)

    return response

def count_words(input):
    words = input.split()
    word_count = len(words)

    return word_count

def handle_action(item):
    action_embedding = create_embedding(item)
    list_actions = get_action_similarity(action_embedding)
    
    if list_actions: # If it is not an empty list
        action = list_actions[0] # The first option will be the most similar

    return action

def handle_complement(action, complement):
    global KNOWN_ROOM, ACTUAL_ROOM

    list_complements = list()

    complement_embedding = create_embedding(complement)

    # Based on the main action found, we will look in an specific dataFrame
    if complement == "user":
            list_complements = [USER]
    else:
        if action == "go" or action == "put": 
            list_complements = get_complement_similarities(complement_embedding, embeddings_data[LOCATIONS])

        elif action == "grab":
            list_complements = get_complement_similarities(complement_embedding, embeddings_data[ITEMS])

        elif action == "find":
            if complement.capitalize() in embeddings_data[NAMES]['name'].values:
                list_complements.append(complement)
                KNOWN_ROOM = False
                ACTUAL_ROOM = None
            else:
                rospy.loginfo("I'm sorry, I don't know that person")
            
        #print(list_complements)

    return list_complements

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)

    entrada = data.data
    entrada_fineTuned = fineTunning(entrada)  # It uses our fine tuned model of ChatGPT

    # split the user_input into smaller sections
    items = entrada_fineTuned.split(", ")

    for item in items: # for each section, we will split it into action and complement
        comands = list() # List of commands to be sent to the robot (action, complement trough ROS)

        if count_words(item) == 1: # Handle single word action (Introduce)
            action = handle_action(item)
            com = command()
            com.action = action
            com.complements = [""]
            comands.append(com)

        elif count_words(item) == 2: # Other actions
            action, complement = item.split()
            action = handle_action(action)
            complement = handle_complement(action, complement)
            com = command()
            com.action = action
            com.complements = complement
            comands.append(com)
            
        else:
            print("** ERROR IN COMMAND SPLITTING **")

        publisher_commands = rospy.Publisher(COMMAND_TOPIC, list_of_commands, queue_size=10)
        publisher_commands.publish(comands)    


if __name__ == "__main__":
    rospy.init_node('TEXT_ANALYSIS_NODE', anonymous=True)
    rospy.Subscriber(RAW_TEXT_INPUT_TOPIC, String, callback)
    rospy.loginfo("HRI analysis started")
    rospy.spin()
