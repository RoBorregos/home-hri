#!/usr/bin/env python3
from openai import OpenAI
import rospy
import pandas as pd
import numpy as np
from dotenv import load_dotenv
import os
from std_msgs.msg import String
from prueba_hri.msg import command, list_of_commands # ESTO HAY QUE CAMBIARLO (nombre del paquete)

# Load the environment variables
load_dotenv()

# Client setup
client = OpenAI(
    api_key=os.getenv("OPENAI_API_KEY")
)

KNOWN_ROOM = False # in the first iteration maybe the robot doesn't know where it is
ACTUAL_ROOM = None # Options: living_room, bedroom, kitchen, batroom

# Constants
SIMILARITY_THRESHOLD = 0.5 # Minimum similarity percentage in which 2 words are considered the same
CONFIDENCE_THRESHOLD = 0.4 # ask for user confirmation 

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
                "content": "You are a service robot for domestic applications. You were developed by RoBorregos team from Tec de Monterrey, from Mexico. You are given general purpose tasks in the form of natural language inside a house environment. You have in your architecture the modules of: navigation, manipulation, person recognition, object detection and human-robot interaction. Your job is to understand the task and divide it to actions proper to your modules, considering a logical flow of the actions. You can ask for clarification if the task is not clear enough. Try to abstract the verbs as much as possible. Divide each action with a semicolon. The actions should be in the form of: 'do x; do y; do z'. For example, for the prompt 'Locate a dish in the kitchen then get it and give it to Angel in the living room', the actions would be: 'go, kitchen; find, dish; grab, dish; go, living room; find, Angel; approach, Angel; give, dish.'. Another example is, for the prompt: 'Tell me what is the biggest object on the tv stand' and its actions are 'remember, location; go, tv stand; identify, biggest + object; go, past location; interact, biggest object information.'. Don't add single quotes"}
    messages = [context]
    messages.append({"role": "user", "content": petition})
    messages.append({"role": "assistant", "content": "remember, location; go, shelf; find, iced tea; pick, iced tea; go, past location; give, iced tea."})
    
    response = client.chat.completions.create(
        model="ft:gpt-3.5-turbo-0125:ixmatix:roborregos:9ArVJ9Nf",
        messages=messages
    )
    response_content = response.choices[0].message.content

    messages.append({"role": "assistant", "content": response_content})

    #print("Fine tuned sentence: ", response_content)

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

    return best_action
        

def get_item_similarities(embeddings_input, df):
    best_item = list()

    df_namesorted, name_similarity = calculate_similarity(embeddings_input, df, 'name_embedding')
    df_categorysorted, category_similarity = calculate_similarity(embeddings_input, df, 'category_embedding')

    if (name_similarity > category_similarity):
        name = df_namesorted.iloc[0]['name'] # Detected name

        # Check confidence
        if name_similarity >= SIMILARITY_THRESHOLD: 
            best_item.append(name)

        elif name_similarity >= CONFIDENCE_THRESHOLD:
            user_response = get_user_confirmation(name)
            if user_response.lower() == 'yes':
                print("** Command confirmed **")
                best_item.append(name)
            else:
                print("** User didn't confirm the command. Cancelling the action. Please try again **")
                return []
        else: 
            print("** SORRY I can not understand you **")
            return []    
                         
    else: # (category_similarity > name_similarity) -> devuelve una lista con todos los elementos de esa categoria
        category = df_categorysorted.iloc[0]['category']    # Detected category

        # Check confidence
        if category_similarity >= SIMILARITY_THRESHOLD: 
            best_item.append(category)

        elif category_similarity >= CONFIDENCE_THRESHOLD: # Si la confianza es baja, se solicita una confirmación al usuario
            user_response = get_user_confirmation(category)
            if user_response.lower() == 'yes':
                print("** Command confirmed **")
                best_item.append(category)
            else:
                print("** User didn't confirm the command. Cancelling the action. Please try again**")
                return []
            
        else: 
            print("** SORRY I can not understand you **")
            return []
   
    return best_item

def get_location_similarities(embeddings_input, df):
    best_loc = list()

    df_namesorted, name_similarity = calculate_similarity(embeddings_input, df, 'name_embedding')
    name = df_namesorted.iloc[0]['name']

    # Check confidence
    if name_similarity >= SIMILARITY_THRESHOLD: 
        best_loc.append(name)

    elif name_similarity >= CONFIDENCE_THRESHOLD:
        user_response = get_user_confirmation(name)
        if user_response.lower() == 'yes':
            print("** Command confirmed **")
            best_loc.append(name)
        else:
            print("** User didn't confirm the command. Cancelling the action. Please try again **")
            return []
    else: 
        print("** SORRY I can not understand you **")
        return []    

    return best_loc


def create_embedding(item):
    response = get_embedding(item)

    return response


def handle_action(action):
    action_embedding = create_embedding(action)
    list_actions = get_action_similarity(action_embedding)
    
    if list_actions: # If it is not an empty list
        action = list_actions[0] # The first option will be the most similar

    return action

def handle_complement(action, complement):
    global KNOWN_ROOM, ACTUAL_ROOM

    list_complements = list()

    complement_embedding = create_embedding(complement)

    # Based on the main action found, we will look in an specific dataFrame
    if action == "go": # go + place
        if complement == "past location":
            list_complements.append(complement)
        else:
            list_complements = get_location_similarities(complement_embedding, embeddings_data[LOCATIONS])
            
    elif action == "find": # find + item or item category
        if complement.capitalize() in embeddings_data[NAMES]['name'].values:
            list_complements.append(complement)
        else:
            list_complements = get_item_similarities(complement_embedding, embeddings_data[ITEMS])

    elif action == "identify": # identify + item
        list_complements.append(complement)

    elif action == "count":
        list_complements.append(complement)

    elif action == "approach" or action == "follow": # approach / follow + person
        if complement == "tracker":
            list_complements.append(complement)
        elif complement.capitalize() in embeddings_data[NAMES]['name'].values:
            list_complements.append(complement)
        else:
            rospy.loginfo("I'm sorry, I don't know that person")
            #print("I'm sorry, I don't know that person")
    
    elif action == "interact" or action == "ask": # interact /ask + request
        list_complements.append(complement)

    elif action == "pick" or action == "place" or action == "grab" or action == "give": # pick / place / grab / give + item
        list_complements = get_item_similarities(complement_embedding, embeddings_data[ITEMS])
   
    elif action == "open" or action == "close":  # open / close + entrance
        list_complements.append(complement)

    elif action == "remember": # something
        list_complements.append(complement)
    
    #print(list_complements)

    return list_complements

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)

    entrada = data.data
    entrada_fineTuned = fineTunning(entrada)  # It uses our fine tuned model of ChatGPT
    
    print("Fine tuned sentence: ", entrada_fineTuned)

    # split the user_input into smaller sections
    items = entrada_fineTuned.split("; ")
    
    for item in items: # for each section, we will split it into action and complement
        comands = list() # List of commands to be sent to the robot (action, complement trough ROS)

        action, complement = item.split(", ")
        action = handle_action(action)
        print(action)
        complement = handle_complement(action, complement)
        print(complement)
        com = command()
        com.action = action
        com.complements = complement
        comands.append(com)
        
        publisher_commands = rospy.Publisher(COMMAND_TOPIC, list_of_commands, queue_size=10)
        publisher_commands.publish(comands)    
        


if __name__ == "__main__":
    rospy.init_node('TEXT_ANALYSIS_NODE', anonymous=True)
    rospy.Subscriber(RAW_TEXT_INPUT_TOPIC, String, callback)
    rospy.loginfo("HRI analysis started")
    rospy.spin()
    