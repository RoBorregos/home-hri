from openai import OpenAI
import pandas as pd
import numpy as np
from dotenv import load_dotenv
import os

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
KNOWN_ROOM = False # en la primera iteracion puede ser que no se sepa la ubicacion
ACTUAL_ROOM = 'bathroom' # Options: living_room, bedroom, kitchen, batroom -> IMPORTANTE ESTO DEBERIA IR CAMBIANDO SEGUN LA UBICACION DEL ROBOT

# Constants
SIMILARITY_THRESHOLD = 0.67 # Minimum similarity percentage in which 2 words are considered the same
CONFIDENCE_THRESHOLD = 0.5 # Grado de incertidumbre -> solicitar confirmacion del usuario

DATAFRAMES_DIR = "dataframes/"
ITEMS = "items"
LOCATIONS = "locations"
NAMES = "names"
ACTIONS = "actions"

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
def calculate_similarity(embeddings_input, data_frame, column_name): 

    data_frame['similarity'] = data_frame[column_name].apply(lambda x: np.dot(x, embeddings_input))
    data_frame_sorted = data_frame.sort_values('similarity', ascending=False)
    max_similarity = data_frame_sorted.iloc[0]['similarity']
    
    return data_frame_sorted, max_similarity

def get_action_similarity(embeddings_input):
    best_action = list()

    action_data_frame_sorted, action_similarity = calculate_similarity(embeddings_input, embeddings_data[ACTIONS], 'action_embedding')

    if action_similarity >= SIMILARITY_THRESHOLD:
        best_action.append(action_data_frame_sorted.iloc[0]['action'])

    elif action_similarity >= CONFIDENCE_THRESHOLD: # Si la confianza es baja, se solicita una confirmación al usuario
        user_response = get_user_confirmation(action_data_frame_sorted.iloc[0]['action'])
        if user_response.lower() != 'yes':
            print("** User didn't confirm the command. Cancelling the action **")
            return
        else:
            print("** Command confirmed **")
            best_action.append(action_data_frame_sorted.iloc[0]['action'])

    else:
        print("** ERROR: No similar action found **")

    print(best_action)

    return best_action


def get_complement_similarities(embeddings_input, data_frame):
    global KNOWN_ROOM, ACTUAL_ROOM

    best_complement = list()

    data_frame_namesorted, name_similarity = calculate_similarity(embeddings_input, data_frame, 'name_embedding')
    data_frame_categorysorted, category_similarity = calculate_similarity(embeddings_input, data_frame, 'category_embedding')

    if (name_similarity > category_similarity): # Coincide con un "name" especifico -> me devuelve el resultado con el "name" mas similar
        if name_similarity >= SIMILARITY_THRESHOLD: 
            best_complement.append(data_frame_namesorted.iloc[0]["name"])
        elif name_similarity >= CONFIDENCE_THRESHOLD: # Si la confianza es baja, se solicita una confirmación al usuario
            user_response = get_user_confirmation(data_frame_namesorted.iloc[0]["name"])
            if user_response.lower() == 'yes':
                print("** Command confirmed **")
                best_complement.append(data_frame_namesorted.iloc[0]["name"])
            else:
                print("** User didn't confirm the command. Cancelling the action **")
        else: 
            print("** ERROR: No similar complement found **")
            return
                     
    else: # En este caso (category_similarity > name_similarity). Coincide con una "category" especifica -> me devuelve una lista con todos los elementos de esa categoria
        detected_category = data_frame_categorysorted.iloc[0]['category']
        data_frame_detected_category = data_frame[data_frame['category'] == detected_category]

        if category_similarity >= SIMILARITY_THRESHOLD: 
            if KNOWN_ROOM: # se devuelve solo los elementos de esta categoria que estan en la habitacion en la que estamos
                filtered_by_category_and_room = data_frame_detected_category[data_frame_detected_category['room'] == ACTUAL_ROOM]["name"].tolist()
                best_complement = filtered_by_category_and_room
            else:
                filtered_by_category = data_frame[data_frame['category'] == detected_category]['name'].tolist()
                best_complement = filtered_by_category

        elif category_similarity >= CONFIDENCE_THRESHOLD: # Si la confianza es baja, se solicita una confirmación al usuario
            user_response = get_user_confirmation(detected_category)
            if user_response.lower() == 'yes':
                print("** Command confirmed **")
                if KNOWN_ROOM:
                    filtered_by_category_and_room = data_frame_detected_category[data_frame_detected_category['room'] == ACTUAL_ROOM]["name"].tolist()
                    best_complement = filtered_by_category_and_room
                else:
                    filtered_by_category = data_frame[data_frame['category'] == detected_category]['name'].tolist()
                    best_complement = filtered_by_category
            else:
                print("** User didn't confirm the command. Cancelling the action **")
        else: 
                print("** ERROR: No similar complement found **")
                return

    KNOWN_ROOM = True
    ACTUAL_ROOM = data_frame[data_frame['name'] == best_complement[0]]['room'].values[0]
    
    print(ACTUAL_ROOM)

    return best_complement

def create_embedding(item):
    response = get_embedding(item)

    return response

def count_words(input):
    words = input.split()
    word_count = len(words)

    return word_count

def handle_single_word_action(item):
    action_embedding = create_embedding(item)
    list_actions = get_action_similarity(action_embedding)
    
    if list_actions: # If it is not an empty list
        action = list_actions[0] # The first option will be the most similar

    return action

def handle_two_word_action(item):
    action, complement = item.split() 

    action_embedding = create_embedding(action)
    complement_embedding = create_embedding(complement)

    list_actions = get_action_similarity(action_embedding)

    # Based on the main action found, we will look in an specific dataFrame
    if list_actions: # If it is not an empty list
        action = list_actions[0] # The first option will be the most similar
        
        if action == "go" or action == "put": 
            if complement == "user":
                list_complements = [USER]
            else:
                list_complements = get_complement_similarities(complement_embedding, embeddings_data[LOCATIONS])
            
        elif action == "grab":
            list_complements = get_complement_similarities(complement_embedding, embeddings_data[ITEMS])

        elif action == "find":
            list_complements = get_complement_similarities(complement_embedding, embeddings_data[NAMES]) 
        
        print(list_complements)

    return action, list_complements

if __name__ == "__main__":

    entrada =  "Move to the sidetable, move to the kitchen counter and leave the apartment"

    entrada_fineTuned = fineTunning(entrada)  # It uses our fine tuned model of ChatGPT

    # split the user_input into smaller sections
    items = entrada_fineTuned.split(", ")

    for item in items: # for each section
        if count_words(item) == 1: # Handle single word action (Introduce)
            
            action = handle_single_word_action(item)

        elif count_words(item) == 2: # Other actions
            action, complement = handle_two_word_action(item)
            
        else:
            print("** ERROR IN COMMAND SPLITTING **")
        
            
