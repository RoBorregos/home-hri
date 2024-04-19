import pandas as pd

# import csv
import os
import numpy as np  
import pickle
import tqdm

from sentence_transformers import SentenceTransformer, util

model = SentenceTransformer("all-MiniLM-L12-v2")

RESPONSES_PATH = 'StopCommands.csv'

with open(RESPONSES_PATH, 'r') as file:
    responses = pd.read_csv(file)

embeddings = []
progress_bar = tqdm.tqdm(total=responses.shape[0], desc="Embedding descriptions", unit="questions")

# print all IDs and their respective responses
for index, row in responses.iterrows():
    try:
        embeddings.append(model.encode(row, convert_to_tensor=True))
    except:
        print(f"Error at index {index}")
    progress_bar.update(1)

# save the embeddings to a file
if os.path.exists("stop_commands.pkl"):
    os.remove("stop_commands.pkl")
    
pickle.dump(embeddings, open("stop_commands.pkl", "wb"))
