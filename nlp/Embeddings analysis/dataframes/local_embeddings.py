import pandas as pd

# import csv
import os
import numpy as np  
import pickle
import tqdm
import pathlib
from sentence_transformers import SentenceTransformer, util

BASE_PATH = pathlib.Path(__file__).parent.absolute()
model = SentenceTransformer("all-MiniLM-L12-v2")

# Modify the categories csv the add new categories
RESPONSES_PATH = os.path.join(BASE_PATH, "categories.csv")

TESTING = True

with open(RESPONSES_PATH, 'r') as file:
    responses = pd.read_csv(file)

print(responses.head())

category_embeddings = {}
progress_bar = tqdm.tqdm(total=responses.shape[0], desc="Embedding descriptions", unit="category")
# print all IDs and their respective responses
for index, row in responses.iterrows():
    try:
        #print(f"ID: {row['ID']}, Own description: {row[name]}, Match description: {row["category"]}")
        category_embedding = {}
        # category_embedding['name'] = 
        category_embedding['category'] = row["category"]
        category_embedding['category_embedding'] = model.encode(row["category"], convert_to_tensor=True)
        category_embeddings[row["category_id"]] = category_embedding
        # print(f"Name: {row['name']}, Category: {row['category']}, Embedding: {category_embedding['name_embedding']}")
    except Exception as e:
        print(e )
        category_embeddings[row["category_id"]] = None
    progress_bar.update(1)

# save the embeddings to a file
if os.path.exists("embeddings_categories.pkl"):
    os.remove("embeddings_categories.pkl")
pickle.dump(category_embeddings, open("embeddings_categories.pkl", "wb"))

def get_embeddings_similarity(object_embedding, category_embedding):
        return util.pytorch_cos_sim(object_embedding, category_embedding).item()

if TESTING:
    while True:
        target_item = input("Enter the item to categorize: ")

        if target_item == "exit":
            break

        item_embedding = model.encode(target_item, convert_to_tensor=True)
        for category_id, category_data in category_embeddings.items():
             if category_data is not None:
                category_data['similarity'] = get_embeddings_similarity(item_embedding, category_data['category_embedding'])
                category_embeddings[category_id] = category_data
        
        possible_item = None

        for category_id, category_data in category_embeddings.items():
            if category_data is not None:
                if possible_item is None:
                    possible_item = category_data
                elif category_data['similarity'] > possible_item['similarity']:
                    possible_item = category_data
        print(f"Category: {possible_item['category']}")
        # print(f"Category: {possible_item['category_embedding']}")
        print(f"Category: {possible_item['similarity']}")

        if target_item == "exit":
            break