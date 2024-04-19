from openai import OpenAI
import pandas as pd
from dotenv import load_dotenv
import os

# Load the environment variables
load_dotenv()

# INICIALIZATION PARAMETERS
DATAFRAMES_DIR = "../dataframes/" 
ITEMS = "items"
LOCATIONS = "locations"
NAMES = "names"
ACTIONS = "actions"

# Load data
data = {
    ACTIONS: os.path.join(DATAFRAMES_DIR, "actions.csv"),
    LOCATIONS: os.path.join(DATAFRAMES_DIR, "locations.csv"),
    ITEMS: os.path.join(DATAFRAMES_DIR, "items.csv"),
    NAMES: os.path.join(DATAFRAMES_DIR, "names.csv")
}

# Client setup
client = OpenAI(
    api_key=os.getenv("OPENAI_API_KEY")
)

# Embedding function
def get_embedding(text, model="text-embedding-3-small"):
   emb = client.embeddings.create(input = [text], model=model).data[0].embedding
   
   return emb


# Process for creating and saving embeddings
def process_data(filename, embedding_columns):
    # Loads data
    df = pd.read_csv(filename)

    # Creates embeddings
    for column in embedding_columns:
        df[f"{column}_embedding"] = df[column].apply(lambda x: get_embedding(x))
        
    # Saves the dataframe to a pickle file (faster than csv)
    df.to_pickle(f"embeddings_{filename.split('/')[-1].split('.')[0]}.pkl")

    #print(df)
    return df 


if __name__ == "__main__":
    process_data(data[ACTIONS], ["action"])
    process_data(data[LOCATIONS], ["name"])
    process_data(data[ITEMS], ["name", "category"])
    process_data(data[NAMES], ["name"])
    
    print("Dataframes created succesfully")
