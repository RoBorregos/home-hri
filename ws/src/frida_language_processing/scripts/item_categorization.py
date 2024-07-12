#!/usr/bin/env python3
"""
ROS node for the Storing groceries task, should return the category of an item or list of items
"""
import rospy
import pandas as pd
import numpy as np
import os
import pickle
from sentence_transformers import SentenceTransformer, util
import yaml
import tqdm
import rospkg
from openai import OpenAI


from frida_hri_interfaces.srv import ItemsCategory, ItemsCategoryResponse

SIMILARITY_THRESHOLD = 0.7 # Minimum similarity percentage in which 2 words are considered the same

rp = rospkg.RosPack()
DATAFRAMES_DIR = rp.get_path('frida_language_processing') + "/scripts/dataframes/"
ITEMS_EXTRACT_TOPIC = "items_category"
OBJECT_EXTRACT_TOPIC = "object_category"

class ItemCategorization:
    """class to handle both services for item categorization"""
    def __init__(self) -> None:
        """Initialize the ROS node"""
        self._node = rospy.init_node("item_categorization")
        self._rate = rospy.Rate(10)
        # rospy.Service(ITEMS_EXTRACT_TOPIC, ItemsCategory, self.extract_category)
        rospy.Service(ITEMS_EXTRACT_TOPIC, ItemsCategory, self.get_local_embedding)
        rospy.Service(OBJECT_EXTRACT_TOPIC, ItemsCategory, self.extract_item)
        self.openai_client = OpenAI(
            api_key=os.getenv("OPENAI_API_KEY")
        )
        self.model = SentenceTransformer("all-MiniLM-L12-v2")
        current_dir = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(current_dir, "assets", "embeddings_items.pkl"), "rb") as f:
            self.items_dataframe = pickle.load(f)

        with open(os.path.join(current_dir, "assets", "embeddings_categories.pkl"), "rb") as f:
            self.category_embeddings = pickle.load(f)

        rospy.spin()
    
    def extract_item(self, req):
        """Service to return the most representative category of the items given"""
        target_item = req.items[0]
        detected_items = req.items[1:]

        item_embedding = self.get_embedding(target_item)
        possible_items = self.get_similar_items(item_embedding, self.items_dataframe)

        response = ItemsCategoryResponse()

        for i in range(len(possible_items)):
            if possible_items.iloc[i]['name'] in detected_items:
                response.category = possible_items.iloc[i]['name']
                break
        
        return response

    def extract_category(self, req):
        """Service to return the most representative category of the items given"""
        items = req.items
        categories = []
        for item in items:
            item_embedding = self.get_embedding(item)
            item_category = self.get_category_similarity(item_embedding, self.items_dataframe)
            categories.append(item_category)

        representative_category = max(set(categories), key=categories.count)
        response = ItemsCategoryResponse()
        response.category = representative_category
        return response

    def local_extract_category(self, req):
        """Service to return the most representative category of an item locally"""
        target_item = req.items[0]
        item_embedding = self.model.encode(target_item, convert_to_tensor=True)
        for category_id, category_data in self.category_embeddings.items():
             if category_data is not None:
                category_data['similarity'] = self.get_embeddings_similarity(item_embedding, category_data['category_embedding'])
                self.category_embeddings[category_id] = category_data
        
        possible_item = None

        for category_id, category_data in self.category_embeddings.items():
            if category_data is not None:
                if possible_item is None:
                    possible_item = category_data
                elif category_data['similarity'] > possible_item['similarity']:
                    possible_item = category_data

        return possible_item['category']

    def get_embedding(self, text):
        """Get the embedding of a given text"""
        return self.openai_client.embeddings.create(input = [text], model="text-embedding-3-small").data[0].embedding

    def get_local_embedding(self, object_embedding, category_embedding):
        return util.pytorch_cos_sim(object_embedding, category_embedding).item()
    
    def get_embeddings_similarity(self, object_embedding, category_embedding):
        return util.pytorch_cos_sim(object_embedding, category_embedding).item()

    def get_similar_items(self, input, dataframe):
        """Get the category of the item based on the similarity of the embedding"""
        # Get name similarity
        dataframe['similarity'] = dataframe['name_embedding'].apply(lambda x: np.dot(x, input))
        sorted_names = dataframe.sort_values('similarity', ascending=False)
        return sorted_names

    def get_category_similarity(self, input, dataframe):
        """Get the category of the item based on the similarity of the embedding"""
        # Get name similarity
        dataframe['similarity'] = dataframe['name_embedding'].apply(lambda x: np.dot(x, input))
        sorted_names = dataframe.sort_values('similarity', ascending=False)
        best_name_similarity = sorted_names.iloc[0]['similarity']

        # Get category similarity 
        dataframe['similarity'] = dataframe['category_embedding'].apply(lambda x: np.dot(x, input))
        sorted_categories = dataframe.sort_values('similarity', ascending=False)
        best_category_similarity = sorted_categories.iloc[0]['similarity']

        # Return the category with the highest similarity, comparing name and category
        if best_name_similarity > best_category_similarity:
            return sorted_names.iloc[0]['category']
        else:
            return sorted_categories.iloc[0]['category']

if __name__ == "__main__":
    try:
        ItemCategorization()
    except rospy.ROSInterruptException:
        pass
