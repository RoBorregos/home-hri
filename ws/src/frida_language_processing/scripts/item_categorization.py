#!/usr/bin/env python3
"""
ROS node for the Storing groceries task, should return the category of an item or list of items
"""
import rospy
from openai import OpenAI
import pandas as pd
import numpy as np
import os
import sys
import pathlib
import rospkg
import pickle

from frida_hri_interfaces.srv import ItemsCategory, ItemsCategoryResponse

SIMILARITY_THRESHOLD = 0.7 # Minimum similarity percentage in which 2 words are considered the same

rp = rospkg.RosPack()
DATAFRAMES_DIR = rp.get_path('frida_language_processing') + "/scripts/dataframes/"
ITEMS = "items"

class ItemCategorization:
    """class to handle both services for item categorization"""
    def __init__(self) -> None:
        """Initialize the ROS node"""
        self._node = rospy.init_node("item_categorization")
        self._rate = rospy.Rate(10)
        rospy.Service("items_category", ItemsCategory, self.extract_category)
        self.openai_client = OpenAI(
            api_key=os.getenv("OPENAI_API_KEY")
        )
        current_dir = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(current_dir, "assets", "embeddings_items.pkl"), "rb") as f:
            self.items_dataframe = pickle.load(f)

        rospy.spin()

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

    def get_embedding(self, text):
        """Get the embedding of a given text"""
        return self.openai_client.embeddings.create(input = [text], model="text-embedding-3-small").data[0].embedding

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
