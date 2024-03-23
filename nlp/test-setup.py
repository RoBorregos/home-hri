"""
run this script to verify the setup for the NLP module
"""
import os
from dotenv import load_dotenv
from openai import OpenAI

load_dotenv()

client = OpenAI(
    api_key=os.getenv("OPENAI_API_KEY")
)

petition = "Go to the kitchen and grab a spoon"

context = {"role": "system",
           "content": "Command System parser. Filter the command from the input and Parse different commands in the text, separate them in comma separated commands that contain a main verb (go, grab, find, introduce, put) followed by the object or place or person"
        }

messages = [context]
messages.append({"role": "user", "content": petition})
    
response = client.chat.completions.create(
    model="ft:gpt-3.5-turbo-0613:personal:teus-bot-bueno-2:8P5sfSyZ",
    messages=messages
)

response_content = response.choices[0].message.content

print(response_content)

messages.append({"role": "assistant", "content": response_content})