"""
Run this script to verify the setup for the NLP feature (without ROS)
"""
import os
from openai import OpenAI

client = OpenAI(
    api_key=os.getenv("OPENAI_API_KEY")
)

petition = "Escort the waving person from the armchair to the bathroom"

while True:
    context = {"role": "system",
            "content": "You are a service robot for domestic applications. You were developed by RoBorregos team from Tec de Monterrey, from Mexico. You are given general purpose tasks in the form of natural language inside a house environment. You have in your architecture the modules of: navigation, manipulation, person recognition, object detection and human-robot interaction. Your job is to understand the task and divide it to actions proper to your modules, considering a logical flow of the actions. You can ask for clarification if the task is not clear enough. Try to abstract the verbs as much as possible. Divide each action with a semicolon. The actions should be in the form of: 'do x; do y; do z'. For example, for the prompt 'Locate a dish in the kitchen then get it and give it to Angel in the living room', the actions would be: 'go, kitchen; find, dish; grab, dish; go, living room; find, Angel; approach, Angel; give, dish.'. Another example is, for the prompt: 'Tell me what is the biggest object on the tv stand' and its actions are 'remember, location; go, tv stand; identify, biggest + object; go, past location; interact, biggest object information.'. Don't add single quotes"
            }

    messages = [context]
    messages.append({"role": "user", "content": petition})
        
    response = client.chat.completions.create(
        model="ft:gpt-3.5-turbo-0125:ixmatix:roborregos:9ArVJ9Nf",
        messages=messages
    )

    response_content = response.choices[0].message.content

    print(response_content)

    messages.append({"role": "assistant", "content": response_content})

    petition = input("Enter a new command or type 'q' to quit:\n")
    if petition == "q":
        break