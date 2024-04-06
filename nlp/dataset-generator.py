"""
Using the `generator.py` script of Competition Template as base for generating the datasets
"""
import random
import re
import warnings
import qrcode
from PIL import Image, ImageDraw, ImageFont
from CompetitionTemplate.command_generator.gpsr_commands import CommandGenerator
from CompetitionTemplate.command_generator.egpsr_commands import EgpsrCommandGenerator

import json
import os
from openai import OpenAI

client = OpenAI(
    api_key=os.getenv('OPENAI_API_KEY')
)

dataset_file = open("nlp-dataset-2.json", "w")
model_name = "ft:gpt-3.5-turbo-0125:ixmatix:roborregos:9ArVJ9Nf"
total_tokens_used = 0
number_of_prompts = 0

"""
Function that processes the command as a prompt sent to the OpenAI API, with the required format as the context
"""
def process_command(prompt):
    global total_tokens_used
    system_prompt = "You are a service robot for domestic applications. You were developed by RoBorregos team from Tec de Monterrey, from Mexico. You are given general purpose tasks in the form of natural language inside a house environment. You have in your architecture the modules of: navigation, manipulation, person recognition, object detection and human-robot interaction. Your job is to understand the task and divide it to actions proper to your modules, considering a logical flow of the actions. You can ask for clarification if the task is not clear enough. Try to abstract the verbs as much as possible. Divide each action with a semicolon. The actions should be in the form of: 'do x; do y; do z'. For example, for the prompt 'Locate a dish in the kitchen then get it and give it to Angel in the living room', the actions would be: 'go, kitchen; find, dish; grab, dish; go, living room; find, Angel; approach, Angel; give, dish.'. Another example is, for the prompt: 'Tell me what is the biggest object on the tv stand' and its actions are 'remember, location; go, tv stand; identify, biggest + object; go, past location; interact, biggest object information.'. Don't add single quotes"
    dataset_instance = {
        "messages": [
            {
                "role": "system",
                "content": system_prompt
            },
            {
                "role": "user",
                "content": ""
            },
            {
                "role": "assistant",
                "content": ""
            }
        ]
    }

    completion = client.chat.completions.create(
        model= model_name,
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": prompt}
        ]
    )
    print(completion.choices[0].message.content)

    dataset_instance["messages"][1]["content"] = prompt
    dataset_instance["messages"][2]["content"] = completion.choices[0].message.content
    total_tokens_used += completion.usage.total_tokens
    dump_to_json(dataset_instance)

"""
Append the generated data in the format of a JSON object to the dataset JSON file
"""
def dump_to_json(data):
    json.dump(data, dataset_file)
    dataset_file.write("\n")

"""
Close the JSON file and print the total tokens used in the process
"""
def close_json_file():
    dataset_file.close()
    print(f"Total tokens used: {total_tokens_used}")


def read_data(file_path):
    with open(file_path, 'r') as file:
        data = file.read()
    return data


def parse_names(data):
    parsed_names = re.findall(r'\|\s*([A-Za-z]+)\s*\|', data, re.DOTALL)
    parsed_names = [name.strip() for name in parsed_names]

    if parsed_names:
        return parsed_names[1:]
    else:
        warnings.warn("List of names is empty. Check content of names markdown file")
        return []


def parse_locations(data):
    parsed_locations = re.findall(r'\|\s*([0-9]+)\s*\|\s*([A-Za-z,\s, \(,\)]+)\|', data, re.DOTALL)
    parsed_locations = [b for (a, b) in parsed_locations]
    parsed_locations = [location.strip() for location in parsed_locations]

    parsed_placement_locations = [location for location in parsed_locations if location.endswith('(p)')]
    parsed_locations = [location.replace('(p)', '') for location in parsed_locations]
    parsed_placement_locations = [location.replace('(p)', '') for location in parsed_placement_locations]
    parsed_placement_locations = [location.strip() for location in parsed_placement_locations]
    parsed_locations = [location.strip() for location in parsed_locations]

    if parsed_locations:
        return parsed_locations, parsed_placement_locations
    else:
        warnings.warn("List of locations is empty. Check content of location markdown file")
        return []


def parse_rooms(data):
    parsed_rooms = re.findall(r'\|\s*(\w+ \w*)\s*\|', data, re.DOTALL)
    parsed_rooms = [rooms.strip() for rooms in parsed_rooms]

    if parsed_rooms:
        return parsed_rooms[1:]
    else:
        warnings.warn("List of rooms is empty. Check content of room markdown file")
        return []


def parse_objects(data):
    parsed_objects = re.findall(r'\|\s*(\w+)\s*\|', data, re.DOTALL)
    parsed_objects = [objects for objects in parsed_objects if objects != 'Objectname']
    parsed_objects = [objects.replace("_", " ") for objects in parsed_objects]
    parsed_objects = [objects.strip() for objects in parsed_objects]

    parsed_categories = re.findall(r'# Class \s*([\w,\s, \(,\)]+)\s*', data, re.DOTALL)
    parsed_categories = [category.strip() for category in parsed_categories]
    parsed_categories = [category.replace('(', '').replace(')', '').split() for category in parsed_categories]
    parsed_categories_plural = [category[0] for category in parsed_categories]
    parsed_categories_plural = [category.replace("_", " ") for category in parsed_categories_plural]
    parsed_categories_singular = [category[1] for category in parsed_categories]
    parsed_categories_singular = [category.replace("_", " ") for category in parsed_categories_singular]

    if parsed_objects or parsed_categories:
        return parsed_objects, parsed_categories_plural, parsed_categories_singular
    else:
        warnings.warn("List of objects or object categories is empty. Check content of object markdown file")
        return []


if __name__ == "__main__":
    names_file_path = 'CompetitionTemplate/names/names.md'
    locations_file_path = 'CompetitionTemplate/maps/location_names.md'
    rooms_file_path = 'CompetitionTemplate/maps/room_names.md'
    objects_file_path = 'CompetitionTemplate/objects/test.md'

    names_data = read_data(names_file_path)
    names = parse_names(names_data)

    locations_data = read_data(locations_file_path)
    location_names, placement_location_names = parse_locations(locations_data)

    rooms_data = read_data(rooms_file_path)
    room_names = parse_rooms(rooms_data)

    objects_data = read_data(objects_file_path)
    object_names, object_categories_plural, object_categories_singular = parse_objects(objects_data)

    generator = CommandGenerator(names, location_names, placement_location_names, room_names, object_names,
                                 object_categories_plural, object_categories_singular)
    egpsr_generator = EgpsrCommandGenerator(generator)
    user_prompt = "'1': Any command,\n" \
                  "'2': Command without manipulation,\n" \
                  "'3': Command with manipulation,\n" \
                  "'4': Batch of three commands,\n" \
                  "'5': Generate EGPSR setup,\n" \
                  "'0': Generate QR code,\n" \
                  "'q': Quit"
    print(user_prompt)
    command = ""
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_L,
        box_size=30,
        border=4,
    )
    last_input = '?'
    try:
        while True:
            # Read user input
            user_input = input()

            # Check user input
            if user_input == '1':
                command = generator.generate_command_start(cmd_category="")

                last_input = "1"
            elif user_input == '2':
                command = generator.generate_command_start(cmd_category="people")
                last_input = "2"
            elif user_input == '3':
                command = generator.generate_command_start(cmd_category="objects")
                last_input = "3"
            elif user_input == '4':
                command_one = generator.generate_command_start(cmd_category="people")
                command_two = generator.generate_command_start(cmd_category="objects")
                command_three = generator.generate_command_start(cmd_category="")
                command_list = [command_one[0].upper() + command_one[1:], command_two[0].upper() + command_two[1:],
                                command_three[0].upper() + command_three[1:]]
                random.shuffle(command_list)
                command = command_list[0] + "\n" + command_list[1] + "\n" + command_list[2]
                last_input = "4"
            elif user_input == "5":
                command = egpsr_generator.generate_setup()
                last_input = "5"
            elif user_input == 'q':
                close_json_file()
                break
            elif user_input == '0':
                if last_input == '4':
                    commands = command_list
                else:
                    commands = [command]
                for c in commands:
                    qr.clear()
                    qr.add_data(c)
                    qr.make(fit=True)

                    img = qr.make_image(fill_color="black", back_color="white")
                    # Create a drawing object
                    draw = ImageDraw.Draw(img)

                    # Load a font
                    font = ImageFont.truetype("Arial.ttf", 30)

                    # Calculate text size and position
                    text_size = draw.textsize(c, font)
                    if text_size[0] > img.size[0]:
                        font = ImageFont.truetype("Arial.ttf", 15)
                        text_size = draw.textsize(c, font)
                    text_position = ((img.size[0] - text_size[0]) // 2, img.size[1] - text_size[1] - 10)

                    # Draw text on the image
                    draw.text(text_position, c, font=font, fill="black")
                    img.show()
            else:
                print(user_prompt)
                continue
            command = command[0].upper() + command[1:]
            print(command)
            process_command(command) # Process the command
            number_of_prompts += 1
            print("Number of prompts requested: ", number_of_prompts)

    except KeyboardInterrupt:
        print("KeyboardInterrupt. Exiting the loop.")

    # for _ in range(500):  # Generate 50 random commands
    #     generator = CommandGenerator(names, location_names, placement_location_names, room_names, object_names,
    #                                  object_categories_plural, object_categories_singular)
    #     command = generator.generate_command_start(cmd_category="")
    #     command = command[0].upper() + command[1:]
    #     print(command)
