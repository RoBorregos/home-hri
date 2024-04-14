# Embedding analysis
Embeddings in NLP implementations offer users flexibility with synonyms, singular, and plural forms, enhancing system robustness and user interaction. For RoboCup@Home 2024, we have integrated embeddings into the nlp framework to better understand and process user commands, improving communication between users and robots. This allows for varied inputs while ensuring command execution accuracy, aligning with competition requirements.

Here you can find the [new command generator](https://github.com/johaq/CompetitionTemplate), which will be used in the competition to tell the tasks to the robot.

## Init NLP setup
First step is to open the **nlp docker** following the instruction on [this file](../README.md)

## Requirements
Once in the docker terminal, it is necessary to install **Pandas**. Pandas is a Python library specialized in data manipulation and analysis.
```bash
pip install pandas
```
## Create embedding for known items
The first step involves creating the embedding value for known elements. For this purpose, we have set up four .csv files (for ease of editing and reading) organized into four categories: actions, items, locations, and names. By running the create_dataset_embeddings.py code, four .pkl files are generated, containing the information from the initial files plus an additional column for the embedding value. These files are saved as .pkl for faster reading.
```bash
ros@marina:/workspace/nlp/Embeddings analysis/dataframes$ python3 create_dataframes_embeddings.py 
Dataframes created succesfully
```
Each time you want to add or modify the .csv files, you have to run de code again to create the new embeddings.

## NLP using embeddings
The robot's architecture incorporates modules for navigation, manipulation, human-robot interaction, and vision. To ensure that user-defined tasks are executed by the appropriate modules, we utilize a fine-tuned chat model that breaks down user commands into small, logically ordered subtasks. Each of these subtasks is referred to as a command, comprising an action followed by a complement. Through embeddings, we compare each action and complement specified by the user with the actions and complements known to each area of the robot's architecture, allowing us to provide the correct information. By comparing words, we calculate the distance between the received word and all known words, and if the distance is less than a certain threshold, it is considered the same word.

To run the embedding analysis, you just have to execute the `test_analysis.py` file and write the user input in a natural way.

```bash
ros@marina:/workspace/nlp/Embeddings analysis$ python3 test-analysis.py
bring me a mug from the kitchen table
Fine tuned sentence:  remember, location; go, kitchen table; find, mug; pick, mug; go, past location; give, mug.
remember
['location']
go
['kitchen table']
find
** WARNING! Did you mean cup?? ** 
Please, confirm with 'yes' or 'no': yes
** Command confirmed **
['cup']
pick
** Command confirmed **
['cup']
go
['past location']
give
** Command confirmed **
['cup']
```


