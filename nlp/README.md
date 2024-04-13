# Init NLP setup

In the root folder (`hri`), create a `.env` file based on `.env.example` and add the credentials. 

Build this image, must be in root directory:

```bash
docker build -f docker/Dockerfile.hri -t roborregos/home:hri-base .
```

Run the image, the `--env-file` passes the environment variables:

```bash
docker run -it --name home-hri --net=host --privileged --env="QT_X11_NO_MITSHM=1" -e DISPLAY=$DISPLAY -eQT_DEBUG_PLUGINS=1 -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/video0:/dev/video0 --user $(id -u):$(id -g) -v $(pwd):/workspace --env-file .env roborregos/home:hri-base bash
```

Test the initial setup:
```bash
python3 nlp/test-setup.py
```

The output should be: `go kitchen, grab spoon`

**Useful command found**

Remove unused docker images:

```bash
docker rmi $(docker images --filter "dangling=true" -q --no-trunc)
```

# Fine-tuning guide

**Current model:** `ft:gpt-3.5-turbo-0125:ixmatix:roborregos:9ArVJ9Nf`

### Requirements

The [new command generator](https://github.com/johaq/CompetitionTemplate) was added as a submodule in this folder. For its proper setup to use it for generating prompts, go to the root folder of `hri` and execute:
```bash
git submodule init
git submodule update --remote --merge --recursive # To update all submodules and point them to the most recent branch
```
Additionally, for the dataset generation script, go to the folder `CompetitionTemplate/command_generator`, and inside the `egpsr_commands.py`, modify the last import for handling relative path calls:
```python
from .gpsr_commands import CommandGenerator
```

### Dataset generation

The `dataset-generator.py` file, based on the `generator.py` of the command generator, provides an index with different types of prompts. Write a number (preffered `1`), and it will display a command, followed by the current fine-tuned model parsed response:

```bash
ros@afr2903u:/workspace/nlp$ python3 dataset-generator.py 
'1': Any command,
'2': Command without manipulation,
'3': Command with manipulation,
'4': Batch of three commands,
'5': Generate EGPSR setup,
'0': Generate QR code,
'q': Quit
1
Give me an iced tea from the shelf
remember, location; go, shelf; find, iced tea; pick, iced tea; go, past location; give, iced tea.
Number of prompts requested:  1
1
Escort the waving person from the armchair to the bathroom
go, armchair; identify, waving + person; approach, tracker; interact, ask to be followed; go, bathroom.
Number of prompts requested:  2
q
Total tokens used: 1869
```

You can watch random commands being displayed, and after writing `q`, prompt examples will be appended to `nlp-dataset-2.json` file in the format for fine-tuning dataset:

```json
{"messages": [{"role": "system", "content": "You are a service robot for domestic applications. You were developed by RoBorregos team from Tec de Monterrey, from Mexico. You are given general purpose tasks in the form of natural language inside a house environment. You have in your architecture the modules of: navigation, manipulation, person recognition, object detection and human-robot interaction. Your job is to understand the task and divide it to actions proper to your modules, considering a logical flow of the actions. You can ask for clarification if the task is not clear enough. Try to abstract the verbs as much as possible. Divide each action with a semicolon. The actions should be in the form of: 'do x; do y; do z'. For example, for the prompt 'Locate a dish in the kitchen then get it and give it to Angel in the living room', the actions would be: 'go, kitchen; find, dish; grab, dish; go, living room; find, Angel; approach, Angel; give, dish.'. Another example is, for the prompt: 'Tell me what is the biggest object on the tv stand' and its actions are 'remember, location; go, tv stand; identify, biggest + object; go, past location; interact, biggest object information.'. Don't add single quotes"}, {"role": "user", "content": "Give me an iced tea from the shelf"}, {"role": "assistant", "content": "remember, location; go, shelf; find, iced tea; pick, iced tea; go, past location; give, iced tea."}]}
....

```

You should review the `assistant` response and modify it. Follow the [Robot actions guide](https://github.com/RoBorregos/home/wiki/Robot-actions-guide,-Rulebook-2024) with the allowed commands to abstract the response and standarize it.

After validating all prompt examples in this file, append all the lines in the `nlp-dataset.json`.

**Don't delete previous lines**, to improve the model, a **larger** dataset is usually the best path. Discuss improvements instead of addition of samples before modifying the current main dataset.

## Fine tuning

When the dataset is ready, execute the `fine-tuner.py` script. It will display feedback on the current status of the procces. It may take a while to finish the training:
```bash
ros@afr2903u:/workspace/nlp$ python3 fine-tuner.py 
File uploaded: file-id
Fine-tuning job ID: ftjob-id
FineTuningJob(id='ftjob-id', created_at=1712375310, error=Error(code=None, message=None, param=None, error=None), fine_tuned_model=None,...
Time elapsed: 1.545234
FineTuningJob(id='ftjob-id', created_at=1712375310, error=Error(code=None, message=None, param=None, error=None), fine_tuned_model=None,...
Time elapsed: 102.24756

Fine-tuning job done!
Model name: ft:gpt-3.5-turbo-0125:suffix:id
go, kitchen; find, dish; pick, dish; go, bedroom; identify, standing + person; approach, tracker; give, dish.
```

When the fine-tuning job is finished, change the model name in the `dataset-generator.py`, the `test-setup.py`, and the `README.md` (this file).

Test and validate the model using the `dataset-generator`.