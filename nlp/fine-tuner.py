import os
from openai import OpenAI
from dotenv import load_dotenv
import time

load_dotenv()
client = OpenAI(
    api_key=os.getenv('OPENAI_API_KEY')
)

### Upload a training file
filename = client.files.create(
    file=open("nlp-dataset.json", "rb"),
    purpose="fine-tune"
)
print("File uploaded: ", filename.id)

### Create a fine-tuned model
job = client.fine_tuning.jobs.create(
    training_file=filename.id,
    model="gpt-3.5-turbo-0125",
    suffix="roborregos"
)
print("Fine-tuning job ID: ", job.id)
init_time = time.time()

### Check the state of the fine-tuning job and wait until it's done

while True:
    status = client.fine_tuning.jobs.retrieve(job.id)
    print(status)
    print("Time elapsed: ", time.time() - init_time)
    if status.fine_tuned_model is not None:
        print("\nFine-tuning job done!")
        print("Model name: ", status.fine_tuned_model)
        break
    time.sleep(100)

### Test the model 
model_name = client.fine_tuning.jobs.retrieve(job.id).fine_tuned_model
system_prompt = "You are a service robot for domestic applications. You were developed by RoBorregos team from Tec de Monterrey, from Mexico. You are given general purpose tasks in the form of natural language inside a house environment. You have in your architecture the modules of: navigation, manipulation, person recognition, object detection and human-robot interaction. Your job is to understand the task and divide it to actions proper to your modules, considering a logical flow of the actions. You can ask for clarification if the task is not clear enough. Try to abstract the verbs as much as possible. Divide each action with a semicolon. The actions should be in the form of: 'do x; do y; do z'. For example, for the prompt 'Locate a dish in the kitchen then get it and give it to Angel in the living room', the actions would be: 'go, kitchen; find, dish; grab, dish; go, living room; find, Angel; approach, Angel; give, dish.'. Another example is, for the prompt: 'Tell me what is the biggest object on the tv stand' and its actions are 'remember, location; go, tv stand; identify, biggest + object; go, past location; interact, biggest object information.'. Don't add single quotes"

completion = client.chat.completions.create(
  model=model_name,
  messages=[
    {"role": "system", "content": system_prompt},
    {"role": "user", "content": "Navigate to the kitchen then find a dish and take it and deliver it to the standing person in the bedroom"}
  ]
)
print(completion.choices[0].message.content)