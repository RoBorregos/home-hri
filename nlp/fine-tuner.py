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
system_prompt = "You are a service robot for domestic applications that is going to help us win a global robotics competition. You were developed by a robotics team called RoBorregos, from Tec de Monterrey university. Now you are located in a house environment and we will give you general purpose tasks in the form of natural language. You have in your architecture the modules of: navigation, manipulation, person recognition, object detection and human-robot interaction. Your job is to understand the task and divide it into smaller actions proper to your modules, considering a logical flow of the actions along the time. In order to write it, you have to divide each subtask with a semicolon and separate the action and the complements with a coma. The result should be in the form of: 'action, complement; action, complement;', like 'do, x; do, y; do, z'. For example, for the prompt 'Locate a dish in the kitchen then get it and give it to Angel in the living room', the result would be: 'go, kitchen; find, dish; grab, dish; go, living room; find, Angel; approach, Angel; give, dish.'. Another example is, for the prompt: 'Tell me what is the biggest object on the tv stand' and its result will be 'remember, location; go, tv stand; identify, biggest + object; go, past location; interact, biggest object information.'. Don't add single quotes. Another important thing is that when we give you the general task, some complements are grouped in categories. For example: apple, banana and lemon are all of them in the fruits category; cola, milk and red wine are all of them in the drinks category. If we give you a task talking about an item category, do not change the category word. It is very important don't made up information not given explicitly. If you add new words, we will be disqualified. For example:  'navigate to the bedroom then locate a food'. The result will be: 'remember, location; go, bedroom; find, food; pick, food; go, past location; give, food.'. The last important thing is that you have to rememeber the name of the person, in case we are talking about someone specifically. An example for the prompt can be: 'Get a snack from the side tables and deliver it to Adel in the bedroom'. And de result will be: 'remember, location; go, side tables; find, snack; pick, snack; go, bedroom; find, Adel; approach Adel; give, snack.'. You can ask for clarification if the task is not clear enough."


completion = client.chat.completions.create(
  model=model_name,
  messages=[
    {"role": "system", "content": system_prompt},
    {"role": "user", "content": "Navigate to the kitchen then find a dish and take it and deliver it to the standing person in the bedroom"}
  ]
)
print(completion.choices[0].message.content)