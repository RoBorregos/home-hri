## ROS NODE using fine tunning + embeddings
This ROS node aims to integrate the fine-tuning work and task analysis using embeddings. To achieve this, the following steps should be followed.

First, it is neccesary to create a ROS package with the following folders:
 - dataframes: containing the previously stored dataframes (.pkl files).
 - msg: containing messages sent via the /speech/processed_commands topic.

To make the node function:
1. (TERMINAL A) Start a roscore.
```bash
marina@marina:~/catkin_ws$ source devel/setup.bash 
marina@marina:~/catkin_ws$ catkin_make
[...]
marina@marina:~/catkin_ws$ roscore
... logging to /home/marina/.ros/log/2cf838ca-f96b-11ee-b3a5-8f372d1bed75/roslaunch-marina.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

```

2. (TERMINAL B) execute the `test-human.py` file, which simulates the user input and publishes it on the /RawInput topic.
```bash
marina@marina:~/catkin_ws$ source devel/setup.bash 
marina@marina:~/catkin_ws$ catkin_make
[...]
marina@marina:~/catkin_ws$ rosrun test_hri test-human.py 
bring me a mug from the kitchen table
```
3. (TERMINAL C) Run the analysis node `test-analysis-simple.py`, which receives input from `/RawInput`, applies the fine-tuning model, analyzes the embeddings, and publishes the result in the form of *action + complement* on the `/speech/processed_commands` topic. 

_WARNING!!_ To run the code first you have to change two things:
- Write the name of your package in: `from test_hri.msg import command, list_of_commands`
- Write the path to your dataframes in `DATAFRAMES_DIR = "/path_to_dataframes"`

```bash
marina@marina:~/catkin_ws$ source devel/setup.bash 
marina@marina:~/catkin_ws$ catkin_make
[...]
marina@marina:~/catkin_ws/src$ rosrun test_hri test-analysis-simple.py  
[INFO] [1713099488.171782]: HRI analysis started
[INFO] [1713273567.973529]: /TEXT_ANALYSIS_NODE_41798_1713273557249 I heard bring me a mug from the kitchen
Fine tuned sentence:  remember, location; go, kitchen; find, mug; pick, mug; go, past location; give, mug.
** WARNING! Did you mean cup?? ** 
Please, confirm with 'yes' or 'no': yes
** Command confirmed **
** Command confirmed **
** Command confirmed **
```

4. (TERMINAL D) You can see the published message in the topic with `echo`
```bash
marina@marina:~/catkin_ws$ rostopic echo /speech/processed_commands
commands: 
  - 
    action: "go"
    complements: 
      - kitchen
---
commands: 
  - 
    action: "find"
    complements: 
      - cup
---
commands: 
  - 
    action: "pick"
    complements: 
      - cup
---
commands: 
  - 
    action: "go"
    complements: 
      - past location
---
commands: 
  - 
    action: "give"
    complements: 
      - cup
---


```