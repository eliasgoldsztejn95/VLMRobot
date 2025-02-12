## Requirements

- python > 3.6
- Firebase account: google.json
- Groq account: api key for llava-vision

## Code explanation

- control.py: This is the state machine that connects with the robot computer. It controls all the other subprocesses.
- realsesne_process.py: Starts the camera and shares the frames with all other subprocesses.
- follow.py: Follow algorithm.
- vlm.py: Sends API requests given a pictue and a prompt. The prompt asks for a json file containing specific descriptions of the picture.
- broadcast.py: Sends notifications to the users. THe notification is broadcasted to all users asking for help. When a user connects it broadcasts a message saying that it got help.
- server_message.py: Manages messages from user.
- server_realsense: Manages video for the user.
