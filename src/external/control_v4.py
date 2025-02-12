import time
import sys
#import serial
from multiprocessing.shared_memory import SharedMemory
import json
from transitions import Machine
import numpy as np
import socket

# Processes involved
PROCESSES = ["camera", "follow", "notification", "server_camera", "server_command", "vlm"]
# Define states
states = ['NAVIGATE', 'FOLLOW', 'REMOTE']

class StateMachine:
    def __init__(self):
        self.state = "NAVIGATE"  # Default state
        self.danger_type = "None"
        self.shared_memory = {}
        self.server_ip = "172.18.8.23"
        self.port = 10093
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.server_ip, self.port))
        print("connected to the server")

        
        # Create shared memory for each process
        for process in PROCESSES:
            self.shared_memory[process] = {
                "command": SharedMemory(create=True, size=20, name=f"shm_{process}_cmd"),
            }
        print("here 2")
        self.shared_memory["vlm"]["output"] = SharedMemory(create=True, size=256, name=f"shm_vlm_out")
        self.shared_memory["follow"]["output"] = SharedMemory(create=True, size=100, name=f"shm_follow_out")
        self.shared_memory["server_command"]["output"] = SharedMemory(create=True, size=100, name=f"shm_server_command_out")
        print("here 3")

        
        # USB serial output
        #self.usb_serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # Initialize state machine
        self.machine = Machine(model=self, states=states, initial='NAVIGATE')
        
        # Define transitions
        self.machine.add_transition('switch_to_follow', 'NAVIGATE', 'FOLLOW', after='on_enter_follow')
        self.machine.add_transition('switch_to_remote', 'NAVIGATE', 'REMOTE', after='on_enter_remote')
        self.machine.add_transition('switch_to_navigate', ['FOLLOW', 'REMOTE'], 'NAVIGATE', after='on_enter_navigate')

    def log(self, message):
        log_message = f"<{self.state}><{message}>"
        print(log_message)
        self.client_socket.sendall(log_message.encode())

    def send_command(self, process, command):
        """ Send a command (start/stop) to a process """
        shm = self.shared_memory[process]["command"]
        shm.buf[:] = b'\x00' * len(shm.buf)  # Clear the buffer
        shm.buf[:len(command)] = command.encode()
        #self.log("SEND", f"{process}: {command}")

    def receive_command(self, process):
        """ Receive output from a process """
        shm = self.shared_memory[process]["command"]
        message = bytes(shm.buf[:]).decode().strip('\x00')
        #self.log("RECEIVE", f"{process}: {message}")
        return message

    def receive_output(self, process):
        """ Receive output from a process """
        shm = self.shared_memory[process]["output"]
        message = bytes(shm.buf[:]).decode().strip('\x00')
        #self.log("RECEIVE", f"{process}: {message}")
        return message

    def decode_vlm(self):
        # Read data from shared memory
        shm = self.shared_memory["vlm"]["output"]
        encoded_data = bytes(shm.buf[:]).decode().strip('\x00')  # Remove trailing null bytes
        data = json.loads(encoded_data)  # Decode bytes to JSON
        
        # Extract required fields
        object = data.get("object close", "Unknown")
        people = data.get("people", False)
        looking_to_camera = data.get("looking to camera", False)
        raising_one_hand = data.get("raising one hand", False)
        raising_two_hands = data.get("raising two hands", False)
        environment = data.get("environment", "Unknown")
        
        # Decision logic
        if object == "tiger":
            return "DANGER", object
        elif people and looking_to_camera and raising_one_hand and not raising_two_hands:
            return "STOP", ""
        elif people and looking_to_camera and raising_two_hands:
            return "FOLLOW", environment
        
        return "NO ACTION", environment

    def on_enter_navigate(self):
        print("Switching to navigate")

    def on_enter_follow(self):
        print("Switching to follow")
        self.send_command("follow", "start")

    def on_enter_remote(self):
        print("Switching to remote")
        self.send_command("notification", "start|" + self.danger_type)
        self.send_command("server_camera", "start")
        self.send_command("server_command", "start")
        print("Finished initialization")

    def run(self):
        """ Main loop that handles state transitions and communication """
        """ Main loop to check shared memory and transition states. """

        time.sleep(20)

        while True:

            if self.state == "NAVIGATE":
                print("Navigating...")
                action_vlm, extra_vlm = self.decode_vlm()
                print(f"action_vlm: {action_vlm}, extra_vlm: {extra_vlm}")

                self.log("")

                if action_vlm == "FOLLOW":
                    self.switch_to_follow()
                elif action_vlm == "DANGER":
                    self.danger_type = extra_vlm
                    self.switch_to_remote()

            if self.state == "REMOTE":
                print("Remoting...")
                remote_camera_status = self.receive_command("server_camera")
                remote_command_status = self.receive_command("server_command")
                remote_command_out = self.receive_output("server_command")
                print(f"remote_command_out: {remote_command_out}")

                self.log(remote_command_out)

                if remote_command_status == "stop" and remote_camera_status == "stop":
                    self.switch_to_navigate()

            if self.state == "FOLLOW":
                print("Following...")
                action_vlm, extra_vlm = self.decode_vlm()
                print(f"action_vlm: {action_vlm}, extra_vlm: {extra_vlm}")

                follow_status = self.receive_command("follow")
                follow_out = self.receive_output("follow")
                print(f"follow_out: {follow_out}")

                self.log(follow_out)

                #if follow_status == "stop" or action_vlm == 'STOP':
                if follow_status == "stop":
                    self.switch_to_navigate()
                    self.send_command("follow", "stop")

            time.sleep(0.1)  # Prevent excessive CPU usage


    def cleanup(self):
        """ Cleanup shared memory """
        for process in PROCESSES:
            self.shared_memory[process]["command"].unlink()
            self.shared_memory[process]["output"].unlink()
        self.usb_serial.close()

if __name__ == "__main__":
    sm = StateMachine()
    try:
        sm.run()
    except KeyboardInterrupt:
        print("here")
        sm.cleanup()
        sys.exit(0)
