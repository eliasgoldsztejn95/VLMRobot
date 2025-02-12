import requests
import time
import os
from multiprocessing.shared_memory import SharedMemory


class KtorClient:
    def __init__(self, ip_address: str, port: int):
        self.base_url = f"http://{ip_address}:{port}"

        ##################################################################
        # Attach to shared memory command buffer
        self.shm_notification_cmd = SharedMemory(name="shm_notification_cmd")
        self.shm_server_command_cmd = SharedMemory(name="shm_server_command_cmd")
        self.shm_server_camera_cmd = SharedMemory(name="shm_server_camera_cmd")
        ##################################################################

    def receive_command_notification(self, shm):
        """ Reads and decodes a command from shared memory, returns False if empty """
        raw_bytes = bytes(shm.buf).split(b'\x00', 1)[0]  # Remove trailing null bytes
        command_str = raw_bytes.decode().strip()
        
        if not command_str:  # If empty, return False
            return False
        
        return command_str.split("|")  # Convert back to list

    def receive_command(self, shm):
        """ Receive output from a process """
        message = bytes(shm.buf[:]).decode().strip('\x00')
        return message

    def send_command(self, shm, command):
        """ Receive output from a process """
        shm.buf[:] = b'\x00' * len(shm.buf)  # Clear the buffer
        shm.buf[:len(command)] = command.encode()

    def broadcast_message(self, title_text, message_text: str):
        url = f"{self.base_url}/broadcast"
        payload = {
            "to": None,
            "notification": {
                "title": title_text,
                "body": message_text
            }
        }
        headers = {"Content-Type": "application/json"}

        try:
            response = requests.post(url, json=payload, headers=headers)
            if response.status_code == 200:
                print("Broadcast sent successfully!")
            else:
                print(f"Failed to send broadcast: {response.status_code}, {response.text}")
        except requests.exceptions.RequestException as e:
            print(f"Error sending broadcast: {e}")
    
    def run(self):
        
        while True:

            notification_cmd = self.receive_command_notification(self.shm_notification_cmd)
            print(f"Reading...{notification_cmd}")
            if notification_cmd and notification_cmd[0] == "start":
                print("Broadcasting")
                self.broadcast_message("Request to Help Robot", "Danger: " + notification_cmd[1])

                while True:
                    server_command_cmd = self.receive_command(self.shm_server_command_cmd)
                    server_camera_cmd = self.receive_command(self.shm_server_camera_cmd)

                    if server_command_cmd == "connected" and server_camera_cmd == "connected":
                        break

                    time.sleep(1)
                
                self.broadcast_message("Got help, thanks!", "")
                self.send_command(self.shm_notification_cmd, "stop")
            
            time.sleep(1)



# Example usage
if __name__ == "__main__":
    ip_address = "localhost"  # Replace with your Ktor server's IP
    port = 8080  # Replace with your Ktor server's port

    ktor_client = KtorClient(ip_address, port)
    ktor_client.run()
