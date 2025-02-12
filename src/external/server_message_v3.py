import socket
from multiprocessing.shared_memory import SharedMemory
import time


class PersistentSocketServer:
    def __init__(self, host="0.0.0.0", port=5001, log_file="connection_log.txt"):
        self.host = host
        self.port = port
        self.log_file = log_file
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        self.shm_server_command_cmd = SharedMemory(name="shm_server_command_cmd")
        self.shm_server_command_out = SharedMemory(name="shm_server_command_out")
        print("Server is running and waiting for connections...")

    def log_message(self, message):
        with open(self.log_file, "a") as log:
            log.write(message + "\n")

    def receive_command(self, shm):
        """ Receive output from a process """
        message = bytes(shm.buf[:]).decode().strip('\x00')
        return message

    def send_command(self, shm, command):
        """ Receive output from a process """
        shm.buf[:] = b'\x00' * len(shm.buf)  # Clear the buffer
        shm.buf[:len(command)] = command.encode()

    def start(self):
        while True:
            server_command_cmd = self.receive_command(self.shm_server_command_cmd)
            if server_command_cmd == "start":
                while True:
                    conn, addr = self.server_socket.accept()
                    print(f"Connected to {addr}")
                    self.log_message("Connection established")
                    self.send_command(self.shm_server_command_cmd, "connected")
                    
                    while True:
                        try:
                            data = conn.recv(1024).decode()
                            if not data:
                                break
                            print(f"Received: {data}")
                            self.send_command(self.shm_server_command_out, data)
                        except ConnectionResetError:
                            break  # Handle abrupt client disconnections
                    
                    print("Client disconnected.")
                    self.send_command(self.shm_server_command_cmd, "stop")
                    self.log_message("Connection finished")
                    conn.close()
                    break

            time.sleep(1)

    def stop(self):
        self.server_socket.close()

if __name__ == "__main__":
    server = PersistentSocketServer()
    server.start()
