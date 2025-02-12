import socket
import cv2
import pyrealsense2 as rs
import numpy as np
import pickle
import struct
from multiprocessing.shared_memory import SharedMemory

class RealSenseClient:
    def __init__(self):
        self.shm_color = SharedMemory(name="shm_color")
        self.shm_depth = SharedMemory(name="shm_depth")
        self.color_shape = (480, 640, 3)
        self.depth_shape = (480, 640)
        self.color_image = np.ndarray(self.color_shape, dtype=np.uint8, buffer=self.shm_color.buf)
        self.depth_image = np.ndarray(self.depth_shape, dtype=np.uint16, buffer=self.shm_depth.buf)

    def get_frames(self):
        return self.color_image, self.depth_image

    def close(self):
        self.shm_color.close()
        self.shm_depth.close()
        self.shm_color.unlink()
        self.shm_depth.unlink()

class StreamingServer:
    def __init__(self, host='0.0.0.0', port=5004):
        self.host = host
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        self.camera = RealSenseClient()
        self.shm_server_camera_cmd = SharedMemory(name="shm_server_camera_cmd")

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
            server_camera_cmd = self.receive_command(self.shm_server_camera_cmd)
            if server_camera_cmd == "start":
                while True:
                    print("Waiting for connection...")
                    conn, addr = self.server_socket.accept()
                    print(f"Connected to {addr}")
                    self.send_command(self.shm_server_camera_cmd, "connected")
                    
                    while True:
                        try:
                            color_frame, _ = self.camera.get_frames()
                            data = pickle.dumps(color_frame)
                            message_size = struct.pack("Q", len(data))
                            conn.sendall(message_size + data)
                        except ConnectionResetError:
                            break  # Handle abrupt client disconnections
                    print("Client disconnected.")
                    self.send_command(self.shm_server_camera_cmd, "stop")
                    conn.close()
                    break

    def stop(self):
        self.server_socket.close()
        self.camera.close()

if __name__ == "__main__":
    server = StreamingServer()
    server.start()
