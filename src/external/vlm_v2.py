import json
import os
import time
import cv2
from multiprocessing import Process, Queue
from multiprocessing.shared_memory import SharedMemory
import numpy as np
import base64
from groq import Groq
import re
import copy
import sys

dir_path = os.path.dirname(os.path.realpath(__file__))

def cleanup_on_exit(signum, frame):
    """ Cleanup shared memory and exit gracefully """
    print("\nCleaning up shared memory and exiting...")
    system.camera.close()  # Properly close shared memory
    sys.exit(0)

class RealSenseClient:
    def __init__(self):
        # Attach to shared memory for color and depth images
        self.shm_color = SharedMemory(name="shm_color")
        self.shm_depth = SharedMemory(name="shm_depth")
        
        # Define shapes for color and depth images
        self.color_shape = (480, 640, 3)  # Color image shape
        self.depth_shape = (480, 640)     # Depth image shape
        
        # Create buffers from shared memory
        self.color_image = np.ndarray(self.color_shape, dtype=np.uint8, buffer=self.shm_color.buf)
        self.depth_image = np.ndarray(self.depth_shape, dtype=np.uint16, buffer=self.shm_depth.buf)

    def update_images(self):
        """ Continuously update the images from shared memory """
        pass  # This function can be used if processing is needed

    def get_color_image(self):
        """ Return the current color image """
        return self.color_image

    def close(self):
        """ Close the shared memory buffers """
        self.shm_color.close()
        self.shm_depth.close()
        self.shm_color.unlink()  # Fully remove from system
        self.shm_depth.unlink()
        pass

class VisualLanguageModel:
    def __init__(self, api_key):
        self.client = Groq(api_key=api_key)

    def process_image(self, image):
        image_path = "captured_image.jpg"
        cv2.imwrite(image_path, image)
        with open(image_path, "rb") as image_file:
            base64_image = base64.b64encode(image_file.read()).decode('utf-8')

        chat_completion = self.client.chat.completions.create(
            model="llama-3.2-11b-vision-preview",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": "In JSON format fill only the following fields: <environment> -office/open space/corridor/caffeteria/crowded area/park,  <object close> -table/elevator/stairs/chair/etc., <people> -true/false, <looking to camera> -true/false, <raising one hand> -true/false, <raising two hands> -true/false in JSON format"},
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"},
                        },
                    ],
                }
            ],
            temperature=1,
            max_tokens=1024,
            top_p=1,
            stream=False,
            response_format={"type": "json_object"},
            stop=None,
        )
        return chat_completion.choices[0].message.content


class ReportVLM:
    def __init__(self, api_key):
        # Initialize the camera and VLM
        self.camera = RealSenseClient()
        self.vlm = VisualLanguageModel(api_key)
        self.shm_vlm_out = SharedMemory(name="shm_vlm_out")

    def send_command(self, data):
        json_data = json.dumps(data)  # Convert dictionary to JSON string
        encoded_data = json_data.encode()  # Encode string to bytes

        if len(encoded_data) > self.shm_vlm_out.size:
            raise ValueError("Data is too large for shared memory buffer!")

        self.shm_vlm_out.buf[:len(encoded_data)] = encoded_data  # Write to shared memory
        self.shm_vlm_out.buf[len(encoded_data):] = b'\x00' * (self.shm_vlm_out.size - len(encoded_data))  # Clear remaining bytes

    def validate_and_filter_vlm_response(self, vlm_response):
        """
        Validates and filters a VLM response JSON structure.
        Args:
            vlm_response (str or dict): The VLM response as a JSON string or object.
        Returns:
            tuple: (bool, dict) - Whether the response is valid and the filtered JSON.
        """
        # Define the expected keys and their data types
        expected_structure = {
            "environment": str,
            "object close": str,
            "people": bool,
            "looking to camera": bool,
            "raising one hand": bool,
            "raising two hands": bool,
        }
        
        # Function to normalize keys
        def normalize_key(key):
            return re.sub(r'[-_\s]+', ' ', key.strip().lower())
        
        try:
            # Parse JSON if input is a string
            if isinstance(vlm_response, str):
                vlm_response = json.loads(vlm_response)
            elif not isinstance(vlm_response, dict):
                return False, None  # Invalid input type
            
            # Validate and normalize the structure
            filtered_data = {}
            for expected_key, expected_type in expected_structure.items():
                normalized_key = normalize_key(expected_key)
                
                # Check if a corresponding key exists (flexible formatting)
                matching_keys = [key for key in vlm_response.keys() if normalize_key(key) == normalized_key]
                if not matching_keys:
                    return False, None  # Missing key
                
                # Validate the type of the value
                if not isinstance(vlm_response[matching_keys[0]], expected_type):
                    return False, None
                
                # Add the normalized key-value pair to the filtered data
                filtered_data[expected_key] = vlm_response[matching_keys[0]]
            
            # Return True and the filtered data
            return True, filtered_data
        
        except (json.JSONDecodeError, ValueError) as e:
            # Handle parsing errors gracefully
            print(f"Error: {e}")
            return False, None

    def save_json(self, vlm_data):
        """ Saves the VLM response as a JSON file """
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        file_name = dir_path + "/vlm_response.json"
        with open(file_name, "w") as f:
            json.dump(vlm_data, f, indent=4)
        print(f"JSON file saved as {file_name}")

    def run(self):
        try:
            while True:
                # Capture color image from the camera
                color_image = self.camera.get_color_image()

                try:
                    # Simulate fetching a VLM response
                    vlm_response = self.vlm.process_image(color_image)
                    print(vlm_response)
                    # Validate and filter the response
                    correct_response, vlm_data = self.validate_and_filter_vlm_response(vlm_response)
                    
                    if correct_response:
                        self.send_command(vlm_data)
                        self.save_json(vlm_data)  # Save the valid JSON response
                    else:
                        print("Invalid VLM Response. Retrying...")
                except Exception as e:
                    # Handle any exceptions from VLM processing
                    print(f"Error occurred: {e}. Retrying...")
                    time.sleep(1)  # Optional: Add a delay to prevent rapid retries
        except KeyboardInterrupt:
            print("Stopped the VLM report generation.")

        finally:
            self.camera.close()

if __name__ == "__main__":
    api_key = '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'  # Replace with your API key
    system = ReportVLM(api_key)
    #system.run()

    system.run()
