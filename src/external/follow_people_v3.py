import torch
import cv2
import pyrealsense2 as rs
import numpy as np
import base64
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2 import model_zoo
from multiprocessing import Process, Queue
from multiprocessing.shared_memory import SharedMemory
import time
import sys

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

    def get_frames(self):
        """ Return the current color image """
        return self.color_image, self.depth_image

    def close(self):
        """ Close the shared memory buffers """
        self.shm_color.close()
        self.shm_depth.close()
        self.shm_color.unlink()  # Fully remove from system
        self.shm_depth.unlink()
        pass

class PersonDetector:
    def __init__(self):
        cfg = get_cfg()
        cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5
        cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")
        cfg.MODEL.DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
        self.predictor = DefaultPredictor(cfg)

    def detect_people(self, color_image):
        outputs = self.predictor(color_image)
        instances = outputs["instances"]
        masks = instances.pred_masks.cpu().numpy()
        boxes = instances.pred_boxes.tensor.cpu().numpy()
        scores = instances.scores.cpu().numpy()
        classes = instances.pred_classes.cpu().numpy()
        return masks, boxes, scores, classes

    @staticmethod
    def get_average_depth(mask, depth_image):
        depth_values = depth_image[mask]
        valid_depths = depth_values[depth_values > 0]
        if valid_depths.size == 0:
            return None
        return np.mean(valid_depths)

    @staticmethod
    def calculate_center_of_mass(mask, frame_width, frame_height):
        y_coords, x_coords = np.nonzero(mask)
        if y_coords.size == 0 or x_coords.size == 0:
            return None
        center_x = int(np.mean(x_coords))
        center_y = int(np.mean(y_coords))
        relative_x = center_x - frame_width // 2
        relative_y = center_y - frame_height // 2
        return relative_x, relative_y


class FollowPerson:
    def __init__(self):
        # Camera
        self.camera = RealSenseClient()
        self.detector = PersonDetector()

        self.shm_follow_cmd = SharedMemory(name="shm_follow_cmd")
        self.shm_follow_out = SharedMemory(name="shm_follow_out")

        # Robot
        self.vel_linear = 0
        self.vel_angular = 0

        # Sign
        self.min_dist = 1000               # Stop if closer than 1 meter but keep following / otherwise follow
        self.max_dist = 5000               # Stop following if person detected is more than 5 meters away
        self.environment = None            # Store environment type

        # Tracking
        self.following = True         # Is following person
        self.last_tracked_position = None  # Store last known position (avg_depth, relative_x, relative_y)
        self.last_detection_time = -1   # Timestamp of the last detection
        self.tracking_threshold = 10        # Threshold in seconds for reassigning tracking

    def receive_command(self, shm):
        """ Receive output from a process """
        message = bytes(shm.buf[:]).decode().strip('\x00')
        return message

    def send_command(self, shm, command):
        """ Receive output from a process """
        shm.buf[:] = b'\x00' * len(shm.buf)  # Clear the buffer
        shm.buf[:len(command)] = command.encode()

    def follow_person(self, relative_x, relative_y, avg_depth, environment):
        # Calculate linear and angular velocities using "Follow the Carrot"
        max_linear_speed = 1.5 if environment in ["park", "open space"] else 1.0
        self.vel_linear = max_linear_speed
        self.vel_angular = -20*relative_x / avg_depth
        print(f"Following person in environment: {environment}, Linear Velocity = {self.vel_linear}, Angular Velocity = {self.vel_angular}")
        self.send_command(self.shm_follow_out, f"{self.vel_linear} {self.vel_angular}")

    def stop_robot(self):
        self.vel_linear = 0
        self.vel_angular = 0
        print(f"Stopping: Linear Velocity = {self.vel_linear}, Angular Velocity = {self.vel_angular}")
        self.send_command(self.shm_follow_out, f"{self.vel_linear} {self.vel_angular}")

    def find_closest_person(self, masks, classes, depth_image, boxes, frame_width, frame_height):
        closest_person = None
        closest_box = None
        closest_distance = float('inf')
        closest_position = None

        for i, mask in enumerate(masks):
            if classes[i] == 0:  # COCO class 0 is 'person'
                avg_depth = self.detector.get_average_depth(mask, depth_image)
                center_of_mass = self.detector.calculate_center_of_mass(mask, frame_width, frame_height)
                if avg_depth is not None and center_of_mass is not None:
                    relative_x, relative_y = center_of_mass
                    # Compare with the last known position
                    if self.last_tracked_position:
                        last_avg_depth, last_x, last_y = self.last_tracked_position
                        distance = np.sqrt((avg_depth - last_avg_depth)**2 + (relative_x - last_x)**2 + (relative_y - last_y)**2)
                    else:
                        distance = avg_depth

                    if distance < closest_distance:
                        closest_distance = distance
                        closest_person = (avg_depth, relative_x, relative_y)
                        closest_position = center_of_mass
                        closest_box = boxes[i]

        return closest_person, closest_position, closest_box

    def is_tracking_time_within_threshold(self):
        if (time.time() - self.last_detection_time) < self.tracking_threshold:
            self.following = True
        else:
            self.following = False
    
    def restart_variables(self):
        self.following = True         # Is following person
        self.last_tracked_position = None  # Store last known position (avg_depth, relative_x, relative_y)
        self.last_detection_time = -1   # Timestamp of the last detection

    def run(self):
        try:
            while True:
                follow_cmd = self.receive_command(self.shm_follow_cmd)
                self.restart_variables()
                color_image, _ = self.camera.get_frames()
                while follow_cmd == "start":
                    # Get follow command
                    follow_cmd = self.receive_command(self.shm_follow_cmd)
                    # Get Realsense image
                    color_image, depth_image = self.camera.get_frames()
                    if color_image is None or depth_image is None:
                        continue

                    # Use Detectron2 to segment people
                    masks, boxes, scores, classes = self.detector.detect_people(color_image)
                    frame_height, frame_width = color_image.shape[:2]

                    # Find closest person relative to last tracked person
                    closest_person, closest_position, closest_box = self.find_closest_person(masks, classes, depth_image, boxes, frame_width, frame_height)

                    if closest_person:
                        avg_depth, relative_x, relative_y = closest_person
                        x1, y1, x2, y2 = map(int, closest_box)  # Assuming `boxes` are for detected persons
                        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        label = f"Person: {avg_depth / 1000:.2f}m, CoM: ({relative_x}, {relative_y})"
                        cv2.putText(color_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.circle(color_image, (relative_x + frame_width // 2, relative_y + frame_height // 2), 5, (0, 0, 255), -1)

                        if avg_depth is not None and relative_x is not None and self.following: # If person detected and following
                            
                            # Stop robot
                            if avg_depth <= self.min_dist:
                                print("Update track time and position")
                                self.last_tracked_position = (avg_depth, relative_x, relative_y)
                                self.last_detection_time = time.time()

                                print("Stop because person too close")
                                self.stop_robot()

                            # Follow person
                            elif avg_depth > self.min_dist and avg_depth < self.max_dist:
                                print("Update track time and position")
                                self.last_tracked_position = (avg_depth, relative_x, relative_y)
                                self.last_detection_time = time.time()

                                print("Follow person")
                                self.follow_person(relative_x, relative_y, avg_depth, self.environment)

                            # Stop robot
                            if avg_depth >= self.max_dist:
                                print("Stop because person is too far")
                                self.stop_robot()
                                self.is_tracking_time_within_threshold()

                        # Stop robot
                        else: # Already passed time
                            print("Lost track completely")
                            self.stop_robot()
                            self.send_command(self.shm_follow_cmd, "stop")

                    # Stop robot
                    else: # If no person tracked
                        if self.following: # If still withing time threshold
                            print("Stop because no people in picture")
                            self.stop_robot()
                            self.is_tracking_time_within_threshold()

                        # Stop robot
                        else: # Already passed time
                            print("Lost track completely")
                            self.stop_robot()
                            self.send_command(self.shm_follow_cmd, "stop")

                    cv2.imshow("RealSense Color", color_image)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                time.sleep(1)
                cv2.imshow("RealSense Color", color_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.camera.stop()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    system = FollowPerson()
    system.run()
