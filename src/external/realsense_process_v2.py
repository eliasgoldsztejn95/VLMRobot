import numpy as np
import cv2
import pyrealsense2 as rs
import sys
import time
import atexit
from multiprocessing.shared_memory import SharedMemory

# Global variables to track shared memory
shm_color = None
shm_depth = None
pipeline = None

def cleanup_resources():
    """Gracefully clean up shared memory and stop the pipeline before exiting."""
    global shm_color, shm_depth, pipeline

    print("\nCleaning up resources...")

    if pipeline:
        try:
            pipeline.stop()  # Ensure pipeline is stopped
            print("Pipeline stopped.")
        except Exception as e:
            print(f"Error stopping pipeline: {e}")

    # Safely close and unlink shared memory
    for shm, name in [(shm_color, "shm_color"), (shm_depth, "shm_depth")]:
        if shm is not None:
            try:
                shm.close()
                shm.unlink()
                print(f"Shared memory {name} cleaned up.")
            except FileNotFoundError:
                print(f"Warning: Shared memory {name} not found.")
            except Exception as e:
                print(f"Error cleaning shared memory {name}: {e}")

    print("Cleanup complete. Exiting...")
    sys.exit(0)

# Register the cleanup function to be called on program exit
atexit.register(cleanup_resources)

def realsense_process():
    global shm_color, shm_depth, pipeline

    try:
        # Initialize RealSense camera
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        pipeline.start(config)

        # Capture first frame to determine buffer size
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            print("Failed to capture initial frames!")
            return

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Create named shared memory buffers
        shm_color = SharedMemory(name="shm_color", create=True, size=color_image.nbytes)
        shm_depth = SharedMemory(name="shm_depth", create=True, size=depth_image.nbytes)

        color_buffer = np.ndarray(color_image.shape, dtype=color_image.dtype, buffer=shm_color.buf)
        depth_buffer = np.ndarray(depth_image.shape, dtype=depth_image.dtype, buffer=shm_depth.buf)

        # Initialize buffers with first frames
        color_buffer[:] = color_image
        depth_buffer[:] = depth_image

        print("Shared memory created. Running RealSense...")

        while True:
            # Capture new frames
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            # Update shared memory
            color_buffer[:] = np.asanyarray(color_frame.get_data())
            depth_buffer[:] = np.asanyarray(depth_frame.get_data())

            time.sleep(0.03)  # 30 FPS

    except KeyboardInterrupt:
        print("\nStopping RealSense process...")

    except Exception as e:
        print(f"Unexpected error: {e}")

    finally:
        cleanup_resources()  # Ensure cleanup

if __name__ == "__main__":
    realsense_process()

