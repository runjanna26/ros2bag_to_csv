import rclpy
import cv2
import numpy as np
import sqlite3
import struct
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rclpy.node import Node
import os

from .include.read_bag import *


'''
How to extract vdo from ros2 bag (foxy)

$ cd .db3 directory folder
$ ros2 run ros2bag_to_csv ros2bag_to_vdo_compressed --ros-args -p topic:=/HERO/InspectionCamera/stream_images -p fps:=10
'''

# Configuration
FPS = 10  # Adjust FPS as needed

bridge = CvBridge()

def extract_and_save_video(bag_file, image_topic, output_video, fps):
    """ Extracts compressed images from a ROS 2 bag and writes directly to video """
    conn = sqlite3.connect(bag_file)
    cursor = conn.cursor()

    # Get topic ID
    cursor.execute("SELECT id FROM topics WHERE name=?", (image_topic,))
    topic_id_row = cursor.fetchone()
    if topic_id_row is None:
        print(f"[ERROR] Topic '{image_topic}' not found in bag.")
        return
    
    topic_id = topic_id_row[0]
    print(f"Found topic ID: {topic_id}")

    # Fetch messages from the topic
    # Fetch all messages from the topic
    cursor.execute("SELECT data FROM messages WHERE topic_id=?", (topic_id,))
    messages = cursor.fetchall()
    total_frames = len(messages)

    if total_frames == 0:
        print("[ERROR] No images found in the bag file.")
        return

    print(f"[INFO] {total_frames} images found. Processing...")

    # Initialize video writer with first image size
    first_msg = deserialize_message(messages[0][0], get_message("sensor_msgs/msg/CompressedImage"))
    first_img = cv2.imdecode(np.frombuffer(first_msg.data, np.uint8), cv2.IMREAD_COLOR)
    height, width, _ = first_img.shape
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

    print(f"[INFO] Writing video to {output_video}")

    frame_count = 0
    for msg_data, in messages:
        try:
            msg = deserialize_message(msg_data, get_message("sensor_msgs/msg/CompressedImage"))
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is not None:
                out.write(img)

                frame_count += 1

                # Show progress every 100 frames
                if frame_count % 100 == 0:
                    print(f"[INFO] Processed {frame_count}/{total_frames} frames...")

        except Exception as e:
            print(f"[ERROR] Failed to decode image: {e}")

    out.release()
    conn.close()
    print("[INFO] Video saved successfully.")

def main(args=None):
    topic_name = None
    fps = None

    db3_file = find_db3_file()
    if db3_file:
        print(f"The .db3 file found is: {db3_file}")
    else:
        print("No .db3 file found in the directory.")

    
    rclpy.init(args=args)
    node = Node("ros2bag_to_vdo_compressed")  # Create a Node instance

    # Read the topic parameter
    node.declare_parameter('topic', None)
    node.declare_parameter('fps', 0)
    if node.has_parameter("topic"):  
        topic_name = node.get_parameter("topic").value
    else:
        print("[ERROR] Missing parameter: topic")
        return
    
    
    if node.has_parameter("fps"):  
        fps = node.get_parameter("fps").value
    else:
        print("[ERROR] Missing parameter: fps")
        return

    print(f"[INFO] Extracting from topic: {topic_name} with {fps} FPS")

    extract_and_save_video(db3_file, topic_name, db3_file.split(".")[0] + '.mp4', fps)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
