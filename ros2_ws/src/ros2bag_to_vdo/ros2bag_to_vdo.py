import rclpy
import cv2
import numpy as np
import sqlite3
import struct
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import os


# Configuration
BAG_FILE        = os.path.expanduser("/home/refine2/refine_ros2bags/refine_bag_20250703_134401/refine_bag_20250703_134401_0.db3")
IMAGE_TOPIC     = "/camera1/image_compressed"
OUTPUT_VIDEO    = "output.mp4"
FPS = 10  # Adjust FPS as needed

bridge = CvBridge()

def extract_images_from_bag(bag_file, image_topic):
    """ Extract images from a ROS 2 bag (Foxy-compatible method using sqlite3) """
    conn = sqlite3.connect(bag_file)
    cursor = conn.cursor()

    # Get topic ID for the image topic
    cursor.execute("SELECT id FROM topics WHERE name=?", (image_topic,))
    topic_id_row = cursor.fetchone()
    if topic_id_row is None:
        print(f"[ERROR] Topic '{image_topic}' not found in bag.")
        return []
    
    topic_id = topic_id_row[0]
    print(f"Found topic ID: {topic_id}")


    # cursor.execute("SELECT id, name FROM topics")
    # all_topics = cursor.fetchall()
    # print("Available topics:")
    # for topic_id, name in all_topics:
    #     print(f"ID: {topic_id}, Name: {name}")

    # Fetch messages from the topic
    cursor.execute("SELECT data FROM messages WHERE topic_id=?", (topic_id,))
    messages = cursor.fetchall()

    images = []
    for msg_data, in messages:
        print(f"Raw message data size: {len(msg_data)} bytes")
        try:
            # Deserialize the ROS 2 Image message
            msg = deserialize_message(msg_data, get_message("sensor_msgs/msg/Image"))
            img = bridge.imgmsg_to_cv2(msg, "bgr8")
            images.append(img)
        except Exception as e:
            print(f"[ERROR] Failed to convert image: {e}")

    
    print(f"Extracted {len(images)} images.")
    conn.close()
    return images

def save_video(images, output_video, fps):
    """ Save images as a video """
    if not images:
        print("[ERROR] No images to save as video.")
        return
    
    height, width, _ = images[0].shape
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

    for img in images:
        out.write(img)

    out.release()
    print(f"[INFO] Video saved: {output_video}")

def main():
    rclpy.init()
    
    print("[INFO] Extracting images from bag...")
    images = extract_images_from_bag(BAG_FILE, IMAGE_TOPIC)
    
    if images:
        print(f"[INFO] {len(images)} images extracted. Saving to video...")
        save_video(images, OUTPUT_VIDEO, FPS)
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
