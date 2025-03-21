import rclpy
import cv2
import numpy as np
import sqlite3
import struct
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import os

# Configuration
BAG_FILE        = os.path.expanduser("/media/runj/RunJ SSD/RunJ/VISTEC/0_Industrials Projects/4_Project_Freelander_HERO/HERO REC DEMO/Day3/rosbag/mt_demo_rec_day3_1/mt_demo_rec_day3_1_0.db3")
IMAGE_TOPIC     = "/HERO/InspectionCamera/stream_images"
OUTPUT_VIDEO    = "output.mp4"
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

def main():
    rclpy.init()
    extract_and_save_video(BAG_FILE, IMAGE_TOPIC, OUTPUT_VIDEO, FPS)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
