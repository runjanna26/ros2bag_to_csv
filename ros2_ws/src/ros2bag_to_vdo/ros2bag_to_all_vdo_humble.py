'''
$ python3 ros2bag_to_all_vdo_humble.py ~/refine_ros2bags/refine_bag_20250703_142243  --fps 30
'''

import rclpy
import cv2
import os
import numpy as np
from tqdm import tqdm
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

def convert_all_compressed_topics_to_video(bag_path, fps=30):
    rclpy.init()

    # Prepare bag reader
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

    # Discover all CompressedImage topics
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()
    type_dict = {topic.name: topic.type for topic in topic_types}
    # reader.close()  ← REMOVE this


    compressed_topics = [t for t in type_dict if 'CompressedImage' in type_dict[t]]

    if not compressed_topics:
        print("❌ No 'CompressedImage' topics found.")
        return

    print(f"[INFO] Found {len(compressed_topics)} CompressedImage topics:")
    for topic in compressed_topics:
        print(f"  • {topic}")

    # Process each topic individually
    for topic in compressed_topics:
        print(f"\n🎬 Converting topic: {topic}")
        if compressed_topics == '/camera2/image_compressed':
            convert_single_topic(bag_path, topic, 15)
        else:
            convert_single_topic(bag_path, topic, fps)

    rclpy.shutdown()

def convert_single_topic(bag_path, topic_name, fps):
    # Set up new reader for this topic
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    # Message type
    topic_types = reader.get_all_topics_and_types()
    type_dict = {topic.name: topic.type for topic in topic_types}
    msg_type_str = type_dict[topic_name]
    msg_type = get_message(msg_type_str)

    # Output file path
    topic_clean = topic_name.strip('/').replace('/', '_')
    output_path = os.path.join(bag_path, f"{topic_clean}_output.mp4")

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = None
    frame_count = 0
    pbar = tqdm(desc=f"  → Writing {topic_clean}", unit="frame", total=None)

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != topic_name:
            continue

        msg = deserialize_message(data, msg_type)
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_img is None:
                continue

            if out is None:
                h, w, _ = cv_img.shape
                out = cv2.VideoWriter(output_path, fourcc, fps, (w, h))

            out.write(cv_img)
            frame_count += 1
            pbar.update(1)

        except Exception as e:
            pbar.write(f"⚠️ Failed to decode frame: {e}")

    pbar.close()
    if out:
        out.release()
        print(f"✅ Saved: {output_path} ({frame_count} frames)")
    else:
        print("❌ No valid frames found.")

# Entry Point
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Convert all CompressedImage topics in ROS2 bag to MP4.")
    parser.add_argument("bag_path", help="Path to ROS2 bag directory")
    parser.add_argument("--fps", type=int, default=30, help="Frame rate (default: 30)")
    args = parser.parse_args()

    convert_all_compressed_topics_to_video(args.bag_path, args.fps)
