'''
$ python3 ros2bag_to_vdo_humble.py ~/refine_ros2bags/refine_bag_20250703_142243 /camera1/image_compressed --fps 30
'''

import rclpy
import cv2
import os
import numpy as np
from tqdm import tqdm
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

def bag_to_video_compressed(bag_path, topic_name, output_path="output.mp4", fps=30):
    rclpy.init()

    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_dict = {topic.name: topic.type for topic in topic_types}
    
    if topic_name not in type_dict:
        raise RuntimeError(f"Topic '{topic_name}' not found in bag.")

    msg_type_str = type_dict[topic_name]
    msg_type = get_message(msg_type_str)

    if 'CompressedImage' not in msg_type_str:
        raise RuntimeError(f"Topic '{topic_name}' is not a CompressedImage.")

    print(f"[INFO] Starting fast conversion: {topic_name} → {output_path}")

    fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Faster codec
    out = None
    frame_count = 0

    pbar = tqdm(desc="Converting frames", unit="frame", total=None)

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
            pbar.write(f"⚠️ Error decoding frame: {e}")

    pbar.close()
    if out:
        out.release()
        print(f"✅ Done: {frame_count} frames → {output_path}")
    else:
        print("❌ No valid frames written.")

    rclpy.shutdown()


# Entry point
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Convert ROS2 bag CompressedImage topic to MP4 with progress.")
    parser.add_argument("bag_path", help="Path to ROS2 bag folder.")
    parser.add_argument("topic_name", help="Topic name (e.g., /camera/image/compressed)")
    parser.add_argument("--output", help="Output video file path. If not given, saves to bag folder.", default=None)
    parser.add_argument("--fps", type=int, default=30, help="Frame rate")

    args = parser.parse_args()
    if args.output is None:
        topic_clean = args.topic_name.strip('/').replace('/', '_')  # e.g. /camera/image -> camera_image
        args.output = os.path.join(args.bag_path, f"{topic_clean}_output.mp4")
    bag_to_video_compressed(args.bag_path, args.topic_name, args.output, args.fps)
