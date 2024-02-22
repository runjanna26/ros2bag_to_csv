#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from .include.read_bag import *
import sys, os

argvs = sys.argv
argc = len(argvs)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('ros2bag_to_csv_node')
        

def main(args=None):
    db3_file = find_db3_file()
    # print(argvs)
    # print(len(argvs))

    if db3_file:
        print(f"The .db3 file found is: {db3_file}")
    else:
        print("No .db3 file found in the directory.")

    # file_url = argvs[1]
    read_write_from_all_topics(find_db3_file(),True)

    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


