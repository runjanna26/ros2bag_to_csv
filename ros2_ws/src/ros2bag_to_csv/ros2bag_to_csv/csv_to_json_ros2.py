#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import os
import sys
import pandas as pd
from datetime import datetime
import json

directory_path = '.'

# Initialize parameters
argvs = sys.argv
argc = len(argvs)

dfs = {}
topic_name = []
null_topic_name = []
result_list = []
name_json_file = ''
def list_csv_files(directory_path="."):
    csv_files = [filename for filename in os.listdir(directory_path) if filename.endswith(".csv")]
    return csv_files

def convert_df_to_dict(df, column_name):
   '''
   - convert dataframe to dictionary change string type to float type of data
   - convert datetime to second according to record time
   '''
   init_ts = convert_timestamp_str_to_datetime(df['time'][0])
   data_list = []
   for i in range(len(df['time'])):
      val = [float(j) for j in df['data'][i][1:-1].split(', ')]
      ts = (convert_timestamp_str_to_datetime(df['time'][i]) - init_ts).total_seconds()
      data_list.append({'val': val, 'ts': ts})
   return {column_name: data_list}



def convert_timestamp_str_to_datetime(timestamp_str):
   '''
   Convert string of time from ros2bag to datetime object
   - ros2bag record with nano second type
   '''
   date_part, nanosecond_part = timestamp_str.split('.')  
   microsecond = str(int(float(nanosecond_part)//1000.0)).zfill(6)
   timestamp_str_modified = f"{date_part}.{microsecond}"  
   datetime_obj = datetime.strptime(timestamp_str_modified, "%Y/%m/%d %H:%M:%S.%f")   
   
   return datetime_obj

def main(args=None):
   csv_files = list_csv_files(directory_path)
   if csv_files:
      print("===[List of .csv files in the directory]===")
      for csv_file in csv_files:
         print(csv_file)

         file_path = os.path.join(directory_path, csv_file)
         file_size = os.path.getsize(file_path)  # Get size of CSV file

         threshold_size = 0  # 0 MB == empty file
         if file_size > threshold_size:  # Check if file size is greater than threshold
            df = pd.read_csv(file_path)
   
            topic_name.append(csv_file.split('-')[1].replace(".csv", ""))
            name_json_file = csv_file.split('-')[0].replace(".db3", "")
            prefix = csv_file.split('-')[1].replace(".csv", "")

            dfs[prefix] = df
         else:
            null_topic_name.append(csv_file.split('-')[1].replace(".csv", ""))
      print("===[List of EMPTY topic]===")
      print(null_topic_name)
      print('===========================================')
      
      for i in range(len(topic_name)):
         result_list.append(convert_df_to_dict(dfs[topic_name[i]], topic_name[i]))
         
      # Merge list of dict to one dict
      merged_dict = result_list[0].copy()
      for d in result_list:
          for key, value in d.items():
              merged_dict[key] = value

      # Convert to JSON file
      with open(name_json_file + ".json", "w") as outfile:
          json.dump(merged_dict, outfile)

   else:
      print("No .csv files found in the directory.")



   
   rclpy.init(args=args)
   minimal_publisher = MinimalPublisher()

   minimal_publisher.destroy_node()
   rclpy.shutdown()




class MinimalPublisher(Node):

    def __init__(self):
      super().__init__('csv_to_json_node')
      # my_param = self.get_parameter('/ros_version').get_parameter_value().string_value

      # self.get_logger().info('Hello %s!' % my_param)
        






if __name__ == '__main__':
    main()