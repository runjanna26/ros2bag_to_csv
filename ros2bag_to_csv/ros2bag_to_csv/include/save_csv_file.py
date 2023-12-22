#!/usr/bin/env python3

import csv
import json
import time
import numpy as np

def save_csv_file(data, csv_file_name, version=0, print_out=False):
    """ Save data to a csv_file_name (use it after 'read_from_all_topics').
    """
    
    # Create csv file
    with open(csv_file_name, mode='w') as csv_file:

        field_names = ['topic_name', 'topic_type', 'time_stamp', 'message']
        writer = None

        nt,line_datas  =  data[0],data[1]
        for index in range(len(line_datas)):
            row_time,row_data  = nt[index],line_datas[index]
            row_time =  '{}.{}'.format(time.strftime('%Y/%m/%d %H:%M:%S', time.localtime(row_time / 1000 / 1000 / 1000)), row_time % (1000 * 1000 * 1000))
            if writer is None:
                field_names = ['time']+list(row_data.keys())
                writer = csv.DictWriter(csv_file,fieldnames=field_names)
                writer.writeheader()
            row_data["time"] = row_time
            writer.writerow(row_data)
    print('Saving', csv_file_name)
