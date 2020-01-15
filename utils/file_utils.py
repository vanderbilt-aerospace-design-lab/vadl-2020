import yaml
import numpy as np
import datetime
import os

def load_yaml(calib_file):
    with open(calib_file) as file:
        yaml_list = yaml.load(file)
        return np.array(yaml_list['camera_matrix']), np.array(yaml_list['dist_coefficients'])

# Generates a file name based off the current date and time.
def create_file_name_date():
    date = datetime.datetime.now()
    return str(date.year) + "_" + str(date.month) + "_" \
           + str(date.day) + "_" + str(date.hour) + "_" + str(date.minute)

# Assumes files in directory are numbered chronologically with the same base name
def create_file_name_chronological(data_dir, file_base, ext):
    # Find next video file name
    ct = 0
    for dirpath, dirnames, filenames in os.walk(data_dir):
        for file in filenames:
            ct += 1

    return data_dir + "/" + file_base + "_{}.".format(ct) + ext

def open_file(dir, file):
    if not os.path.exists(dir):
        os.mkdir(dir)

    return open(file, "w")