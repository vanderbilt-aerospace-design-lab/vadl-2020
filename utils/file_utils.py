import yaml
import numpy as np

def load_yaml(calib_file):
    with open(calib_file) as file:
        yaml_list = yaml.load(file)
        return np.array(yaml_list['camera_matrix']), np.array(yaml_list['dist_coefficients'][0])
