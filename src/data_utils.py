import cv2
import numpy as np
import pandas as pd

def read_camera(path):
    img = cv2.imread(path)
    return img

def read_point_cloud(path):
    pcl = np.fromfile(path, dtype=np.float32)
    pcl = pcl.reshape(-1, 4)
    return pcl

def read_imu(path):
    pass

def read_gpu(path):
    pass
