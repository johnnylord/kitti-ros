import cv2
import numpy as np
import pandas as pd

VEHICLES = [ 'Van', 'Car', 'Tram', 'Truck' ]
OBJECTS_OF_INTRESET = [ 'Pedestrian', 'Cyclist' ] + VEHICLES

TRACKING_COLUMN = [
    'frame', 'tid', 'type', 'truncated', 'occluded', 'alpha',
    'xmin', 'ymin', 'xmax', 'ymax',
    'height', 'width', 'length',
    'pos_x', 'pos_y', 'pos_z', 'rotate_y', ]

COLOR_MAP = {
    'Car': (255, 0, 0),
    'Pedestrian': (0, 255, 0),
    'Cyclist': (0, 0, 255) }

def read_camera(path, bboxes, types):
    img = cv2.imread(path)
    for bbox, type_ in zip(bboxes, types):
        pt1 = tuple([ int(v) for v in bbox[:2]])
        pt2 = tuple([ int(v) for v in bbox[2:]])
        color = COLOR_MAP[type_]
        img = cv2.rectangle(img, pt1, pt2, color=color, thickness=3)
    return img

def read_point_cloud(path):
    pcl = np.fromfile(path, dtype=np.float32)
    pcl = pcl.reshape(-1, 4)
    return pcl

def read_tracking(path):
    # Read tracking result
    df = pd.read_csv(path, header=None, sep=" ")
    df.columns = TRACKING_COLUMN

    # Normalize & Filter result
    df.loc[df.type.isin(VEHICLES), 'type'] = 'Car'
    df = df[df.type.isin(OBJECTS_OF_INTRESET)]

    return df
