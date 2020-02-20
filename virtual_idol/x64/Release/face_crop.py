import dlib
import numpy as np
import cv2
import math
from global_config import *


face_detector = dlib.get_frontal_face_detector()
dlib_landmark_model = dlib_landmarks_fp
face_regressor = dlib.shape_predictor(dlib_landmark_model)

#make sure shape of the image is (height, width, channel)
def crop_face(img_ori):
    image = None
    shape = img_ori.shape
    img_ = cv2.resize(img_ori, (int(shape[1] / scale_size), int(shape[0] / scale_size)),
                      interpolation=cv2.INTER_LINEAR)
    
    rects = face_detector(img_, 1)
    if len(rects) == 0:
        return None, None
    rect = rects[0]
    rect_ = [int(rect.left() * scale_size), int(rect.top() * scale_size), int(rect.right() * scale_size),
             int(rect.bottom() * scale_size)]
    
    rect_[0] = max(rect_[0], 0)
    rect_[1] = max(rect_[1], 0)
    w = rect_[2] - rect_[0]
    h = rect_[3] - rect_[1]

    size = max(w, h) / 2
    zero = [(rect_[2] + rect_[0]) / 2, (rect_[3] + rect_[1]) / 2]
    
    if w != h:
        rect_[0] = max(int(zero[0] - size), 0)
        rect_[2] = int(zero[0] + size)

        rect_[1] = max(int(zero[1] - size), 0)
        rect_[3] = int(zero[1] + size)
    rect_ = dlib.rectangle(rect_[0], rect_[1], rect_[2], rect_[3])
    pts = face_regressor(img_ori, rect_).parts()
    pts = np.array([[pt.x, pt.y] for pt in pts])
    w = (pts[16][0] - pts[0][0])
    h = int(1.5 * (pts[8][1] - pts[27][1]))
    size = int(max(h, w) / 2)
    img = img_ori[max(pts[30][1] - size, 0): pts[30][1] + size, max(pts[30][0] - size, 0): pts[30][0] + size]
    image = img
    return image, [pts[30][1] - size, pts[30][1] + size, pts[30][0] - size, pts[30][0] + size]
