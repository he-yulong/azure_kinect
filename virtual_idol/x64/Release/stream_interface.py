## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################q
import numpy as np
import cv2
import dlib
from predict import *
from face_crop import *
import time

def interface(img):
    # frame = cv2.resize(img, CAMERA_SIZE, interpolation=cv2.INTER_LINEAR)
    # color_frame = np.asanyarray(frame)
    color_image = cv2.flip(img, 1, dst=None)
    
    face_img, point = crop_face(color_image)
    if not face_img is None:
        #cv2.imshow("test", face_img);  # test
        #cv2.waitKey(10)
        #result = create_data_camera_unreal(face_img)
        #print(result)
        send_socket(create_data_camera(face_img))
        #return result
        return []
    return []

if __name__ == "__main__":
    img = cv2.imread('./face_capture/18.jpg')
    print(interface(img))