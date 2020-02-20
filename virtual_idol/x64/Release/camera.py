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


data_path = './face_capture/'

i = 0

vc = cv2.VideoCapture(0)

try:
    while True:
        start = time.time()
        # Wait for a coherent pair of frames: depth and color
        _, frame = vc.read()
        frame = cv2.resize(frame, CAMERA_SIZE, interpolation=cv2.INTER_LINEAR)
        color_frame = np.asanyarray(frame)
        # Stack both images horizontally
        color_image = cv2.flip(color_frame, 1, dst=None)

        face_img, point = crop_face(color_image)
        if not face_img is None:
            #send_socket(create_data_camera(face_img))
            send_socket(create_data_camera_unreal(face_img))
            #cv2.imwrite(data_path + str(i) + '.jpg', color_image)
            color_image = cv2.rectangle(color_image, (point[2], point[0]), (point[3], point[1]), (0, 255, 0), 2)   
            i += 1
        cv2.imshow('frame', color_image)
        end = time.time()
        print('frame time cost:' + str(end - start))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            send_socket({'face_data': 'none'})
            print('frame_count: ' + str(i))
            break


finally:

    # Stop streaming
    print('finish')
