## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import numpy as np
import cv2
import dlib
from predict import *
from face_crop import *
import time
import threading
import sys
import ctypes
import inspect

ACC_FACTOR = 1

def _async_raise(tid, exctype):
    """raises the exception, performs cleanup if needed"""
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        # """if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")
 
 
def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)


def _fix(controller, temp):
    for i in range(len(controller)):
        controller[i] = min(temp[i], max(controller[i], temp[i]))
    return controller

data_path = './face_capture/'

i = 0

sample_inteval = float(1. / sample_rate)

bs_current = None
bs_direction = None
bs_temp = None
point_temp = None
color_image = None

vc = cv2.VideoCapture(0)

def face_capture():
    try:

        global bs_temp
        global point_temp
        global bs_direction
        global bs_current
        global color_image
        while True:
            start_t = time.time()
            # Wait for a coherent pair of frames: depth and color

            # Convert images to numpy arrays
            _, frame = vc.read()
            frame = cv2.resize(frame, CAMERA_SIZE, interpolation=cv2.INTER_LINEAR)
            color_frame = np.asanyarray(frame)

            bs = None
            # Stack both images horizontally
            color_image = cv2.flip(color_frame, 1, dst=None)
            flip_time = time.time() - start_t
            #print("flip time: ", flip_time)
            face_img, point = crop_face(color_image)
            face_time = time.time() - flip_time - start_t
            #print("face time: ", face_time)
            if not face_img is None:
                # bs = create_data_camera_unreal(face_img)
                bs = create_data_camera(face_img)['face_data']
                bs_time = time.time() - face_time - flip_time - start_t
                #print("bs time: ", bs_time)
                point_temp = point
                color_image = cv2.rectangle(color_image, (point[2], point[0]), (point[3], point[1]), (0, 255, 0), 2)
            end_t = time.time()
            if not bs is None:
                if bs_current is None:
                    bs_current = bs
                bs_time = time.time() - face_time - flip_time - start_t
                if not bs_temp is None:
                    bs_direction = np.array(bs) - np.array(bs_current)
                bs_temp = bs
            if sample_inteval - end_t + start_t > 0:
                time.sleep(sample_inteval - end_t + start_t)
    finally:
        # Stop streaming
        print('finish')


t = threading.Thread(target=face_capture)
t.start()
bs_interval = float(1. / bs_rate)
i = 0
while True:
    start_t = time.time()
    if not bs_current is None and not bs_direction is None:
        bs_current = _fix(bs_current + bs_direction * sample_inteval * float(bs_rate / sample_rate * ACC_FACTOR),
                          bs_temp)
        send_socket({"face_data": list(bs_current), "body_data": []})
        #send_socket(list(bs_current))
        end_t = time.time()
        if bs_interval - end_t + start_t > 0:
            time.sleep(bs_interval - end_t + start_t)
        #print('sleep:', bs_interval - end_t + start_t)
        i += 1
    else:
        time.sleep(bs_interval)
        print('None face captured!')
    if not color_image is None:
        cv2.imshow('', color_image)
        cv2.imwrite(data_path + str(i) + '.jpg', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            send_socket({'face_data': 'none'})
            print('frame_count: ' + str(i))
            stop_thread(t)
            break


