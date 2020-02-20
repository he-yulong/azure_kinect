## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
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


ACC_FACTOR = 3


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

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

data_path = './face_capture/'

i = 0

sample_inteval = float(1. / sample_rate)

bs_current = None
bs_direction = None
bs_temp = None
point_temp = None
color_image = None

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
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_frame = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            bs = None
            # Stack both images horizontally
            color_image = cv2.flip(color_frame, 1, dst=None)
            flip_time = time.time() - start_t
            #print("flip time: ", flip_time)
            face_img, point = crop_face(color_image)
            face_time = time.time() - flip_time - start_t
            #print("face time: ", face_time)
            if not face_img is None:
                bs = create_data_camera_unreal(face_img)
                #bs = create_data_camera(face_img)['face_data']
                #send_socket(create_data_camera(face_img))
                bs_time = time.time() - face_time - flip_time - start_t
                #print("bs time: ", bs_time)
                point_temp = point
                color_image = cv2.rectangle(color_image, (point[2], point[0]), (point[3], point[1]), (0, 255, 0), 2)
            end_t = time.time()
            #print('time cost:', end_t - start_t)
            #if ((end_t - start_t) <= sample_inteval or bs_temp is None) and not bs is None:
            if not bs is None:
                if bs_current is None:
                    bs_current = bs
                bs_time = time.time() - face_time - flip_time - start_t
                if not bs_temp is None:
                    bs_direction = np.array(bs)- np.array(bs_current)
                bs_temp = bs
            if sample_inteval - end_t + start_t > 0:
                time.sleep(sample_inteval - end_t + start_t)
    finally:
        # Stop streaming
        pipeline.stop()


t = threading.Thread(target=face_capture)
t.start()
bs_interval = float(1. / bs_rate)
i = 0
while True:
    start_t = time.time()
    if not bs_current is None and not bs_direction is None:
        #print(len(bs_current[31], bs_direction[31])
        bs_current = _fix(bs_current + bs_direction * sample_inteval * float(bs_rate / sample_rate * ACC_FACTOR), bs_temp)
        #send_socket({"face_data": list(bs_current), "body_data": []})
        send_socket(list(bs_current))
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
        #cv2.imwrite(data_path + str(i) + '.jpg', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            #send_socket({'face_data': 'none'})
            print('frame_count: ' + str(i))
            stop_thread(t)
            break


