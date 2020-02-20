# -*- coding: utf-8 -*-
import os,re,time,json,shutil,random,threading
from datetime import datetime, timedelta
import socket

from global_config import *
import math
import numpy as np

CONTROLLER_PATH = './controller_config.txt'
ACTIVATED_PTS = {}

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

class NpEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        else:
            return super(NpEncoder, self).default(obj)

def send_socket(data):
    print((IP, port), data)
    data_ = json.dumps(data, cls=NpEncoder)
    s.sendto(data_.encode(), (IP, port))
    print('[Send offset data to controller!]')


data = "eyeBlinkRight,eyeWideRight,mouthLowerDownLeft,mouthRollUpper,cheekSquintLeft,mouthDimpleRight,browInnerUp,eyeLookInLeft,mouthPressLeft,mouthStretchRight,browDownLeft,mouthFunnel,noseSneerLeft,eyeLookOutLeft,eyeLookInRight,mouthLowerDownRight,browOuterUpRight,mouthLeft,cheekSquintRight,jawOpen,eyeBlinkLeft,jawForward,mouthPressRight,noseSneerRight,jawRight,mouthShrugLower,eyeSquintLeft,eyeLookOutRight,mouthFrownLeft,cheekPuff,mouthStretchLeft,tongueOut,mouthRollLower,mouthUpperUpRight,mouthShrugUpper,eyeSquintRight,eyeLookDownLeft,mouthSmileLeft,eyeWideLeft,mouthClose,jawLeft,mouthDimpleLeft,mouthFrownRight,mouthPucker,mouthRight,eyeLookUpLeft,browDownRight,mouthSmileRight,mouthUpperUpLeft,browOuterUpLeft,eyeLookUpRight,eyeLookDownRight".split(',')
face_data = ["browInnerUp", "browDownLeft", "browDownRight", "browOuterUpLeft", "browOuterUpRight", "eyeLookUpLeft", "eyeLookUpRight", "eyeLookDownLeft", "eyeLookDownRight", "eyeLookInLeft", "eyeLookInRight", "eyeLookOutLeft", "eyeLookOutRight", "eyeBlinkLeft", "eyeBlinkRight", "eyeSquintLeft", "eyeSquintRight", "eyeWideLeft", "eyeWideRight", "noseSneerLeft", "noseSneerRight",
"mouthLowerDownLeft","cheekSquintLeft","mouthDimpleRight","mouthPressLeft","mouthStretchRight","mouthFunnel","mouthLowerDownRight","mouthLeft","cheekSquintRight","jawOpen","jawForward","mouthPressRight","jawRight","mouthShrugLower","mouthFrownLeft","cheekPuff","mouthStretchLeft","mouthRollLower","mouthUpperUpRight","mouthShrugUpper","mouthSmileLeft","mouthClose","jawLeft","mouthDimpleLeft","mouthFrownRight","mouthPucker","mouthRight","mouthSmileRight","mouthUpperUpLeft","mouthRollUpper"]
def transform(bs_data):
    dict1 = {}
    result = []
    for i in range(len(bs_data)):
        dict1[data[i]] = bs_data[i]
    for k in range(len(face_data)):
        result.append(dict1[face_data[k]])
    return result


if __name__ == '__main__':
    bs_data = [0.0469483807682991, 0.5384506583213806, 0.5367223620414734, 0.0, 0.0, 0.2692825496196747, 0.26497596502304077, 0.0, 0.0, 0.0, 0.4391936659812927, 0.33316200971603394, 0.0, 0.0, 0.0, 0.7045563459396362, 0.704266369342804, 0.5426825881004333, 0.5434309244155884, 0.31090158224105835, 0.29444482922554016, 0.005539034027606249, 0.29620978236198425, 0.10147984325885773, 0.056318093091249466, 0.026608511805534363, 0.2682250440120697, 0.14344824850559235, 0.29246291518211365, 0.00545020867139101, 0.10158316045999527, 0.05444159358739853, 0.08055151253938675, 0.0232651699334383, 0.060809843242168427, 0.10658431053161621, 0.0, 0.025338485836982727, 0.23851308226585388, 0.05494000017642975, 0.15643011033535004, 0.30240675806999207, 0.20485065877437592, 0.01677986979484558, 0.0, 0.04870281741023064, 0.0, 0.15563185513019562, 0.0009015927207656205, 0.2264779508113861, 0.15602780878543854, 0.037170130759477615]
    #bs_data = transform(bs_data)
    print(len(bs_data))
    # send_socket(bs_data)
    send_socket({"face_data": list(bs_data[:21]) + list(bs_data[22:]), "body_data": []})
    send_socket({'face_data': 'none'})




