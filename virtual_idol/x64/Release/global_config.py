import torch
import os
import sys
import json
#import urllib

root_path=os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + os.path.sep + ".")+'/'
sys.path.append(root_path)
root_path=os.path.join(os.getcwd(), "")
sys.path.append(root_path)
root_path = ''

weights = open("face_config.json", encoding='utf-8')
face_weights = json.load(weights)
config = open("global_config.json", encoding='utf-8')
global_config = json.load(config)

dlib_landmarks_fp = root_path + 'models/shape_predictor_68_face_landmarks.dat'
bs_model = root_path + 'models/bs_model.pth'

IMAGE_SIZE = global_config["IMAGE_SIZE"]
CAMERA_SIZE = tuple(global_config["CAMERA_SIZE"])
scale_size = global_config["scale_size"]
sample_rate = global_config["sample_rate"]
bs_rate = global_config["bs_rate"]
IP = global_config["IP"]
port = global_config["port"]

USE_GPU = (global_config["USE_GPU"] == 1)
if USE_GPU:
    os.environ["CUDA_VISIBLE_DEVICES"] = global_config["DEVICE_INDEX"]
#torch.cuda.is_available()

'''
IMAGE_SIZE = 192

weights = open("face_config.json", encoding='utf-8')
face_weights = json.load(weights)

CAMERA_SIZE = (600, 480)

USE_GPU = torch.cuda.is_available()

dlib_landmarks_fp = root_path + 'models/shape_predictor_68_face_landmarks.dat'
# if not os.path.exists(dlib_landmarks_fp):
#     urllib.urlretrieve("", dlib_landmarks_fp)

scale_size = 6

bs_model = root_path + 'models/bs_model.pth'
# if not os.path.exists(bs_model):
#     urllib.urlretrieve("", bs_model)

#sample rate: the fps of model 
sample_rate = 7
bs_rate = 30

IP = '127.0.0.1'
#IP = '172.27.40.135'
port = 8999
'''

