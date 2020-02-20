from fine_tune_model import *
from global_config import *
import torch
import cv2
import sys
import os
from PIL import Image
from torch.autograd import Variable
import numpy as np
from udp_sender import *
import time
import math

from torchvision import datasets, models, transforms

model = blenderNet(USE_GPU)
model.eval()

normalize_mean = [0.485, 0.456, 0.406]
normalize_std = [0.229, 0.224, 0.225]

data_transforms = transforms.Compose([
    transforms.Scale(IMAGE_SIZE),
    transforms.CenterCrop(IMAGE_SIZE),
    transforms.ToTensor(),
    transforms.Normalize(normalize_mean, normalize_std)
])

if USE_GPU:
    model.load_state_dict(torch.load(bs_model))
else:
    model.load_state_dict(torch.load(bs_model, map_location=lambda storage, loc: storage))


def weighted_bs(bs_data):
    bs_data[13] = 0.5 * math.tanh(face_weights["left_eye"]["alpha"] * bs_data[13] - face_weights["left_eye"]["beta"]
                                  * face_weights["left_eye"]["power"]) + 0.5
    bs_data[14] = 0.5 * math.tanh(face_weights["right_eye"]["alpha"] * bs_data[14] - face_weights["right_eye"]["beta"]
                                  * face_weights["right_eye"]["power"]) + 0.5
    bs_data[1] = min(max(bs_data[1] * face_weights["left_brow"]["scale"], 0), 1)
    bs_data[3] = min(max(bs_data[3] * face_weights["left_brow"]["scale"], 0), 1)
    bs_data[2] = min(max(bs_data[2] * face_weights["right_brow"]["scale"], 0), 1)
    bs_data[4] = min(max(bs_data[4] * face_weights["right_brow"]["scale"], 0), 1)
    mouth_left = [25, 29, 36, 38, 42, 44, 45]
    for d in mouth_left:
        bs_data[d] = min(max(bs_data[d] * face_weights["left_mouthMove"]["scale"], 0), 1)
    mouth_right = [24, 26, 33, 34, 46, 48, 49]
    for d in mouth_right:
        bs_data[d] = min(max(bs_data[d] * face_weights["right_mouthMove"]["scale"], 0), 1)
    return bs_data


def predict_single_image(inputs):
    if USE_GPU:
        inputs = inputs.cuda()
    outputs = model(inputs)
    if USE_GPU:
        prediction = outputs.cpu().detach().numpy()[0]
    else:
        prediction = outputs.detach().numpy()[0]
    return prediction


def create_data(image):
    img = Image.open(image).convert('RGB')
    img = data_transforms(img)
    tensor = Variable(torch.unsqueeze(img, dim=0).float(), requires_grad=False)
    bs_data = predict_single_image(tensor)
    # bs_data[13] = 0.5 * math.tanh((1. * bs_data[13] - 0.7) * 5) + 0.5
    # bs_data[14] = 0.5 * math.tanh((1. * bs_data[14] - 0.7) * 5) + 0.5
    bs_data = weighted_bs(bs_data)
    return {"face_data": list(bs_data[:21]) + list(bs_data[22:]), "body_data": []}


def create_data_unreal(image):
    img = Image.open(image).convert('RGB')
    img = data_transforms(img)
    tensor = Variable(torch.unsqueeze(img, dim=0).float(), requires_grad=False)
    bs_data = predict_single_image(tensor)
    # bs_data[13] = 0.5 * math.tanh((1. * bs_data[13] - 0.7) * 5) + 0.5
    # bs_data[14] = 0.5 * math.tanh((1. * bs_data[14] - 0.7) * 5) + 0.5
    bs_data = weighted_bs(bs_data)
    return list(bs_data)


def create_data_camera(image):
    img = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    img = data_transforms(img)
    tensor = Variable(torch.unsqueeze(img, dim=0).float(), requires_grad=False)
    bs_data = predict_single_image(tensor)
    # bs_data[13] = 0.5 * math.tanh((1. * bs_data[13] - 0.7) * 5) + 0.5
    # bs_data[14] = 0.5 * math.tanh((1. * bs_data[14] - 0.7) * 5) + 0.5
    # bs_data = transform(bs_data)
    bs_data = weighted_bs(bs_data)
    return {"face_data": list(bs_data[:21]) + list(bs_data[22:]), "body_data": []}


def create_data_camera_unreal(image):
    img = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    img = data_transforms(img)
    tensor = Variable(torch.unsqueeze(img, dim=0).float(), requires_grad=False)
    bs_data = predict_single_image(tensor)
    # bs_data[13] = 0.5 * math.tanh((1. * bs_data[13] - 0.7) * 5) + 0.5
    # bs_data[14] = 0.5 * math.tanh((1. * bs_data[14] - 0.7) * 5) + 0.5
    # bs_data = transform(bs_data)
    bs_data = weighted_bs(bs_data)
    return list(bs_data)


if __name__ == '__main__':
    use_gpu = USE_GPU
    if use_gpu:
        torch.cuda.set_device(0)
    '''
    send_socket(create_data('/home/seallhf/test_face1.jpg'))
    send_socket(create_data('/home/seallhf/test_face3.jpg'))
    send_socket(create_data('/home/seallhf/FC_1_53.jpg'))
    send_socket(create_data('/home/seallhf/pytorch/test_face/1575443342.jpg'))
    send_socket(create_data('/home/seallhf/pytorch/test_face/1575552089.jpg'))
    send_socket(create_data('/home/seallhf/pytorch/test_face/1575552093.jpg'))
    '''
    for i in range(155):
        start = time.time()
        send_socket(create_data_unreal('./face_capture/' + str(i) + '.jpg'))
        end = time.time()
        span = end - start
        if span <= 0.1:
            time.sleep(0.1 - span)

    print("time span: " + str(end - start))
    send_socket({'face_data': 'none'})
    np.rot90()
