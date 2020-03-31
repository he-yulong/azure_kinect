import sys
sys.path.append('./')
import numpy
import main

def f1(data):
    return []

sk = main.SingleKinect(f1, 0)
sk.Open()
sk.Running(51000)
sk.Close()
