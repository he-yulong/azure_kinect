import sys
sys.path.append('./')
import main
from stream_interface import interface

def f1(data):
    print('hello')
    return []

sk = main.SingleKinect(interface, 0)
sk.Open()
sk.Running(1000)
sk.Close()
