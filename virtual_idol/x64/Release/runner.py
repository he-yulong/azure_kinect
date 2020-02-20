import sys
sys.path.append('./')
import main
from stream_interface import interface

def hello(frame):
  print("hello")
  print(frame)


sk = main.SingleKinect(interface, 0)
sk.Open()
sk.Running(500)
sk.Close()
