import cv2
import numpy as np
from matplotlib import pyplot as plt
from skimage.util import img_as_ubyte, img_as_float
from skimage import exposure
import time
import socket
import threading
import math

USE_VIDEO = True
DO_MINES = True
DO_ROBOT = True
RECORD = True
START_MINES = 8

F_NAME = "robot_mode.jpg"

DEG_TO_RAD = math.pi/180.0
RAD_TO_DEG = 180.0/math.pi

# must be 3:4 H:W ratio
RESOLUTION = np.array([640, 480]) # x, y
FPS = 10

# scale vs 640, 480
IDX_SCALE = RESOLUTION[0] / 640.0

def mpl_show(img):

    if len(img.shape) > 2:
        b, g, r = cv2.split(img)
        img_mpl = cv2.merge([r, g, b])
    else:
        img_mpl = img
    plt.axis("off")
    plt.imshow(img_mpl, "gray")
    plt.show()


def cv2_cross(img, idx, size, colour, t=1):
    y, x = idx
    cv2.line(img, tuple([y - size, x - size]), tuple([y + size, x + size]), colour, t)
    cv2.line(img, tuple([y + size, x - size]), tuple([y - size, x + size]), colour, t)

def simple_brighten(img, a=1.0, b=0.0):
    new_img = np.zeros_like(img)
    for y in range(img.shape[1]):
        for x in range(img.shape[0]):
            new_img = np.clip(a*img[y,x] + b, 0, 255)
    return new_img


def gamma_brighten(img, g=0.2):
    return apply_fn(img, lambda x: np.clip(pow(x / 255.0, g) * 255.0, 0, 255))    



def apply_fn(img, fn):
    
    lookUpTable = np.empty((1,256), np.uint8)
    for i in range(256):
        lookUpTable[0,i] = np.clip(fn(i), 0, 255)
    
    return cv2.LUT(img, lookUpTable)

class Stopwatch():

    def __init__(self):
        self.t_start = 0

    def start(self):
        self.t_start = time.time()

    def stop(self, print_result=True):
        if self.t_start != 0:
            delta_t = time.time() - self.t_start
            self.t_start = 0
            if print_result:
                print(f"timer: {delta_t}s")

            return delta_t
        
        return 0
        

def cv2_text(image, txt, idx, colour):
    font = cv2.FONT_HERSHEY_SIMPLEX 
    fontScale = 0.5    
    thickness = 1
    
    # Using cv2.putText() method 
    return cv2.putText(image, txt, idx, font,  
                       fontScale, colour, thickness, cv2.LINE_AA) 