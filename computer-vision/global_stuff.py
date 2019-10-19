import cv2
import numpy as np
from matplotlib import pyplot as plt
from skimage.util import img_as_ubyte, img_as_float
from skimage import exposure
import time
import socket
import threading

USE_VIDEO = True
DO_MINES = True
DO_ROBOT = True
F_NAME = "pics/arena3-robot.jpg"

# must be 3:4 H:W ratio
RESOLUTION = np.array([640, 480]) # x, y
FPS = 10

# scale vs 640, 480
IDX_SCALE = RESOLUTION[0] / 640.0

def mpl_show(img):

    if len(img.shape) > 2:
        b, g, r=cv2.split(img)
        img_mpl = cv2.merge([r, g, b])
    else:
        img_mpl = img
    plt.axis("off")
    plt.imshow(img_mpl,'gray') #convert to bgr
    plt.show()


def cv2_cross(img, idx, size, colour, t=1):
    y, x = idx
    cv2.line(img, tuple([y - size, x - size]), tuple([y + size, x + size]), colour, t)
    cv2.line(img, tuple([y + size, x - size]), tuple([y - size, x + size]), colour, t)