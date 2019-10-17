import cv2
import numpy as np
from matplotlib import pyplot as plt
from skimage.util import img_as_ubyte
from skimage import exposure
import time

USE_VIDEO = False

# must be 3:4 H:W ratio
RESOLUTION = np.array([640, 480]) # x, y
FPS = 10

# scale vs 640, 480
IDX_SCALE = RESOLUTION[0] / 640.0

def mpl_show(img):

    plt.axis("off")
    plt.imshow(img[...,::-1],'gray') #convert to bgr
    plt.show()