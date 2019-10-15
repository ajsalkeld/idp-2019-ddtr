import cv2
import numpy as np
from matplotlib import pyplot as plt
from skimage.util import img_as_ubyte
from skimage import exposure
import time

import "arena"

fname = "pics/arena3-1.jpg"

img = cv2.imread(fname, cv2.IMREAD_GRAYSCALE)
h, w = np.shape(img)

