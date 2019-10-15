import cv2
import numpy as np
from matplotlib import pyplot as plt
from skimage.util import img_as_ubyte
from skimage import exposure
import time

def simple_brighten(img, a=1.0, b=0.0):
    new_img = np.zeros_like(img)
    for y in range(img.shape[1]):
        for x in range(img.shape[0]):
            new_img = np.clip(a*img[y,x] + b, 0, 255)
    return new_img

def apply_fn(img, fn):
    
    lookUpTable = np.empty((1,256), np.uint8)
    for i in range(256):
        lookUpTable[0,i] = np.clip(fn(i), 0, 255)
    
    return cv2.LUT(img, lookUpTable)


def gamma_brighten(img, g=0.2):
    return apply_fn(img, lambda x: np.clip(pow(x / 255.0, g) * 255.0, 0, 255))    

# currently untested, basically copy/pasted from __main__
def find_mines(img):
    bright = cv2.bitwise_not(gamma_brighten(img, 0.4))
    sobel = cv2.Sobel(bright, cv2.CV_64F, 0, 1, ksize=25)
    mean = sobel.mean()

    for y in range(h):
        for x in range(w):
            sobel[y, x] = abs(sobel[y, x] - mean)

    max_val = sobel.max()

    sobel2 = exposure.rescale_intensity(sobel, in_range=(0, max_val))
    sobel2 = img_as_ubyte(sobel2)

    ret, threshed = cv2.threshold(sobel2, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    closed = cv2.morphologyEx(threshed, cv2.MORPH_CLOSE, np.ones((25, 25),np.uint8))
    
    contours, hierarchy = cv2.findContours(closed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  
    centroids = []
    for c in contours:
        M = cv2.moments(c)
        centroids.append((int(M['m10']/M['m00']), int(M['m01']/M['m00'])))

    return centroids

if __name__ == "__main__":

    print("starting")
    tstart = time.time()

    fname = "pics/arena3-4.jpg"

    img = cv2.imread(fname, cv2.IMREAD_GRAYSCALE)

    h, w = np.shape(img)

    print(h, w)

    bright = cv2.bitwise_not(gamma_brighten(img, 0.4))
    sobel = cv2.Sobel(bright, cv2.CV_64F, 0, 1, ksize=25)
    mean = sobel.mean()

    for y in range(h):
        for x in range(w):
            sobel[y, x] = abs(sobel[y, x] - mean)

    max_val = sobel.max()

    sobel2 = exposure.rescale_intensity(sobel, in_range=(0, max_val))
    sobel2 = img_as_ubyte(sobel2)

    ret, threshed = cv2.threshold(sobel2, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    closed = cv2.morphologyEx(threshed, cv2.MORPH_CLOSE, np.ones((25, 25),np.uint8))


    colour_img = cv2.imread(fname)

    contours, hierarchy = cv2.findContours(closed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        (x, y), radius = cv2.minEnclosingCircle(c)
        centre = (int(x), int(y))
        radius = int(radius)
        contoured = cv2.circle(colour_img, centre, radius, (0,255,0), 2)

    # cntrs_img = cv2.drawContours(colour_img, contours, -1, (0,255,0), 2)
    colour_img = cv2.imread(fname) # because the drawContours function modifies it

    tend = time.time()
    print(f"done. time: {tend - tstart} s")


    to_draw = [contoured]#[colour_img, sobel, contoured]


    for i, img in enumerate(to_draw):
        plt.subplot(1, len(to_draw),i+1)
        plt.axis("off")
        plt.imshow(img,'gray')

    plt.show()
