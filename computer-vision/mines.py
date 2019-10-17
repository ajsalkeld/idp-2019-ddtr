from global_stuff import *

from arena import *

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


def find_mines(img, mask, img_x, img_y):

    h, w = np.shape(img)

    # img = filled_mask(img, mask)

    ksize = 15
    bright = cv2.bitwise_not(gamma_brighten(img, 0.4))
    sobel = cv2.Sobel(bright, cv2.CV_64F, 0, 1, ksize=ksize)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
    dilated_mask = cv2.erode(mask, kernel)
    sobel = cv2.multiply(sobel, img_as_float(dilated_mask))
    
    mean = sobel.mean()

    for y in range(h):
        for x in range(w):
            sobel[y, x] = abs(sobel[y, x] - mean)

    max_val = sobel.max()

    sobel = exposure.rescale_intensity(sobel, in_range=(0, max_val))
    sobel = img_as_ubyte(sobel)

    ret, threshed = cv2.threshold(sobel, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    closed = cv2.morphologyEx(threshed, cv2.MORPH_CLOSE, np.ones((11, 11),np.uint8))
   
    # mpl_show(closed)

    contours, hierarchy = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    cv2.drawContours(mask, contours, -1, (0, 0, 0))

    mine_deets = []
    for c in contours:
        (x, y), radius = cv2.minEnclosingCircle(c)
        centre = Point(idx=(img_x + x, img_y + y))
        radius = round(radius)

        mine_deets.append((centre, radius))

        # M = cv2.moments(c)
        # if M['m00'] > 0:
        #     centroids.append((int(M['m10']/M['m00']), int(M['m01']/M['m00'])))

    return mine_deets
