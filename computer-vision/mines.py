from global_stuff import *

from arena import *

def find_mines(img, mask, img_x, img_y):

    h, w = np.shape(img)

    ksize = 15 # for sobel and dilation
    
    bright = gamma_brighten(img, 0.4)
    
    blur_to_norm = cv2.GaussianBlur(bright, (35, 35), cv2.BORDER_DEFAULT)
    normalised = cv2.divide(img_as_float(bright), img_as_float(blur_to_norm))

    # mpl_show(normalised)

    mean = normalised.mean()
    ret, trunced = cv2.threshold(normalised, mean, mean, cv2.THRESH_TRUNC)

    # mpl_show(normalised)
    # mpl_show(trunced)

    mean = trunced.mean()

    sobel = cv2.Sobel(trunced, cv2.CV_64F, 0, 1, ksize=ksize)
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

    # mpl_show(sobel)

    ret, threshed = cv2.threshold(sobel, 30, 255, cv2.THRESH_BINARY)
    
    closed = cv2.morphologyEx(threshed, cv2.MORPH_CLOSE, np.ones((15, 15),np.uint8))
   
    # mpl_show(closed)

    # closed = cv2.medianBlur(closed, 5)

    contours, ret = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    cv2.drawContours(mask, contours, -1, (0, 0, 0))

    mine_deets = []
    for c in contours:


        M = cv2.moments(c)
        if M['m00'] > 0:
            centre = Point(idx=(img_x + M['m10']/M['m00'], img_y + M['m01']/M['m00']))
            area = cv2.contourArea(c)
            radius = area**0.5

            if area < 500:
                mine_deets.append((centre, radius))


            # (x, y), radius = cv2.minEnclosingCircle(c)
            # centre = Point(idx=(img_x + x, img_y + y))
            # radius = round(radius)

    # because it will think noise is mines without the contrast.
    if len(mine_deets) > 10:
        print("no mines.") 
        mine_deets = []

    return mine_deets, closed
