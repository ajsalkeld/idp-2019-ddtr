# function to filter the frame and try to find mines
from global_stuff import *

from arena import *

def find_mines(img, mask, detection_thresh, img_x, img_y):

    h, w = np.shape(img)

    img = img.copy()
    mask = mask.copy()

    # mpl_show(img)

    ksize = 15 # for sobel and dilation
    
    # brighten the image to improve contrast
    bright = gamma_brighten(img, 0.4)
    
    # divide each pixel by heavily-blurred image to remove 
    # the effects of slowly-varying table brightness
    blur_to_norm = cv2.GaussianBlur(bright, (35, 35), cv2.BORDER_DEFAULT)
    normalised = cv2.divide(img_as_float(bright), img_as_float(blur_to_norm))

    # mpl_show(normalised)

    # we don't care about bright spots (likely to be annoying specular artefacts)
    # and all mines will show up just from their dark parts
    # - so truncate the brightness of the image at the mean
    mean = normalised.mean()
    ret, trunced = cv2.threshold(normalised, mean, mean, cv2.THRESH_TRUNC)

    # mpl_show(trunced)

    # sobel is a type of gradient kernel to detect edges
    # use a vertical sobel as most scratches/markings on the table are vertical
    sobel = cv2.Sobel(trunced, cv2.CV_64F, 0, 1, ksize=ksize) # makes image emphasising edges
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
    
    # dilate the mask (i.e. erode its complement) to get rid of 
    # sharp gradient at mask edges
    dilated_mask = cv2.erode(mask, kernel)
    sobel = cv2.multiply(sobel, img_as_float(dilated_mask))
    
    # find absolute value of gradient (mean should be ~0)
    mean = sobel.mean()
    for y in range(h):
        for x in range(w):
            sobel[y, x] = abs(sobel[y, x] - mean)

    # convert from float back to 0-255 integer pixel values
    max_val = sobel.max()
    sobel = exposure.rescale_intensity(sobel, in_range=(0, max_val))
    sobel = img_as_ubyte(sobel)

    # binary-threshold brightness based on input value 
    # - this changes the sensitivity to mines
    ret, threshed = cv2.threshold(sobel, detection_thresh, 255, cv2.THRESH_BINARY)

    # 'closing' is a morphological image operation which dilates then
    # erodes white pixels. This gets rid of 'holes' enclosed by white pixels.
    # since the gradient will be at the edges of mines, this converts
    # them to equal-sized blobs rather than outlines
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (30, 30))
    closed = cv2.morphologyEx(threshed, cv2.MORPH_CLOSE, close_kernel)
   
    # mpl_show(closed)

    # find contours of white pixels
    contours, ret = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    cv2.drawContours(mask, contours, -1, (0, 0, 0))

    # extract details about contours, only returning blobs with sensible areas
    mine_deets = []
    for c in contours:

        M = cv2.moments(c)
        if M['m00'] > 0:
            centre = Point(idx=(img_x + M['m10']/M['m00'], img_y + M['m01']/M['m00']))
            area = cv2.contourArea(c)
            radius = area**0.5

            if 20 < area < 400:
                # print("mine:", centre.pos, area)
                mine_deets.append((centre, radius))

    return mine_deets