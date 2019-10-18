from global_stuff import *

HUE_NORMALISE = 179.0/360.0

# hue values
TURQOISE = np.array([180, 220])
VIOLET = np.array([230, 270])
PINK = np.array([270, 310])

# potentially better (more distributed):
YELLOW = np.array([35, 65])
BLUE = np.array([190, 230])
PURPLE = np.array([250, 290])

# tape
GREEN = np.array([70, 190])
RED1 = np.array([0, 60])
RED2 = np.array([320, 360])

DS_RATIO = 1

def detect_robot(img):

    # downsampled = cv2.resize(img, tuple([int(x) for x in RESOLUTION/DS_RATIO]))

    # mpl_show(downsampled)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    imgs = []

    for colour_range in (GREEN,):

        colour_min, colour_max = (colour_range * HUE_NORMALISE).astype(int)
        img = cv2.inRange(hsv, np.array([colour_min, 100, 100]), np.array([colour_max, 255, 255]))

        imgs.append(img)
