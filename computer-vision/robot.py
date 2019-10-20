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
GREEN = np.array([90, 140])
RED1 = np.array([0, 20])
RED2 = np.array([340, 360])

DS_RATIO = 1

robot_mask_name = "pics/calib/robot_mode_mask.jpg"
ROBOT_MASK = cv2.imread(robot_mask_name)
ROBOT_MASK = cv2.resize(ROBOT_MASK, tuple(RESOLUTION))

def detect_robot(img):

    sw = Stopwatch()
    sw.start()

    # downsampled = cv2.resize(img, tuple([int(x) for x in RESOLUTION/DS_RATIO]))

    # TODO: different method -
    # robot will occlude the lines and fuck with this

    img = cv2.bitwise_and(img, ROBOT_MASK)

    # mpl_show(img)

    # get rid of white

    # def maxabs(pxl):
    #     return max(abs(pxl[0] - pxl[1]), abs(pxl[1] - pxl[]))

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    h, s, v = cv2.split(hsv)

    def get_colour_contours(colour_range):

        colour_min, colour_max = (colour_range * HUE_NORMALISE).astype(int)
        in_range = cv2.inRange(hsv, np.array([colour_min, 150, 40]), np.array([colour_max, 255, 255]))
        
        contours, ret = cv2.findContours(in_range, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        return contours

    green_cs = get_colour_contours(GREEN)
    red_cs = get_colour_contours(RED1)
    red_cs.extend(get_colour_contours(RED2))

    print(len(green_cs), len(red_cs))

    def biggest_n_cs(contours, n):
        return sorted(contours, key=lambda c: cv2.contourArea(c), reverse=True)[:n]

    def get_line_params(contour):
        params = cv2.fitLine(contour, cv2.DIST_L2,0,0.01,0.01)
        params = [p[0] for p in params]
        print(params)
        return params

    def draw_line_from_params(img, line_params, colour):
        [vx, vy, x, y] = line_params
        # rows,cols = img.shape[:2]
        # lefty = int((-x*vy/vx) + y)
        # righty = int(((cols-x)*vy/vx)+y)
        return cv2.line(img, (x,y), (int(x + 30*vx), int(y + 30*vy)), colour, 1)

    green_cs = biggest_n_cs(green_cs, 2)
    red_cs = biggest_n_cs(red_cs, 2)
    
    green_lines = [get_line_params(c) for c in green_cs]
    red_lines = [get_line_params(c) for c in red_cs]

    robot_img = np.zeros_like(hsv)

    i = 0
    for line in green_lines:
        i+= 100
        robot_img = draw_line_from_params(robot_img, line, (i, 255, 0))

    i = 0
    for line in red_lines:
        i+= 100
        robot_img = draw_line_from_params(robot_img, line, (i, 0, 255))

    # algorithm:
    # assemble lines into pairs:
    # - find red with orth. red - corner now known
    # - find green with orth. green - corner again known
    # 
    # - ()

    sw.stop()

    mpl_show(robot_img)



    # mpl_show(img)

    

