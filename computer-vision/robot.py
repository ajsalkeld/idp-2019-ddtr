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
RED1 = np.array([0, 20])
RED2 = np.array([340, 360])

DS_RATIO = 1



SQRT2 = 2**0.5
_1_SQRT2 = 1/SQRT2

def get_line_params(contour):
    params = cv2.fitLine(contour, cv2.DIST_L2,0,0.01,0.01)
    params = [p[0] for p in params]
    # print(params)
    return params


def draw_line_from_params(img, line_v, line_pt, colour):
    [vx, vy] = line_v
    [x, y] = line_pt
    rows,cols = img.shape[:2]
    cv2_cross(img, (int(x + 30*vx), int(y + 30*vy)), 3, colour, 1)
    return cv2.line(img, (int(x - 30*vx), int(y - 30*vy)), (int(x + 30*vx), int(y + 30*vy)), colour, 1)


def find_intersect_coeffs(line_vs, line_pts):

    # solve equation system for intersect
    m = np.transpose(np.array([line_vs[0], -line_vs[1]]))
    c = line_pts[1] - line_pts[0]
    
    try:
        x = np.linalg.solve(m, c)
    except:
        print("singular matrix")
        return None

    return x


# fit vectors which are 100% perpendicular (rather than roughly)
def fit_orthog_vs(line_vs):

    halfway = sum(line_vs)
    halfway /= np.sqrt(np.dot(halfway, halfway))

    # print("line_vs", line_vs)
    # print("halfway", halfway)
    
    rot_45_ccw = np.array([[_1_SQRT2, -_1_SQRT2], [_1_SQRT2, _1_SQRT2]])
    rot_45_cw  = np.array([[_1_SQRT2, _1_SQRT2], [-_1_SQRT2, _1_SQRT2]])

    vy = np.matmul(halfway, rot_45_ccw)
    vx = np.matmul(halfway, rot_45_cw)
    line_vs = np.array([vx, vy])

    # print("new line_vs", line_vs)

    return line_vs


def detect_robot(img):

    sw = Stopwatch()
    sw.start()

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    h, s, v = cv2.split(hsv)

    def get_colour_contours(colour_range):

        colour_min, colour_max = (colour_range * HUE_NORMALISE).astype(int)
        in_range = cv2.inRange(hsv, np.array([colour_min, 100, 40]), np.array([colour_max, 255, 255]))
        
        # mpl_show(in_range)

        contours, ret = cv2.findContours(in_range, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        return contours

    red_cs = get_colour_contours(RED1)
    red_cs.extend(get_colour_contours(RED2))

    if len(red_cs) < 2:
        print("not enough contours")
        return None

    red_cs = sorted(red_cs, key=lambda c: cv2.contourArea(c), reverse=True)[:2]
    
    red_lines = [get_line_params(c) for c in red_cs]

    robot_img = np.zeros_like(hsv)
    robot_img1 = np.zeros_like(hsv)

    # cv2.drawContours(img, red_cs, -1, (0, 0, 255), -1)



    v1 = np.array(red_lines[0][:2])
    v2 = np.array(red_lines[1][:2])
    line_vs = np.array([v1, v2])

    a1 = np.array(red_lines[0][2:])
    a2 = np.array(red_lines[1][2:])
    line_pts = np.array([a1, a2])


    # make sure vectors are roughly orthogonal (sanity check)
    theta = (180/np.pi) * np.arccos(abs(np.dot(line_vs[0], line_vs[1])))
    
    if (abs(90-theta) > 30):
        print("more than 30 degrees from orthogonal:", theta)
        return None

    coeffs = find_intersect_coeffs(line_vs, line_pts)

    # point vectors away from intersect point
    for i, coeff in enumerate(coeffs):
        if coeff > 0:
            coeffs[i] = -coeffs[i]
            line_vs[i] = -line_vs[i]

    # using this rough solution make sure we have the x and y
    # vectors in the right order (cross product should be positive)
    cross_prod = line_vs[0][0] * line_vs[1][1] - line_vs[1][0] * line_vs[0][1]


    if cross_prod > 0:
        line_vs = [v for v in reversed(line_vs)]
        line_pts = [v for v in reversed(line_pts)]


    # fit orthogonal basis to these vectors
    line_vs = fit_orthog_vs(line_vs)

    # find new intersect coeffs
    coeffs = find_intersect_coeffs(line_vs, line_pts)

    intersect_coords_1 = line_pts[0] + coeffs[0] * line_vs[0]
    intersect_coords_2 = line_pts[1] + coeffs[1] * line_vs[1]
    cv2_cross(img, intersect_coords_1.astype(int), 3, (255, 255, 255), 2)
    cv2_cross(img, intersect_coords_2.astype(int), 3, (255, 255, 255), 2)

    # make sure intersection coords agree (sanity check)
    diff = intersect_coords_1 - intersect_coords_2
    if any(abs(i) > 0.1 for i in diff):
        print("intersect coords disagree: ", intersect_coords_1, intersect_coords_2)
        return None

    intersect_coords = (0.5 * (intersect_coords_1 + intersect_coords_2)).astype(int)

    # coordinate system now fully established
    # vector 0 is robot x, vector 1 is robot y

    # x cyan, y yellow, magenta origin
    img = draw_line_from_params(img, line_vs[0], line_pts[0], (255, 255, 0)) 
    img = draw_line_from_params(img, line_vs[1], line_pts[1], (0, 255, 255))
    cv2_cross(img, intersect_coords, 3, (255, 0, 255), 2)


    sw.stop(False)

    # mpl_show(robot_img)
    if not USE_VIDEO:
        mpl_show(img)

    return img


    

