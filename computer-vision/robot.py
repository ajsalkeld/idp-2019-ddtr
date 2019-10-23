from global_stuff import *
from arena import *
import udpComms as udpc

# udpc.setup()

pt = Point

HUE_NORMALISE = 179.0/360.0

# # hue values
# TURQOISE = np.array([180, 220])
# VIOLET = np.array([230, 270])
# PINK = np.array([270, 310])

# # potentially better (more distributed):
# YELLOW = np.array([35, 65])
# BLUE = np.array([190, 230])
# PURPLE = np.array([250, 290])

# tape
RED1 = np.array([0, 20])
RED2 = np.array([340, 360])


MAX_THETA_RATE = 45*DEG_TO_RAD # 90 degrees per second

class Robot():

    def __init__(self):
        self.w_m = 0.175
        self.h_m = 0.155
        self.dims = np.array([self.h_m, self.w_m])

        self.origin_history = []
        self.axes_history = []
        self.timestamps = []

        # target rotation and position
        self.t_rot = None
        self.t_pos = None

        self.theta_err = None
        self.latest_cmd = None

    def draw_coord_sys(self, img):

        est = self.get_pos_estimate()

        if est is None:
            return img
        
        origin, axes = est

        # x cyan, y yellow, magenta origin
        img = draw_line_from_params(img, origin, axes[0], (255, 255, 0)) 
        img = draw_line_from_params(img, origin, axes[1], (0, 255, 255))
        
        cv2_cross(img, pt(idx=origin).idx, 3, (255, 0, 255), 2)
        cv2_cross(img, pt(pos=self.get_centre_coords()).idx, 3, (255, 255, 255), 1)

        cv2_text(img, "latest command: " + str(self.latest_cmd), (50, 50), (255, 255, 255))
        cv2_text(img, "rotation error: " + str(self.theta_err), (50, 100), (255, 255, 255))

        # mpl_show(robot_img)
        if not USE_VIDEO:
            mpl_show(img)

        return img


    def get_pos_estimate(self):

        if len(self.origin_history) < 1: # make this 3
            return None

        # punish records which were a long time ago
        weights = np.exp(8.0*(np.array(self.timestamps) - self.timestamps[-1]))      
        # print(weights)

        def weighted_ave(arr):
            return sum([weights[i] * arr[i] for i in range(len(weights))])/sum(weights)
    
        # print(self.origin_history)
        return weighted_ave(self.origin_history), weighted_ave(self.axes_history)


    def get_centre_coords(self):

        origin, dirs = self.get_pos_estimate()
        
        offset = (self.dims - 0.005)/2

        return pt(idx=origin).pos + np.matmul(dirs, offset)


    def record_pos(self, origin, axes):

        ts = time.time()
        outrageous_speed = 0.4 # m/s

        est = self.get_pos_estimate()

        if est is None:
            # print("pos history", self.origin_history)
            # print("recording at", origin, axes)

            self.origin_history.append(origin)
            self.axes_history.append(axes)
            self.timestamps.append(ts)
        
        else:
            distance = pythag(pt(idx=origin).pos - pt(idx=est[0]).pos)

            if distance > 0.05 and distance / (ts - self.timestamps[0]) > outrageous_speed:
                print("didn't record - moved too fast")
                return

            else:
                # print("recording at", origin, axes)
                self.origin_history.append(origin)
                self.axes_history.append(axes)
                self.timestamps.append(ts)

        if len(self.origin_history) > 4:
            del self.origin_history[0]
            del self.axes_history[0]
            del self.timestamps[0]



    def update_pos(self, img):
        img = cv2.bitwise_and(img, ROBOT_MASK)

        res = detect_robot(img)

        if res is None:
            return img

        origin, axes = res

        self.record_pos(origin, axes)


    # in real world coords
    def set_target(self, pos, rot):
        self.t_pos = pos

        if abs(1.0-pythag(rot)) > 0.01:
            rot = rot / pythag(rot)

        self.t_rot = rot



    def do_control(self):
        
        vals = self.get_pos_estimate()

        if vals is None:
            return

        real_rot = vals[1][1] # heading vector
        real_pos = self.get_centre_coords()

        sign = lambda x: x/abs(x) if abs(x) > 0.000001 else 1

        # print("target_rot", self.t_rot)
        # print("actual_rot", real_rot)

        if self.t_rot is not None:
            d_prod = np.dot(self.t_rot, real_rot)
            c_prod = self.t_rot[0] * real_rot[1] - self.t_rot[1] * real_rot[0]

            # print("c_prod", c_prod)

            theta_err = np.arccos(d_prod)
            if c_prod < 0:
                theta_err = -theta_err

            self.theta_err = theta_err
            # print("theta error (rad):", theta_err)

            if abs(theta_err) > DEG_TO_RAD * 10:

                # TODO: stop moving forward
                
                if abs(theta_err) > DEG_TO_RAD * 15:
                    theta_rate = - MAX_THETA_RATE * sign(theta_err)
                
                else:
                    theta_rate = - MAX_THETA_RATE * theta_err / (DEG_TO_RAD * 15)

                if theta_rate > 0:
                    self.send_cmd("turn right until")

                else:
                    self.send_cmd("turn left until")

                
            else:
                self.send_cmd("stop")


    def send_cmd(self, cmd_txt):
        if cmd_txt != self.latest_cmd:
            udpc.sendCommand(cmd_txt.encode())
            self.latest_cmd = cmd_txt


robot = Robot()

SQRT2 = 2**0.5
_1_SQRT2 = 1/SQRT2


def get_line_params(contour):
    params = cv2.fitLine(contour, cv2.DIST_L2,0,0.01,0.01)
    params = [p[0] for p in params]
    # print(params)
    return params


def draw_line_from_params(img, line_pt, line_v, colour):
    [vx, vy] = line_v
    [x, y] = line_pt
    rows,cols = img.shape[:2]
    cv2_cross(img, (int(x + 50*vx), int(y + 50*vy)), 3, colour, 1)
    return cv2.line(img, (int(x), int(y)), (int(x + 50*vx), int(y + 50*vy)), colour, 1)


def pythag(v):
    return np.sqrt(np.dot(v, v))


def norm(v):
    return v / np.sqrt(np.dot(v, v))


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

    halfway = norm(sum(line_vs))
    

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

    cv2.drawContours(img, red_cs, -1, (0, 0, 255), -1)



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


    for pt in line_pts:
        dist = pythag(pt - intersect_coords)
        if dist > max(robot.dims/IDX_TO_COORD_SF):
            print("too far to intersect")

    
    # coordinate system now fully established
    # vector 0 is robot x, vector 1 is robot y

    sw.stop(False)

    return intersect_coords, line_vs



    

