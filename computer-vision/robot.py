from global_stuff import *
from arena import *
from udpComms import CURR_UDP_DATA

if USE_VIDEO:
    import udpComms as udpc



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


MAX_THETA_RATE = 0.35
MAX_TRANS_RATE = 0.5

POS_DEADBAND_W = 0.03 # m

class Robot():

    def __init__(self):
        self.w_m = 0.175
        self.h_m = 0.155
        self.dims = np.array([self.h_m, self.w_m])
        self.centre_offset = np.array([0.065, 0.095])
        self.b_box1_offset = np.array([-0.250, -0.250])
        self.b_box1_wh = np.array([0.500, 0.500])
        # self.b_box2_offset = self.b_box1_offset
        # self.b_box2_wh = np.array([0.06, 0.1])
        # self.b_box2_offset[0] += self.b_box1_wh[0]/2 - self.b_box2_wh[0]/2
        # self.b_box2_offset[1] += self.b_box1_wh[1] - 0.01        

        if not USE_VIDEO:
            self.origin_history = []
            self.axes_history = []
            self.timestamps = []

        else:
            self.origin_history = []
            self.axes_history = []
            self.timestamps = []
        
        self.states = [
            "moving from start pos"
            "looking for mines",
            "moving to mine y",
            "moving to mine x",
            "picking up mine",
            "moving to bin x",
            "moving to bin y",
            "rotating to bin",
            "depositing mine",
            "moving to end pos"
        ]

        self.mine_types = ["live", "dead"]

        # self.state = "moving from start pos"
        # self.prev_state = ""

        self.state = "looking for mines"
        self.prev_state = "moving from start pos"
        self.target_mine = None
        self.has_mine = False

        self.wants_mine_data_flag = False
        self.mine_locs = None

        # target rotation and position
        self.achieved_target = False
        self.t_rot = None
        self.t_pos = None
        self.t_start_pos = None
        self.ok_to_translate = False
        self.outside_deadband = False

        self.l_cmd = 0
        self.r_cmd = 0
        
        self.theta_err = None
        self.pos_err = None
        self.latest_cmd = None
        self.latest_cmd_time = time.time()
        self.latest_rcv_time = time.time()

        if USE_VIDEO:
            udpc.rxthread.start()
            self.send_cmd("reset")

    def update_state(self):

        if self.state == "moving from start pos":
            
            if self.achieved_target:
                self.state = "looking for mines"
            else:
                # blindly rotate and then move forward
                self.achieved_target = True
            
        elif self.state == "looking for mines":
            if self.state != self.prev_state:
                self.wants_mine_data_flag = True

            if not self.wants_mine_data():

                self.mine_locs = [mine.pos for mine in self.mine_locs]
                self.mine_locs.sort(key=lambda loc: loc[0], reverse = True)
                
                for mine in self.mine_locs:
                    if mine[1] > 0.4 and mine[1] < 2.1 - 0.4:
                        self.target_mine = mine
                        self.state = "moving to mine"
                        break
                    
                    print("mine is close to edge of camera")
                else:
                    print("all mines left are close to edge of camera")
                    self.target_mine = self.mine_locs[0]
                    self.state = "moving to edge mine"


                print("picked mine at", self.target_mine)


        #should have a best mine identified
        elif "moving to mine" in self.state:
            
            if self.target_mine is None:
                self.state = "looking for mines"

            else:
                if self.state == "moving to mine":
                    self.state = "moving to mine y"

                if self.state == "moving to mine y":
                    if self.state != self.prev_state:
                        # print("setting target pos")
                        self.set_target_pos(np.array([SAFE_LINE_X_POS, self.target_mine[1]]))
                    if self.achieved_target:
                        self.state = "moving to mine x"

                if self.state == "moving to mine x":
                    if self.state != self.prev_state:
                        self.set_target_pos(self.target_mine)

                    if USE_VIDEO and udpc.CURR_UDP_DATA is not None:
                        if udpc.CURR_UDP_DATA["carryingMine"]:
                            is_live = udpc.CURR_UDP_DATA["livemine"] 

                            if is_live:
                                self.state = "moving to bin"
                                self.has_mine = "live"
                            else:
                                self.state = "moving to bin"
                                self.has_mine = "dead"


        # can't go directly to these or CV will lose Nina
        elif "moving to edge mine" in self.state:
            
            if self.target_mine is None:
                self.state = "looking for mines"

            else:
                if self.state == "moving to edge mine":
                    self.state = "moving to edge mine centre line"

                if self.state == "moving to edge mine centre line":
                    if self.state != self.prev_state:
                        self.set_target_pos(np.array([SAFE_LINE_X_POS, 1.2]))
                    if self.achieved_target:
                        self.state = "moving to edge mine x"

                if self.state == "moving to edge mine x":
                    if self.state != self.prev_state:
                        self.set_target_pos(np.array([self.target_mine[0], 1.2]))
                    if self.achieved_target:
                        self.state = "moving to edge mine y"

                if self.state == "moving to edge mine y":
                    if self.state != self.prev_state:
                        self.set_target_pos(self.target_mine)
                        

                    if USE_VIDEO and udpc.CURR_UDP_DATA is not None:
                        if udpc.CURR_UDP_DATA["carryingMine"]:
                            self.state = "blind reverse after edge mine pickup"

        elif self.state == "blind reverse after edge mine pickup":

            self.remove_target_pos()
            self.blind_start_time = time.time()
            if time.time() - self.blind_start_time < 2:
                self.set_motors(-MAX_TRANS_RATE, -MAX_TRANS_RATE)
            else:
                self.send_cmd("stop")
                
                if USE_VIDEO and udpc.CURR_UDP_DATA is not None:
                    is_live = udpc.CURR_UDP_DATA["livemine"] 

                    if is_live:
                        self.state = "moving to bin"
                        self.has_mine = "live"
                    else:
                        self.state = "moving to bin"
                        self.has_mine = "dead"

                    self.target_mine[1] = (self.target_mine[1] - 1.2) * 0.9 + 1.2

        
        elif "moving to bin" in self.state:
            if not self.has_mine:
                self.state = "looking for mines"

            if self.state == "moving to bin":
                self.state = "moving to bin x"

            if self.state == "moving to bin x":
                if self.state != self.prev_state:
                    self.set_target_pos(np.array([SAFE_LINE_X_POS, self.target_mine[1]]))
                if self.achieved_target:
                    self.state = "moving to mine y"

            if self.state == "moving to bin y":
                if self.has_mine == "live":
                    if self.state != self.prev_state:
                        self.set_target_pos(np.array([SAFE_LINE_X_POS, LIVE_DEPOSIT_Y_POS]))
                elif self.has_mine == "dead":
                    if self.state != self.prev_state:
                        self.set_target_pos(np.array([SAFE_LINE_X_POS, DEAD_DEPOSIT_Y_POS]))
                
                if self.achieved_target:
                    self.state = "rotating to bin"


            if self.state == "rotating to bin":
                # self.set_target_rot(np.array([1, 0]))
                self.send_cmd("stop")

                if self.achieved_target:
                    self.state = "depositing mine"
                    #send a packet to nina

        elif self.state == "depositing mine":
            #wait for a packet from nina
            self.state = "looking for mines"


        update_again = False
        if self.prev_state != self.state:
            update_again = True

        self.prev_state = self.state

        if update_again:
            self.update_state()



    def illustrate(self, img):

        est = self.get_pos_estimate()

        if est is None:
            return img
        
        origin, axes = est
        centre = self.get_centre_coords()
        c_idx = pt(pos=centre).idx

        # x cyan, y yellow, magenta centre, green heading
        img = draw_line_from_params(img, origin, axes[0], (255, 255, 0)) 
        img = draw_line_from_params(img, origin, axes[1], (0, 255, 255))
        cv2_cross(img, c_idx, 3, (255, 0, 255), 2)
        img = draw_line_from_params(img, c_idx, axes[1], (0, 255, 0))


        bbox_ctr = self.get_bbox_ctr()
        if bbox_ctr is not None:
            cv2.drawContours(img, [bbox_ctr], 0, (255, 255, 255), 1)


        # img = cv2.bitwise_and(img, ROBOT_MASK)
        
        if self.t_pos is not None:

            t_idx = pt(pos=self.t_pos).idx
            t_s_idx = pt(pos=self.t_start_pos).idx

            # desired centre position & line towards it - white
            cv2_cross(img, t_idx, 3, (255, 255, 255), 2)
            cv2.line(img, tuple(t_idx), tuple(t_s_idx), (255, 255, 255), 1)

            db_offset_dir = (t_idx - t_s_idx)
            db_offset_dir = np.array([db_offset_dir[1], -db_offset_dir[0]])
            db_offset_dir = db_offset_dir/pythag(db_offset_dir)

            db_offset_idx = (db_offset_dir * (POS_DEADBAND_W / IDX_TO_COORD_SF)).astype(int)

            # line deadband in orange
            cv2.line(img, tuple(t_idx + db_offset_idx), tuple(t_s_idx + db_offset_idx), (0, 127, 255), 1)
            cv2.line(img, tuple(t_idx - db_offset_idx), tuple(t_s_idx - db_offset_idx), (0, 127, 255), 1)

        if self.t_rot is not None:

            # img = draw_line_from_params(img, c_idx, axes[1], (255, 0, 0))
            img = draw_line_from_params(img, c_idx, self.t_rot, (255, 127, 0))
            # img = draw_line_from_params(img, c_idx, axes[1], (255, 0, 0))


        if self.target_mine is not None:
            cv2_cross(img, pt(pos=self.target_mine).idx, 3, (127, 127, 255), 2)



        # useful info
        if self.latest_cmd is not None:
            cv2_text(img, f"latest command: {self.latest_cmd}", (50, 50), (255, 255, 255))
        
        if self.theta_err is not None:
            cv2_text(img, f"rotation error: {self.theta_err:.2f} rad", (50, 65), (255, 255, 255))

        if self.pos_err is not None:
            cv2_text(img, f"target pos dist: {self.pos_err:.2f} m", (50, 80), (255, 255, 255))


        cv2_text(img, f"left wheel:  {round(self.l_cmd*255)}", (50, 95), (255, 255, 255))
        cv2_text(img, f"right wheel: {round(self.r_cmd*255)}", (50, 110), (255, 255, 255))
        cv2_text(img, f"ok to translate: {self.ok_to_translate}", (50, 125), (255, 255, 255))
        cv2_text(img, f"state: {self.state}", (50, 155), (255, 255, 255))
        cv2_text(img, f"centre: {centre[0]:.2f}, {centre[1]:.2f} m", (50, 170), (255, 255, 255))

        if USE_VIDEO and udpc.CURR_UDP_DATA is not None:
            cv2_text(img, f"Nina's status:", (50, 230), (255, 255, 255))

            for i, (key, val) in enumerate(udpc.CURR_UDP_DATA.items()):
                cv2_text(img, f"{key}:{val}", (60, 230 + 20 + 15 * i), (255, 255, 255))




        # mpl_show(robot_img)
        # if not USE_VIDEO:
        #     mpl_show(img)

        return img



    def get_bbox_ctr(self):

        v = self.get_pos_estimate()
        if v is None:
            return None

        origin, axes = v

        centre = self.get_centre_coords()

        c_idx = pt(pos=centre).idx
        a = np.matmul(axes, self.b_box1_offset / IDX_TO_COORD_SF)

        bound_cntr = np.array([ tuple([int(x), int(y)]) for x, y in [
            c_idx + a, 
            c_idx + a + self.b_box1_wh[1]*axes[1] / IDX_TO_COORD_SF,
            c_idx + a + (self.b_box1_wh[0]*axes[0] + self.b_box1_wh[1]*axes[1])  / IDX_TO_COORD_SF,
            c_idx + a + self.b_box1_wh[0]*axes[0] / IDX_TO_COORD_SF
        ]], dtype=np.int32)

        return bound_cntr

    def get_centre_coords(self):

        v = self.get_pos_estimate()
        if v is None:
            return None

        origin, dirs = v

        distorted = pt(idx=origin).pos + np.matmul(dirs, self.centre_offset)

        arena_centre = np.array([1.2, 1.2])

        radius = distorted - arena_centre

        relative = radius / (1.2 * SQRT2)

        correction = - 0.10 * relative

        return distorted + correction

        



    def get_pos_estimate(self):

        if len(self.origin_history) < 1: # make this 3
            return None

        # punish records which were a long time ago
        weights = np.exp(5.0*(np.array(self.timestamps) - self.timestamps[-1]))      
        # print(weights)

        def weighted_ave(arr):
            return sum([weights[i] * arr[i] for i in range(len(weights))])/sum(weights)
    
        # print(self.origin_history)
        return weighted_ave(self.origin_history), weighted_ave(self.axes_history)


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

        if len(self.origin_history) > 6:
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


    # in real world coords - also sets target rot
    def set_target_pos(self, t_pos, force = False):

        if force:
            self.remove_target_pos()

        centre = self.get_centre_coords()

        if centre is None:
            return False
        
        if pythag(t_pos - centre) < 0.03 and not force:
            print("already at target pos")
            return False

        self.achieved_target = False
        self.ok_to_translate = False

        self.t_pos = t_pos
        self.t_start_pos = centre
        
        rot = t_pos - centre

        self.set_target_rot(rot)

        return True


    def set_target_rot(self, rot):

        self.achieved_target = False

        if abs(1.0-pythag(rot)) > 0.01:
            rot = rot / pythag(rot)

        self.t_rot = rot


    def remove_target_pos(self):

        self.achieved_target = False
        self.t_rot = None
        self.t_pos = None
        self.t_start_pos = None
        self.ok_to_translate = False
        self.outside_deadband = False


    def control_to_target(self):
        
        vals = self.get_pos_estimate()

        if vals is None:
            return

        axes = vals[1]
        real_rot = axes[1] # heading vector
        real_pos = self.get_centre_coords()

        sign = lambda x: x/abs(x) if abs(x) > 0.000001 else 1

        # print("target_rot", self.t_rot)
        # print("actual_rot", real_rot)

        self.l_cmd = 0
        self.r_cmd = 0

        curr_t_rot = self.t_rot

        update_motors = True

        if self.t_pos is not None:
            self.pos_err = pythag(self.t_pos - real_pos)
            
            t_line_vec = self.t_pos - self.t_start_pos
            t_line_vec = t_line_vec / pythag(t_line_vec)

            d_vec = self.t_pos - real_pos

            further_d_vec = d_vec + 0.1*t_line_vec # 10 cm past
            curr_t_rot = d_vec / pythag(d_vec)
            
            if self.pos_err < 0.02:
                # made it to correct position!
                self.achieved_target = True
                self.t_pos = None
                self.t_rot = None
                self.send_cmd("stop")
                update_motors = False

        if self.t_rot is not None:

            d_prod = np.dot(curr_t_rot, real_rot)
            c_prod = curr_t_rot[0] * real_rot[1] - curr_t_rot[1] * real_rot[0]

            # print("c_prod", c_prod)

            theta_err = np.arccos(d_prod)
            if c_prod < 0:
                theta_err = -theta_err

            self.theta_err = theta_err
            # print("theta error (rad):", theta_err)

            if abs(theta_err) > DEG_TO_RAD * 3:
                
                if abs(theta_err) > DEG_TO_RAD * 20:
                    theta_rate = - MAX_THETA_RATE * sign(theta_err)
                
                else:
                    theta_rate = - MAX_THETA_RATE * theta_err / (DEG_TO_RAD * 20)

                self.l_cmd += theta_rate
                self.r_cmd -= theta_rate

            
            # hysteresis on ok_to_translate
            if abs(theta_err) < DEG_TO_RAD * 5:
                if self.t_pos is None:
                    self.achieved_target = True
                    self.t_rot = None
                else:
                    if not self.ok_to_translate:
                        print("ok to translate!")
                        self.send_cmd("stop")
                        update_motors = False
                        self.ok_to_translate = True


            if self.t_pos is not None:

                # check if outside the deadband and if so stop the robot
                # (so that the robot can rotate towards the target)
                perp_dist_from_t = d_vec[0] * t_line_vec[1] - d_vec[1] * t_line_vec[0]
                

                # self.outside_deadband provides hysteresis
                if abs(perp_dist_from_t) > POS_DEADBAND_W and not self.outside_deadband:
            
                    self.ok_to_translate = False
                    print("stopped - outside deadband")
                    self.send_cmd("stop")
                    update_motors = False

                    self.set_target_rot(d_vec)
                    self.outside_deadband = True

                if abs(perp_dist_from_t) < POS_DEADBAND_W / 2 and self.outside_deadband:
                    self.outside_deadband = False


                if self.ok_to_translate:                

                    # heading the right direction and ready to move forward.
                    if self.pos_err > 0.5: #m

                        self.l_cmd += MAX_TRANS_RATE
                        self.r_cmd += MAX_TRANS_RATE
                    
                    else:

                        self.l_cmd += MAX_TRANS_RATE * (self.pos_err + 0.5)/1.5
                        self.r_cmd += MAX_TRANS_RATE * (self.pos_err + 0.5)/1.5


            if update_motors:
                self.set_motors()

    def set_motors(self):

        if abs(self.l_cmd) < 0.01 and abs(self.r_cmd) < 0.01:
            self.send_cmd("stop")
            return

        l_int = int(round(self.l_cmd * 255))
        r_int = int(round(self.r_cmd * 255))

        self.send_cmd(f"run:0:{l_int}:{r_int}")
        
        

    # max rate of 5 Hz
    def send_cmd(self, cmd_txt):

        t_now = time.time() 

        if cmd_txt != self.latest_cmd and t_now - self.latest_cmd_time > 0.2:
            if USE_VIDEO:
                print("sending:", cmd_txt)
                udpc.sendCommand(cmd_txt.encode())

            if cmd_txt == "stop":
                time.sleep(0.2)
            self.latest_cmd = cmd_txt
            self.latest_cmd_time = t_now


    def recv_update(self):

        t_now = time.time() 
        cmd_txt = "get status"
        if t_now - self.latest_rcv_time > 0.5:
            if USE_VIDEO:
                udpc.sendCommand(cmd_txt.encode())

            self.latest_rcv_time = t_now


    def wants_mine_data(self):
        return self.wants_mine_data_flag


    def set_mine_locs(self, mine_centres):

        if len(mine_centres) > 0:
            self.mine_locs = mine_centres
            self.wants_mine_data_flag = False

        else:
            print("no mines detected")


Nina = Robot()

SQRT2 = 2**0.5 #correct
_1_SQRT2 = 1/SQRT2 #correct


def get_line_params(contour):
    params = cv2.fitLine(contour, cv2.DIST_L2,0,0.01,0.01)
    params = [p[0] for p in params]
    # print(params)
    return params


def draw_line_from_params(img, line_pt, line_v, colour):
    [vx, vy] = line_v
    [x, y] = line_pt
    rows,cols = img.shape[:2]
    # cv2_cross(img, (int(x + 50*vx), int(y + 50*vy)), 3, colour, 1)
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

    cv2.drawContours(img, red_cs, -1, (255, 255, 255), -1)

    # mpl_show(img)

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
    # cv2_cross(img, intersect_coords_1.astype(int), 3, (255, 255, 255), 2)
    # cv2_cross(img, intersect_coords_2.astype(int), 3, (255, 255, 255), 2)

    # mpl_show(img)

    # make sure intersection coords agree (sanity check)
    diff = intersect_coords_1 - intersect_coords_2
    if any(abs(i) > 0.1 for i in diff):
        print("intersect coords disagree: ", intersect_coords_1, intersect_coords_2)
        return None

    intersect_coords = (0.5 * (intersect_coords_1 + intersect_coords_2)).astype(int)


    for pt in line_pts:
        dist = pythag(pt - intersect_coords)
        if dist > max(Nina.dims/IDX_TO_COORD_SF):
            print("too far to intersect")

    
    # coordinate system now fully established
    # vector 0 is robot x, vector 1 is robot y

    sw.stop(False)

    return intersect_coords, line_vs



    

