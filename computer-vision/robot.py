# code to locate the robot in a frame, and Robot class
from global_stuff import *
from arena import *
from udpComms import CURR_UDP_DATA

if USE_VIDEO:
    import udpComms as udpc


pt = Point

MAX_THETA_RATE = 0.5
MAX_TRANS_RATE = 1

CV_N_MINE_CUTOFF = 2

POS_DEADBAND_W = 0.03 # m

class Robot():

    # ---------------------------------------------------------
    # initialisation
    # ---------------------------------------------------------

    def __init__(self):
        self.w_m = 0.175
        self.h_m = 0.155
        self.dims = np.array([self.h_m, self.w_m])
        self.centre_offset = np.array([0.065, 0.095])
        self.b_box1_offset = np.array([-0.250, -0.250])
        self.b_box1_wh = np.array([0.500, 0.500])

        self.competition_start_t = time.time()

        if not USE_VIDEO:
            self.origin_history = []
            self.axes_history = []
            self.timestamps = []

        else:
            self.origin_history = []
            self.axes_history = []
            self.timestamps = []

        
        # non-rigorous list of states
        self.states = [
            "rotating from start pos",
            "moving from start pos",
            "looking for mines",
            "moving to mine y",
            "moving to mine x",
            "moving to edge mine y",
            "moving to edge mine x",
            "picking up mine",
            "moving to bin x",
            "moving to bin y",
            "rotating to bin",
            "depositing mine",
            "moving to end pos"
        ]

        self.mine_types = ["live", "dead"]

        # initial state
        self.state = "rotating from start pos"
        self.prev_state = ""

        # internal state variables
        self.first_mine = True
        self.target_mine = None
        self.target_mine_idx = None
        self.has_mine = False
        self.looking_for_mines = False
        self.n_mines_known = START_MINES
        self.deposit_counts = { "live" : 0, "dead" : 0 }
        self.search_start_pos = np.array([SAFE_LINE_X_POS, SEARCH_START_Y_POS])
        self.deposit_time = time.time()
        self.look_mines_time = time.time()

        self.wants_mine_data_flag = False
        self.mine_locs = None

        # initialise target rotation and position vals
        self.remove_target_pos()

        self.l_cmd = 0
        self.r_cmd = 0
        
        self.theta_err = None
        self.pos_err = None
        self.latest_cmd_times = {}
        self.latest_cmd = None
        self.latest_rcv_time = time.time()

        # start comms with the Arduino
        if USE_VIDEO:
            udpc.rxthread.start()
            self.send_cmd("reset")



    # ---------------------------------------------------------
    # state machine
    # ---------------------------------------------------------

    # rudimentary state machine logic
    # uses text to store state in order to be
    # simple and human-readable
    def update_state(self, second_run=False):


        # decide if the current target mine can be driven directly towards
        # or if Nina should first move to search_start pos to avoid
        # hitting a mine in the bottom blind band
        def no_collide_chance():
            if self.target_mine is not None:
                if self.first_mine:
                    self.first_mine = False
                    print("no collide chance (first mine)")
                    return True
                
                if self.has_mine == "dead":
                    return True

                if self.target_mine[1] < self.target_mine[0] * 0.5 + RESOLUTION[1] * 0.5:
                    print("no collide chance (good region)")
                    return True
                else:
                    print("collide chance (bad region)")
                    return False

            return False # default just in case


        # decide if the Nina will move out of the camera's range when
        # picking up the mine, and therefore whether a blind reverse is
        # required after the pickup
        def needs_blind_reverse():
            if self.target_mine is not None:
                if self.target_mine[1] < MINE_AREA_TOP_LEFT.pos[1] + MINE_BLIND_Y_THRESH:
                    return True
                if self.target_mine[1] > MINE_AREA_BOT_RIGHT.pos[1] - MINE_BLIND_Y_THRESH:
                    return True

                return False
            return True # default just in case


        # ensure Nina returns home after 5m 36s have passed
        if time.time() - self.competition_start_t > 60 * 5.6:
            if "end pos" not in self.state:
                self.state = "moving to end pos"

        # blindly move out of start position (outside of camera range)
        if "from start pos" in self.state:
            
            if self.state == "rotating from start pos":
                if self.state != self.prev_state:
                    self.set_blind_cmd(0.7 / MAX_THETA_RATE, -MAX_THETA_RATE, MAX_THETA_RATE)

                if self.achieved_target:
                    self.state = "moving from start pos"

            if self.state == "moving from start pos":
                if self.state != self.prev_state:
                    self.set_blind_cmd(3 / MAX_TRANS_RATE, MAX_TRANS_RATE/2, MAX_TRANS_RATE/2)

                if self.achieved_target:
                    if self.get_centre_coords() is None:
                        self.set_blind_cmd(0.5 / MAX_TRANS_RATE, MAX_TRANS_RATE/2, MAX_TRANS_RATE/2)
                    else:
                        self.state = "looking for mines"
            

        # flag to main.py if mine positions are required,
        # then choose the optimal mine to pick up:
        #
        # - when leaving the start position, pick up the rightmost mine
        # on the way to the search start position (to optimise speed)
        #
        # - for remainder of mines, if there are any outside the bottom blind
        # band, drive directly towards them from the search start position
        # in order of increasing distance (this ensures no collisions)
        #
        # - finally, navigate to mines in the bottom blind band in such a 
        # way that Nina will not go out of the camera range
        #
        # - if a mine is at a safe angle from the search start position
        # (i.e. we are confident that Nina will not crash into a mine in the
        # bottom blind band), drive directly towards it rather than moving
        # to the start position first to avoid wasting time
        #
        # when approaching a mine, send a packet to Nina to say she can start
        # looking for mines - the Arduino code will take over once one is detected.
        # continue polling status updates until told that a mine is being carried,
        # then progress to next state depending on mine type
        if self.state == "looking for mines":
            if self.state != self.prev_state:
                self.prev_state = "looking for mines"

                if self.n_mines_known > CV_N_MINE_CUTOFF:
                    self.wants_mine_data_flag = True


            if not self.wants_mine_data():

                print("mine locs:", self.mine_locs)

                if self.first_mine:
                    self.mine_locs.sort(key=lambda loc: loc[0], reverse=True)
                else:
                    self.mine_locs.sort(key=lambda loc: pythag(loc - self.search_start_pos))
                
                print("  deciding on target mine...")
                for idx, mine in enumerate(self.mine_locs):
                    if mine[1] > MINE_AREA_BOT_RIGHT.pos[1] - MINE_BLIND_Y_THRESH:
                        print("    mine is close to edge of camera")
                        continue

                    if self.first_mine and mine[1] < MINE_AREA_TOP_LEFT.pos[1] + MINE_BLIND_Y_THRESH:
                        print("    mine is close to edge of camera")
                        continue

                    self.target_mine = mine
                    self.target_mine_idx = idx

                    if no_collide_chance():
                        self.state = "moving to mine pos"
                    else:
                        self.state = "moving to mine search start pos"

                    break
                    
                else:
                    print("    all mines left are close to edge of camera")
                    self.target_mine = self.mine_locs[0]
                    self.target_mine_idx = 0
                    self.state = "moving to edge mine search start pos"

                print("  picked mine at", self.target_mine)


        if "moving to mine" in self.state:
            
            if self.target_mine is None:
                self.state = "looking for mines"

            else:
                if self.state == "moving to mine search start pos":
                    if self.state != self.prev_state:
                        self.prev_state = "moving to mine search start pos"
                        self.set_target_pos(self.search_start_pos, rough=True)

                    if self.achieved_target:
                        self.state = "moving to mine pos"

                if self.state == "moving to mine pos":
                    if self.state != self.prev_state:
                        self.prev_state = "moving to mine pos"
                        self.set_target_pos(self.target_mine)

                    if USE_VIDEO and udpc.CURR_UDP_DATA is not None and time.time() - self.deposit_time > 10:
                        if self.theta_err is not None and self.pos_err is not None:
                            if self.pos_err < 0.25 and self.theta_err < 3 * DEG_TO_RAD and not self.looking_for_mines:
                                self.send_cmd("look mines")
                                self.look_mines_time = time.time()

                        if udpc.CURR_UDP_DATA["carryingMine"]:
                            # print("carrying mine")
                            self.looking_for_mines = False
                            self.state = "blind reverse after mine pickup"

                        if time.time() - self.look_mines_time > 10 and udpc.CURR_UDP_DATA["lookForMines"]:
                            print("hung around for too long looking - picking target mine again")
                            self.state = "looking for mines"


        # can't go directly to these or CV will lose Nina
        if "moving to edge mine" in self.state:
            
            if self.target_mine is None:
                self.state = "looking for mines"

            else:
                if self.state == "moving to edge mine search start pos":
                    if self.state != self.prev_state:
                        self.prev_state = "moving to edge mine search start pos"
                        self.set_target_pos(self.search_start_pos, rough=True)

                    if self.achieved_target:
                        self.state = "moving to edge mine x"

                if self.state == "moving to edge mine x":
                    if self.state != self.prev_state:
                        self.set_target_pos(np.array([self.target_mine[0], SEARCH_START_Y_POS]), rough=True)
                        self.prev_state = "moving to edge mine x"

                    if self.achieved_target:
                        self.state = "moving to edge mine y"

                if self.state == "moving to edge mine y":
                    if self.state != self.prev_state:
                        self.set_target_pos(self.target_mine)
                        self.prev_state = "moving to edge mine y"
                  
                    if USE_VIDEO and udpc.CURR_UDP_DATA is not None:
                        if self.theta_err is not None and self.pos_err is not None:
                            if self.pos_err < 0.25 and self.theta_err < 3 * DEG_TO_RAD and not self.looking_for_mines:
                                self.send_cmd("look mines")      

                        if udpc.CURR_UDP_DATA["carryingMine"]:
                            self.looking_for_mines = False
                            self.state = "blind reverse after mine pickup"


        if self.state == "blind reverse after mine pickup":


            if self.state != self.prev_state:
                self.state = "blind reverse after mine pickup"
                if needs_blind_reverse():
                    self.set_blind_cmd(3/MAX_TRANS_RATE, -MAX_TRANS_RATE/2, -MAX_TRANS_RATE/2)
                else:
                    self.achieved_target = True

            if self.achieved_target:
                if USE_VIDEO and udpc.CURR_UDP_DATA is not None:
                    is_live = udpc.CURR_UDP_DATA["livemine"] 

                    if is_live:
                        self.has_mine = "live"
                    else:
                        self.has_mine = "dead"

                    if no_collide_chance():
                        self.state = "moving to bin " + self.has_mine
                    else:
                        self.state = "moving to bin"
        

        # move to hardcoded live and dead mine bin positions, then rotate
        # towards the bin and drive blindly towards it.
        # for the live bin, approach diagonally to avoid being lost by camera.
        # make incremental changes to deposit position to avoid
        # dropping mines in exactly the same place.
        if "moving to bin" in self.state:
            if not self.has_mine:
                self.state = "looking for mines"

            if self.state == "moving to bin":

                if self.state != self.prev_state:
                    self.prev_state = "moving to bin"
                    self.set_target_pos(self.search_start_pos, rough=True)
                if self.achieved_target:
                    if self.has_mine == "live":
                        self.state = "moving to bin live"
                    else:
                        self.state = "moving to bin dead"

            if self.state == "moving to bin live":

                if self.state != self.prev_state:
                    self.prev_state = "moving to bin live"
                    pos_shift = 0#0.01 * (1 - 0.666*self.deposit_counts["live"])
                    self.set_target_pos(np.array([SAFE_LINE_X_POS - pos_shift, LIVE_DEPOSIT_Y_POS + pos_shift]))

                if self.achieved_target:
                    self.state = "moving to bin rotation"

            elif self.state == "moving to bin dead":

                if self.state != self.prev_state:
                    self.prev_state = "moving to bin dead"
                    y_pos = DEAD_DEPOSIT_Y_POS + 0.02 * (1 - 0.666*self.deposit_counts["dead"])
                    self.set_target_pos(np.array([SAFE_LINE_X_POS, y_pos]))

                if self.has_mine == "live":
                    self.state = "moving to bin live"

                if self.achieved_target:
                    self.state = "moving to bin rotation"

            if self.state == "moving to bin rotation":
                
                if self.state != self.prev_state:
                    self.prev_state = "moving to bin rotation"
                    if self.has_mine == "live":
                        self.set_target_rot(np.array([1, 1]))
                    else:
                        self.set_target_rot(np.array([1, 0]))

                if self.achieved_target:
                    self.state = "moving to bin close"

            if self.state == "moving to bin close":

                if self.state != self.prev_state:
                    self.prev_state = "moving to bin close"
                    if self.has_mine == "live":
                        self.set_blind_cmd(2.8/MAX_TRANS_RATE, MAX_TRANS_RATE/2, MAX_TRANS_RATE/2)
                    elif self.has_mine == "dead":
                        self.set_blind_cmd(1.5/MAX_TRANS_RATE, MAX_TRANS_RATE/2, MAX_TRANS_RATE/2)
                if self.achieved_target:
                    self.state = "depositing mine"


        # tell Nina to drop the mine.
        # delete the mine from the list of known mines. (this is
        # overridden if we are above CV_N_MINE_CUTOFF mines left
        # by the mine detection.)
        # then transfer to 'looking for mines' state during deposit process
        # so that no time is wasted while running mine detection algorithm.
        # if no mines left, move to end position.
        if self.state == "depositing mine":

            if self.state != self.prev_state:
                self.prev_state = "depositing mine"
                self.send_cmd("drop mine")
                self.deposit_time = time.time()

            if time.time() - self.deposit_time > 2:

                self.n_mines_known -= 1
                self.deposit_counts[self.has_mine] += 1
                print(f"incrementing {self.has_mine} count")

                print("removing mine location from mine_locs")
                del self.mine_locs[self.target_mine_idx]

                if self.n_mines_known == 0:
                    self.state = "moving to end pos"
                else:
                    print("looking for mines")
                    self.state = "looking for mines"

        
        # move to just in front of end position then blindly move 
        # forward (out of camera range) into it.
        if "moving to end pos" in self.state:
            if self.state == "moving to end pos":
                if self.state != self.prev_state:
                    self.send_cmd("return home")
                    self.prev_state = "moving to end pos"
                    self.set_target_pos(np.array([SAFE_LINE_X_POS, 1.0]), rough=True)
                if self.achieved_target:
                    self.state = "moving to end pos 2"

            if self.state == "moving to end pos 2":
                if self.state != self.prev_state:
                    self.prev_state = "moving to end pos 2"
                    self.set_target_pos(np.array([2.05, 0.5]), rough=True)
                if self.achieved_target:
                    self.state = "moving to end pos blind"
            
            if self.state == "moving to end pos blind":
                if self.state != self.prev_state:
                    self.prev_state = "moving to end pos blind"
                    self.set_blind_cmd(3.0/MAX_TRANS_RATE, MAX_TRANS_RATE/2, MAX_TRANS_RATE/2)
                if self.achieved_target:
                    self.state = "at end pos"
                    self.send_cmd("reset")
                    # easy way to quit program and handle cleanup
                    raise Exception("finished") 


        if self.state != self.prev_state and not second_run:
            print("second run:", self.prev_state, self.state)
            self.update_state(second_run=True)

        self.prev_state = self.state



    # ---------------------------------------------------------
    # illustration of frame with useful display data
    # ---------------------------------------------------------

    def illustrate(self, img):

        est = self.get_pos_estimate()

        if est is None:
            return img
        
        origin, axes = est
        centre = self.get_centre_coords()
        c_idx = pt(pos=centre).idx

        # x: cyan, y: yellow, magenta: centre, green: heading
        img = draw_line_from_params(img, origin, axes[0], (255, 255, 0)) 
        img = draw_line_from_params(img, origin, axes[1], (0, 255, 255))
        cv2_cross(img, c_idx, 3, (255, 0, 255), 2)
        img = draw_line_from_params(img, c_idx, axes[1], (0, 255, 0))


        bbox_ctr = self.get_bbox_ctr()
        if bbox_ctr is not None:
            cv2.drawContours(img, [bbox_ctr], 0, (255, 255, 255), 1)


        if self.t_pos is not None:

            t_idx = pt(pos=self.t_pos).idx
            t_s_idx = pt(pos=self.t_start_pos).idx

            # desired centre position & line towards it: white
            cv2_cross(img, t_idx, 3, (255, 255, 255), 2)
            cv2.line(img, tuple(t_idx), tuple(t_s_idx), (255, 255, 255), 1)

            db_offset_dir = (t_idx - t_s_idx)
            db_offset_dir = np.array([db_offset_dir[1], -db_offset_dir[0]])
            db_offset_dir = db_offset_dir/pythag(db_offset_dir)

            db_offset_idx = (db_offset_dir * (POS_DEADBAND_W / IDX_TO_COORD_SF)).astype(int)

            # line deadband: orange
            cv2.line(img, tuple(t_idx + db_offset_idx), tuple(t_s_idx + db_offset_idx), (0, 127, 255), 1)
            cv2.line(img, tuple(t_idx - db_offset_idx), tuple(t_s_idx - db_offset_idx), (0, 127, 255), 1)

        if self.t_rot is not None:
            img = draw_line_from_params(img, c_idx, self.t_rot, (255, 127, 0))


        if self.target_mine is not None:
            cv2_cross(img, pt(pos=self.target_mine).idx, 3, (127, 127, 255), 2)


        # useful info displayed on screen in text
        if self.latest_cmd is not None:
            cv2_text(img, f"latest command: {self.latest_cmd}", (50, 50), (255, 255, 255))
        
        if self.theta_err is not None:
            cv2_text(img, f"rotation error: {self.theta_err:.2f} rad", (50, 65), (255, 255, 255))

        if self.pos_err is not None:
            cv2_text(img, f"target pos dist: {self.pos_err:.2f} m", (50, 80), (255, 255, 255))

        cv2_text(img, f"number of mines known: {self.n_mines_known}", (50, 95), (255, 255, 255))
        cv2_text(img, f"deposited: {self.deposit_counts['live']}/4 (live), {self.deposit_counts['dead']}/4 (dead)", (50, 110), (255, 255, 255))
        cv2_text(img, f"ok to translate: {self.ok_to_translate}", (50, 125), (255, 255, 255))
        cv2_text(img, f"state: {self.state}", (50, 155), (255, 255, 255))
        cv2_text(img, f"centre: {centre[0]:.2f}, {centre[1]:.2f} m", (50, 170), (255, 255, 255))
        cv2_text(img, f"time elapsed: {time.time() - self.competition_start_t:.2f} s", (50, 185), (255, 255, 255))
        cv2_text(img, f"current task: {self.state}", (50, 200), (255, 255, 255))


        if USE_VIDEO and udpc.CURR_UDP_DATA is not None:
            cv2_text(img, f"Nina's status:", (50, 250), (255, 255, 255))

            for i, (key, val) in enumerate(udpc.CURR_UDP_DATA.items()):
                cv2_text(img, f"{key}:{val}", (60, 250 + 20 + 15 * i), (255, 255, 255))


        return img


    # ---------------------------------------------------------
    # position estimation & recording
    # ---------------------------------------------------------

    # returns an opencv contour object which the robot is guaranteed
    # to be fully encapsulated in - this is used as a mask for mine detection
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

    
    # based on detected robot axes locations and intersections,
    # correct for camera distortion and robot dimensions
    # to return a centre position in real-world coords.
    def get_centre_coords(self):

        v = self.get_pos_estimate()
        if v is None:
            return None

        origin, dirs = v

        distorted = pt(idx=origin).pos + np.matmul(dirs, self.centre_offset)

        arena_centre = np.array([1.2, 1.2])

        radius = distorted - arena_centre

        relative = radius / (1.2 * SQRT2)

        correction = - 0.15 * relative

        return distorted + correction

        
    # returns an exponentially-convolved rolling mean estimation
    # of Nina's detection positions & axes.
    def get_pos_estimate(self):

        # if insufficient data, return None.
        if len(self.origin_history) < 1:
            return None

        # punish recorded positions which were a long time ago
        weights = np.exp(5.0*(np.array(self.timestamps) - self.timestamps[-1]))      

        def weighted_ave(arr):
            return sum([weights[i] * arr[i] for i in range(len(weights))])/sum(weights)
    
        # print(self.origin_history)
        return weighted_ave(self.origin_history), weighted_ave(self.axes_history)


    # records a detected Nina position, if it's sensible (no teleportation)
    # (there are other validation checks in detect_robot() itself)
    def record_pos(self, origin, axes):

        ts = time.time()
        outrageous_speed = 0.4 # m/s

        est = self.get_pos_estimate()

        # if no existing positions on record, trust this data
        if est is None:
            self.origin_history.append(origin)
            self.axes_history.append(axes)
            self.timestamps.append(ts)
        
        else:
            distance = pythag(pt(idx=origin).pos - pt(idx=est[0]).pos)

            # if speed greater than outrageous_speed 
            # (accounting for noise w/ first clause), ignore
            if distance > 0.05 and distance / (ts - self.timestamps[0]) > outrageous_speed:
                print("didn't record - moved too fast")
                return

            else:
                self.origin_history.append(origin)
                self.axes_history.append(axes)
                self.timestamps.append(ts)

        if len(self.origin_history) > 6:
            del self.origin_history[0]
            del self.axes_history[0]
            del self.timestamps[0]


    # calls detect_robot on masked image and self.record_pos with result
    def update_pos(self, img):
        img = cv2.bitwise_and(img, ROBOT_MASK)

        res = detect_robot(img)

        if res is None:
            return img

        origin, axes = res

        self.record_pos(origin, axes)



    # ---------------------------------------------------------
    # control & communications with Nina
    # ---------------------------------------------------------

    # set motor values to blindly follow for a set period of time
    def set_blind_cmd(self, t, l, r):
        
        self.remove_target_pos()

        self.moving_blind = True
        self.blind_start_time = time.time()
        self.l_cmd = l
        self.r_cmd = r
        self.blind_timeout_time = t


    # set target position in real world coords - also sets target rot
    def set_target_pos(self, t_pos, force = False, rough = False):

        if force:
            self.remove_target_pos()


        centre = self.get_centre_coords()

        if centre is None:
            return False
        
        if pythag(t_pos - centre) < 0.03 and not force:
            print("already at target pos")
            return False

        self.rough_target = rough
        self.achieved_target = False
        self.ok_to_translate = False

        self.t_pos = t_pos
        self.t_start_pos = centre
        
        rot = t_pos - centre

        self.set_target_rot(rot)

        return True


    # set target orientation (as a heading vector)
    def set_target_rot(self, rot):

        self.achieved_target = False

        if abs(1.0-pythag(rot)) > 0.01:
            rot = rot / pythag(rot)

        self.t_rot = rot


    # wipe misc state variables
    def remove_target_pos(self):

        self.achieved_target = False
        self.rough_target = False
        self.t_rot = None
        self.t_pos = None
        self.t_start_pos = None
        self.ok_to_translate = False
        self.outside_deadband = False
        self.moving_blind = False
        self.blind_timeout_time = 0


    # logic to move Nina to a target position/orientation.
    #
    # when moving to a target position, Nina will:
    # - rotate until facing towards the target
    # - then start moving forward at full speed
    # - perpendicular deviations from target line are compensated with additional rotation
    # - there is fixed 'deadband' around the target line in which Nina is allowed to be.
    #   * this is to prevent Nina from accidentally crashing into mines during motion.
    #   * if Nina exits the deadband, she will stop, rotate to the correct orientation,
    #     then start moving forwards again until she's inside the deadband
    # - when near the target position/rotation, rotation/translation speed decreases
    # - Nina's target orientation is a few cm behind the target position to prevent 
    #   extreme orientation changes due to large errors when nearby.
    # - on achieving a target, self.achieved_target is set to True, which is
    #   the primary form of communication with Nina's state machine.
    # - a target is achieved when within a certain distance/angle from the target.
    # - if 'self.rough_target' is True, Nina will move more quickly and 
    #   with more lenient achieved_target conditions, to optimise speed when
    #   fine positioning is unecessary.
    # 
    # there is also code here to handle blind motion
    # and target rotations without target positions.
    def control_to_target(self):
        
        # blind motion handling
        if self.moving_blind:
            if time.time() - self.blind_start_time < self.blind_timeout_time:
                self.set_motors()
            else:
                self.send_cmd("stop")
                self.achieved_target = True
                self.moving_blind = False
                self.pos_err = 999
                self.theta_err = 999
            return


        vals = self.get_pos_estimate()

        if vals is None:
            return

        axes = vals[1]
        real_rot = axes[1] # heading vector
        real_pos = self.get_centre_coords()

        sign = lambda x: x/abs(x) if abs(x) > 0.000001 else 1

        self.l_cmd = 0
        self.r_cmd = 0

        curr_t_rot = self.t_rot

        update_motors = True
        # if there is a target position, calculate some useful
        # quantities and check if target has been achieved.
        if self.t_pos is not None:
            self.pos_err = pythag(self.t_pos - real_pos)
            
            t_line_vec = self.t_pos - self.t_start_pos
            t_line_vec = t_line_vec / pythag(t_line_vec)

            d_vec = self.t_pos - real_pos

            further_d_vec = d_vec + 0.05*t_line_vec # 5 cm past
            curr_t_rot = d_vec / pythag(d_vec)
            
            if self.pos_err < 0.02 or (self.rough_target and self.pos_err < 0.05):
                # made it to correct position!
                self.achieved_target = True
                self.t_pos = None
                self.t_rot = None
                self.send_cmd("stop")
                update_motors = False


        # if there is a target rotation (this is always the case
        # when there is a target position), do rotation control
        if self.t_rot is not None:

            # calculate angle error from dot product
            # ensure angle has correct sense using cross produt
            d_prod = np.dot(curr_t_rot, real_rot)
            c_prod = curr_t_rot[0] * real_rot[1] - curr_t_rot[1] * real_rot[0]

            theta_err = np.arccos(d_prod)
            if c_prod < 0:
                theta_err = -theta_err

            self.theta_err = theta_err

            if abs(theta_err) > DEG_TO_RAD * 3:
                
                # first if statement prevents rotation back and forth
                # when facing directly away form target due to latency -
                # always pick one direction when >150 degrees away
                if abs(theta_err) > DEG_TO_RAD * 150:
                    theta_rate = - MAX_THETA_RATE              
                # rotate at max speed when >50 degrees away  
                elif abs(theta_err) > DEG_TO_RAD * 50:
                    theta_rate = - MAX_THETA_RATE * sign(theta_err)
                # rotate at half max speed when >25 degrees away
                elif abs(theta_err) > DEG_TO_RAD * 25:
                    theta_rate = - MAX_THETA_RATE  * sign(theta_err) / 2
                # rotate at speed proportional to angle error when <=25 degrees away
                else:
                    theta_rate = - MAX_THETA_RATE  * self.theta_err / (30 * DEG_TO_RAD) 

                self.l_cmd += theta_rate
                self.r_cmd -= theta_rate


            # correct rotation and no target pos -> achieved target
            elif self.t_pos is None:
                self.achieved_target = True
                self.pos_err = 999
                self.theta_err = 999
                self.t_rot = None

            # correct rotation and target pos -> ok to translate
            elif not self.ok_to_translate:
                print("ok to translate")
                self.send_cmd("stop")
                update_motors = False
                self.ok_to_translate = True

            
            # hysteresis on ok_to_translate
            if abs(theta_err) > DEG_TO_RAD * 10 and self.ok_to_translate:
                print("not ok to translate")
                self.send_cmd("stop")
                update_motors = False
                self.ok_to_translate = False
                

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
                    if self.rough_target:
                        if self.pos_err > 0.2: # m
                            self.l_cmd += MAX_TRANS_RATE
                            self.r_cmd += MAX_TRANS_RATE
                        
                        else:
                            self.l_cmd += MAX_TRANS_RATE * 0.75
                            self.r_cmd += MAX_TRANS_RATE * 0.75

                    else:
                        if self.pos_err > 0.35: # m
                            self.l_cmd += MAX_TRANS_RATE
                            self.r_cmd += MAX_TRANS_RATE
                        
                        else:
                            self.l_cmd += MAX_TRANS_RATE * (self.pos_err + 0.5)/1.5
                            self.r_cmd += MAX_TRANS_RATE * (self.pos_err + 0.5)/1.5


            if update_motors:
                self.set_motors()

    # l_cmd and r_cmd are float between -1.0 and 1.0
    # sent values are integers between -255 and 255
    def set_motors(self):

        # effectively 0 speed -> stop
        if abs(self.l_cmd) < 0.01 and abs(self.r_cmd) < 0.01:
            self.send_cmd("stop")
            return

        # avoid sending > 255 but maintain ratio
        max_cmd = max(self.l_cmd, self.r_cmd)
        if max_cmd > 1:
            self.l_cmd /= max_cmd
            self.r_cmd /= max_cmd


        l_int = int(round(self.l_cmd * 255))
        r_int = int(round(self.r_cmd * 255))

        self.send_cmd(f"run:0:{l_int}:{r_int}")
        
        
    # max rate of 2.5 Hz for any particular command
    # latest send times are stored in self.latest_cmd_times
    def send_cmd(self, cmd_txt, verbose=False):

        t_now = time.time() 

        command = cmd_txt.split(":")[0]

        if command in self.latest_cmd_times:
            latest_time = self.latest_cmd_times[command]
            if (t_now - latest_time) < 0.4:
                if verbose:
                    print("didn't send command:", cmd_txt)
                return

    
        if USE_VIDEO:
            if verbose or cmd_txt in ("look mines", "stop mines"):
                print("sending:", cmd_txt)
                if cmd_txt == "look mines":
                    self.looking_for_mines = True
                else:
                    self.looking_for_mines = False
            udpc.sendCommand(cmd_txt.encode())
        
        self.latest_cmd_times[command] = time.time()
        self.latest_cmd = cmd_txt



    # at 2Hz, send "get status" packets
    def recv_update(self):

        t_now = time.time() 
        cmd_txt = "get status"
        if t_now - self.latest_rcv_time > 0.5:
            if USE_VIDEO:
                udpc.sendCommand(cmd_txt.encode())

            self.latest_rcv_time = t_now




    # ---------------------------------------------------------
    # updating mine locations logic
    # ---------------------------------------------------------

    # getter
    def wants_mine_data(self):
        return self.wants_mine_data_flag


    # update mine_centres if sensible
    def set_mine_locs(self, mine_centres):
                
        if len(mine_centres) == 0:
            print("no mines detected.")
            return

        if self.n_mines_known is None:
            self.n_mines_known = len(mine_centres)
        else:
            if len(mine_centres) != self.n_mines_known and len(mine_centres) != START_MINES:
                print(f"  {len(mine_centres)} mines left - expected {self.n_mines_known}.")
                if len(mine_centres) == self.n_mines_known + 1:
                    self.n_mines_known = len(mine_centres)

        self.mine_locs = [mine.pos for mine in mine_centres]
        self.wants_mine_data_flag = False


    def reset(self):
        self.remove_target_pos()
        self.state = "rotating from start pos"
        self.prev_state = ""

Nina = Robot()

SQRT2 = 2**0.5
_1_SQRT2 = 1/SQRT2


# ---------------------------------------------------------
# computer vision for detecting 
# ---------------------------------------------------------

# wrapper to get important stuff from opencv line fitting function
def get_line_params(contour):
    params = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
    params = [p[0] for p in params]
    return params


# for illustration purposes
def draw_line_from_params(img, line_pt, line_v, colour):
    [vx, vy] = line_v
    [x, y] = line_pt
    rows,cols = img.shape[:2]
    return cv2.line(img, (int(x), int(y)), (int(x + 50*vx), int(y + 50*vy)), colour, 1)


# basic utility functions for geometry
def pythag(v):
    return np.sqrt(np.dot(v, v))


def norm(v):
    return v / np.sqrt(np.dot(v, v))


# solve equation system for intersect
def find_intersect_coeffs(line_vs, line_pts):

    m = np.transpose(np.array([line_vs[0], -line_vs[1]]))
    c = line_pts[1] - line_pts[0]
    
    try:
        x = np.linalg.solve(m, c)
    except:
        print("singular matrix")
        return None

    return x


# fit vectors which are 100% perpendicular (rather than roughly)
# to a pair of roughly perpendicular vectors
def fit_orthog_vs(line_vs):

    halfway = norm(sum(line_vs))
    
    rot_45_ccw = np.array([[_1_SQRT2, -_1_SQRT2], [_1_SQRT2, _1_SQRT2]])
    rot_45_cw  = np.array([[_1_SQRT2, _1_SQRT2], [-_1_SQRT2, _1_SQRT2]])

    vy = np.matmul(halfway, rot_45_ccw)
    vx = np.matmul(halfway, rot_45_cw)
    line_vs = np.array([vx, vy])

    # print("new line_vs", line_vs)

    return line_vs


# tape hue values - use two ranges for red
# as it is at both ends of the spectrum
HUE_NORMALISE = 179.0/360.0
RED1 = np.array([0, 20])
RED2 = np.array([340, 360])


# main detection function
def detect_robot(img):

    # split into HSV colour space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)


    # threshold the hue into a colour range (called with ranges defined above)
    def get_colour_contours(colour_range):

        colour_min, colour_max = (colour_range * HUE_NORMALISE).astype(int)
        in_range = cv2.inRange(hsv, np.array([colour_min, 100, 40]), 
                                    np.array([colour_max, 255, 255]))
        
        contours, ret = cv2.findContours(in_range, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        return contours


    red_cs = get_colour_contours(RED1)
    red_cs.extend(get_colour_contours(RED2))

    # validation: at least 2 contours should exist
    if len(red_cs) < 2:
        # print("not enough contours")
        return None

    # sort contours by area
    red_cs = sorted(red_cs, key=lambda c: cv2.contourArea(c), reverse=True)[:2]
    
    # fit lines to the two largest contours
    red_lines = [get_line_params(c) for c in red_cs]]

    cv2.drawContours(img, red_cs, -1, (255, 255, 255), -1)

    # unpack the line params: vi are vectors, ai are points
    v1 = np.array(red_lines[0][:2])
    v2 = np.array(red_lines[1][:2])
    line_vs = np.array([v1, v2])

    a1 = np.array(red_lines[0][2:])
    a2 = np.array(red_lines[1][2:])
    line_pts = np.array([a1, a2])

    # validation: lines are orthogonal (roughly)
    theta = (180/np.pi) * np.arccos(abs(np.dot(line_vs[0], line_vs[1])))
    if (abs(90-theta) > 30):
        # print("more than 30 degrees from orthogonal:", theta)
        return None

    coeffs = find_intersect_coeffs(line_vs, line_pts)

    # point vectors away from intersect point if not already
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
    
    # validation: make sure intersection coords agree (should always happen)
    diff = intersect_coords_1 - intersect_coords_2
    if any(abs(i) > 0.1 for i in diff):
        # print("intersect coords disagree: ", intersect_coords_1, intersect_coords_2)
        return None

    intersect_coords = (0.5 * (intersect_coords_1 + intersect_coords_2)).astype(int)


    # validation: intersection is less than Nina dimensions
    # away from line centres of area
    for pt in line_pts:
        dist = pythag(pt - intersect_coords)
        if dist > max(Nina.dims/IDX_TO_COORD_SF):
            # print("too far to intersect")
            return None

    
    # coordinate system now fully established
    # vector 0 is robot x, vector 1 is robot y
    return intersect_coords, line_vs