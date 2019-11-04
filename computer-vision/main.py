from global_stuff import *

import mines
from robot import Nina

if USE_VIDEO:
    import udpComms as udpc

from arena import *
pt = Point


def do_mine_stuff(frame, nina_mask_ctr, n_mines_known):

    frame_grey = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    x1, y1 = MINE_AREA_TOP_LEFT.cv_tup
    x2, y2 = MINE_AREA_BOT_RIGHT.cv_tup
    mine_area_img = frame_grey[y1:y2, x1:x2].copy()

    arena_mask_cpy = ARENA_MASK.copy()

    if nina_mask_ctr is not None:
        cv2.drawContours(arena_mask_cpy, [nina_mask_ctr], 0, (0, 0, 0), -1)

    mine_area_mask = arena_mask_cpy[y1:y2, x1:x2]
    
    BASE_DETECTION_THRESH = 30

    detection_thresh = BASE_DETECTION_THRESH
    mine_details = mines.find_mines(mine_area_img, mine_area_mask, detection_thresh, x1, y1)

    # start at high threshold (detect fewer mines) and increase until enough mines found
    if n_mines_known is not None:
        while len(mine_details) < n_mines_known:
            if detection_thresh <= 10:
                print("too many attempts. Giving up")
                return []

            detection_thresh -= 5
            print(f"not enough mines ({len(mine_details)} / {n_mines_known}) - reducing threshold to {detection_thresh}")
            mine_details = mines.find_mines(mine_area_img, mine_area_mask, detection_thresh, x1, y1)
        
        # probably garbage due to no mines being in the field
        # - no contrast so thinks lots of mines in arena
        if len(mine_details) > n_mines_known + 4:
            print(f"too many mines ({len(mine_details)} / {n_mines_known}) - returning none")
            return []

        # when done, reduce threshold 1 more time in case Nina failed to pick up
        detection_thresh -= 5
        print(f"all mines detected, but reducing threshold once to {detection_thresh} in case Nina missed one")
        mine_details_2 = mines.find_mines(mine_area_img, mine_area_mask, detection_thresh, x1, y1)

        if len(mine_details_2) > len(mine_details) + 2 or len(mine_details_2) < len(mine_details):
            print("never mind, using previous threshold")
        else:
            mine_details = mine_details_2

        print(f"mines detected: {len(mine_details)} / {n_mines_known}")

    else:
        print(f"mines detected: {len(mine_details)}")

    return mine_details


def show_img(frame):
    
    # frame = cv2.resize(frame, tuple(2*RESOLUTION))

    if USE_VIDEO:
        cv2.imshow("frame", frame) 
        # cv2.waitKey(0)
    else:
        mpl_show(frame)

def illustrate(frame, mine_details=[]):

    draw_arena_features(frame)
    
    if len(mine_details) > 0:
        # print("mine locations & rads: (m)")
        for j, (centre, radius) in enumerate(mine_details):
            cv2_cross(frame, centre.cv_tup, 2, (0,100,255), 1)
            # print(f"pos: {centre.pos}")

    # robot.detect_robot(frame)

    # Display the resulting frame
    return frame
 

general_vid_params = {
    "h_res" : {"idx" : 3, "setval" : RESOLUTION[0]},
    "v_res" : {"idx" : 4, "setval" : RESOLUTION[1]},
    "fps" : {"idx" : 5, "setval" : 10},
    "brightness" : {"idx" : 11, "setval" : 32.0}
}

mine_vid_params = {
    # this sets resolution lower
    "h_res" : {"idx" : 3, "setval" : RESOLUTION[0]},
    "v_res" : {"idx" : 4, "setval" : RESOLUTION[1]},
    "saturation" : {"idx" : 12, "setval" : 28.0},
    "exposure" : {"idx" : 15, "setval" : -7.0} # -8.0 is max exposure
}

robot_vid_params = {
    "h_res" : {"idx" : 3, "setval" : RESOLUTION[0]},
    "v_res" : {"idx" : 4, "setval" : RESOLUTION[1]},
    "saturation" : {"idx" : 12, "setval" : 28.0},
    "exposure" : {"idx" : 15, "setval" : -4.0}
}

# USE THESE WHEN DONE SO OTHER PEOPLE DON'T REALISE (old camera exposure)
default_vid_params = {
    "saturation" : {"idx" : 12, "setval" : 28.0},
    "exposure" : {"idx" : 15, "setval" : -7.0}
}

def set_params(cap, params, verbose=False):

    for key, data in params.items():
        if verbose: print(f"{key}: {cap.get(data['idx'])}, set to {data['setval']}")
        ret = cap.set(data["idx"], data["setval"])
        if (not ret) and verbose:
            print("failed")

    time.sleep(1)


def get_frame(cap):
    ret, frame = cap.read()
    return cv2.bitwise_and(frame, cv2.cvtColor(ARENA_MASK, cv2.COLOR_GRAY2RGB))


if __name__ == "__main__":

    if USE_VIDEO:

        print("setting up camera... (takes a while!)")
        start_setup = time.time()
        print("initialising comms...")
        cap = cv2.VideoCapture(0)
        print("initialising comms done.")
        set_params(cap, general_vid_params)
        set_params(cap, robot_vid_params)

        end_setup = time.time()

        print(f"done. setup took {end_setup - start_setup} seconds (jheez)")

        start_input = input("press enter to begin!")

        Nina.competition_start_t = time.time()

        mine_data = []
        i = 0

        while True:

            if Nina.wants_mine_data() and DO_MINES:
                
                print("getting mines")

                set_params(cap, mine_vid_params)

                m_frame = get_frame(cap)
                cv2.imwrite("mine_mode.jpg", m_frame)

                mine_data = do_mine_stuff(m_frame, Nina.get_bbox_ctr(), Nina.n_mines_known)

                # print(mine_data)
                # continue

                Nina.set_mine_locs([centre for centre, rad in mine_data])

                set_params(cap, robot_vid_params)
                

            r_frame = get_frame(cap)
            cv2.imwrite("robot_mode.jpg", r_frame)
        
            r_frame = cv2.resize(r_frame, tuple(RESOLUTION))

            Nina.update_pos(r_frame)
            Nina.recv_update()

            # if i < 10:
            #     Nina.set_target_pos(np.array([1.0, 1.5]))

            # if Nina.achieved_target:
            #     Nina.wants_mine_data_flag = True
            #     # Nina.set_target_pos(np.random.rand(2)*1.4 + 0.5)
            #     print("set new target")

            Nina.update_state()

            Nina.illustrate(r_frame)
            
            Nina.control_to_target()

            to_show = illustrate(r_frame, mine_data)

            show_img(to_show)

            # time.sleep(2)

            # i += 1
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                Nina.send_cmd("stop", True)
                break
            # if key & 0xFF == ord('s'):
            #     i += 1
            #     cv2.imwrite("save{i}.jpg", to_show)
            if key & 0xFF == ord('r'):
                Nina.send_cmd("reset", True)
            if key & 0xFF == ord('d'):
                Nina.send_cmd("drop mine", True)
            if key & 0xFF == ord('l'):
                Nina.send_cmd("look mines", True)
            if key & 0xFF == ord('k'):
                Nina.send_cmd("stop mines", True)
            if key & 0xFF == ord('s'):
                Nina.send_cmd("stop", True)
            if key & 0xFF == ord('o'):
                Nina.send_cmd("stop", True)
                time.sleep(2)
                Nina.reset()


        set_params(cap, default_vid_params)

        print("releasing camera 'capture' object")
        cap.release()
        cv2.destroyAllWindows()

        udpc.STOP_THREAD = True


    else:
        print(f"running on single frame (fpath: {F_NAME})")

        img = cv2.imread(F_NAME)
        img = cv2.resize(img, tuple(RESOLUTION))

        if DO_ROBOT:
            Nina.update_pos(img)


        mine_data = []
        if DO_MINES:
            mine_data = do_mine_stuff(img, Nina.get_bbox_ctr(), START_MINES)

        if DO_ROBOT:
            # don't want to illustrate before giving to mine algorithm
            Nina.illustrate(img)
            # Nina.set_target_pos(np.array([2.0, 2.0]))
            # Nina.control_to_target()

        illustrate(img, mine_data)
        mpl_show(img)
