# program entry point - contains main loop and interacts with arena.py, robot.py, arena.py
# NOTE - this code is for Python 3.6 onwards (there are f-strings)
from global_stuff import *

from robot import Nina
from arena import *
from mines import get_mine_coords

if USE_VIDEO:
    import udpComms as udpc

pt = Point

# image shower function which works whether using video or not
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


    # Display the resulting frame
    return frame
 

# on startup
general_vid_params = {
    "h_res" : {"idx" : 3, "setval" : RESOLUTION[0]},
    "v_res" : {"idx" : 4, "setval" : RESOLUTION[1]},
    "fps" : {"idx" : 5, "setval" : 10},
    "brightness" : {"idx" : 11, "setval" : 32.0}
}

# increased exposure
mine_vid_params = {
    # this sets resolution lower
    "h_res" : {"idx" : 3, "setval" : RESOLUTION[0]},
    "v_res" : {"idx" : 4, "setval" : RESOLUTION[1]},
    "saturation" : {"idx" : 12, "setval" : 28.0},
    "exposure" : {"idx" : 15, "setval" : -7.0} # -8.0 is max exposure
}

# reduced exposure
robot_vid_params = {
    "h_res" : {"idx" : 3, "setval" : RESOLUTION[0]},
    "v_res" : {"idx" : 4, "setval" : RESOLUTION[1]},
    "saturation" : {"idx" : 12, "setval" : 28.0},
    "exposure" : {"idx" : 15, "setval" : -4.0}
}

# change video params back to what they were before we connected
default_vid_params = {
    "saturation" : {"idx" : 12, "setval" : 28.0},
    "exposure" : {"idx" : 15, "setval" : -7.0}
}


# utility function together with above parameter sets
# to interact with the camera
# (opencv interface isn't particularly friendly)
def set_params(cap, params, verbose=False):

    for key, data in params.items():
        if verbose: print(f"{key}: {cap.get(data['idx'])}, set to {data['setval']}")
        ret = cap.set(data["idx"], data["setval"])
        if (not ret) and verbose:
            print("failed")

    # exposure adjustment takes time
    if "exposure" in params:
        time.sleep(1) 


# obtain frame from camera, masked with ARENA_MASK
def get_frame(cap):
    ret, frame = cap.read()
    return cv2.bitwise_and(frame, cv2.cvtColor(ARENA_MASK, cv2.COLOR_GRAY2RGB))


# program entry point
if __name__ == "__main__":

    # this is used during actual competition
    if USE_VIDEO:

        print("setting up camera... (takes a while!)")
        start_setup = time.time()
        print("initialising comms...")
        cap = cv2.VideoCapture(0)
        print("initialising comms done.")
        set_params(cap, general_vid_params)
        set_params(cap, robot_vid_params)

        # to save a video file of the computer vision - not much overhead here
        if RECORD:
            out_vid = cv2.VideoWriter("recording.avi", 
                                    cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'),
                                    10, tuple(RESOLUTION))

        end_setup = time.time()

        print(f"done. setup took {end_setup - start_setup} seconds (jheez)")

        start_input = input("press enter to begin!")

        Nina.competition_start_t = time.time()

        mine_data = []
        i = 0

        # main loop
        while True:

            # switch to mine params and obtain mine positions when desired
            # (at start and after deposits until robot.CV_N_MINE_CUTOFF left)
            if Nina.wants_mine_data() and DO_MINES:
                
                print("getting mines...")

                set_params(cap, mine_vid_params)

                m_frame = get_frame(cap)
                cv2.imwrite("mine_mode.jpg", m_frame) # for debugging/single-frame use

                mine_data = get_mine_coords(m_frame, Nina.get_bbox_ctr(), Nina.n_mines_known)

                Nina.set_mine_locs([centre for centre, rad in mine_data])

                set_params(cap, robot_vid_params)
                

            r_frame = get_frame(cap)
            cv2.imwrite("robot_mode.jpg", r_frame) # for debugging/single-frame use
        
            r_frame = cv2.resize(r_frame, tuple(RESOLUTION)) # just in case

            # locate Nina in the frame
            Nina.update_pos(r_frame)

            # receive data from Nina (2 Hz)
            Nina.recv_update()

            # update Nina internal state machine
            Nina.update_state()

            # draw for cv display
            Nina.illustrate(r_frame)
            
            # based on position, update command to Nina
            Nina.control_to_target()

            to_show = illustrate(r_frame, mine_data)

            if RECORD:
                out_vid.write(to_show)

            show_img(to_show)

            # keys to test functions of Nina
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                Nina.send_cmd("stop", True)
                break
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


        # clean up on exit
        set_params(cap, default_vid_params)

        print("releasing camera 'capture' object")
        cap.release()
        cv2.destroyAllWindows()

        if RECORD:
            out_vid.release()

        udpc.STOP_THREAD = True


    # run on single frame for testing
    else:
        print(f"running on single frame (fpath: {F_NAME})")

        img = cv2.imread(F_NAME)
        img = cv2.resize(img, tuple(RESOLUTION))

        if DO_ROBOT:
            Nina.update_pos(img)

        mine_data = []
        if DO_MINES:
            mine_data = get_mine_coords(img, Nina.get_bbox_ctr(), START_MINES)

        if DO_ROBOT:
            # don't want to illustrate before giving to mine algorithm - do it here
            Nina.illustrate(img)

        illustrate(img, mine_data)
        mpl_show(img)
