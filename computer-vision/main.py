from global_stuff import *

import mines
from robot import robot
import udpComms
from arena import *
pt = Point



def do_mine_stuff(frame):

    frame_grey = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    x1, y1 = MINE_AREA_TOP_LEFT.cv_tup
    x2, y2 = MINE_AREA_BOT_RIGHT.cv_tup
    mine_area_img = frame_grey[y1:y2, x1:x2].copy()
    mine_area_mask = ARENA_MASK[y1:y2, x1:x2].copy()
    # add robot to mask when it's detected
    # mask required to eliminate extreme gradients
    
    mine_details, img2 = mines.find_mines(mine_area_img, mine_area_mask, x1, y1)
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
    "saturation" : {"idx" : 12, "setval" : 28.0},
    "exposure" : {"idx" : 15, "setval" : -7.0} # -8.0 is max exposure
}

robot_vid_params = {
    "saturation" : {"idx" : 12, "setval" : 28.0},
    "exposure" : {"idx" : 15, "setval" : -4.0} # this sets resolution lower
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

    time.sleep(0.5)


def get_frame(cap):
    ret, frame = cap.read()
    return cv2.bitwise_and(frame, cv2.cvtColor(ARENA_MASK, cv2.COLOR_GRAY2RGB))


if __name__ == "__main__":

    if USE_VIDEO:

        # udpComms.setup()

        print("setting up camera... (takes a while!)")
        start_setup = time.time()
        print("initialising comms...")
        cap = cv2.VideoCapture(0)
        print("initialising comms done.")
        set_params(cap, general_vid_params)
        set_params(cap, robot_vid_params)

        end_setup = time.time()

        print(f"done. setup took {end_setup - start_setup} seconds (jheez)")

        mines_every = 150
        i = -2
        mine_data = []
        while True:
            i += 1

            if (i == mines_every or i == -1) and DO_MINES:
                i = 0

                set_params(cap, mine_vid_params)

                m_frame = get_frame(cap)
                cv2.imwrite("mine_mode.jpg", m_frame)
                # show_img(m_frame)


                mine_data = do_mine_stuff(m_frame)

                m_frame = illustrate(m_frame, mine_data)

                # time.sleep(0.5)


                set_params(cap, robot_vid_params)

            elif DO_ROBOT:
                r_frame = get_frame(cap)

                cv2.imwrite("robot_mode.jpg", r_frame)

                # r_frame = cv2.resize(r_frame, tuple(RESOLUTION))
                

                robot.update_pos(r_frame)
                
                robot.draw_coord_sys(r_frame)

                robot.set_target(np.array([1.0, 1.0]), np.array([1.0, 0.0]))

                robot.do_control()

                to_show = illustrate(r_frame, mine_data)

                show_img(to_show)

                # time.sleep(2)


            if cv2.waitKey(1) & 0xFF == ord('q'):
                    break


        set_params(cap, default_vid_params)
        # When everything done, release the capture
        cap.release()
        cv2.destroyAllWindows()


    else:
        print(f"running on single frame (fpath: {F_NAME})")

        img = cv2.imread(F_NAME)
        img = cv2.resize(img, tuple(RESOLUTION))

        if DO_ROBOT:
            robot.update_pos(img)
            robot.draw_coord_sys(img)

        mine_data = []
        if DO_MINES:
            mine_data = do_mine_stuff(img)

        illustrate(img, mine_data)
        mpl_show(img)
