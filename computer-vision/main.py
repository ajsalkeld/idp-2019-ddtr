from global_stuff import *

import mines
from arena import *
pt = Point

F_NAME = "pics/arena3-1.jpg"


def do_stuff(frame):

    start_t = time.time()

    frame = cv2.bitwise_and(frame, ARENA_MASK)

    x1, y1 = MINE_AREA_TOP_LEFT.cv_tup
    x2, y2 = MINE_AREA_BOT_RIGHT.cv_tup

    mine_area_img = frame[y1:y2, x1:x2].copy()
    mine_area_mask = ARENA_MASK[y1:y2, x1:x2].copy()
    # add robot to mask when it's detected
    # mask required to eliminate extreme gradients
    
    mine_details = mines.find_mines(mine_area_img, mine_area_mask, x1, y1)
    
    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)

    print("mine locations & rads: (m)")
    for j, (centre, radius) in enumerate(mine_details):
        if radius < 15 * IDX_SCALE:
            cv2_cross(frame, centre.cv_tup, 2, (0,100,255),1)
            print(f"pos: {centre.pos}")

    draw_arena_features(frame)

    end_t = time.time()

    print(f"identification took {end_t-start_t}s.")

    # Display the resulting frame
    if USE_VIDEO:
        cv2.imshow("frame", frame)
        # cv2.waitKey(0)
    else:
        mpl_show(frame)
 


if USE_VIDEO:
    print("setting up camera... (takes a while!)")
    start_setup = time.time()
    print("initialising comms...")
    cap = cv2.VideoCapture(0)
    print("initialising comms done.")
    print("setting resolution...")
    # property enums: h = 3, w = 4, fps = 5
    ret = cap.set(3, RESOLUTION[0])
    ret &= cap.set(4, RESOLUTION[1])
    ret &= cap.set(5, 10)
    if ret:
        print("successfully set resolution.")
    else:
        print("unsuccessful.")
        quit()

    end_setup = time.time()

    print(f"done. setup took {end_setup - start_setup} seconds (jheez)")

    i = -1
    while True:
        i += 1

        # Capture frame-by-frame
        ret, frame = cap.read()

        # Our operations on the frame come here
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if i == 1:
            i = 0
            do_stuff(grey)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


else:
    print(f"running on single frame (fpath: {F_NAME})")

    grey = cv2.imread(F_NAME, cv2.IMREAD_GRAYSCALE)

    do_stuff(cv2.resize(grey, tuple(RESOLUTION)))
