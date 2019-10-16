from global_stuff import *

import mines
from arena import *

F_NAME = "pics/arena3-1.jpg"


def do_stuff(frame):
    # crop to arena boundaries
    # missing ~30cm off top and bottom
    # frame = frame[Y:Y+H,X:X+W]

    # mine_details = mines.find_mines(frame)
    
    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)

    # for j, (centre, radius) in enumerate(mine_details):
    #     cv2.circle(frame, centre, radius, (0,255,0), 2)

    cv2.rectangle(frame, tuple(ARENA_TOP_LEFT), tuple(BOTTOM_RIGHT), (0, 0, 255), 1)

    # Display the resulting frame
    if USE_VIDEO:
        cv2.imshow("frame", frame)
        # cv2.waitKey(0)
    else:
        plt.axis("off")
        plt.imshow(frame,'gray')
        plt.show()
 



mine_pos_lst = []

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

    do_stuff(grey)
