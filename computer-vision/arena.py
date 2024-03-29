# definition of arena coordinates and conversions, masks, and Point class
from global_stuff import *


    
#-------------------------------------------------
# idx/prop conversions - defined here so they can be
# used in initial arena calculations below
#-------------------------------------------------
def prop_to_idx(in_coords):
    return (np.array(in_coords) * RESOLUTION).astype(int)

def idx_to_prop(in_idx):
    return (np.array(in_idx) / RESOLUTION)


#-------------------------------------------------
# initial calibration
#-------------------------------------------------

# x, y
GRID_TOP_LEFT_IDX = prop_to_idx([32/640, 10/480])
GRID_BOT_RIGHT_IDX = prop_to_idx([543/640, 450/480])
GRID_TOP_LEFT_COORD = np.array([0.0, 0.3])
GRID_BOT_RIGHT_COORD = np.array([2.1, 2.1])

IDX_TO_COORD_SF = (GRID_BOT_RIGHT_COORD - GRID_TOP_LEFT_COORD) / (GRID_BOT_RIGHT_IDX - GRID_TOP_LEFT_IDX)


# used for simple conversions between image index-space and real positions
class Point():

    # 'idx' is pixel coordinate in full image
    # 'prop' is idx/resolution
    # 'coord' (or no label) is physical distance from top left (m)


    #-------------------------------------------------
    # conversions - static methods now
    #-------------------------------------------------
    
    prop_to_idx = staticmethod(prop_to_idx)
    idx_to_prop = staticmethod(idx_to_prop)

    @staticmethod
    def idx_to_pos(in_idx):
        return (np.array(in_idx) - GRID_TOP_LEFT_IDX) * IDX_TO_COORD_SF + GRID_TOP_LEFT_COORD

    @staticmethod
    def pos_to_idx(in_pos):
        return (in_pos - GRID_TOP_LEFT_COORD) / IDX_TO_COORD_SF + GRID_TOP_LEFT_IDX



    #-------------------------------------------------
    # initialiser
    #-------------------------------------------------
    def __init__(self, pos=None, idx=None, prop=None):
        
        if pos is not None:
            self.pos = pos
    
        elif idx is not None:
            self.idx = idx

        elif prop is not None:
            self.prop = prop

        else:
            raise TypeError("Please use one of pos=, idx=, prop=")

    #-------------------------------------------------
    # getters & setters
    # Internally, only self._pos is stored
    # Accessing other members such as self.idx and
    # self.prop invokes these getters and setters
    # this ensures consistency
    #-------------------------------------------------
    @property
    def pos(self):
        return self._pos
    
    @pos.setter
    def pos(self, val):
        self._pos = np.array(val)
    
    @property
    def idx(self):
        return (self.pos_to_idx(self._pos) + 0.5).astype(int)
    
    @idx.setter
    def idx(self, val):
        self._pos = self.idx_to_pos(val)
    
    @property
    def prop(self):
        return self.idx_to_prop(self.pos_to_idx(self._pos))
    
    @prop.setter
    def prop(self, val):
        self._pos = self.idx_to_pos(self.prop_to_idx(val))
    
    @property
    def cv_tup(self):
        return tuple(
            [np.clip(self.idx[0], 0, RESOLUTION[0]-1), 
            np.clip(self.idx[1], 0, RESOLUTION[1]-1)]
        )
    


    #-------------------------------------------------
    # operator overrides
    #-------------------------------------------------
    def __add__(self, other):
        return Point(pos=self.pos + other.pos)
    
    def __sub__(self, other):
        return Point(pos=self.pos - other.pos)

    def __mul__(self, other):

        if type(other) in (int, float):
            return Point(pos=self.pos * other)

        else:
            raise TypeError("invalid type for multiplication")
    
    def __truediv__(self, other):

        if type(other) in (int, float):
            return Point(pos=self.pos / other)

        else:
            raise TypeError("invalid type for division")

    def __repr__(self):
        return f"pos: {self.pos}, idx: {self.idx}, prop: {self.prop}"


    def __iter__(self):
        self.n = -1
        return self

    def __next__(self):
        if self.n < 1:
            self.n += 1
            return self._pos[self.n]
        else:
            raise StopIteration


# sanity check
if __name__ == "__main__":
    
    x = Point(idx=(200, 200))
    y = Point(pos=(1.0, 2.0))

    print(y.pos)
    print(y.idx)

    y.idx = (200, 200)
    x += Point(pos=(0.4, 0.4))
    print(x, y.pos)
    print((y*2 -x).cv_tup)


# this masks out the red 'live mine' bin
# so that Nina is the only red thing on the arena
robot_mask_name = "pics/calib/robot_mask.jpg"
ROBOT_MASK = cv2.imread(robot_mask_name)
ROBOT_MASK = cv2.resize(ROBOT_MASK, tuple(RESOLUTION))

# this masks out the off-table regions of the image
arena_mask_name = "pics/calib/arena3mask.jpg"
ARENA_MASK = cv2.imread(arena_mask_name, cv2.IMREAD_GRAYSCALE)
ARENA_MASK = cv2.resize(ARENA_MASK, tuple(RESOLUTION))


TAPE_THICKNESS = 0.018

ARENA_TOP_LEFT = Point(pos=(0.0, 0.0))
ARENA_BOT_RIGHT = Point(pos=(2.4, 2.4))
ARENA_IDX_H, ARENA_IDX_W = ARENA_BOT_RIGHT - ARENA_TOP_LEFT

MINE_AREA_TOP_LEFT = Point(pos=(0.3, 0.3))
MINE_AREA_BOT_RIGHT = Point(pos=(2.4-0.6, 2.1))


MINE_AREA_DILATION = Point(pos=(0.04, 0.04))

MINE_AREA_TOP_LEFT -= MINE_AREA_DILATION
MINE_AREA_BOT_RIGHT += MINE_AREA_DILATION


LIVE_DEPOSIT_TOP_LEFT = Point(pos=(2.4-0.29, 2.4-0.3))
LIVE_DEPOSIT_BOT_RIGHT = Point(pos=(2.4-TAPE_THICKNESS/2, 2.4-TAPE_THICKNESS/2))

DEAD_DEPOSIT_TOP_LEFT = Point(pos=(2.4-0.29, 2.4-0.9))
DEAD_DEPOSIT_BOT_RIGHT = Point(pos=(2.4-TAPE_THICKNESS/2, 2.4-0.6-TAPE_THICKNESS/2))

SAFE_LINE_X_POS = 2.4 - 0.5
LIVE_DEPOSIT_Y_POS = 2.4 - 0.525
DEAD_DEPOSIT_Y_POS = 2.4 - 0.75
MINE_BLIND_Y_THRESH = 0.2
SEARCH_START_Y_POS = 2.4 - 0.65


LHS_BOUND_LINE_TOP = Point(pos=(2.4-0.5, 0))
LHS_BOUND_LINE_BOT = Point(pos=(2.4-0.5, 2.4))

RHS_BOUND_LINE_TOP = Point(pos=(2.4-0.3, 0))
RHS_BOUND_LINE_BOT = Point(pos=(2.4-0.3, 2.4))


# illustrates the display frame with arena features
def draw_arena_features(img):

    # arena outline
    cv2.rectangle(img, ARENA_TOP_LEFT.cv_tup, ARENA_BOT_RIGHT.cv_tup, (0, 255, 255), 1)
    
    # mine area (loose bound - used for detection)
    cv2.rectangle(img, MINE_AREA_TOP_LEFT.cv_tup, MINE_AREA_BOT_RIGHT.cv_tup, (255, 0, 255), 1)
    
    # mine area (precise)
    cv2.rectangle(img, (MINE_AREA_TOP_LEFT + MINE_AREA_DILATION).cv_tup, 
                        (MINE_AREA_BOT_RIGHT - MINE_AREA_DILATION).cv_tup, (255, 100, 100), 1)

    # mine area - cutoff lines for close to blind regions
    cv2.line(img, Point(pos=(MINE_AREA_BOT_RIGHT.pos[0], MINE_AREA_BOT_RIGHT.pos[1] - MINE_BLIND_Y_THRESH)).cv_tup,
                    Point(pos=(MINE_AREA_TOP_LEFT.pos[0],  MINE_AREA_BOT_RIGHT.pos[1] - MINE_BLIND_Y_THRESH)).cv_tup, (255, 50, 50), 1)
    cv2.line(img, Point(pos=(MINE_AREA_BOT_RIGHT.pos[0],  MINE_AREA_TOP_LEFT.pos[1] + MINE_BLIND_Y_THRESH)).cv_tup,
                    Point(pos=(MINE_AREA_TOP_LEFT.pos[0], MINE_AREA_TOP_LEFT.pos[1] + MINE_BLIND_Y_THRESH)).cv_tup, (255, 50, 50), 1)
  
    # cv2.line(img, Point(pos=(MINE_AREA_BOT_RIGHT.pos[0], MINE_BLIND_BAND)).cv_tup,
                    # Point(pos=(MINE_AREA_TOP_LEFT.pos[0], MINE_BLIND_BAND)).cv_tup, (255, 50, 50), 1)


    # live mine deposit
    # cv2.rectangle(img, LIVE_DEPOSIT_TOP_LEFT.cv_tup, LIVE_DEPOSIT_BOT_RIGHT.cv_tup, (0, 0, 255), 2)
    cv2_cross(img, Point(pos=(SAFE_LINE_X_POS, LIVE_DEPOSIT_Y_POS)).idx, 3, (255, 255, 255), 1)
    cv2_cross(img, Point(pos=(SAFE_LINE_X_POS, DEAD_DEPOSIT_Y_POS)).idx, 3, (255, 255, 255), 1)

    # dead mine deposit
    # cv2.rectangle(img, DEAD_DEPOSIT_TOP_LEFT.cv_tup, DEAD_DEPOSIT_BOT_RIGHT.cv_tup, (0, 255, 0), 2)
 
    # # bounding lines
    # cv2.rectangle(img, LHS_BOUND_LINE_TOP.cv_tup, LHS_BOUND_LINE_BOT.cv_tup, (255, 0, 0), 1)
    # cv2.rectangle(img, RHS_BOUND_LINE_TOP.cv_tup, RHS_BOUND_LINE_BOT.cv_tup, (255, 0, 0), 1)
 
