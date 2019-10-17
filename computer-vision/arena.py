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
ARENA_TOP_LEFT_IDX = prop_to_idx([0.047, 0.0])
ARENA_BOT_RIGHT_IDX = prop_to_idx([0.96, 0.999])
ARENA_IDX_H, ARENA_IDX_W = ARENA_BOT_RIGHT_IDX - ARENA_TOP_LEFT_IDX

GRID_TOP_LEFT_IDX = prop_to_idx([86/1600, 38/1200])
GRID_BOT_RIGHT_IDX = prop_to_idx([1542/1600, 1128/1200])
GRID_TOP_LEFT_COORD = np.array([0.0, 0.3])
GRID_BOT_RIGHT_COORD = np.array([2.4, 2.1])

IDX_TO_COORD_SF = (GRID_BOT_RIGHT_COORD - GRID_TOP_LEFT_COORD) / (GRID_BOT_RIGHT_IDX - GRID_TOP_LEFT_IDX)


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
        return tuple(self.idx)
    


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



if __name__ == "__main__":
    
    x = Point(idx=(200, 200))
    y = Point(pos=(1.0, 2.0))

    print(y.pos)
    print(y.idx)

    y.idx = (200, 200)
    x += Point(pos=(0.4, 0.4))
    print(x, y.pos)
    print((y*2 -x).cv_tup)