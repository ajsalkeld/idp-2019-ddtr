from global_stuff import *

def prop_to_idx(in_coords):
    return (np.array(in_coords) * RESOLUTION).astype(int)

# x, y
ARENA_TOP_LEFT = prop_to_idx([0.047, 0.0])
BOTTOM_RIGHT = prop_to_idx([0.96, 0.999])
H, W = BOTTOM_RIGHT - ARENA_TOP_LEFT
Y, X = ARENA_TOP_LEFT