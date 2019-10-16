from global_stuff import *

def prop_to_idx(in_coords):
    return (np.array(in_coords) * RESOLUTION).astype(int)

def idx_to_prop(in_idx):
    return (np.array(in_idx) / RESOLUTION)

# 'idx' is pixel coordinate in full image
# 'prop' is idx/resolution
# 'coord' (or no label) is physical distance from top left (m)

# x, y
ARENA_TOP_LEFT_IDX = prop_to_idx([0.047, 0.0])
ARENA_BOT_RIGHT_IDX = prop_to_idx([0.96, 0.999])
ARENA_IDX_H, ARENA_IDX_W = ARENA_BOT_RIGHT_IDX - ARENA_TOP_LEFT_IDX

GRID_TOP_LEFT_IDX = np.array([86, 38])
GRID_BOT_RIGHT_IDX = np.array([1542, 1128])
GRID_TOP_LEFT_COORD = np.array([0.0, 0.3])
GRID_BOT_RIGHT_COORD = np.array([2.4, 2.1])

IDX_TO_COORD_SF = (GRID_BOT_RIGHT_COORD - GRID_TOP_LEFT_COORD) / (GRID_BOT_RIGHT_IDX - GRID_TOP_LEFT_IDX)

def idx_to_pos(in_idx):
    idx = np.array(in_idx)
    return (idx - GRID_TOP_LEFT_IDX) * IDX_TO_COORD_SF + GRID_TOP_LEFT_COORD

print(idx_to_pos(0.9*ARENA_BOT_RIGHT_IDX))




    
