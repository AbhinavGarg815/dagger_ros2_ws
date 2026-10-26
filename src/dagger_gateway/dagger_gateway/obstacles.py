"""Live obstacles round the rover for the app (owner, 2026-10-26): what it plans round now.

Nav2's global costmap (odom, 0.1 m, 40 m rolling, sent whole at 1 Hz) holds the LiDAR's
obstacles; the app draws the cells marked lethal (100: something there -- not the inflated
margin round it) within RANGE_M of the rover, merged to CELL_M squares, about once a second.
Cells go out as whole steps from an anchor, the rover's own place, in the frame the app draws the
rover in: lat/lon, or metres on the picture of a map that is not placed.
"""
import math

RANGE_M = 10.0          # owner's choice, 2026-10-26
CELL_M = 0.2            # 2 x 2 costmap cells
MAX_CELLS = 3000        # nearest first beyond this (a dense scene stays a small message)
LETHAL = 100            # nav2_costmap_2d: 254 LETHAL_OBSTACLE -> 100 in the OccupancyGrid


def occupied_near(info, data, here, range_m=RANGE_M, cell_m=CELL_M, max_cells=MAX_CELLS):
    """Centres (x, y) in the grid's frame of the cell_m squares holding a lethal costmap cell
    within range_m of `here` (x, y): `info` is the grid's MapMetaData (origin not rotated, as a
    rolling costmap's is), `data` its row-major values. Nearest first, at most max_cells."""
    res, w, h = info.resolution, info.width, info.height
    ox, oy = info.origin.position.x, info.origin.position.y
    c0 = max(0, int((here[0] - range_m - ox) / res))
    c1 = min(w - 1, int((here[0] + range_m - ox) / res))
    r0 = max(0, int((here[1] - range_m - oy) / res))
    r1 = min(h - 1, int((here[1] + range_m - oy) / res))
    squares = set()
    for r in range(r0, r1 + 1):
        row = r * w
        y = oy + (r + 0.5) * res
        for c in range(c0, c1 + 1):
            if data[row + c] >= LETHAL:
                x = ox + (c + 0.5) * res
                if math.hypot(x - here[0], y - here[1]) <= range_m:
                    squares.add((math.floor(x / cell_m), math.floor(y / cell_m)))
    centres = [((i + 0.5) * cell_m, (j + 0.5) * cell_m) for i, j in squares]
    centres.sort(key=lambda p: math.hypot(p[0] - here[0], p[1] - here[1]))
    return centres[:max_cells]


def as_cells(anchor, points, cell_m=CELL_M):
    """(i, j) whole cell_m steps of each point from the anchor, both (x, y) metres in the app's
    frame (east/north, or the picture's x/y); duplicates merged, the list flattened."""
    out, seen = [], set()
    for x, y in points:
        k = (round((x - anchor[0]) / cell_m), round((y - anchor[1]) / cell_m))
        if k not in seen:
            seen.add(k)
            out.extend(k)
    return out
