"""Global navigation block. Owner: Melvyn.

Find the path from the robot to the target that avoids the cars of the grid.

The search is A* on the grid of cells, with four neighbours and a small cost
for each change of direction, so that among the shortest paths the robot gets
the one with the fewest turns in place. Dijkstra and breadth-first search are
the same search with other settings (see _search and METHODS).
"""

import heapq

from src import common

# Neighbours are always tried in this order: right, up, left, down.
# With a fixed order, two equally short paths are always broken the same way.
_MOVES = ((0, 1), (-1, 0), (0, -1), (1, 0))

# Proposal for the team, see the README: let the robot start from a car cell,
# for example after it steered away from an obstacle. Off until agreed.
_ALLOW_START_ON_CAR = False


def plan(grid, pose, goal):
    """Find the shortest path from the robot to the target.

    grid  array of ROWS x COLS, 0 = free, 1 = car.
    pose  (x, y, theta) of the robot, in metres and radians.
    goal  (x, y) of the target, in metres.

    Gives back the path: a list of (x, y) waypoints in metres, in driving
    order.
        - The start is not in the list.
        - Waypoints are cell centres, except the last one: the exact goal.
        - An empty list [] means no path: the robot or the goal is outside the
          lot or on a car, or the goal cannot be reached.

    Use common.xy_to_cell and common.cell_to_xy to go between metres and cells.
    """
    if grid is None or pose is None or goal is None:
        return []

    start = common.xy_to_cell(pose[0], pose[1])
    end = common.xy_to_cell(goal[0], goal[1])
    if not (_inside(grid, start) and _inside(grid, end)):
        return []
    if grid[end] != 0:
        return []
    if grid[start] != 0 and not _ALLOW_START_ON_CAR:
        return []

    target = (float(goal[0]), float(goal[1]))
    if start == end:
        return [target]

    cost, heuristic, turns = METHODS[METHOD]
    cells, _ = _search(grid, start, end, cost, heuristic, turns)
    if not cells:
        return []
    return [common.cell_to_xy(row, col) for row, col in cells[1:-1]] + [target]


def _search(grid, start, end, cost, heuristic, turns=False):
    """Best-first search on the grid, from cell start to cell end.

    cost(grid, cell)        cost of driving into a cell.
    heuristic(cell, end)    estimate of the cost left; 0 everywhere gives
                            Dijkstra, and with cost 1 everywhere, breadth-first.
    turns                   if True, each change of direction costs _TURN_COST
                            more. The search then remembers the direction the
                            robot arrived from: a state is (cell, direction).

    Gives back (cells, explored):
        cells     the cells of the path, start and end included; [] if none.
        explored  the set of cells the search expanded.
    """
    first = (start, None)              # no direction yet at the start
    order = 0                          # breaks ties first-in, first-out
    todo = [(heuristic(start, end), order, first)]
    cost_so_far = {first: 0}
    came_from = {first: None}
    done = set()
    explored = set()

    while todo:
        _, _, state = heapq.heappop(todo)
        if state in done:
            continue
        cell, direction = state
        if cell == end:
            return [s[0] for s in _walk_back(came_from, state)], explored
        done.add(state)
        explored.add(cell)

        for nxt in _neighbours(grid, cell):
            move = (nxt[0] - cell[0], nxt[1] - cell[1])
            step = cost(grid, nxt)
            if turns and direction is not None and move != direction:
                step += _TURN_COST
            new_state = (nxt, move if turns else None)
            new_cost = cost_so_far[state] + step
            if new_state not in cost_so_far or new_cost < cost_so_far[new_state]:
                cost_so_far[new_state] = new_cost
                came_from[new_state] = state
                order += 1
                heapq.heappush(todo, (new_cost + heuristic(nxt, end), order, new_state))

    return [], explored


def _neighbours(grid, cell):
    """The free cells next to cell, inside the lot, in the order of _MOVES."""
    row, col = cell
    result = []
    for d_row, d_col in _MOVES:
        nxt = (row + d_row, col + d_col)
        if _inside(grid, nxt) and grid[nxt] == 0:
            result.append(nxt)
    return result


def _inside(grid, cell):
    """True if cell is on the grid. Checked before indexing: in NumPy a
    negative index wraps around to the other side without any error."""
    row, col = cell
    return 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]


def _walk_back(came_from, last):
    """Rebuild the list of states from the start to last."""
    states = []
    state = last
    while state is not None:
        states.append(state)
        state = came_from[state]
    return states[::-1]


# --- Settings of the search ---------------------------------------------------

_NEAR_CAR_COST = 2     # extra cost to drive into a cell that touches a car
_TURN_COST = 0.5       # extra cost of one change of direction, in moves


def _cost_one(grid, cell):
    """Every move costs the same."""
    return 1


def _cost_near_cars(grid, cell):
    """A move costs more when the cell touches a car, sides or corners
    included, so the path keeps away from the cars when it can."""
    row, col = cell
    around = grid[max(row - 1, 0):row + 2, max(col - 1, 0):col + 2]
    return 1 + (_NEAR_CAR_COST if around.any() else 0)


def _h_zero(cell, end):
    """No estimate: the search becomes Dijkstra (breadth-first if every
    move costs the same)."""
    return 0


def _h_manhattan(cell, end):
    """Number of moves left if there were no cars: never too high, so A*
    still finds the shortest path."""
    return abs(cell[0] - end[0]) + abs(cell[1] - end[1])


# The four variants compared in the benchmark: (cost, heuristic, turns).
METHODS = {
    "bfs":           (_cost_one,       _h_zero,      False),
    "dijkstra_cars": (_cost_near_cars, _h_zero,      False),
    "astar":         (_cost_one,       _h_manhattan, False),
    "astar_turns":   (_cost_one,       _h_manhattan, True),
}

# Variant used by plan(). Chosen with the benchmark (155 test lots): same
# length as the shortest path, about a quarter fewer turns than plain A*, and
# half the cells explored by breadth-first search.
METHOD = "astar_turns"
