"""A* path planning on a 2D occupancy grid (0 = free, 1 = obstacle)."""
import heapq
import math

SQRT2 = math.sqrt(2)
MOVES_4 = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)]
MOVES_8 = MOVES_4 + [(1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]


def heuristic(a, b, diagonal=True):
    """Admissible distance estimate (octile for 8-way, Manhattan for 4-way)."""
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    if diagonal:
        return (dx + dy) + (SQRT2 - 2) * min(dx, dy)
    return dx + dy


def astar(grid, start, goal, allow_diagonal=True):
    """Return the shortest path as a list of (x, y) cells, or None if unreachable."""
    height, width = grid.shape

    def blocked(x, y):
        return not (0 <= x < width and 0 <= y < height) or grid[y, x] != 0

    if blocked(*start) or blocked(*goal):
        return None

    moves = MOVES_8 if allow_diagonal else MOVES_4
    counter = 0  # tie-breaker so the heap never compares nodes
    open_heap = [(heuristic(start, goal, allow_diagonal), 0.0, counter, start)]
    came_from = {}
    g_score = {start: 0.0}
    closed = set()

    while open_heap:
        _, g, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]
        closed.add(current)

        for dx, dy, cost in moves:
            nx, ny = current[0] + dx, current[1] + dy
            if blocked(nx, ny):
                continue
            # forbid cutting corners diagonally between two obstacles
            if dx and dy and (blocked(current[0] + dx, current[1]) or blocked(current[0], current[1] + dy)):
                continue
            new_g = g + cost
            nb = (nx, ny)
            if new_g < g_score.get(nb, float("inf")):
                g_score[nb] = new_g
                came_from[nb] = current
                counter += 1
                heapq.heappush(open_heap, (new_g + heuristic(nb, goal, allow_diagonal), new_g, counter, nb))
    return None
