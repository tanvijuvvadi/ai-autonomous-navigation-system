import numpy as np
from src.planner import astar


def test_straight_line():
    grid = np.zeros((5, 5), dtype=np.uint8)
    path = astar(grid, (0, 0), (4, 0), allow_diagonal=False)
    assert path[0] == (0, 0) and path[-1] == (4, 0) and len(path) == 5


def test_unreachable_goal_returns_none():
    grid = np.zeros((5, 5), dtype=np.uint8)
    grid[:, 2] = 1  # full wall
    assert astar(grid, (0, 0), (4, 4)) is None


def test_path_avoids_obstacles():
    grid = np.zeros((5, 5), dtype=np.uint8)
    grid[0:4, 2] = 1  # wall with a gap at the bottom
    path = astar(grid, (0, 0), (4, 0))
    assert path is not None
    assert all(grid[y, x] == 0 for x, y in path)
