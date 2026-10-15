"""Virtual 2D world: static obstacles, moving obstacles, start and goal."""
import numpy as np
from .planner import astar

MOVES = [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)]


class GridWorld:
    def __init__(self, width=40, height=30, density=0.18, n_dynamic=0, seed=42):
        self.width, self.height, self.seed = width, height, seed
        self.rng = np.random.default_rng(seed)
        self.start = (1, 1)
        self.goal = (width - 2, height - 2)
        self.grid = self._generate(density)
        self.dynamic = self._spawn_dynamic(n_dynamic)  # list of [x, y]

    def _generate(self, density):
        w, h = self.width, self.height
        grid = np.zeros((h, w), dtype=np.uint8)
        grid[0, :] = grid[-1, :] = 1
        grid[:, 0] = grid[:, -1] = 1
        for _ in range(int(density * w * h / 4)):
            bw, bh = int(self.rng.integers(1, 4)), int(self.rng.integers(1, 4))
            x = int(self.rng.integers(1, w - bw))
            y = int(self.rng.integers(1, h - bh))
            grid[y:y + bh, x:x + bw] = 1
        for (x, y) in (self.start, self.goal):  # keep a 3x3 area around start/goal free
            grid[max(1, y - 1):min(h - 1, y + 2), max(1, x - 1):min(w - 1, x + 2)] = 0
        return grid

    def _spawn_dynamic(self, n):
        cells, tries = [], 0
        while len(cells) < n and tries < 2000:
            tries += 1
            x = int(self.rng.integers(1, self.width - 1))
            y = int(self.rng.integers(1, self.height - 1))
            far_from_start = abs(x - self.start[0]) + abs(y - self.start[1]) > 6
            far_from_goal = abs(x - self.goal[0]) + abs(y - self.goal[1]) > 6
            if self.grid[y, x] == 0 and far_from_start and far_from_goal and [x, y] not in cells:
                cells.append([x, y])
        return cells

    def in_bounds(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def is_blocked(self, x, y):
        """True for out-of-bounds, static obstacles and moving obstacles."""
        if not self.in_bounds(x, y) or self.grid[y, x] == 1:
            return True
        return any(d[0] == x and d[1] == y for d in self.dynamic)

    def step_dynamic(self, agent_pos):
        """Random-walk each moving obstacle one cell (never onto walls, others or the agent)."""
        occupied = {tuple(d) for d in self.dynamic}
        for d in self.dynamic:
            dx, dy = MOVES[int(self.rng.integers(0, len(MOVES)))]
            nx, ny = d[0] + dx, d[1] + dy
            if (self.in_bounds(nx, ny) and self.grid[ny, nx] == 0
                    and (nx, ny) not in occupied and (nx, ny) != tuple(agent_pos)):
                occupied.discard((d[0], d[1]))
                d[0], d[1] = nx, ny
                occupied.add((nx, ny))


def create_solvable_world(width, height, density, n_dynamic, seed, max_tries=100):
    """Try consecutive seeds until the static map has a start->goal route."""
    for k in range(max_tries):
        world = GridWorld(width, height, density, n_dynamic, seed + k)
        if astar(world.grid, world.start, world.goal) is not None:
            return world
    raise RuntimeError("Could not generate a solvable map. Lower the obstacle density.")
