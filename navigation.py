"""Navigation agent: sense -> update belief -> (re)plan -> move, with metrics."""
import math
import time

import numpy as np

from .planner import astar


def path_length(path):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


class NavigationAgent:
    def __init__(self, world, sensors, allow_diagonal=True, patience=15):
        self.world = world
        self.sensors = sensors
        self.allow_diagonal = allow_diagonal
        self.patience = patience                      # how long to wait when no path exists
        self.belief = np.zeros_like(world.grid)       # starts as "everything is free" (optimistic)
        self.pos = world.start
        self.trail = [self.pos]
        self.path = None
        self.replans = 0
        self.emergency_stops = 0
        self.frames = []                              # per-step snapshots for visualization

    def _update_belief(self, obs):
        """Overwrite what we can currently see (this also lets us forget moved obstacles)."""
        for (x, y) in obs.visible_cells:
            self.belief[y, x] = 1 if (x, y) in obs.obstacle_cells else 0

    @staticmethod
    def _corner_cut(a, b, blocked):
        """True if a diagonal move a->b squeezes between two blocked cells."""
        dx, dy = b[0] - a[0], b[1] - a[1]
        return bool(dx and dy and (blocked(a[0] + dx, a[1]) or blocked(a[0], a[1] + dy)))

    def _path_blocked(self):
        """Path is invalid if a cell became blocked OR a diagonal now cuts a corner."""
        bel = lambda x, y: bool(self.belief[y, x])
        if any(self.belief[y, x] for x, y in self.path[1:]):
            return True
        return any(self._corner_cut(a, b, bel) for a, b in zip(self.path, self.path[1:]))

    def _record(self, step, obs, ranges, replanned):
        self.frames.append({
            "step": step, "pos": self.pos,
            "path": list(self.path) if self.path else [],
            "trail": list(self.trail),
            "belief": self.belief.copy(),
            "dynamic": [tuple(d) for d in self.world.dynamic],
            "boxes": obs.boxes,
            "min_range": float(ranges.min()),
            "replanned": replanned, "replans": self.replans,
        })

    def run(self, max_steps=600):
        t0 = time.perf_counter()
        status, stalled, first_plan_done = "max_steps_reached", 0, False

        for step in range(max_steps):
            if self.pos == self.world.goal:
                break
            obs = self.sensors.perceive(self.world, self.pos)           # PERCEIVE
            self._update_belief(obs)
            _, ranges = self.sensors.lidar_scan(self.world, self.pos)

            replanned = False
            if self.path is None or self._path_blocked():               # DECIDE
                self.path = astar(self.belief, self.pos, self.world.goal, self.allow_diagonal)
                replanned = True
                if first_plan_done:
                    self.replans += 1
                first_plan_done = True

            if self.path is None:                                       # no route right now: wait
                stalled += 1
                self._record(step, obs, ranges, replanned)
                if stalled > self.patience:
                    status = "no_path"
                    break
                self.world.step_dynamic(self.pos)
                continue
            stalled = 0
            self._record(step, obs, ranges, replanned)

            nxt = self.path[1]                                          # ACT
            if (self.world.is_blocked(*nxt)                             # safety net: never drive into a hazard
                    or self._corner_cut(self.pos, nxt, self.world.is_blocked)):
                self.emergency_stops += 1
                self.path = None
                self.world.step_dynamic(self.pos)
                continue
            self.path = self.path[1:]
            self.pos = nxt
            self.trail.append(nxt)
            self.world.step_dynamic(self.pos)

        if self.pos == self.world.goal:
            status = "success"
        return self._metrics(status, time.perf_counter() - t0)

    def _metrics(self, status, runtime):
        optimal = astar(self.world.grid, self.world.start, self.world.goal, self.allow_diagonal)
        actual = path_length(self.trail)
        opt_len = path_length(optimal) if optimal else float("nan")
        return {
            "status": status,
            "success": status == "success",
            "steps": len(self.trail) - 1,
            "path_length": round(actual, 2),
            "optimal_length_static_map": round(opt_len, 2),
            "path_efficiency": round(opt_len / actual, 3) if actual > 0 else None,
            "replans": self.replans,
            "emergency_stops": self.emergency_stops,
            "collisions": 0,   # the safety check above prevents entering blocked cells
            "obstacles_discovered": int(self.belief.sum()),
            "runtime_seconds": round(runtime, 3),
        }
