"""Perception: simulated camera + OpenCV obstacle detection, plus a LiDAR-style range scan."""
from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class Observation:
    visible_cells: list = field(default_factory=list)   # in-bounds cells the sensor can see
    obstacle_cells: set = field(default_factory=set)    # cells detected as obstacles
    boxes: list = field(default_factory=list)           # bounding boxes (x, y, w, h) in grid cells
    image: np.ndarray = None                            # the processed camera mask


class SensorSuite:
    def __init__(self, view_radius=5, n_rays=16, max_range=6.0, pixels_per_cell=10):
        self.view_radius = view_radius
        self.n_rays = n_rays
        self.max_range = max_range
        self.scale = pixels_per_cell

    def perceive(self, world, pos):
        """Simulate a camera looking at the (2r+1)x(2r+1) window around the agent, then detect obstacles."""
        r, s = self.view_radius, self.scale
        size = 2 * r + 1
        ox, oy = pos[0] - r, pos[1] - r

        # 1) Simulated camera frame: obstacles are white, free space is black
        img = np.zeros((size * s, size * s), dtype=np.uint8)
        for j in range(size):
            for i in range(size):
                if world.is_blocked(ox + i, oy + j):
                    img[j * s:(j + 1) * s, i * s:(i + 1) * s] = 255

        # 2) Preprocessing: blur (noise reduction) + threshold
        img = cv2.GaussianBlur(img, (5, 5), 0)
        _, mask = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

        # 3) Detection: contours -> bounding boxes
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for c in contours:
            px, py, pw, ph = cv2.boundingRect(c)
            boxes.append((ox + px // s, oy + py // s, max(1, round(pw / s)), max(1, round(ph / s))))

        # 4) Convert the mask back to grid cells (sample the centre pixel of each cell)
        obs = Observation(image=mask, boxes=boxes)
        for j in range(size):
            for i in range(size):
                wx, wy = ox + i, oy + j
                if not world.in_bounds(wx, wy):
                    continue
                obs.visible_cells.append((wx, wy))
                if mask[j * s + s // 2, i * s + s // 2] > 0:
                    obs.obstacle_cells.add((wx, wy))
        return obs

    def lidar_scan(self, world, pos):
        """Cast rays around the agent; return (angles, distances in cells). Mirrors an ultrasonic/LiDAR sensor."""
        angles = np.linspace(0, 2 * np.pi, self.n_rays, endpoint=False)
        ranges = []
        for a in angles:
            d = 0.0
            while d < self.max_range:
                d += 0.25
                x = int(round(pos[0] + d * np.cos(a)))
                y = int(round(pos[1] + d * np.sin(a)))
                if world.is_blocked(x, y):
                    break
            ranges.append(min(d, self.max_range))
        return angles, np.array(ranges)
