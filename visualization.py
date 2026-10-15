"""Dashboard rendering: ground truth (left) vs. what the AI believes (right)."""
import matplotlib
matplotlib.use("Agg")  # render off-screen so it works on any machine
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle
from PIL import Image

TRUTH_CMAP = ListedColormap(["#f4f4f4", "#2b2b2b", "#e67e22"])  # free, wall, moving obstacle
BELIEF_CMAP = ListedColormap(["#f4f4f4", "#2b2b2b"])


class Visualizer:
    def __init__(self, world, view_radius):
        self.world = world
        self.r = view_radius
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 4.8), dpi=100)

    def render(self, rec):
        """Draw one recorded step and return it as an RGB array."""
        w = self.world
        for ax in (self.ax1, self.ax2):
            ax.clear()
            ax.set_xticks([])
            ax.set_yticks([])

        truth = w.grid.astype(float)
        for (x, y) in rec["dynamic"]:
            truth[y, x] = 2
        self.ax1.imshow(truth, cmap=TRUTH_CMAP, vmin=0, vmax=2)
        self.ax1.set_title("Simulated world (ground truth)")
        self.ax2.imshow(rec["belief"], cmap=BELIEF_CMAP, vmin=0, vmax=1)
        self.ax2.set_title("AI belief map + planned path")

        tx, ty = zip(*rec["trail"])
        self.ax1.plot(tx, ty, "-", color="#3498db", lw=2, label="trail")
        px, py = rec["pos"]
        self.ax1.add_patch(Rectangle((px - self.r - .5, py - self.r - .5), 2 * self.r + 1, 2 * self.r + 1,
                                     fill=False, edgecolor="cyan", lw=1.2))  # camera field of view
        for (bx, by, bw, bh) in rec["boxes"]:
            self.ax1.add_patch(Rectangle((bx - .5, by - .5), bw, bh, fill=False, edgecolor="red", lw=1))

        if rec["path"]:
            ex, ey = zip(*rec["path"])
            self.ax2.plot(ex, ey, "-", color="#2ecc71", lw=2)
        for ax in (self.ax1, self.ax2):
            ax.plot(*w.start, "s", color="green", ms=9)
            ax.plot(*w.goal, "*", color="gold", ms=15, mec="k")
            ax.plot(px, py, "o", color="#e74c3c", ms=8, mec="k")

        self.fig.suptitle(f"Step {rec['step']}   |   replans: {rec['replans']}   |   "
                          f"nearest obstacle: {rec['min_range']:.1f} cells"
                          + ("   |   REPLANNED" if rec["replanned"] else ""))
        self.fig.tight_layout()
        self.fig.canvas.draw()
        return np.asarray(self.fig.canvas.buffer_rgba())[:, :, :3].copy()

    def save_png(self, rec, path):
        self.render(rec)
        self.fig.savefig(path)


def save_gif(rgb_frames, path, fps=8):
    imgs = [Image.fromarray(f) for f in rgb_frames]
    imgs[0].save(path, save_all=True, append_images=imgs[1:], duration=int(1000 / fps), loop=0)
