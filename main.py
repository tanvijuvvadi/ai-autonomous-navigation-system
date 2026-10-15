"""Entry point: python main.py --dynamic 3"""
import argparse
import json
import logging
import os

import cv2

from src.environment import create_solvable_world
from src.navigation import NavigationAgent
from src.perception import SensorSuite
from src.visualization import Visualizer, save_gif

log = logging.getLogger("nav")


def parse_args():
    p = argparse.ArgumentParser(description="AI-Based Autonomous Navigation System (simulation)")
    p.add_argument("--width", type=int, default=40)
    p.add_argument("--height", type=int, default=30)
    p.add_argument("--density", type=float, default=0.18, help="obstacle density (0-0.35)")
    p.add_argument("--dynamic", type=int, default=0, help="number of moving obstacles")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--view-radius", type=int, default=5, help="camera window radius in cells")
    p.add_argument("--max-steps", type=int, default=600)
    p.add_argument("--no-diagonal", action="store_true", help="4-way movement instead of 8-way")
    p.add_argument("--live", action="store_true", help="show a live OpenCV window while rendering")
    p.add_argument("--output-dir", default="outputs")
    return p.parse_args()


def main():
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(message)s", datefmt="%H:%M:%S")
    os.makedirs(args.output_dir, exist_ok=True)

    world = create_solvable_world(args.width, args.height, args.density, args.dynamic, args.seed)
    log.info("World %dx%d | seed=%d | moving obstacles=%d | start=%s goal=%s",
             world.width, world.height, world.seed, len(world.dynamic), world.start, world.goal)

    sensors = SensorSuite(view_radius=args.view_radius)
    agent = NavigationAgent(world, sensors, allow_diagonal=not args.no_diagonal)
    log.info("Navigation started...")
    result = agent.run(max_steps=args.max_steps)

    for k, v in result.items():
        log.info("%-26s %s", k, v)
    with open(os.path.join(args.output_dir, "metrics.json"), "w") as f:
        json.dump(result, f, indent=2)

    # Render the dashboard (at most ~120 frames, always include the last)
    frames = agent.frames
    stride = max(1, len(frames) // 120)
    idx = sorted(set(list(range(0, len(frames), stride)) + [len(frames) - 1]))
    vis = Visualizer(world, args.view_radius)
    rgb = []
    for i in idx:
        img = vis.render(frames[i])
        rgb.append(img)
        if args.live:
            cv2.imshow("Autonomous Navigation (press q to stop)", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            if cv2.waitKey(30) & 0xFF == ord("q"):
                break
    if args.live:
        cv2.destroyAllWindows()

    vis.save_png(frames[0], os.path.join(args.output_dir, "initial_plan.png"))
    vis.save_png(frames[-1], os.path.join(args.output_dir, "final_frame.png"))
    save_gif(rgb, os.path.join(args.output_dir, "navigation.gif"))
    log.info("Saved: metrics.json, initial_plan.png, final_frame.png, navigation.gif in '%s/'", args.output_dir)


if __name__ == "__main__":
    main()
