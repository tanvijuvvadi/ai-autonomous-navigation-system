"""Run many randomized scenarios and report aggregate metrics.

Usage: python benchmark.py --runs 30
Use these real numbers in your README and resume.
"""
import argparse
import json

import numpy as np

from src.environment import create_solvable_world
from src.navigation import NavigationAgent
from src.perception import SensorSuite

SCENARIOS = {
    "Static obstacles": dict(density=0.18, n_dynamic=0),
    "Dense map": dict(density=0.28, n_dynamic=0),
    "4 moving obstacles": dict(density=0.18, n_dynamic=4),
    "8 moving obstacles": dict(density=0.18, n_dynamic=8),
}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--runs", type=int, default=30)
    p.add_argument("--view-radius", type=int, default=5)
    args = p.parse_args()

    rows, raw = [], {}
    for name, cfg in SCENARIOS.items():
        res = []
        for seed in range(args.runs):
            world = create_solvable_world(40, 30, cfg["density"], cfg["n_dynamic"], 1000 + seed * 101)
            res.append(NavigationAgent(world, SensorSuite(view_radius=args.view_radius)).run())
        ok = [r for r in res if r["success"]]
        raw[name] = res
        rows.append((name,
                     100 * len(ok) / len(res),
                     np.mean([r["steps"] for r in ok]) if ok else float("nan"),
                     np.mean([r["replans"] for r in ok]) if ok else float("nan"),
                     np.mean([r["path_efficiency"] for r in ok]) if ok else float("nan"),
                     sum(r["collisions"] for r in res)))

    header = "| Scenario | Success rate | Avg steps | Avg replans | Avg path efficiency | Collisions |\n|---|---|---|---|---|---|"
    lines = [header] + [f"| {n} | {s:.0f}% ({args.runs} runs) | {st:.1f} | {rp:.1f} | {pe:.3f} | {c} |"
                        for n, s, st, rp, pe, c in rows]
    table = "\n".join(lines)
    print(table)
    with open("docs/benchmark_results.md", "w") as f:
        f.write(f"# Benchmark results ({args.runs} randomized maps per scenario)\n\n{table}\n")
    with open("docs/benchmark_raw.json", "w") as f:
        json.dump(raw, f, indent=1)


if __name__ == "__main__":
    main()
