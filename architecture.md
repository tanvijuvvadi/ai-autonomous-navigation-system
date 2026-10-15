# Architecture

```
┌──────────────── SIMULATED WORLD (environment.py) ────────────────┐
│ grid map + static obstacles + moving obstacles + start/goal       │
└───────┬──────────────────────────────────────────────▲───────────┘
        │ true state                                   │ move one cell
        ▼                                              │
┌───────────────────────────┐  observation  ┌─────────────────────┐
│ PERCEPTION (perception.py)│ ────────────► │ BELIEF MAP          │
│ 1 render camera window    │               │ (what the AI knows) │
│ 2 blur + threshold        │               └──────────┬──────────┘
│ 3 contours → boxes        │                          ▼
│ 4 LiDAR-style ray scan    │               ┌─────────────────────────┐
└───────────────────────────┘               │ DECISION (navigation.py)│
                                            │ path blocked → replan   │
                                            └──────────┬──────────────┘
                                                       ▼
                                            ┌─────────────────────┐
                                            │ PLANNER (planner.py)│
                                            │ A* on belief map    │
                                            └──────────┬──────────┘
                                                       ▼
                                            ┌─────────────────────────┐
                                            │ SAFETY + CONTROL        │──► world
                                            └─────────────────────────┘
```

## Data flow
world → camera image → contours → obstacle cells → belief map → A* path → safety check → next cell → world updates → repeat.

## Key design decisions
- **Belief map, not ground truth.** The planner never reads the true grid, so replanning is meaningful.
- **Optimistic initial belief.** Unknown space is assumed free; the agent corrects itself as it sees more.
- **Event-triggered replanning.** A\* reruns only when the path is invalidated, not every step.
- **Safety layer.** Independent of the planner, it rejects moves into occupied cells and diagonal moves that squeeze between obstacles.
- **Metrics.** Success, steps, replans, path efficiency and emergency stops are logged per run.

## Mapping to real hardware
| Simulation | Real robot |
|---|---|
| `perceive()` | Camera + detector (e.g. YOLO) |
| `lidar_scan()` | HC-SR04 / LiDAR |
| Move one cell | Velocity commands to motor driver |
| Belief map | Occupancy grid from SLAM |
