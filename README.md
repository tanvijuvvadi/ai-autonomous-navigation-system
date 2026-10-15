# 🚗 AI-Based Autonomous Navigation System

A simulation-based autonomous navigation stack in Python. A virtual robot starts with **no knowledge of the map**, perceives its surroundings with an OpenCV pipeline, builds a belief map of what it has seen, plans routes with **A\***, and **replans in real time** when it discovers new or moving obstacles.

![Demo](docs/images/demo.gif)

*Left: ground truth (orange = moving obstacles, cyan box = camera view, red boxes = detections). Right: what the AI believes, plus its planned path.*

## 📌 Overview
Every step the agent runs a **perceive → update belief → plan → act** loop, the same architecture used in autonomous vehicles and warehouse robots, implemented here in a fully reproducible 2D simulator that needs no GPU or hardware.

## ❓ Problem Statement
Mobile robots must reach a goal safely in environments that are only partly known and may change. This project demonstrates the software side of that problem: sensing, world modelling, planning, replanning and safety checks.

## 🏭 Industry Relevance
The perception → world model → planner → safety layer structure underlies self-driving cars, warehouse AMRs, delivery robots, drones and industrial safety vehicles. This project is a simplified version of that structure, not a production system.

## 🛠 Tech Stack
Python · NumPy · OpenCV · Matplotlib · Pillow · pytest

## 🧠 Architecture
```
World → Simulated Camera → OpenCV Detection → Belief Map → A* Planner → Safety Check → Move → World
```
| Module | Responsibility |
|---|---|
| `src/environment.py` | Grid world, static and random-walk moving obstacles |
| `src/perception.py` | Simulated camera, blur → threshold → contour detection, LiDAR-style ray scan |
| `src/planner.py` | A\* (8-connected, octile heuristic, no corner cutting) |
| `src/navigation.py` | Decision loop, event-triggered replanning, safety layer, metrics |
| `src/visualization.py` | Dashboard frames, PNGs, GIF |
| `benchmark.py` | Multi-scenario evaluation |

More detail: [docs/architecture.md](docs/architecture.md)

## 📂 Folder Structure
```
├── main.py              # CLI entry point
├── benchmark.py         # randomized evaluation
├── src/                 # environment, perception, planner, navigation, visualization
├── tests/               # unit and scenario tests
├── docs/                # architecture, benchmark results, images
├── notebooks/           # experiments
└── outputs/             # generated per run (git-ignored)
```

## ⚙️ Installation
```bash
git clone https://github.com/<your-username>/ai-autonomous-navigation-system.git
cd ai-autonomous-navigation-system
python -m venv venv
venv\Scripts\activate          # Windows   |   source venv/bin/activate  (Mac/Linux)
pip install -r requirements.txt
```
Python 3.10+ recommended.

## ▶️ How to Run
```bash
python -m pytest                       # run tests
python main.py                         # static-obstacle run
python main.py --dynamic 4 --seed 7    # with moving obstacles
python main.py --density 0.28 --view-radius 3   # harder, short-sighted robot
python main.py --live                  # live OpenCV window (press q to stop)
python benchmark.py --runs 30          # aggregate evaluation
```
Outputs land in `outputs/`: `metrics.json`, `initial_plan.png`, `final_frame.png`, `navigation.gif`.

## 🎮 Simulation Workflow
1. Generate a random map and verify a route exists on the static layout
2. The agent "photographs" a local window; OpenCV blur, threshold and contours detect obstacles
3. The belief map is updated (observed cells overwrite old beliefs, so moved obstacles are forgotten)
4. If the current path is blocked, A\* replans on the belief map; otherwise the agent steps forward
5. A final safety check refuses any move into an occupied cell or through a pinched diagonal
6. Metrics and visuals are saved

## 📊 Results
Measured with `python benchmark.py --runs 30` (30 randomized 40×30 maps per scenario, camera radius 5):

| Scenario | Success rate | Avg steps | Avg replans | Avg path efficiency | Collisions |
|---|---|---|---|---|---|
| Static obstacles | 100% (30 runs) | 43.9 | 10.0 | 0.964 | 0 |
| Dense map | 100% (30 runs) | 49.8 | 13.6 | 0.941 | 0 |
| 4 moving obstacles | 100% (30 runs) | 44.5 | 11.0 | 0.957 | 0 |
| 8 moving obstacles | 100% (30 runs) | 44.4 | 11.8 | 0.956 | 0 |

*Path efficiency = optimal length on the true static map ÷ actual length driven (1.0 is perfect; below 1.0 because the agent discovers the map as it goes). "Collisions" is 0 by design: the safety layer rejects unsafe moves and counts them separately as `emergency_stops` in `metrics.json`.*

## 🖼 Screenshots
| Environment | Planned path |
|---|---|
| ![](docs/images/02_environment.png) | ![](docs/images/03_path_planning.png) |

| Replanning moment | Final run |
|---|---|
| ![](docs/images/04_replanning.png) | ![](docs/images/05_final_run.png) |

## ⚠️ Limitations
2D grid world · perfect, noise-free sensor · random-walk obstacles · no robot dynamics · detection is classical CV, not a learned model.

## 🔮 Future Improvements
YOLOv8 detection · sensor noise and occlusion · lane detection · ROS 2 nodes · CARLA integration · SLAM · reinforcement-learning comparison · multi-agent · warehouse and drone variants · real hardware (Raspberry Pi + camera + ultrasonic sensor)

## 🎓 Learning Outcomes
Graph search (A\*), perception pipelines, belief-state mapping, replanning, simulation-based testing, modular Python, unit testing, Git workflow.

## 👤 Author
**Your Name** · [LinkedIn](https://linkedin.com/in/your-profile) · [GitHub](https://github.com/your-username)

Licensed under MIT.
