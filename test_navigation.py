from src.environment import create_solvable_world
from src.navigation import NavigationAgent
from src.perception import SensorSuite


def test_agent_reaches_goal_on_static_map():
    world = create_solvable_world(40, 30, 0.18, 0, 42)
    result = NavigationAgent(world, SensorSuite()).run()
    assert result["success"] and result["collisions"] == 0


def test_agent_never_enters_blocked_cell_with_moving_obstacles():
    world = create_solvable_world(40, 30, 0.18, 4, 7)
    agent = NavigationAgent(world, SensorSuite())
    agent.run()
    assert all(world.grid[y, x] == 0 for x, y in agent.trail)


def test_agent_never_cuts_corners_and_is_not_better_than_optimal():
    for seed in (42, 7, 3):
        world = create_solvable_world(40, 30, 0.18, 0, seed)
        agent = NavigationAgent(world, SensorSuite())
        result = agent.run()
        for a, b in zip(agent.trail, agent.trail[1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            if dx and dy:
                assert world.grid[a[1], a[0] + dx] == 0 and world.grid[a[1] + dy, a[0]] == 0
        assert result["path_efficiency"] <= 1.0
