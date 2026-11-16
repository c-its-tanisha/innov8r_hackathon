# sim/test_engine.py (Person A) - run from the project folder:  python sim/test_engine.py
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sim.engine import Simulation


class OneShot:
    """Test controller: sends a list of commands on tick 0, records all events."""
    def __init__(self, commands, at_tick=0):
        self.commands, self.at_tick, self.events = commands, at_tick, []
    def decide(self, state):
        return self.commands if state["tick"] == self.at_tick else []
    def on_events(self, events):
        self.events.extend(events)


CROSSING = [
    {"robot_id": 1, "goal": [18, 2], "purpose": "dropoff", "task_id": 1},
    {"robot_id": 2, "goal": [18, 9], "purpose": "dropoff", "task_id": 2},
    {"robot_id": 3, "goal": [1, 3], "purpose": "pickup", "task_id": 3},
]


def run(mode, commands=CROSSING, ticks=200):
    sim = Simulation(planner_mode=mode)
    ctrl = OneShot(commands)
    sim.set_controller(ctrl)
    for _ in range(ticks):
        sim.step()
    return sim, ctrl


def test_state_is_json_and_has_contract_keys():
    sim = Simulation()
    s = sim.get_state()
    json.dumps(s)
    for k in ("tick", "grid", "stations", "blocked", "robots", "human", "reservations", "sim_metrics"):
        assert k in s, k
    for k in ("id", "x", "y", "heading", "battery", "status", "goal", "purpose", "task_id", "path"):
        assert k in s["robots"][0], k
    assert len(s["robots"]) == 3


def test_three_robots_arrive_with_zero_collisions():
    sim, ctrl = run("spacetime")
    arrived = {e["robot_id"] for e in ctrl.events if e["type"] == "arrived" and e["task_id"]}
    assert arrived == {1, 2, 3}, arrived
    assert sim.metrics["collisions"] == 0, sim.metrics


def test_idle_robots_walk_home():
    sim, _ = run("spacetime")
    for r in sim.robots:
        assert (r.x, r.y) == r.home, (r.id, (r.x, r.y), r.home)


def test_blocked_cell_triggers_reroute_and_robot_still_arrives():
    sim = Simulation()
    ctrl = OneShot([CROSSING[0]])
    sim.set_controller(ctrl)
    for _ in range(4):
        sim.step()
    r1 = sim.robots[0]
    ahead = r1.path[5]
    sim.block_cell(ahead[0], ahead[1])
    for _ in range(150):
        sim.step()
    assert sim.metrics["reroutes"] >= 1
    assert any(e["type"] == "arrived" and e["task_id"] == 1 for e in ctrl.events)
    assert sim.metrics["collisions"] == 0


def test_battery_drains_and_charger_refills():
    sim = Simulation()
    cmds = [{"robot_id": 1, "goal": [18, 2], "purpose": "dropoff", "task_id": 1}]
    ctrl = OneShot(cmds)
    sim.set_controller(ctrl)
    for _ in range(60):
        sim.step()
    low = sim.robots[0].battery
    assert low < 100.0
    sim.set_controller(OneShot([{"robot_id": 1, "goal": [0, 11], "purpose": "charge", "task_id": None}], at_tick=sim.tick))
    for _ in range(120):
        sim.step()
    assert sim.robots[0].battery > low
    assert sim.robots[0].battery >= 99.0        # fully charged (walking back home costs 0.2)


def test_unreachable_goal_gives_no_path_event():
    sim = Simulation()
    ctrl = OneShot([{"robot_id": 1, "goal": [3, 4], "purpose": "pickup", "task_id": 9}])
    sim.set_controller(ctrl)
    sim.step()
    assert any(e["type"] == "no_path" and e["task_id"] == 9 for e in ctrl.events)


def test_same_inputs_give_same_result():
    a, _ = run("spacetime")
    b, _ = run("spacetime")
    assert json.dumps(a.get_state(), sort_keys=True) == json.dumps(b.get_state(), sort_keys=True)


def test_estimate():
    sim = Simulation()
    e = sim.estimate(1, [18, 2])
    assert e["steps"] and e["battery_needed"] > 0


def test_head_on_traffic_zero_collisions_over_many_tasks():
    cmds_a = [{"robot_id": 1, "goal": [18, 2], "purpose": "dropoff", "task_id": 1},
              {"robot_id": 2, "goal": [1, 3], "purpose": "pickup", "task_id": 2},
              {"robot_id": 3, "goal": [18, 9], "purpose": "dropoff", "task_id": 3}]
    cmds_b = [{"robot_id": 1, "goal": [1, 8], "purpose": "pickup", "task_id": 4},
              {"robot_id": 2, "goal": [18, 9], "purpose": "dropoff", "task_id": 5},
              {"robot_id": 3, "goal": [1, 3], "purpose": "pickup", "task_id": 6}]
    sim = Simulation()
    ctrl = OneShot(cmds_a)
    sim.set_controller(ctrl)
    for _ in range(120):
        sim.step()
    ctrl.commands, ctrl.at_tick = cmds_b, sim.tick
    for _ in range(150):
        sim.step()
    done = {e["task_id"] for e in ctrl.events if e["type"] == "arrived" and e["task_id"]}
    assert done == {1, 2, 3, 4, 5, 6}, done
    assert sim.metrics["collisions"] == 0, sim.metrics


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("PASS", name)
    base, _ = run("basic")
    smart, _ = run("spacetime")
    print("Info (same crossing scenario): baseline collisions =", base.metrics["collisions"],
          "| smart collisions =", smart.metrics["collisions"])
    print("All engine tests passed.")
