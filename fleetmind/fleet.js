// fleet.js  (Person 2)  -- tasks + allocation.
// STARTER VERSION: highest priority first, nearest idle robot. Person 2 replaces pickRobot()
// with the cost-based allocator (distance + battery + workload + priority) and logs the breakdown.
const Fleet = {
  addTask(pickup, drop, priority) {
    const t = { id: Sim.nextTaskId++, pickup, drop, priority, status: 'pending', createdAt: Sim.tick };
    Sim.tasks.push(t);
    Log.push('TASK', `#${t.id} ${pickup.name}->${drop.name} priority ${priority}`);
    return t;
  },

  pickRobot(task) {
    let best = null, bestCost = Infinity;
    for (const r of Sim.robots) {
      if (r.status !== 'idle') continue;
      const c = Nav.cost(r, task.pickup);
      if (c < bestCost) { bestCost = c; best = r; }
    }
    return best;
  },

  step() {
    // 1. assign pending tasks (highest priority first)
    const pending = Sim.tasks.filter(t => t.status === 'pending').sort((a, b) => b.priority - a.priority || a.id - b.id);
    for (const t of pending) {
      const r = this.pickRobot(t);
      if (!r) break;
      const path = Nav.plan(r.id, r, t.pickup, Sim.tick);
      if (!path) { Log.push('WARN', `no route to ${t.pickup.name} for task #${t.id}`); continue; }
      t.status = 'assigned'; r.task = t; r.status = 'toPickup'; r.path = path;
      Log.push('ASSIGN', `task #${t.id} -> robot ${r.id}`);
    }
    // 2. progress robots through their task states
    for (const r of Sim.robots) {
      if (r.path.length) continue;                       // still moving
      if (r.status === 'toPickup') {
        const path = Nav.plan(r.id, r, r.task.drop, Sim.tick);
        if (path) { r.status = 'toDrop'; r.path = path; Log.push('PICKUP', `robot ${r.id} picked task #${r.task.id}`); }
        else Log.push('WARN', `robot ${r.id} cannot route to ${r.task.drop.name}`);
      } else if (r.status === 'toDrop') {
        r.task.status = 'done'; r.task.doneAt = Sim.tick;
        Log.push('DELIVER', `robot ${r.id} delivered task #${r.task.id}`);
        Nav.release(r.id); r.task = null; r.status = 'idle';
      }
    }
  },
};
