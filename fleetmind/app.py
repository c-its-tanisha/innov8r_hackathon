"""FleetMind AI: Simulated Fleet Digital Twin server (Person B).
Run:  python app.py          (fake sim, works with no Person A code)
      set SIM=real & python app.py     (Windows cmd)  /  $env:SIM="real"; python app.py   (PowerShell)
"""
import os
import random
import sys
import threading
import time
import traceback

from flask import Flask, render_template
from flask_socketio import SocketIO, emit

from fleet import metrics as fm
from fleet.manager import FleetManager

SIM_KIND = os.environ.get('SIM', 'fake').lower()
if SIM_KIND == 'real':
    try:
        from sim.engine import Simulation as SimClass
    except Exception as exc:                                    # noqa: BLE001
        sys.exit(f'SIM=real but sim/engine.py could not be imported: {exc}\n'
                 'Ask Person A, or run with SIM=fake.')
else:
    from fleet.fake_sim import FakeSimulation as SimClass

MODES = {'smart': ('spacetime', 'cost'), 'baseline': ('basic', 'fcfs')}
LIVE_CSV = os.path.join('results', 'live_runs.csv')     # live demo runs; benchmark.csv stays clean
DEMO_PRIORITIES = [1, 1, 2, 1, 3, 2, 1, 3]

app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading', cors_allowed_origins='*')
lock = threading.RLock()          # RLock: the sim calls manager.decide() inside step(), and the manager may call sim.estimate()
G = {'mode': 'smart', 'seed': 42, 'paused': True, 'hz': 4}
sim = None
mgr = None


def build(mode):
    """(Re)create sim + manager for a mode. Caller holds the lock."""
    global sim, mgr
    planner, alloc = MODES[mode]
    if sim is None:
        sim = SimClass(seed=G['seed'], planner_mode=planner)
    else:
        sim.reset(G['seed'], planner)
    mgr = FleetManager(allocator_mode=alloc)
    if os.environ.get('USE_ESTIMATE') == '1' and hasattr(sim, 'estimate'):
        mgr.allocator.estimator = lambda rid, goal: sim.estimate(rid, goal)
    sim.set_controller(mgr)
    mgr.observe(sim.get_state())


def payload():
    st = sim.get_state()
    light = {k: v for k, v in st.items() if k != 'grid'}      # grid is sent once in 'init'
    data = {'sim': light, 'mode': G['mode'], 'paused': G['paused'], 'hz': G['hz']}
    data.update(mgr.snapshot())
    return data


def push():
    with lock:
        data = payload()
    socketio.emit('state', data)


def toast(message):
    emit('toast', {'message': message})


def save_live_run():
    with lock:
        k = mgr.kpi()
        if k['tasks_total'] > 0:
            run_id = f"live-{time.strftime('%Y%m%d-%H%M%S')}"
            fm.append_row(LIVE_CSV, fm.make_row(run_id, G['seed'], sim.get_state()['planner_mode'], mgr.mode, k))


def loop():
    while True:
        if G['paused']:
            socketio.sleep(0.1)
            continue
        try:
            with lock:
                sim.step()
                data = payload()
            socketio.emit('state', data)
        except Exception:                                       # noqa: BLE001
            traceback.print_exc()
            G['paused'] = True
            socketio.emit('server_error', {'message': 'Simulation error, paused. See the server terminal.'})
        socketio.sleep(1.0 / max(1, G['hz']))


@app.route('/')
def index():
    return render_template('index.html', sim_kind=SIM_KIND)


@socketio.on('connect')
def on_connect():
    with lock:
        st = sim.get_state()
        init = {'grid': st['grid'], 'stations': st['stations'], 'sim_kind': SIM_KIND}
        data = payload()
    emit('init', init)
    emit('state', data)


@socketio.on('add_task')
def on_add_task(d):
    try:
        with lock:
            mgr.add_task(d['pickup'], d['drop'], int(d.get('priority', 1)))
    except (ValueError, KeyError) as exc:
        toast(f'Could not add task: {exc}')
    push()


@socketio.on('demo_batch')
def on_demo_batch(_d=None):
    with lock:
        st = sim.get_state()
        pk = [s['id'] for s in st['stations']['pickups']]
        dr = [s['id'] for s in st['stations']['drops']]
        rng = random.Random(G['seed'])          # same tasks in smart and baseline: fair live comparison
        for prio in DEMO_PRIORITIES:
            mgr.add_task(rng.choice(pk), rng.choice(dr), prio)
    push()


@socketio.on('block_cell')
def on_block(d):
    try:
        with lock:
            sim.block_cell(int(d['x']), int(d['y']))
    except Exception as exc:                                    # noqa: BLE001
        toast(f'Could not block cell: {exc}')
    push()


@socketio.on('unblock_cell')
def on_unblock(d):
    try:
        with lock:
            sim.unblock_cell(int(d['x']), int(d['y']))
    except Exception as exc:                                    # noqa: BLE001
        toast(f'Could not unblock cell: {exc}')
    push()


@socketio.on('spawn_human')
def on_spawn(d=None):
    try:
        with lock:
            g = sim.get_state()['grid']
            wps = (d or {}).get('waypoints') or [[g['w'] // 2, 1], [g['w'] // 2, g['h'] - 2]]
            sim.spawn_human(wps)
    except Exception as exc:                                    # noqa: BLE001
        toast(f'Could not spawn human: {exc}')
    push()


@socketio.on('set_mode')
def on_set_mode(d):
    mode = (d or {}).get('mode')
    if mode not in MODES:
        return
    save_live_run()
    with lock:
        G['mode'] = mode
        build(mode)
    push()


@socketio.on('reset')
def on_reset(_d=None):
    save_live_run()
    with lock:
        build(G['mode'])
    push()


@socketio.on('pause')
def on_pause(_d=None):
    G['paused'] = True
    push()


@socketio.on('resume')
def on_resume(_d=None):
    G['paused'] = False
    push()


@socketio.on('set_speed')
def on_speed(d):
    try:
        G['hz'] = max(1, min(30, int(d['hz'])))
    except (KeyError, TypeError, ValueError):
        pass
    push()


with lock:
    build('smart')

if __name__ == '__main__':
    socketio.start_background_task(loop)
    print(f'FleetMind AI dashboard ({SIM_KIND} sim): http://localhost:5000')
    try:
        socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)
    except TypeError:                                          # older flask-socketio
        socketio.run(app, host='0.0.0.0', port=5000)
