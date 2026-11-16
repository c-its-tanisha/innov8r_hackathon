(() => {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const COLORS = ['#0e7c86', '#d6336c', '#5c940d', '#e67700', '#7048e8', '#1971c2'];

  function showBanner(text, isError) {
    const b = $('banner');
    b.textContent = text;
    b.className = 'banner' + (isError ? ' error' : '');
    b.hidden = false;
  }

  if (typeof io === 'undefined') {
    showBanner('socket.io.min.js is missing. Download it into the static folder (see README), then reload.', true);
    return;
  }

  const socket = io();
  const cv = $('cv');
  const ctx = cv.getContext('2d');
  let grid = null, stations = null, S = null, cell = 30, allocSel = null;
  const colorIds = {};

  const colorOf = (id) => {
    if (!(id in colorIds)) colorIds[id] = COLORS[Object.keys(colorIds).length % COLORS.length];
    return colorIds[id];
  };
  const fmt = (v, d = 1) => (v === null || v === undefined ? '-' : (typeof v === 'number' ? +v.toFixed(d) : v));
  const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

  // ---------- socket ----------
  socket.on('connect', () => { $('conn').className = 'dot'; $('connText').textContent = 'Live'; });
  socket.on('disconnect', () => { $('conn').className = 'dot off'; $('connText').textContent = 'Disconnected'; });
  socket.on('init', (d) => { grid = d.grid; stations = d.stations; buildSelects(); resize(); draw(); });
  socket.on('state', (d) => { S = d; render(); });
  socket.on('toast', (d) => toast(d.message));
  socket.on('server_error', (d) => showBanner(d.message, true));

  let toastTimer = null;
  function toast(msg) {
    const t = $('toast');
    t.textContent = msg; t.hidden = false;
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => { t.hidden = true; }, 3500);
  }

  // ---------- controls ----------
  $('btnStart').onclick = () => socket.emit('resume');
  $('btnPause').onclick = () => socket.emit('pause');
  $('btnReset').onclick = () => socket.emit('reset');
  $('speed').onchange = (e) => socket.emit('set_speed', { hz: +e.target.value });
  $('modeSmart').onclick = () => socket.emit('set_mode', { mode: 'smart' });
  $('modeBase').onclick = () => socket.emit('set_mode', { mode: 'baseline' });
  $('btnDemo').onclick = () => socket.emit('demo_batch');
  $('btnHuman').onclick = () => socket.emit('spawn_human', {});
  $('showRes').onchange = () => draw();
  $('btnAdd').onclick = () => addTask(+$('selPrio').value);
  $('btnUrgent').onclick = () => addTask(3);
  function addTask(priority) {
    socket.emit('add_task', { pickup: $('selPickup').value, drop: $('selDrop').value, priority });
  }
  function buildSelects() {
    $('selPickup').innerHTML = stations.pickups.map((s) => `<option>${esc(s.id)}</option>`).join('');
    $('selDrop').innerHTML = stations.drops.map((s) => `<option>${esc(s.id)}</option>`).join('');
  }

  cv.addEventListener('click', (e) => {
    if (!grid) return;
    const r = cv.getBoundingClientRect();
    const x = Math.floor((e.clientX - r.left) / cell), y = Math.floor((e.clientY - r.top) / cell);
    if (x < 0 || y < 0 || x >= grid.w || y >= grid.h) return;
    if (isStation(x, y) || isShelf(grid.cells[y][x])) return;
    const blocked = S && S.sim.blocked.some((b) => b[0] === x && b[1] === y);
    socket.emit(blocked ? 'unblock_cell' : 'block_cell', { x, y });
  });
  window.addEventListener('resize', () => { resize(); draw(); });

  // ---------- helpers ----------
  const isShelf = (v) => !(v === 0 || v === '.' || v === ' ' || v === false || v === null || v === undefined);
  function isStation(x, y) {
    if (!stations) return false;
    const all = stations.pickups.concat(stations.drops, [stations.charger]);
    return all.some((s) => s.x === x && s.y === y);
  }
  function headingAngle(h) {
    if (typeof h === 'number') return (h * Math.PI) / 180;
    const m = { E: 0, S: Math.PI / 2, W: Math.PI, N: -Math.PI / 2, RIGHT: 0, DOWN: Math.PI / 2, LEFT: Math.PI, UP: -Math.PI / 2 };
    const a = m[String(h).toUpperCase()];
    return a === undefined ? 0 : a;
  }
  const carrying = (rid) => S && S.tasks.some((t) => t.robot_id === rid && t.status === 'picked');

  // ---------- canvas ----------
  function resize() {
    if (!grid) return;
    const w = cv.parentElement.clientWidth;
    cell = Math.max(14, Math.floor(w / grid.w));
    const dpr = window.devicePixelRatio || 1;
    cv.style.width = cell * grid.w + 'px';
    cv.style.height = cell * grid.h + 'px';
    cv.width = cell * grid.w * dpr;
    cv.height = cell * grid.h * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  const cx = (x) => x * cell + cell / 2;
  const sq = (x, y, pad = 0) => ctx.fillRect(x * cell + pad, y * cell + pad, cell - 2 * pad, cell - 2 * pad);

  function draw() {
    if (!grid) return;
    const W = grid.w, H = grid.h;
    ctx.clearRect(0, 0, W * cell, H * cell);
    ctx.fillStyle = '#eceff1'; ctx.fillRect(0, 0, W * cell, H * cell);
    ctx.strokeStyle = '#d5dbe0'; ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = 0; x <= W; x++) { ctx.moveTo(x * cell + .5, 0); ctx.lineTo(x * cell + .5, H * cell); }
    for (let y = 0; y <= H; y++) { ctx.moveTo(0, y * cell + .5); ctx.lineTo(W * cell, y * cell + .5); }
    ctx.stroke();

    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
      if (isShelf(grid.cells[y][x]) && !isStation(x, y)) {
        ctx.fillStyle = '#8596a6'; sq(x, y, 1);
        ctx.fillStyle = '#6f8091'; ctx.fillRect(x * cell + 1, y * cell + cell - 4, cell - 2, 3);
      }
    }
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.font = `700 ${Math.max(10, cell * .34)}px Bahnschrift, Segoe UI, sans-serif`;
    stations.pickups.forEach((s) => { ctx.fillStyle = '#2f9e44'; sq(s.x, s.y, 2); ctx.fillStyle = '#fff'; ctx.fillText(s.id, cx(s.x), cx(s.y)); });
    stations.drops.forEach((s) => { ctx.fillStyle = '#1971c2'; sq(s.x, s.y, 2); ctx.fillStyle = '#fff'; ctx.fillText(s.id, cx(s.x), cx(s.y)); });
    const c = stations.charger;
    ctx.fillStyle = '#f2b705'; sq(c.x, c.y, 2); ctx.fillStyle = '#16212b'; ctx.fillText('CHG', cx(c.x), cx(c.y));

    if (!S) return;
    const sim = S.sim;

    sim.blocked.forEach(([x, y]) => {
      ctx.fillStyle = 'rgba(201,42,42,.35)'; sq(x, y, 1);
      ctx.strokeStyle = '#c92a2a'; ctx.lineWidth = 2; ctx.beginPath();
      ctx.moveTo(x * cell + 5, y * cell + 5); ctx.lineTo((x + 1) * cell - 5, (y + 1) * cell - 5);
      ctx.moveTo((x + 1) * cell - 5, y * cell + 5); ctx.lineTo(x * cell + 5, (y + 1) * cell - 5); ctx.stroke();
    });

    if ($('showRes').checked && sim.reservations) {
      sim.reservations.forEach(([x, y, t, rid]) => {
        if (t - sim.tick < 0 || t - sim.tick > 8) return;
        ctx.globalAlpha = 0.28; ctx.fillStyle = colorOf(rid); sq(x, y, cell * 0.28); ctx.globalAlpha = 1;
      });
    }

    if (sim.human) {
      ctx.fillStyle = 'rgba(232,89,12,.28)';
      (sim.human.predicted || []).forEach(([x, y]) => sq(x, y, 3));
    }

    sim.robots.forEach((r) => {
      if (!r.path || !r.path.length) return;
      ctx.strokeStyle = colorOf(r.id); ctx.globalAlpha = 0.45; ctx.lineWidth = 3; ctx.lineJoin = 'round';
      ctx.beginPath(); ctx.moveTo(cx(r.x), cx(r.y));
      r.path.forEach((p) => ctx.lineTo(cx(p[0]), cx(p[1])));
      ctx.stroke(); ctx.globalAlpha = 1;
      if (r.goal) {
        ctx.strokeStyle = colorOf(r.id); ctx.lineWidth = 2;
        const gx = cx(r.goal[0]), gy = cx(r.goal[1]), k = cell * 0.2;
        ctx.beginPath(); ctx.moveTo(gx - k, gy - k); ctx.lineTo(gx + k, gy + k); ctx.moveTo(gx + k, gy - k); ctx.lineTo(gx - k, gy + k); ctx.stroke();
      }
    });

    if (sim.human) {
      const h = sim.human;
      ctx.fillStyle = '#e8590c'; ctx.beginPath(); ctx.arc(cx(h.x), cx(h.y), cell * 0.3, 0, 7); ctx.fill();
      ctx.fillStyle = '#fff'; ctx.font = `700 ${Math.max(10, cell * .3)}px sans-serif`; ctx.fillText('H', cx(h.x), cx(h.y));
    }

    sim.robots.forEach((r) => {
      const x = cx(r.x), y = cx(r.y), rad = cell * 0.36;
      if (r.status === 'waiting') { ctx.strokeStyle = '#f2b705'; ctx.lineWidth = 4; ctx.beginPath(); ctx.arc(x, y, rad + 4, 0, 7); ctx.stroke(); }
      if (r.status === 'charging') { ctx.strokeStyle = '#4c8dd6'; ctx.lineWidth = 3; ctx.setLineDash([4, 3]); ctx.beginPath(); ctx.arc(x, y, rad + 4, 0, 7); ctx.stroke(); ctx.setLineDash([]); }
      ctx.fillStyle = colorOf(r.id); ctx.beginPath(); ctx.arc(x, y, rad, 0, 7); ctx.fill();
      const a = headingAngle(r.heading);
      ctx.fillStyle = '#fff'; ctx.beginPath();
      ctx.moveTo(x + Math.cos(a) * rad * 0.95, y + Math.sin(a) * rad * 0.95);
      ctx.lineTo(x + Math.cos(a + 2.5) * rad * 0.6, y + Math.sin(a + 2.5) * rad * 0.6);
      ctx.lineTo(x + Math.cos(a - 2.5) * rad * 0.6, y + Math.sin(a - 2.5) * rad * 0.6);
      ctx.fill();
      ctx.fillStyle = '#16212b'; ctx.font = `700 ${Math.max(10, cell * .3)}px Bahnschrift, sans-serif`;
      ctx.fillText(r.id, x, y - rad - 6);
      if (carrying(r.id)) { ctx.fillStyle = '#f2b705'; ctx.fillRect(x + rad * 0.3, y - rad - 2, 8, 8); ctx.strokeStyle = '#16212b'; ctx.lineWidth = 1; ctx.strokeRect(x + rad * 0.3, y - rad - 2, 8, 8); }
    });
  }

  // ---------- panels ----------
  function render() {
    const smart = S.mode === 'smart';
    $('tick').textContent = S.sim.tick;
    const mb = $('modeBadge');
    mb.textContent = smart ? 'Smart: space-time A* + cost allocator' : 'Baseline: basic planner + first-come-first-served';
    mb.className = 'badge ' + (smart ? 'smart' : 'base');
    $('modeSmart').className = smart ? 'on' : '';
    $('modeBase').className = smart ? '' : 'on';
    $('btnStart').disabled = !S.paused;
    $('btnPause').disabled = S.paused;
    if (S.paused && S.sim.tick === 0) showBanner('Press Start to run the simulation. Add tasks first with "Load 8 demo tasks".', false);
    else if ($('banner').className === 'banner') $('banner').hidden = true;
    draw();
    renderRobots(); renderTasks(); renderKpi(); renderLog(); renderAlloc();
  }

  function renderRobots() {
    $('robots').innerHTML = S.sim.robots.map((r) => {
      const b = Math.max(0, Math.min(100, r.battery));
      const col = b > 50 ? '#2f9e44' : b > 25 ? '#e67700' : '#c92a2a';
      const task = r.task_id ? `Task #${r.task_id} (${esc(r.purpose || '')})` : (r.purpose ? esc(r.purpose) : 'No task');
      const thr = S.mode === 'smart' ? 35 : 15;
      return `<div class="rcard" style="border-left-color:${colorOf(r.id)}">
        <div class="head"><span class="name">Robot ${esc(r.id)}${carrying(r.id) ? ' (carrying)' : ''}</span><span class="chip ${esc(r.status)}">${esc(r.status)}</span></div>
        <div class="bar" title="Charge threshold ${thr}%"><i style="width:${b}%;background:${col}"></i><b style="left:${thr}%"></b></div>
        <div class="sub">${b.toFixed(0)}% battery. ${task}</div></div>`;
    }).join('');
  }

  function renderTasks() {
    const active = S.tasks.filter((t) => t.status !== 'done').sort((a, b) => b.effective_priority - a.effective_priority || a.id - b.id);
    const done = S.tasks.filter((t) => t.status === 'done').sort((a, b) => b.done_tick - a.done_tick).slice(0, 6);
    const row = (t) => `<tr><td>#${t.id}</td><td>${esc(t.pickup)} to ${esc(t.drop)}</td>
      <td><span class="prio p${t.priority}">${t.priority}</span></td>
      <td><span class="chip ${t.status}">${t.status}</span></td>
      <td>${t.robot_id === null ? '-' : 'R' + esc(t.robot_id)}</td>
      <td>${t.status === 'done' ? (t.done_tick - t.created_tick) + ' ticks' : (S.sim.tick - t.created_tick) + ' waiting'}</td></tr>`;
    $('tasks').innerHTML = S.tasks.length
      ? `<table><tr><th>ID</th><th>Route</th><th>Prio</th><th>Status</th><th>Robot</th><th>Time</th></tr>${active.concat(done).map(row).join('')}</table>`
      : '<p class="small">No tasks yet. Add one or load the demo tasks.</p>';
  }

  function renderKpi() {
    const k = S.kpi;
    const items = [
      ['Tasks done', `${k.tasks_done} / ${k.tasks_total}`, 'wide'],
      ['Avg delivery (ticks)', fmt(k.avg_delivery_ticks)], ['Avg urgent delivery', fmt(k.avg_urgent_delivery_ticks)],
      ['Utilization %', fmt(k.utilization_pct)], ['Distance travelled', fmt(k.total_distance)],
      ['Collisions', k.collisions], ['Deadlocks', k.deadlocks], ['Reroutes', k.reroutes], ['Waits', k.waits],
      ['Battery used %', fmt(k.battery_used), 'wide'],
    ];
    $('kpi').innerHTML = items.map(([l, v, w]) => `<div class="${w || ''}"><b>${v}</b><span>${l}</span></div>`).join('');
  }

  function renderLog() {
    $('log').innerHTML = S.event_log.slice().reverse().map((e) =>
      `<li class="${esc(e.type)}"><time>t${e.tick}</time>${esc(e.text)}</li>`).join('') || '<li>No events yet.</li>';
  }

  function renderAlloc() {
    const log = S.alloc_log;
    if (!log.length) return;
    const entry = (allocSel !== null && log.find((e) => e.tick + ':' + e.task_id === allocSel)) || log[log.length - 1];
    let why;
    if (entry.chosen === null) why = `Task #${entry.task_id}: no eligible robot. Battery cannot cover the trip plus the return to the charger, so it waits.`;
    else if (entry.mode === 'fcfs') why = `Task #${entry.task_id} went to Robot ${entry.chosen}: first idle robot by id. Distance and battery are ignored.`;
    else why = `Task #${entry.task_id} (priority ${entry.priority}) went to Robot ${entry.chosen}: lowest cost among eligible robots.` +
      (entry.preempted ? ` It took over from task #${entry.preempted}, which returned to the queue.` : '');
    $('allocWhy').textContent = why;
    const cost = entry.mode === 'cost';
    $('allocTable').innerHTML = `<table><tr><th>Robot</th><th>To pickup</th><th>Battery</th><th>Done</th><th>Cost</th><th>Eligible</th></tr>` +
      entry.table.map((t) => `<tr class="${t.robot === entry.chosen ? 'pick' : (cost && !t.eligible ? 'bad' : '')}">
        <td>${esc(t.robot)}</td><td>${fmt(t.d1, 0)}</td><td>${fmt(t.battery, 0)}%</td><td>${t.load}</td>
        <td>${t.cost === null ? '-' : t.cost}</td><td>${t.eligible ? 'yes' : 'no'}</td></tr>`).join('') + '</table>';
    $('allocHist').innerHTML = log.slice(-8).reverse().map((e) => {
      const key = e.tick + ':' + e.task_id;
      return `<button data-k="${key}" class="${e === entry ? 'on' : ''}">#${e.task_id} to ${e.chosen === null ? 'none' : 'R' + esc(e.chosen)}</button>`;
    }).join('');
    const newest = log[log.length - 1].tick + ':' + log[log.length - 1].task_id;
    $('allocHist').querySelectorAll('button').forEach((b) => {
      b.onclick = () => { allocSel = b.dataset.k === newest ? null : b.dataset.k; renderAlloc(); };
    });
  }
})();
