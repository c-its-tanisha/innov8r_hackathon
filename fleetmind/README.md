# FleetMind AI: Simulated Fleet Digital Twin (Person B)

Always call this a **Simulated Fleet Digital Twin**. There are no physical robots.

## Setup (Windows, PowerShell)
    python -m venv .venv
    .venv\Scripts\activate
    pip install -r requirements.txt
    Invoke-WebRequest https://cdn.socket.io/4.7.5/socket.io.min.js -OutFile static\socket.io.min.js

## Run
    python app.py                         # fake sim, open http://localhost:5000
    $env:SIM="real"; python app.py        # Person A's sim (sim/engine.py)
    $env:USE_ESTIMATE="1"                 # optional: use sim.estimate() for robot->pickup distance

## Test
    python -m unittest fleet.test_fleet -v

## Benchmark (real sim only; never quote numbers from --fake)
    $env:SIM="real"; python -m scenarios.demo --seeds 1 2 3 4 5     # writes results/benchmark.csv

Live runs from the dashboard are appended to results/live_runs.csv when you press Reset or switch mode.
