# brushbots_simulator
Simulator for a swarm of brushbots

## Linux/macOS

### 1. Setup (to be exectued only the very first time)

In a terminal:
```
python3 -m venv brushbot_simulator_venv
source brushbot_simulator_venv/bin/activate
python3 -m pip install -r requirements.txt
deactivate
```

### 2. Run example scripts

In a terminal:
```
source brushbot_simulator_venv/bin/activate
python3 example_1_go_to_origin_algorithm.py
deactivate
```

## Windows

### 1. Download and install python

Version 3.7.9 for 64-bit OS available [here](https://www.python.org/ftp/python/3.7.9/python-3.7.9-amd64.exe)

### 2. Setup (to be exectued only the very first time)

In the command prompt:
```
python3 -m venv brushbot_simulator_venv
brushbot_simulator_venv/Scripts/activate
python3 -m pip install -r requirements.txt
deactivate
```

### 3. Run examples scripts

In the command prompt:
```
brushbot_simulator_venv/Scripts/activate
python3 example_1_go_to_origin_algorithm.py
deactivate
```

## Write your own script

Starting from one of the example scripts, write your own function `compute_velocities_and_leds` to compute velocity and led commands to control the robots!
