from brushbot_simulator import BrushbotSimulator
from scipy.spatial.distance import cdist
import numpy as np

bb_sim = BrushbotSimulator()

ID_ROBOTS = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,18,19,20,21]
# SIR algorithm parameters
# motion
V_SPEED = 0.5
W_SPEED = 100.0
SWITCH_TIMEOUT = {x: 20 + 60 * np.random.rand() for x in ID_ROBOTS}
TURNING_TIME = {x: 20 + 20 * np.random.rand() for x in ID_ROBOTS}
TURNING_DIRECTION = {x: -1 + 2 * np.random.randint(0,2) for x in ID_ROBOTS}
timers_motion = {x: 0 for x in ID_ROBOTS}
# population
D_THRESH = 0.15
TIME_TO_RECOVER = 100
TIME_INCUBATION = 25
# Other SIR parameter setup examples
# time_to_recover=100, time_incubation=25, d_thresh=0.15
# time_to_recover=50, time_incubation=6, d_thresh=0.15
# time_to_recover=100, time_incubation=25, d_thresh=0.1
# time_to_recover=50, time_incubation=6, d_thresh=0.1
COLORS = np.array([[0,0,255],[255,0,0],[0,255,0],[255,255,0]])  # S, I, R, Vaccinated
state = {x: int(0) for x in ID_ROBOTS}
state[1] = 1
timers_pop = {x: 0 for x in ID_ROBOTS}
timers_set = {x: False for x in ID_ROBOTS}

def compute_velocities_and_leds(p):
    global ID_ROBOTS, V_SPEED, W_SPEED, SWITCH_TIMEOUT, TURNING_TIME, TURNING_DIRECTION, timers_motion, D_THRESH, TIME_TO_RECOVER, TIME_INCUBATION, COLORS, state, timers_pop, timers_set

    vel = np.zeros((len(ID_ROBOTS),2))
    leds = {x: 6*[0] for x in ID_ROBOTS}

    if p is not None:
        d2 = cdist(p[:, :2], p[:, :2])
        new_state = state
        for i in range(p.shape[0]):
            id_i = ID_ROBOTS[i]
            # susceptible -> infected/infectious
            for j in range(p.shape[0]):
                id_j = ID_ROBOTS[j]
                if not(state[id_i] == 3 or state[id_j] == 3):
                    if d2[i][j] < D_THRESH:
                        if state[id_i] == 1 and state[id_j] == 0:
                            if timers_pop[id_i] >= TIME_INCUBATION:
                                new_state[id_j] = 1
                        if state[id_i] == 0 and state[id_j] == 1:
                            if timers_pop[id_j] >= TIME_INCUBATION:
                                new_state[id_i] = 1
            # infected/infectious -> recovered
            if state[id_i] == 1:
                if not timers_set[id_i]:
                    timers_set[id_i] = True
                else:
                    timers_pop[id_i] += 1
                if timers_pop[id_i] >= TIME_TO_RECOVER:
                    state[id_i] = 2
            state = new_state

        # velocity controller
        for i in range(p.shape[0]):
            id_i = ID_ROBOTS[i]
            if timers_motion[id_i] < SWITCH_TIMEOUT[id_i]:
                vel[i, 0] = V_SPEED
                vel[i, 1] = 0
                timers_motion[id_i] += 1
            elif timers_motion[id_i] < SWITCH_TIMEOUT[id_i] + TURNING_TIME[id_i]:
                vel[i, 0] = 0
                vel[i, 1] = TURNING_DIRECTION[id_i] * W_SPEED
                timers_motion[id_i] += 1
            else:
                vel[i, 0] = V_SPEED
                vel[i, 1] = 0
                timers_motion[id_i] = 0

        # led controller
        for i in range(p.shape[0]):
            id_i = ID_ROBOTS[i]
            leds[id_i][0:3] = COLORS[state[id_i]][:]
            leds[id_i][3:] = COLORS[state[id_i]][:]

        for i in range(vel.shape[0]):
            vel[i, 0] = min(max(1*vel[i, 0], 0), 2)
            vel[i, 1] = min(max(10*vel[i, 1], -100), 100)

    return vel, leds

while True:
    p = bb_sim.get_poses()
    v, leds = compute_velocities_and_leds(p)
    bb_sim.set_velocities_and_leds(v, leds)
    bb_sim.step()
