from brushbot_simulator import BrushbotSimulator
from scipy.spatial.distance import cdist
import numpy as np

bb_sim = BrushbotSimulator()

ID_ROBOTS = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,18,19,20,21]

def compute_velocities_and_leds(p):
    global ID_ROBOTS

    vel = np.zeros((len(ID_ROBOTS),2))
    leds = {x: 6*[0] for x in ID_ROBOTS}

    if p is not None:
        # velocity controller
        for i in range(p.shape[0]):
            vel[i, 0] = 0.1 * np.sqrt(p[i,0]**2+p[i,1]**2)
            delta_th = - np.arctan2(p[i,0],p[i,1]) + np.pi - p[i,2]
            vel[i, 1] = 10 * np.arctan2(np.cos(delta_th), np.sin(delta_th))

        # led controller
        for i in range(p.shape[0]):
            id_i = ID_ROBOTS[i]
            if np.sqrt(p[i,0]**2+p[i,1]**2) > 0.1:
                leds[id_i][0:3] = [255,0,0]
                leds[id_i][3:] = [255,0,0]
            else:
                leds[id_i][0:3] = [0,255,0]
                leds[id_i][3:] = [0,255,0]

        for i in range(vel.shape[0]):
            vel[i, 0] = min(max(1*vel[i, 0], 0), 2)
            vel[i, 1] = min(max(10*vel[i, 1], -100), 100)

    return vel, leds

while True:
    p = bb_sim.get_poses()
    v, leds = compute_velocities_and_leds(p)
    bb_sim.set_velocities_and_leds(v, leds)
    bb_sim.step()
