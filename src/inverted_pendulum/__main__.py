from .controller import Controller, LQRController, SwingUpController, wrap_angle
from .visualizer import Visualizer
from .plant import Plant
import numpy as np
import math
import time
from enum import Enum, auto
class Mode(Enum):
    SWING_UP = auto()
    BALANCE = auto()
g = 9.81
r = 8.0
A = np.array([
        [0, 1, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 1],
        [0, 0 ,g/r, 0]]
    )
B = np.matrix_transpose(np.array([[0, 1, 0, 1/r]]))
Q = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])
R = np.array([[1]])
U_MAX = 5.0
CATCH_ANGLE = 0.3  # switch to LQR inside this (rad)
DROP_ANGLE = 0.6   # fall back to swing-up outside this (rad)
if __name__ == "__main__":
    balancer = LQRController(A, B, Q, R, u_max=U_MAX)
    swing_up = SwingUpController(gain=0.05, gain_x=0.5, gain_v=1.0, r=r, u_max=U_MAX)
    visualizer = Visualizer([-10, 10])
    plant = Plant([0.0, 0.0, math.pi/2.0, 0.0], r, g, bounds=[-10, 10])
    dt = 0.01  # 100 Hz physics
    sim_dt = dt * 1.7  # sped up to compensate for slow drawing
    sim_time = 0.0
    steps = 0
    wall_start = time.perf_counter()
    frame = 0
    mode = Mode.SWING_UP
    while True:
        start = time.perf_counter()
        phi = wrap_angle(plant.state[2])
        if mode == Mode.SWING_UP and abs(phi) < CATCH_ANGLE:
            mode = Mode.BALANCE
            print(f"t={sim_time:6.2f}s: caught, switching to BALANCE")
        elif mode == Mode.BALANCE and abs(phi) > DROP_ANGLE:
            mode = Mode.SWING_UP
            print(f"t={sim_time:6.2f}s: dropped, switching to SWING_UP")

        u = balancer.control(plant.state) if mode == Mode.BALANCE else swing_up.control(plant.state)
        plant.step(u, sim_dt)
        if frame % 4 ==0:
            visualizer.update(plant.state, r)
        frame+=1
        sim_time += sim_dt
        steps += 1
        if steps % 100 == 0:
            wall_time = time.perf_counter() - wall_start
            print(f"sim {sim_time:6.2f}s | wall {wall_time:6.2f}s | "
                  f"speed {sim_time / wall_time:4.2f}x real time | "
                  f"avg loop {1000 * wall_time / steps:5.1f} ms (target {1000 * dt:.0f} ms) | mode {mode.name}")
        time.sleep(max(0.0, dt - (time.perf_counter() - start)))
