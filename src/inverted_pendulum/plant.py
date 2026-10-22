from numpy.typing import NDArray
import numpy as np
class Plant:
    """Nonlinear cart-pole. State is [x, x_dot, phi, phi_dot], phi = 0 upright, u = cart acceleration."""
    def __init__(self, initial_state: NDArray, r: float, g: float = 9.81, bounds = [-10, 10]):
        self.state = np.asarray(initial_state, dtype=float)
        self.r = r
        self.g = g
        self.bounds = bounds

    def step(self, u: NDArray, dt: float) -> NDArray:
        lo, hi = self.bounds
        x, v, phi, w = self.state
        a = float(u[0])

        # At a wall, block any push further into it; pushing away is still allowed
        if (x <= lo and a < 0) or (x >= hi and a > 0):
            a = 0.0

        alpha = (self.g * np.sin(phi) + a * np.cos(phi)) / self.r

        # Semi-implicit Euler: update velocities first, then positions
        v += a * dt
        w += alpha * dt
        x += v * dt
        phi += w * dt

        # Hard stop: clamp to the wall and kill the cart's velocity
        if x < lo or x > hi:
            x = np.clip(x, lo, hi)
            v = 0.0

        self.state = np.array([x, v, phi, w]) #+ np.array([0.0, (np.random.random()-0.5)/10.0, 0.0, 0.0])
        return self.state
