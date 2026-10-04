from numpy.typing import NDArray
import numpy as np
from typing import Protocol
from scipy.linalg import solve_continuous_are

def wrap_angle(phi: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (phi + np.pi) % (2 * np.pi) - np.pi

class Controller(Protocol):
    def control(self, x: NDArray) -> NDArray:
        ...
class LQRController(Controller):
    def __init__(self, A:NDArray, B:NDArray, Q: NDArray, R: NDArray, u_max: float):
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.u_max = u_max
        P = solve_continuous_are(self.A, self.B, self.Q, self.R, e=None, s=None, balanced=True)
        self.K = np.linalg.inv(R) @ B.T @ P
        print(self.K)
    def control(self, x: NDArray) -> NDArray:
        # LQR is linearized about phi = 0, so feed it the wrapped angle (phi may be 2*pi after a swing-up)
        x_wrapped = np.array([x[0], x[1], wrap_angle(x[2]), x[3]])
        return np.clip(-self.K @ x_wrapped, -self.u_max, self.u_max)
class SwingUpController(Controller):
    """Energy-based swing-up: pump energy toward the upright energy, with a weak spring centering the cart."""
    def __init__(self, gain: float, gain_x: float, gain_v: float, r: float, u_max: float, g: float = 9.81):
        self.gain = gain
        self.gain_x = gain_x
        self.gain_v = gain_v
        self.r = r
        self.g = g
        self.u_max = u_max
        self.E_desired = r * g  # energy of the pendulum at rest upright

    def control(self, x: NDArray) -> NDArray:
        pos, vel, phi, w = x
        E = 0.5 * self.r**2 * w**2 + self.g * self.r * np.cos(phi)
        # dE/dt = r * u * w * cos(phi), so pushing along sign(w*cos(phi)) adds energy
        pump = np.clip(self.gain * (self.E_desired - E), -1, 1) * np.sign(w * np.cos(phi))
        u = self.u_max * pump - self.gain_x * pos - self.gain_v * vel
        return np.array([np.clip(u, -self.u_max, self.u_max)])
