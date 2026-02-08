from dataclasses import dataclass
import numpy as np
from scipy import signal
@dataclass
class Boundary:
    r0: np.ndarray
    v0: np.ndarray
    m0: float
    rT: np.ndarray
    vT: np.ndarray
    throt1: float       # thrust lower bound ()
    throt2: float       # thrust upper bound ()
    T_max: float
    V_max: float
    y_gs: float = 30.0 #glide_slope angle (deg)
    theta_deg: float = 120.0  # pointing cone half-angle (deg) relative to +z
    rp3: float = 0.0  # radius for p3 target

@dataclass
class ParamsLCVX:
    N: int
    dt: float
    g: np.ndarray
    a: float

params = ParamsLCVX(
        N=100,
        dt=0.30154781, #tf/N
        g=np.array([-9.81, 0.0, 0.0]),
        a = 1/(262.99*9.81),
    )
bnd = Boundary(
    r0=np.array([1422.7, -1.6, 0.9]),
    v0=np.array([-7.38, 0.0, 0.0]),
    m0=5600.0,
    throt1= 0.2, #throt
    throt2= 0.8,
    T_max= 176600.0,
    rT=np.array([0.0, 0.0, 0.0]),
    vT=np.array([0.0, 0.0, 0.0]),
    V_max = 90.0,
    theta_deg=45.0,
    y_gs=30.0,
    rp3 = np.array([0.0,0.0,0.0])
)

    