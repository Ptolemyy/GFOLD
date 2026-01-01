# solver_lcvx_log.py
"""
LCvx solver faithful to Acikmese & Ploen (Soft Landing LCvx) with integrated
lexicographic (P4 -> P3) runner.

Core model (discrete, fixed dt):
    r_{k+1} = r_k + dt * v_k
    v_{k+1} = v_k + dt * (g + u_k)
    z_{k+1} = z_k - alpha * sigma_k * dt         (z = ln m, alpha = 1/(Isp*g0))

Lossless convexification constraints:
    ||u_k||_2 <= sigma_k
    n^T u_k >= cos(theta) * sigma_k              (pointing cone about +z)

Thrust magnitude bounds (paper's exponential form) approximated by Eq. (37):
    rho1 * exp(-z0_k) * [1 - (z_k - z0_k) + 0.5 (z_k - z0_k)^2] <= sigma_k
    sigma_k <= rho2 * exp(-z0_k) * [1 - (z_k - z0_k)]

Provides:
- lcvx_solve(...)   : Solve P3 (fuel-min), P4 (error-min), or Weighted
- run_lexicographic: Stage-1 P4 -> Stage-2 P3 with hard caps based on P4
- plot_lexi(...)   : Make plots for the lexicographic planned trajectories only
"""

import numpy as np
import cvxpy as cp
from config import *
import os
os.environ["CMAKE_GENERATOR"] = "Visual Studio 17 2022"
os.environ["CMAKE_GENERATOR_PLATFORM"] = "x64"

from cvxpygen import cpg

def _calculate_parameter(expression, *args, **kwargs):
        """Helper function to create parameters from expressions with values"""
        return cp.Parameter(*args, value=expression.value, **kwargs)
# ----------------------------- Core Solver -----------------------------
def e(i):
    return signal.unit_impulse(3,i) 

def lcvx_solve(problem_type):
    '''params'''
    
    log_m0_ = np.log(bnd.m0)
    rho1_ = bnd.throt1 * bnd.T_max
    rho2_ = bnd.throt2 * bnd.T_max
    cos_theta_deg_ = np.cos(np.deg2rad(bnd.theta_deg))
    sin_y_gs_ = np.sin(np.deg2rad(bnd.y_gs))
    
    N = params.N
    dt = cp.Parameter(name='dt', value=params.dt, nonneg=True)
    g = cp.Parameter(3, name='g', value=params.g)   
    g_dt = _calculate_parameter(g * dt,3, name='g_dt')
    a = cp.Parameter(name='a', value=params.a, nonneg=True)
    a_dt = _calculate_parameter(a * dt, name='a_dt', nonneg=True)
    g_dt_sq = _calculate_parameter(g*dt*dt, 3, name="g_dt_sq")
    dt_squared = _calculate_parameter(dt**2, name="dt_squared")
    '''boundary conditions'''
    r0 = cp.Parameter(3, name='r0', value=bnd.r0)
    v0 = cp.Parameter(3, name='v0', value=bnd.v0)
    log_m0 = cp.Parameter(name='m0', value=log_m0_,nonneg=True)
    sin_y_gs = cp.Parameter(name='sin_y_gs', value=sin_y_gs_, nonneg=True)
    cos_theta_deg = cp.Parameter(name='cos_theta_deg', value=cos_theta_deg_, nonneg=True)
    rT = bnd.rT
    vT = bnd.vT
    V_max = bnd.V_max
    rp3 = cp.Parameter(3, name='rp3', value=bnd.rp3)
    
    z0 = cp.Parameter(N, name='z0')
    mu_2 = cp.Parameter(N, name='mu_2', nonneg=True)
    mu_1 = cp.Parameter(N, name='mu_1', nonneg=True)
    
    c_z0_ = []
    c_mu2_ = []
    c_mu1_ = []
    
    for k in range(0,N):
        z00_term = bnd.m0 - a.value * rho2_ * (k) * dt.value  # see ref [2], eq 34,35,36
        z00 = np.log(z00_term)
        mu_2_ = 1 / (rho2_ * np.exp(-z00))
        mu_1_ = 1 / (rho1_ * np.exp(-z00))
        c_z0_.append(z00)
        c_mu2_.append(mu_2_)
        c_mu1_.append(mu_1_)
    
    z0.value = c_z0_
    mu_2.value = c_mu2_
    mu_1.value = c_mu1_
    
    if problem_type=='p3':
        program = 3
    elif problem_type=='p4':
        program = 4
    
    # Decision variables
    r = cp.Variable((N, 3),"r")
    v = cp.Variable((N, 3),"v")
    z = cp.Variable(N,"z")            # ln m
    u = cp.Variable((N, 3),"u")         # specific thrust: Tc/m
    s = cp.Variable(N,"s")          # ||Tc||

    # Constraints
    constraints = []
    # Initial conditions
    constraints += [r[0, :] == r0]
    constraints += [v[0, :] == v0]
    constraints += [v[-1, :] == vT]

    constraints += [z[0] == log_m0]
    
    if program == 3:
        constraints += [r[-1, 0] == 0]
    elif program == 4:
        constraints += [r[-1, :] == rp3]
        #con += [norm(E*(x[0:3,N-1]-rf))<=norm(rp3-rf)] # CONVEX <= CONVEX (?)

    for k in range(0,N):
        # Dynamics --> v = A(w)*x + B*(g + u)
        if k != N - 1:
            acc = (u[k+1, :] + u[k, :])/2
            constraints += [
                r[k+1, :] == r[k, :] + (v[k, :] + v[k+1, :]) * dt / 2 + (acc*dt_squared+g_dt_sq) * (1/2),
                v[k+1, :] == v[k, :] + acc*dt + g_dt,
            ]
            constraints += [z[k+1] == z[k] - (a_dt / 2) * (s[k]+s[k+1])] # mass decreases
        #constraints += [cp.norm(E*(r[k, :]- rp3)) - c.T*(r[k, :]- rp3) <= 0 ] # glideslope, full generality # (5)
        #constraints += [cp.norm((r[k,:]-rT)[0:2]) - c.T[0]*(r[k,0]-rT[0]) <= 0] # glideslope, specific, but faster
        constraints += [r[k, 0] >= cp.norm(r[k,:]) * sin_y_gs]
        constraints += [cp.norm(v[k, :]) <= V_max]# velocity
        
        constraints += [cp.norm(u[k, :]) <= s[k]] # limit thrust magnitude & also therefore, mass
        constraints += [ u[k, 0] >= cos_theta_deg*s[k]]
        
        constraints += [(1 - (z[k] - z0[k]) + 0.5 * (z[k] - z0[k])**2) <= s[k] * mu_1[k]]
        constraints += [s[k] * mu_2[k] <= (1 - (z[k] - z0[k]))]
    
    if program == 3:
        print('-----------------------------')
        objective=cp.Minimize(cp.norm(r[-1,:]-rT))
        problem=cp.Problem(objective,constraints=constraints)
        return problem
        print('-----------------------------')
    elif program == 4:
        print('-----------------------------')
        objective=cp.Minimize(cp.sum(s))
        problem=cp.Problem(objective,constraints=constraints)
        return problem
        print('-----------------------------')

if __name__ == '__main__':
    p3 = lcvx_solve('p3')
    p4 = lcvx_solve('p4')
    cpg.generate_code(p3, code_dir="p3_cpg_solver", solver=cp.ECOS,wrapper=False)
    cpg.generate_code(p4, code_dir="p4_cpg_solver", solver=cp.ECOS,wrapper=False)