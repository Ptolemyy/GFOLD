#include "GFOLD_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

extern "C" {
#include "cpg_workspace.h"
#include "cpg_solve.h"
}

GFOLDSolver::GFOLDSolver(const GFOLDConfig& cfg) : cfg_(cfg) {}

void GFOLDSolver::set_config(const GFOLDConfig& cfg) {
    cfg_ = cfg;
}

void GFOLDSolver::apply_initial_conditions() const {
    // Initial position/velocity
    cpg_update_r0(0, cfg_.r0[0]);
    cpg_update_r0(1, cfg_.r0[1]);
    cpg_update_r0(2, cfg_.r0[2]);

    cpg_update_v0(0, cfg_.v0[0]);
    cpg_update_v0(1, cfg_.v0[1]);
    cpg_update_v0(2, cfg_.v0[2]);

    // Mass and angular constraints
    cpg_update_log_m0(std::log(cfg_.m0));
    cpg_update_sin_y_gs(std::sin(cfg_.glide_slope_deg * M_PI / 180.0));
    cpg_update_cos_theta_deg(std::cos(cfg_.max_angle_deg * M_PI / 180.0));
}

void GFOLDSolver::update_state() const {
    const double dt = cfg_.tf / static_cast<double>(cfg_.steps);
    const double fuel_consumption = 1.0 / (cfg_.g0 * cfg_.Isp);

    cpg_update_dt(dt);
    cpg_update_a_dt(fuel_consumption * dt);
    cpg_update_g_dt(0, -cfg_.g0 * dt);
    //cpg_update_dt_squared(dt * dt);
    //cpg_update_g_dt_sq(0, -cfg_.g0 * dt * dt);

    const double throt1 = cfg_.throttle_min;
    const double throt2 = cfg_.throttle_max;

    for (int i = 0; i < cfg_.steps; ++i) {
        double mi = cfg_.m0 - fuel_consumption * dt * cfg_.T_max * throt2 * i;
        if (mi <= 1.0) mi = 1.0;

        const double z0 = std::log(mi);
        const double c_z0_exp = std::exp(-z0); // exp(-z0)
        const double mu_1 = 1.0 / (throt1 * cfg_.T_max * c_z0_exp);
        const double mu_2 = 1.0 / (throt2 * cfg_.T_max * c_z0_exp);

        cpg_update_z0(i, z0);
        cpg_update_mu_1(i, mu_1);
        cpg_update_mu_2(i, mu_2);
    }
}

bool GFOLDSolver::solve() {
    apply_initial_conditions();
    update_state();

    cpg_solve();

    if (CPG_Result.info->status != 0) {
        std::cerr << "Solver status: " << CPG_Result.info->status << " (non-zero)\n";
        return false;
    }
    return true;
}

int GFOLDSolver::status() const {
    return CPG_Result.info ? CPG_Result.info->status : -999;
}

GFOLDThrustProfile GFOLDSolver::compute_thrust_profile() const {
    GFOLDThrustProfile prof;
    const int steps = cfg_.steps;

    prof.t.reserve(steps);
    prof.thrust_N.reserve(steps);
    prof.angle_deg.reserve(steps);

    double* ux = CPG_Result.prim->u;
    double* uy = CPG_Result.prim->u + steps;
    double* uz = CPG_Result.prim->u + 2 * steps;
    double* z  = CPG_Result.prim->z;

    const double dt = cfg_.tf / static_cast<double>(steps);
    const double RAD2DEG = 180.0 / M_PI;

    for (int i = 0; i < steps; ++i) {
        const double vx = ux[i], vy = uy[i], vz = uz[i];
        const double m  = std::exp(z[i]);
        const double norm_u = std::sqrt(vx*vx + vy*vy + vz*vz);

        prof.thrust_N.push_back(norm_u * m);

        double cosv = 1.0;
        if (norm_u > 0.0) {
            cosv = vx / norm_u;
            cosv = std::clamp(cosv, -1.0, 1.0);
        }
        prof.angle_deg.push_back(std::acos(cosv) * RAD2DEG);
        prof.t.push_back(i * dt);
    }

    return prof;
}
