#include "find_tf_2.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>

extern "C" {
#include "cpg_workspace.h"
#include "cpg_solve.h"
}

SearchResult find_best_tf(const GFOLDConfig& cfg_in, double a, double b, int iters, bool save_last_traj) {
    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();

    SearchResult res;
    constexpr double kThrottleSwitchTf = 5.0;
    constexpr double kThrottleMaxShortTf = 1.0;
    constexpr double kGlideSlopeShortTfDeg = 90.0;

    GFOLDSolver solver(cfg_in);
    const int steps = cfg_in.steps;
    bool last_eval_feasible = false;

    auto capture_last_traj = [&]() {
        double* rx = CPG_Result.prim->r;
        double* ry = CPG_Result.prim->r + steps;
        double* rz = CPG_Result.prim->r + 2 * steps;
        double* vx = CPG_Result.prim->v;
        double* vy = CPG_Result.prim->v + steps;
        double* vz = CPG_Result.prim->v + 2 * steps;
        double* z = CPG_Result.prim->z;

        res.last_rx.assign(rx, rx + steps);
        res.last_ry.assign(ry, ry + steps);
        res.last_rz.assign(rz, rz + steps);
        res.last_vx.assign(vx, vx + steps);
        res.last_vy.assign(vy, vy + steps);
        res.last_vz.assign(vz, vz + steps);
        res.last_m_traj.resize(steps);
        for (int i = 0; i < steps; ++i) {
            res.last_m_traj[i] = std::exp(z[i]);
        }
        res.has_last_traj = true;
    };

    auto f = [&](double x) -> double {
        res.solve_calls++;
        GFOLDConfig trial = cfg_in;
        trial.tf = x;
        trial.throttle_max = (x < kThrottleSwitchTf) ? kThrottleMaxShortTf : cfg_in.throttle_max;
        trial.glide_slope_deg = (x < kThrottleSwitchTf) ? kGlideSlopeShortTfDeg : cfg_in.glide_slope_deg;
        solver.set_config(trial);
        if (!solver.solve()) {
            last_eval_feasible = false;
            return -std::numeric_limits<double>::infinity(); // infeasible
        }

        last_eval_feasible = true;
        const int last_idx = steps - 1;
        const double mass = std::exp(CPG_Result.prim->z[last_idx]);

        res.feasible = true;
        if (mass > res.best_m) { res.best_m = mass; res.best_tf = x; }
        return mass;
    };

    constexpr double CGOLD = 0.3819660112501051; // 1 - 1/phi
    constexpr double eps   = 1e-12;

    double x = a + 0.5 * (b - a);
    double w = x, v = x;
    double fx = f(x), fw = fx, fv = fx;

    double d = 0.0, e = 0.0;

    for (int iter = 0; iter < iters; ++iter) {
        const double m  = 0.5 * (a + b);
        const double tol = std::sqrt(eps) * (std::fabs(x) + 1.0);

        if (std::fabs(x - m) <= 2.0 * tol - 0.5 * (b - a)) break;

        double p = 0.0, q = 0.0, r = 0.0;
        bool do_parabola = (std::fabs(e) > tol) &&
                           (fx > -std::numeric_limits<double>::infinity()) &&
                           (fw > -std::numeric_limits<double>::infinity()) &&
                           (fv > -std::numeric_limits<double>::infinity());

        if (do_parabola) {
            r = (x - w) * (fx - fv);
            q = (x - v) * (fx - fw);
            p = (x - v) * q - (x - w) * r;
            q = 2.0 * (q - r);
            if (q > 0.0) p = -p;
            q = std::fabs(q);

            const double etemp = e;
            e = d;

            if (std::fabs(p) < std::fabs(0.5 * q * etemp) &&
                p > q * (a - x) &&
                p < q * (b - x)) {
                d = p / q;
            } else {
                e = (x < m) ? (b - x) : (a - x);
                d = CGOLD * e;
            }
        } else {
            e = (x < m) ? (b - x) : (a - x);
            d = CGOLD * e;
        }

        double u = x + ((std::fabs(d) >= tol) ? d : (d > 0 ? tol : -tol));
        double fu = f(u);

        if (fu >= fx) {
            if (u >= x) a = x; else b = x;
            v = w; fv = fw;
            w = x; fw = fx;
            x = u; fx = fu;
        } else {
            if (u < x) a = u; else b = u;
            if (fu >= fw) {
                v = w; fv = fw;
                w = u; fw = fu;
            } else if (fu >= fv) {
                v = u; fv = fu;
            }
        }
    }

    if (save_last_traj) {
        if (last_eval_feasible) {
            capture_last_traj();
        } else {
            // Final eval may be infeasible; try one explicit final-point solve.
            GFOLDConfig final_trial = cfg_in;
            final_trial.tf = x;
            final_trial.throttle_max = (x < kThrottleSwitchTf) ? kThrottleMaxShortTf : cfg_in.throttle_max;
            final_trial.glide_slope_deg = (x < kThrottleSwitchTf) ? kGlideSlopeShortTfDeg : cfg_in.glide_slope_deg;
            solver.set_config(final_trial);
            if (solver.solve()) {
                capture_last_traj();
            } else if (res.feasible) {
                // Fallback once to best_tf so mode0 can still export a trajectory for plotting.
                final_trial.tf = res.best_tf;
                final_trial.throttle_max =
                    (res.best_tf < kThrottleSwitchTf) ? kThrottleMaxShortTf : cfg_in.throttle_max;
                final_trial.glide_slope_deg =
                    (res.best_tf < kThrottleSwitchTf) ? kGlideSlopeShortTfDeg : cfg_in.glide_slope_deg;
                solver.set_config(final_trial);
                if (solver.solve()) {
                    capture_last_traj();
                }
            }
        }
    }

    auto t1 = clock::now();
    res.elapsed_sec = std::chrono::duration<double>(t1 - t0).count();
    return res;
}
