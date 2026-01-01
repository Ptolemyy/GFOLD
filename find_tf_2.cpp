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

SearchResult find_best_tf(const GFOLDConfig& cfg_in, double a, double b, int iters) {
    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();

    SearchResult res;

    GFOLDSolver solver(cfg_in);
    const int steps = cfg_in.steps;

    auto f = [&](double x) -> double {
        res.solve_calls++;
        GFOLDConfig trial = cfg_in;
        trial.tf = x;
        solver.set_config(trial);
        if (!solver.solve()) return -std::numeric_limits<double>::infinity(); // infeasible

        const double mass = std::exp(CPG_Result.prim->z[steps - 1]);
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

    auto t1 = clock::now();
    res.elapsed_sec = std::chrono::duration<double>(t1 - t0).count();
    return res;
}
