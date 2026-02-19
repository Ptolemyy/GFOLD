#define NOMINMAX
#include "GFOLD_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>
#include <iostream>

#include <pybind11/embed.h>
#include <matplotlibcpp17/pyplot.h>

extern "C" {
#include "cpg_workspace.h"
#include "cpg_solve.h"
}

namespace py = pybind11;

// Evaluate m(tf). If infeasible, return nullopt.
static std::optional<double> eval_m(GFOLDSolver& solver, GFOLDConfig& cfg, double tf)
{
    cfg.tf = tf;
    solver.set_config(cfg);
    if (!solver.solve()) {
        return std::nullopt;
    }
    return std::exp(CPG_Result.prim->z[cfg.steps - 1]);
}

// Coarse scan to find feasible bracket and best m(tf) sample.
struct Bracket {
    double a;
    double b;
    double best_tf;
    double best_m;
    std::vector<double> tfs;
    std::vector<double> ms;
};

static Bracket coarse_bracket(GFOLDSolver& solver, GFOLDConfig& cfg,
                              double tf_min, double tf_max, double tf_step)
{
    Bracket out;
    out.a = tf_min;
    out.b = tf_max;
    out.best_tf = tf_min;
    out.best_m = -std::numeric_limits<double>::infinity();

    bool seen_feasible = false;
    double first_feas = 0.0;
    double last_feas = 0.0;

    for (double tf = tf_min; tf <= tf_max + 1e-12; tf += tf_step) {
        auto mopt = eval_m(solver, cfg, tf);
        if (!mopt.has_value()) {
            std::cout << "tf = " << tf << " infeasible, status = "
                      << solver.status() << "\n";
            continue;
        }
        double m = *mopt;

        out.tfs.push_back(tf);
        out.ms.push_back(m);

        if (!seen_feasible) {
            seen_feasible = true;
            first_feas = tf;
        }
        last_feas = tf;

        if (m > out.best_m) {
            out.best_m = m;
            out.best_tf = tf;
        }

        std::cout << "tf = " << tf << ", m = " << m
                  << " status = " << solver.status() << "\n";
    }

    if (!seen_feasible) {
        out.a = tf_min;
        out.b = tf_min;
        out.best_tf = tf_min;
        out.best_m = -std::numeric_limits<double>::infinity();
        return out;
    }

    out.a = first_feas;
    out.b = last_feas;
    return out;
}

// Golden-section maximize inside feasible bracket.
static std::pair<double,double> golden_maximize(
    GFOLDSolver& solver, GFOLDConfig& cfg,
    double a, double b,
    int max_iter = 60,
    double tol = 1e-4)
{
    const double phi = (1.0 + std::sqrt(5.0)) / 2.0;
    const double invphi = 1.0 / phi;

    double c = b - (b - a) * invphi;
    double d = a + (b - a) * invphi;

    auto fc_opt = eval_m(solver, cfg, c);
    auto fd_opt = eval_m(solver, cfg, d);

    auto fval = [](const std::optional<double>& x) {
        return x.has_value() ? *x : -std::numeric_limits<double>::infinity();
    };

    double fc = fval(fc_opt);
    double fd = fval(fd_opt);

    for (int it = 0; it < max_iter && (b - a) > tol; ++it) {
        if (fc < fd) {
            a = c;
            c = d;
            fc = fd;

            d = a + (b - a) * invphi;
            fd_opt = eval_m(solver, cfg, d);
            fd = fval(fd_opt);
        } else {
            b = d;
            d = c;
            fd = fc;

            c = b - (b - a) * invphi;
            fc_opt = eval_m(solver, cfg, c);
            fc = fval(fc_opt);
        }
    }

    std::vector<double> cand = {a, b, c, d};
    double best_tf = cand[0];
    double best_m  = -std::numeric_limits<double>::infinity();

    for (double tf : cand) {
        auto mopt = eval_m(solver, cfg, tf);
        double m = mopt.has_value() ? *mopt : -std::numeric_limits<double>::infinity();
        if (m > best_m) {
            best_m = m;
            best_tf = tf;
        }
    }
    return {best_tf, best_m};
}

// Compute thrust profile from current CPG_Result.
static void compute_thrust_profiles(
    double tf,
    int steps,
    std::vector<double>& t,
    std::vector<double>& Th,
    std::vector<double>& u_angle_deg)
{
    t.clear(); Th.clear(); u_angle_deg.clear();
    t.reserve(steps); Th.reserve(steps); u_angle_deg.reserve(steps);

    double* ux = CPG_Result.prim->u;
    double* uy = CPG_Result.prim->u + steps;
    double* uz = CPG_Result.prim->u + 2 * steps;
    double* z  = CPG_Result.prim->z;

    const double dt = tf / steps;
    const double RAD2DEG = 180.0 / M_PI;

    for (int i = 0; i < steps; ++i) {
        double vx = ux[i];
        double vy = uy[i];
        double vz = uz[i];
        double m = std::exp(z[i]);
        double norm_u = std::sqrt(vx * vx + vy * vy + vz * vz);
        Th.push_back(norm_u * m);

        double cos_val = 1.0;
        if (norm_u > 0.0) {
            cos_val = vx / norm_u;
            cos_val = std::min(1.0, std::max(-1.0, cos_val));
        }
        double angle_rad = std::acos(cos_val);
        u_angle_deg.push_back(angle_rad * RAD2DEG);

        t.push_back(i * dt);
    }
}

int main()
{
    // Problem setup: copy from main.cpp
    GFOLDConfig cfg;
    cfg.steps = 100;
    cfg.tf = 57.29;
    cfg.g0 = 3.71;
    cfg.Isp = 2000.0/3.71;
    cfg.T_max = 24000.0;
    cfg.throttle_min = 0.2;
    cfg.throttle_max = 0.8;
    cfg.m0 = 2000.0;
    cfg.r0[0] = 2400.00000;
    cfg.r0[1] = 450.00000;
    cfg.r0[2] = -330.00000;
    cfg.v0[0] = -10.0000;
    cfg.v0[1] = -40.00;
    cfg.v0[2] = 10.0000;
    cfg.glide_slope_deg = 30.0;
    cfg.max_angle_deg = 45.0;

    GFOLDSolver solver(cfg);

    const double tf_min  = 30.0;
    const double tf_max  = 100.0;
    const double tf_step = 1;

    Bracket br = coarse_bracket(solver, cfg, tf_min, tf_max, tf_step);
    if (br.ms.empty()) {
        std::cout << "No feasible tf in [" << tf_min << "," << tf_max << "].\n";
        return 0;
    }

    std::cout << "coarse best tf = " << br.best_tf << ", m = " << br.best_m
              << " (feasible range approx [" << br.a << ", " << br.b << "])\n";

    auto [best_tf, best_m] = golden_maximize(solver, cfg, br.a, br.b, /*max_iter=*/80, /*tol=*/1e-4);
    std::cout << "golden best tf = " << best_tf << ", m = " << best_m << "\n";

    //cfg.tf = best_tf;
    cfg.tf = best_tf;
    solver.set_config(cfg);
    if (!solver.solve()) {
        std::cout << "Best tf solution infeasible, status = "
                  << solver.status() << '\n';
        return 0;
    }

    std::vector<double> t, Th, u_angle;
    compute_thrust_profiles(best_tf, cfg.steps, t, Th, u_angle);

    py::scoped_interpreter guard{};
    auto plt = matplotlibcpp17::pyplot::import();

    auto fig = plt.figure(
        Args(),
        Kwargs("figsize"_a = py::make_tuple(10, 8))
    );

    auto ax1 = fig.add_subplot(Args(6, 1, 1));
    ax1.plot(Args(br.tfs, br.ms), Kwargs("color"_a = "blue", "linewidth"_a = 2.0));
    ax1.set_xlabel(Args("tf"));
    ax1.set_ylabel(Args("m(tf)"));
    ax1.set_title(Args("m(tf) (feasible points only)"));
    ax1.grid(Args(true));

    auto ax4 = fig.add_subplot(Args(6, 1, 4));
    ax4.plot(Args(t, Th), Kwargs("color"_a = "blue"));

    double y1 = cfg.T_max * cfg.throttle_max;
    double y2 = cfg.T_max * cfg.throttle_min;

    ax4.plot(Args(t, std::vector<double>(t.size(), y1)),
             Kwargs("linestyle"_a="--", "color"_a="red"));
    ax4.plot(Args(t, std::vector<double>(t.size(), y2)),
             Kwargs("linestyle"_a="--", "color"_a="red"));
    ax4.set_title(Args("Thrust (N)"));

    auto ax5 = fig.add_subplot(Args(6, 1, 5));
    ax5.plot(Args(t, std::vector<double>(t.size(), cfg.max_angle_deg)),
             Kwargs("linestyle"_a="--", "color"_a="red"));
    ax5.plot(Args(t, u_angle), Kwargs("color"_a="blue"));
    ax5.set_title(Args("Thrust angle (deg)"));

    plt.show();
    return 0;
}
