#define NOMINMAX
#include "GFOLD_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>
#include <iostream>

#include <pybind11/embed.h>
#include <matplotlibcpp17/mplot3d.h>
#include <matplotlibcpp17/pyplot.h>

namespace py = pybind11;

// Evaluate m(tf). If infeasible, return nullopt.
static std::optional<double> eval_m(GFOLDSolver& solver, GFOLDConfig& cfg, double tf)
{
    cfg.tf = tf;
    solver.set_config(cfg);
    if (!solver.solve()) {
        return std::nullopt;
    }
    const double m = solver.terminal_mass();
    if (m == m) return m;
    return std::nullopt;
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

int main()
{
    // Problem setup: copy from main.cpp
    GFOLDConfig cfg;
    cfg.steps = 10;
    cfg.tf = 2.728452;
    cfg.g0 = 9.810000;
    cfg.Isp = 252.085000;
    cfg.T_max = 169370.000000;
    cfg.throttle_min = 0.300000;
    cfg.throttle_max = 1.000000;
    cfg.m0 = 5481.000000;
    cfg.r0[0] = 53.900000;
    cfg.r0[1] = -0.200000;
    cfg.r0[2] = 0.000000;
    cfg.v0[0] = -42.560000;
    cfg.v0[1] = 0.070000;
    cfg.v0[2] = -6.220000;
    cfg.glide_slope_deg = 30.000000;
    cfg.max_angle_deg = 20.000000;

    GFOLDSolver solver(cfg);

    const double tf_min  = 1.0;
    const double tf_max  = 6.0;
    const double tf_step = 0.1;

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

    GFOLDThrustProfile prof = solver.compute_thrust_profile();
    GFOLDSolution sol = solver.solution();

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
    ax4.plot(Args(prof.t, prof.thrust_N), Kwargs("color"_a = "blue"));

    double y1 = cfg.T_max * cfg.throttle_max;
    double y2 = cfg.T_max * cfg.throttle_min;

    ax4.plot(Args(prof.t, std::vector<double>(prof.t.size(), y1)),
             Kwargs("linestyle"_a="--", "color"_a="red"));
    ax4.plot(Args(prof.t, std::vector<double>(prof.t.size(), y2)),
             Kwargs("linestyle"_a="--", "color"_a="red"));
    ax4.set_title(Args("Thrust (N)"));

    auto ax5 = fig.add_subplot(Args(6, 1, 5));
    ax5.plot(Args(prof.t, std::vector<double>(prof.t.size(), cfg.max_angle_deg)),
             Kwargs("linestyle"_a="--", "color"_a="red"));
    ax5.plot(Args(prof.t, prof.angle_deg), Kwargs("color"_a="blue"));
    ax5.set_title(Args("Thrust angle (deg)"));

    if (sol.steps > 0 &&
        static_cast<int>(sol.rx.size()) == sol.steps &&
        static_cast<int>(sol.ry.size()) == sol.steps &&
        static_cast<int>(sol.rz.size()) == sol.steps) {
        auto fig_r = plt.figure(
            Args(),
            Kwargs("figsize"_a = py::make_tuple(8, 6))
        );
        auto axr = fig_r.add_subplot(Args(1, 1, 1), Kwargs("projection"_a = "3d"));
        axr.plot(Args(sol.rx, sol.ry, sol.rz),
                 Kwargs("color"_a = "tab:blue", "linewidth"_a = 2.0));
        axr.set_title(Args("R 3D Trajectory"));
        axr.set_xlabel(Args("Up (x)"));
        axr.set_ylabel(Args("North (y)"));
        axr.set_zlabel(Args("East (z)"));
        axr.grid(Args(true));
    } else {
        std::cout << "Skip R 3D plot: invalid r trajectory.\n";
    }

    plt.show();
    return 0;
}
