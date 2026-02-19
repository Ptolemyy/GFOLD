#define NOMINMAX
#include <matplotlibcpp17/mplot3d.h>
#include <matplotlibcpp17/pyplot.h>
#include "GFOLD_solver.hpp"
extern "C" {
#include "cpg_workspace.h"
#include "cpg_solve.h"
}
#include <chrono>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace py = pybind11;

int main()
{
    // Problem setup (single N consistent with generated solver)
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

    auto start = std::chrono::steady_clock::now();
    const bool ok = solver.solve();
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "[main] N=" << cfg.steps << " solve time: " << duration.count() << " ms\n";

    if (!ok) {
        std::cout << "Infeasible for N=" << cfg.steps << "\n";
        return 1;
    }

    GFOLDThrustProfile profile = solver.compute_thrust_profile();
    GFOLDConfig plot_cfg = solver.config(); // capture cfg for plotting bounds

    // ---- plot ----
    py::scoped_interpreter guard{};
    auto plt = matplotlibcpp17::pyplot::import();
    auto fig = plt.figure(Args(), Kwargs("figsize"_a = py::make_tuple(10, 6)));

    auto ax1 = fig.add_subplot(Args(2, 1, 1));
    ax1.plot(Args(profile.t, profile.thrust_N), Kwargs("linewidth"_a = 2.0));
    const double T_max  = plot_cfg.T_max;
    ax1.plot(Args(profile.t, std::vector<double>(profile.t.size(), T_max * plot_cfg.throttle_max)), Kwargs("linestyle"_a="--"));
    ax1.plot(Args(profile.t, std::vector<double>(profile.t.size(), T_max * plot_cfg.throttle_min)), Kwargs("linestyle"_a="--"));
    ax1.set_title(Args("Thrust (N)"));
    ax1.grid(Args(true));

    auto ax2 = fig.add_subplot(Args(2, 1, 2));
    ax2.plot(Args(profile.t, profile.angle_deg), Kwargs("linewidth"_a = 2.0));
    ax2.set_title(Args("Thrust angle (deg)"));
    ax2.grid(Args(true));

    plt.show();
    return 0;
}
