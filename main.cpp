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
    const int built_N = GFOLD_CPG_N; // passed via CMake target_compile_definitions
    std::vector<int> test_Ns = {100, 75, 50, 25, 10};

    GFOLDThrustProfile profile; // keep the one we plot (N=100 expected)
    GFOLDConfig plot_cfg{};     // capture cfg used for the plotted profile
    
    for (int N : test_Ns) {
        if (N != built_N) {
            std::cout << "[main] Skip N=" << N
                      << " (rebuild with -DGFOLD_CPG_N=" << N << " to benchmark)\n";
            continue;
        }

        // Problem setup
        GFOLDConfig cfg;
        cfg.steps = N;
        cfg.tf = 31.726;
        cfg.g0 = 9.81;
        cfg.Isp = 262.9;
        cfg.T_max = 176400.0;
        cfg.throttle_min = 0.2;
        cfg.throttle_max = 0.8;
        cfg.m0 = 5400.0;
        cfg.r0[0] = 1379.700000;
        cfg.r0[1] = -6.300;
        cfg.r0[2] = -5.100;
        cfg.v0[0] = -7.38;
        cfg.v0[1] = 0.0;
        cfg.v0[2] = 0.0;
        cfg.glide_slope_deg = 30.0;
        cfg.max_angle_deg = 45.0;

        GFOLDSolver solver(cfg);

        auto start = std::chrono::steady_clock::now();
        const bool ok = solver.solve();
        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "[main] N=" << N << " solve time: " << duration.count() << " ms\n";

        if (!ok) {
            std::cout << "Infeasible for N=" << N << "\n";
            continue;
        }

        if (N == 100) {
            profile = solver.compute_thrust_profile();
            plot_cfg = solver.config(); // capture cfg for plotting bounds
        }
    }

    // ---- plot ----
    py::scoped_interpreter guard{};\
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
