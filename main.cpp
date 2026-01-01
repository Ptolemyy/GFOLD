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
    // Problem setup (callers can change these before solving)
    GFOLDConfig cfg;
    cfg.steps = 100;
    cfg.tf = 57.29;
    cfg.g0 = 3.71;
    cfg.Isp = 2000.0/3.71;
    cfg.T_max = 24000.0;
    cfg.throttle_min = 0.2;
    cfg.throttle_max = 0.8;
    cfg.m0 = 2000.0;
    cfg.r0[0] = 2400.0;
    cfg.r0[1] = 450.0;
    cfg.r0[2] = -330.0;
    cfg.v0[0] = -10.0;
    cfg.v0[1] = -40.0;
    cfg.v0[2] = 10.0;
    cfg.glide_slope_deg = 30.0;
    cfg.max_angle_deg = 45.0;

    GFOLDSolver solver(cfg);
    
    auto start = std::chrono::steady_clock::now();
    const bool ok = solver.solve();
    auto end = std::chrono::steady_clock::now(); 
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Solve time: " << duration.count() << " ms\n";

    std::cout << "Solver status: " << solver.status() << "\n";
    if (!ok) {
        std::cout << "Infeasible\n";
        system("pause");
        return 0;
    }

    std::vector<double> vec_x;
    std::vector<double> vec_y;
    std::vector<double> vec_z;
    std::vector<double> vec_ux;
    std::vector<double> vec_uy;
    std::vector<double> vec_uz;

    const int steps = solver.config().steps;
    double *px = CPG_Result.prim->r, *pyv = CPG_Result.prim->r + steps, *pz = CPG_Result.prim->r + 2*steps,
            *ux = CPG_Result.prim->u, *uy = CPG_Result.prim->u + steps, *uz = CPG_Result.prim->u + 2*steps;

    for (int i = 0; i < steps; i++) {
        vec_x.push_back(*(px+i));
        vec_y.push_back(*(pyv+i));
        vec_z.push_back(*(pz+i));
        vec_ux.push_back(*(ux+i));
        vec_uy.push_back(*(uy+i));
        vec_uz.push_back(*(uz+i));
    }

    //pybind11::scoped_interpreter guard{};
    //auto plt = matplotlibcpp17::pyplot::import();
    //matplotlibcpp17::mplot3d::import();
//
    //auto fig = plt.figure(Args(), Kwargs("figsize"_a = py::make_tuple(10, 7)));
    //auto ax = fig.add_subplot(Args(), Kwargs("projection"_a = "3d"));
    //ax.plot(Args(vec_z, vec_y, vec_x), Kwargs("color"_a = "green", "linewidth"_a = 2.0));
    //ax.quiver(Args(vec_z, vec_y, vec_x, vec_uz, vec_uy, vec_ux),
    //            Kwargs("linewidth"_a = 1, "length"_a = 30, "normalize"_a = true, "color"_a = "red"));
    //ax.set_xlim(Args(-500, 500));
    //ax.set_ylim(Args(-500, 500));
    //ax.set_xlabel(Args("X"));
    //ax.set_ylabel(Args("Y")); 
    //ax.set_zlabel(Args("Z"));
//
//
    //plt.show();

    GFOLDThrustProfile profile = solver.compute_thrust_profile();

    // ---- plot ----
    py::scoped_interpreter guard{};
    auto plt = matplotlibcpp17::pyplot::import();
    auto fig = plt.figure(Args(), Kwargs("figsize"_a = py::make_tuple(10, 6)));

    auto ax1 = fig.add_subplot(Args(2, 1, 1));
    ax1.plot(Args(profile.t, profile.thrust_N), Kwargs("linewidth"_a = 2.0));
    const double T_max  = cfg.T_max;
    ax1.plot(Args(profile.t, std::vector<double>(profile.t.size(), T_max * cfg.throttle_max)), Kwargs("linestyle"_a="--"));
    ax1.plot(Args(profile.t, std::vector<double>(profile.t.size(), T_max * cfg.throttle_min)), Kwargs("linestyle"_a="--"));
    ax1.set_title(Args("Thrust (N)"));
    ax1.grid(Args(true));

    auto ax2 = fig.add_subplot(Args(2, 1, 2));
    ax2.plot(Args(profile.t, profile.angle_deg), Kwargs("linewidth"_a = 2.0));
    ax2.set_title(Args("Thrust angle (deg)"));
    ax2.grid(Args(true));

    plt.show();
    return 0;
}