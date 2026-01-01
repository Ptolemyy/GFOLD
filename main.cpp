#define NOMINMAX
#include <matplotlibcpp17/mplot3d.h>
#include <matplotlibcpp17/pyplot.h>
extern "C" {
#include "cpg_workspace.h"
#include "cpg_solve.h"
}
#include <chrono>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <vector>
static constexpr int N = 100;
static void update_state(double tf)
{
    const double dt = tf / double(N);

    // 你现在用的 alpha=1/2000 恰好等于论文的 5e-4（先按你当前写法保留）
    const double fuel_consumption = 1.0 / 2000.0;

    cpg_update_dt(dt);
    cpg_update_a_dt(fuel_consumption * dt);
    cpg_update_g_dt(0, -3.71 * dt);

    // 如果你的 solver 没有 dt_squared / g_dt_sq 的 update 接口，请删掉这两行
    cpg_update_dt_squared(dt * dt);
    cpg_update_g_dt_sq(0, -3.71 * dt * dt);

    // z0/mu 的长度：你当前版本似乎是 N（如果越界就改成 i < N-1）
    for (int i = 0; i < N; ++i) {
        double mi = 2000.0 - fuel_consumption * dt * 24000.0 * 0.8 * i;
        //if (mi <= 1.0) mi = 1.0;

        const double z00 = std::log(mi);
        const double c_z0_exp = std::exp(-z00); // exp(-z0)

        // mu = 1/(rho*exp(-z0)) = exp(z0)/rho
        const double mu_1 = 1.0 / (0.2 * 24000.0 * c_z0_exp);
        const double mu_2 = 1.0 / (0.8 * 24000.0 * c_z0_exp);

        cpg_update_z0(i, z00);
        cpg_update_mu_1(i, mu_1);
        cpg_update_mu_2(i, mu_2);
    }
}

static void compute_thrust_profiles(double tf, int steps,
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
        const double vx = ux[i], vy = uy[i], vz = uz[i];
        const double m  = std::exp(z[i]);
        const double norm_u = std::sqrt(vx*vx + vy*vy + vz*vz);

        Th.push_back(norm_u * m);

        // (kept consistent with your earlier version) angle w.r.t. x-axis:
        double cosv = 1.0;
        if (norm_u > 0.0) {
            cosv = vx / norm_u;
            cosv = std::min(1.0, std::max(-1.0, cosv));
        }
        u_angle_deg.push_back(std::acos(cosv) * RAD2DEG);
        t.push_back(i * dt);
    }
}

int main()
{
    cpg_update_r0(0, 2400.0); //x
    cpg_update_r0(1, 450.0); //y
    cpg_update_r0(2, -330.0); //z
    cpg_update_v0(0, -10.0); //vx
    cpg_update_v0(1, -40.0); //vy
    cpg_update_v0(2, 10.0); //vz
    //cpg_update_r0(0, 2400.0);
    //cpg_update_r0(1, 3400.0);
    //cpg_update_r0(2, 0.0);
    //cpg_update_v0(0, -40);
    //cpg_update_v0(1, 45.0);
    //cpg_update_v0(2, 0.0);
    cpg_update_sin_y_gs(std::sin(30.0 * M_PI / 180.0));
    cpg_update_cos_theta_deg(std::cos(45.0 * M_PI / 180.0));
    update_state(57.29); //tf
    auto start = std::chrono::steady_clock::now();
    cpg_solve();
    auto end = std::chrono::steady_clock::now(); 
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    printf("Solve time: %ld ms \n", duration.count());

    printf("Solver status: %d\n", CPG_Result.info->status);
    if (CPG_Result.info->status != 0) {
        printf("Infeasible\n");
        return 0;
    }

    std::vector<double> vec_x;
    std::vector<double> vec_y;
    std::vector<double> vec_z;
    std::vector<double> vec_ux;
    std::vector<double> vec_uy;
    std::vector<double> vec_uz;

    int steps = 100;
    double *px = CPG_Result.prim->r, *py = CPG_Result.prim->r + steps, *pz = CPG_Result.prim->r + 2*steps,
            *ux = CPG_Result.prim->u, *uy = CPG_Result.prim->u + steps, *uz = CPG_Result.prim->u + 2*steps;

    for (int i = 0; i < steps; i++) {
        vec_x.push_back(*(px+i));
        vec_y.push_back(*(py+i));
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

    std::vector<double> t, Th, u_angle;
    compute_thrust_profiles(57.29, steps, t, Th, u_angle);

    // ---- plot ----
    py::scoped_interpreter guard{};
    auto plt = matplotlibcpp17::pyplot::import();
    auto fig = plt.figure(Args(), Kwargs("figsize"_a = py::make_tuple(10, 6)));

    auto ax1 = fig.add_subplot(Args(2, 1, 1));
    ax1.plot(Args(t, Th), Kwargs("linewidth"_a = 2.0));
    const double T_max  = 24000.0;
    ax1.plot(Args(t, std::vector<double>(t.size(), T_max * 0.8)), Kwargs("linestyle"_a="--"));
    ax1.plot(Args(t, std::vector<double>(t.size(), T_max * 0.2)), Kwargs("linestyle"_a="--"));
    ax1.set_title(Args("Thrust (N)"));
    ax1.grid(Args(true));

    auto ax2 = fig.add_subplot(Args(2, 1, 2));
    ax2.plot(Args(t, u_angle), Kwargs("linewidth"_a = 2.0));
    ax2.set_title(Args("Thrust angle (deg)"));
    ax2.grid(Args(true));

    plt.show();

    return 0;
}

