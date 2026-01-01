#define NOMINMAX  // 防止 Windows 头文件定义 min/max 宏污染 std::max
#include <vector>
#include <iostream>
#include <cmath>
#include <limits>
#include <optional>
#include <algorithm>

#include <pybind11/embed.h>
#include <matplotlibcpp17/pyplot.h>

extern "C" {
#include "cpg_workspace.h"
#include "cpg_solve.h"
}

static constexpr int N = 100;

// 你的更新（保持不动）
void update_state(double tf)
{
    double dt = tf / double(N);
    double fuel_consumption = (1.0 / 2000.0);

    cpg_update_dt(dt);
    cpg_update_a_dt(fuel_consumption * dt);
    cpg_update_g_dt(0, -3.71 * dt);
    cpg_update_dt_squared(dt * dt);
    cpg_update_g_dt_sq(0, -3.71 * dt * dt);

    for (int i = 0; i < N; i++) {
        double mi = 2000.0 - fuel_consumption * dt * 24000.0 * 0.8 * i;
        if (mi <= 1.0) mi = 1.0;

        double z00 = std::log(mi);
        double c_z0_exp = std::exp(-z00);
        double mu_1 = 1.0 / (0.2 * 24000.0 * c_z0_exp); // = exp(z0)/rho1
        double mu_2 = 1.0 / (0.8 * 24000.0 * c_z0_exp); // = exp(z0)/rho2

        cpg_update_z0(i, z00);
        cpg_update_mu_1(i, mu_1);
        cpg_update_mu_2(i, mu_2);
    }
}

// 评估 m(tf)。按你的规则：status!=0 => 无解 => 返回 nullopt
static std::optional<double> eval_m(double tf)
{
    update_state(tf);
    cpg_solve();

    if (CPG_Result.info->status != 0) {
        return std::nullopt;
    }
    return std::exp(CPG_Result.prim->z[N - 1]);
}

// 先用粗扫描找到一个“可行的峰值附近”作为 bracket
struct Bracket {
    double a;
    double b;
    double best_tf;
    double best_m;
    std::vector<double> tfs;
    std::vector<double> ms;
};

static Bracket coarse_bracket(double tf_min, double tf_max, double tf_step)
{
    Bracket out;
    out.a = tf_min;
    out.b = tf_max;
    out.best_tf = tf_min;
    out.best_m  = -std::numeric_limits<double>::infinity();

    bool seen_feasible = false;
    double first_feas = 0, last_feas = 0;

    for (double tf = tf_min; tf <= tf_max + 1e-12; tf += tf_step) {
        auto mopt = eval_m(tf);
        if (!mopt.has_value()) {
            std::cout << "tf = " << tf << " infeasible, status = "
                      << CPG_Result.info->status << "\n";
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
                  << " status = " << CPG_Result.info->status << "\n";
    }

    if (!seen_feasible) {
        // 整段无解
        out.a = tf_min;
        out.b = tf_min;
        out.best_tf = tf_min;
        out.best_m = -std::numeric_limits<double>::infinity();
        return out;
    }

    // 可行区间的粗估计：只在可行区间内做后续搜索
    out.a = first_feas;
    out.b = last_feas;
    return out;
}

// 黄金分割最大化（假设在 bracket 内“近似单峰”；若不是单峰，也比全扫省得多但可能找到局部峰）
static std::pair<double,double> golden_maximize(
    double a, double b,
    int max_iter = 60,
    double tol = 1e-4)
{
    const double phi = (1.0 + std::sqrt(5.0)) / 2.0;
    const double invphi = 1.0 / phi;

    // 内点
    double c = b - (b - a) * invphi;
    double d = a + (b - a) * invphi;

    auto fc_opt = eval_m(c);
    auto fd_opt = eval_m(d);

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
            fd_opt = eval_m(d);
            fd = fval(fd_opt);
        } else {
            b = d;
            d = c;
            fd = fc;

            c = b - (b - a) * invphi;
            fc_opt = eval_m(c);
            fc = fval(fc_opt);
        }
    }

    // 在 a,b,c,d 中挑最好（但 a/b 可能没算过就再算一下）
    std::vector<double> cand = {a, b, c, d};
    double best_tf = cand[0];
    double best_m  = -std::numeric_limits<double>::infinity();

    for (double tf : cand) {
        auto mopt = eval_m(tf);
        double m = mopt.has_value() ? *mopt : -std::numeric_limits<double>::infinity();
        if (m > best_m) {
            best_m = m;
            best_tf = tf;
        }
    }
    return {best_tf, best_m};
}

// thrust profile 计算（保持你的逻辑不动）
void compute_thrust_profiles(
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
    // 初始状态（保持你现在的）
    cpg_update_r0(0, 2400.0); // y
    cpg_update_r0(1, 450.0);  // z
    cpg_update_r0(2, -330.0); // x
    cpg_update_v0(0, -10.0);  // vy
    cpg_update_v0(1, -40.0);  // vz
    cpg_update_v0(2, 10.0);   // vx

    const double tf_min  = 10.0;
    const double tf_max  = 100.0;
    const double tf_step = 0.5;

    // 1) 先粗扫：拿到可行区间 + 画 m(tf) 用的数据 + 一个峰值附近
    Bracket br = coarse_bracket(tf_min, tf_max, tf_step);

    if (br.ms.empty()) {
        std::cout << "No feasible tf in [" << tf_min << "," << tf_max << "].\n";
        return 0;
    }

    std::cout << "coarse best tf = " << br.best_tf << ", m = " << br.best_m
              << " (feasible range approx [" << br.a << ", " << br.b << "])\n";

    // 2) 在可行区间内做黄金分割最大化（把扫描步长带来的误差再压下去）
    auto [best_tf, best_m] = golden_maximize(br.a, br.b, /*max_iter=*/80, /*tol=*/1e-4);
    std::cout << "golden best tf = " << best_tf << ", m = " << best_m << "\n";

    // 3) 用 best_tf 重新求解一次，画 thrust / angle
    update_state(best_tf);
    cpg_solve();
    if (CPG_Result.info->status != 0) {
        std::cout << "Best tf solution infeasible, status = "
                  << CPG_Result.info->status << '\n';
        return 0;
    }

    const int steps = 100;
    std::vector<double> t, Th, u_angle;
    compute_thrust_profiles(best_tf, steps, t, Th, u_angle);

    // ====== 画图 ======
    py::scoped_interpreter guard{};
    auto plt = matplotlibcpp17::pyplot::import();

    auto fig = plt.figure(
        Args(),
        Kwargs("figsize"_a = py::make_tuple(10, 8))
    );

    // 子图1：粗扫的 m(tf)
    auto ax1 = fig.add_subplot(Args(6, 1, 1));
    ax1.plot(Args(br.tfs, br.ms), Kwargs("color"_a = "blue", "linewidth"_a = 2.0));
    ax1.set_xlabel(Args("tf"));
    ax1.set_ylabel(Args("m(tf)"));
    ax1.set_title(Args("m(tf) (feasible points only)"));
    ax1.grid(Args(true));

    // 子图4：Thrust
    auto ax4 = fig.add_subplot(Args(6, 1, 4));
    ax4.plot(Args(t, Th), Kwargs("color"_a = "blue"));

    const double T_max     = 24000;
    const double throt1    = 0.2;
    const double throt2    = 0.8;
    const double theta_deg = 45.0;

    double y1 = T_max * throt2;
    double y2 = T_max * throt1;

    ax4.plot(Args(t, std::vector<double>(t.size(), y1)),
             Kwargs("linestyle"_a="--", "color"_a="red"));
    ax4.plot(Args(t, std::vector<double>(t.size(), y2)),
             Kwargs("linestyle"_a="--", "color"_a="red"));
    ax4.set_title(Args("Thrust (N)"));

    // 子图5：Thrust angle
    auto ax5 = fig.add_subplot(Args(6, 1, 5));
    ax5.plot(Args(t, std::vector<double>(t.size(), theta_deg)),
             Kwargs("linestyle"_a="--", "color"_a="red"));
    ax5.plot(Args(t, u_angle), Kwargs("color"_a="blue"));
    ax5.set_title(Args("Thrust angle (deg)"));

    plt.show();
    return 0;
}
