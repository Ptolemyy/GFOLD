#include <cmath>
#include <optional>
#include <chrono>
#include <iostream>
#include <limits>

static constexpr int N = 100;

extern "C" {
#include "cpg_workspace.h"
#include "cpg_solve.h"
}

// ========== Your parameter update (keep consistent with your generator) ==========
static void update_state(double tf)
{
    const double dt = tf / double(N);
    // alpha = 1/2000 = 5e-4 (matches the paper's alpha numerically)
    const double fuel_consumption = 1.0 / 2000.0;

    cpg_update_dt(dt);
    cpg_update_a_dt(fuel_consumption * dt);
    cpg_update_g_dt(0, -3.71 * dt);

    // If your generated solver doesn't have these, delete these two lines:
    cpg_update_dt_squared(dt * dt);
    cpg_update_g_dt_sq(0, -3.71 * dt * dt);

    // NOTE: If z0/mu arrays are length N-1 in your solver, change i < N-1
    for (int i = 0; i < N; ++i) {
        double mi = 2000.0 - fuel_consumption * dt * 24000.0 * 0.8 * i;
        if (mi <= 1.0) mi = 1.0;

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

// ========== Base evaluator: status!=0 => infeasible ==========
static std::optional<double> eval_m(double tf)
{
    update_state(tf);
    cpg_solve();

    // User rule: status!=0 => no solution
    if (CPG_Result.info->status != 0) return std::nullopt;

    return std::exp(CPG_Result.prim->z[N - 1]);
}

struct SearchResult {
    double best_tf = std::numeric_limits<double>::quiet_NaN();
    double best_m  = -std::numeric_limits<double>::infinity();
    double elapsed_sec = 0.0;
    long long solve_calls = 0;
};

static SearchResult brent_max_feasible(double a, double b, int iters = 20) {
    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();

    SearchResult res;

    auto f = [&](double x) -> double {
        res.solve_calls++;
        auto mx = eval_m(x);
        if (!mx) return -std::numeric_limits<double>::infinity(); // 无解视为 -inf
        // 记录全局最优
        if (*mx > res.best_m) { res.best_m = *mx; res.best_tf = x; }
        return *mx;
    };

    constexpr double CGOLD = 0.3819660112501051; // 1 - 1/phi
    constexpr double eps   = 1e-12;

    // 初始化：x,w,v 都在区间内
    double x = a + 0.5 * (b - a);
    double w = x, v = x;
    double fx = f(x), fw = fx, fv = fx;

    double d = 0.0, e = 0.0;

    for (int iter = 0; iter < iters; ++iter) {
        const double m  = 0.5 * (a + b);
        const double tol = std::sqrt(eps) * (std::fabs(x) + 1.0);

        // 收敛判据（你也可以不提前停，严格跑满 iters）
        if (std::fabs(x - m) <= 2.0 * tol - 0.5 * (b - a)) break;

        double p = 0.0, q = 0.0, r = 0.0;
        bool do_parabola = (std::fabs(e) > tol) &&
                           (fx > -std::numeric_limits<double>::infinity()) &&
                           (fw > -std::numeric_limits<double>::infinity()) &&
                           (fv > -std::numeric_limits<double>::infinity());

        if (do_parabola) {
            // 抛物线插值（用 x,w,v 三点）
            r = (x - w) * (fx - fv);
            q = (x - v) * (fx - fw);
            p = (x - v) * q - (x - w) * r;
            q = 2.0 * (q - r);
            if (q > 0.0) p = -p;
            q = std::fabs(q);

            const double etemp = e;
            e = d;

            // 接受抛物线步的条件：步长在区间内，且不过大
            if (std::fabs(p) < std::fabs(0.5 * q * etemp) &&
                p > q * (a - x) &&
                p < q * (b - x)) {
                d = p / q;
            } else {
                // 退回黄金步
                e = (x < m) ? (b - x) : (a - x);
                d = CGOLD * e;
            }
        } else {
            // 黄金步
            e = (x < m) ? (b - x) : (a - x);
            d = CGOLD * e;
        }

        double u = x + ((std::fabs(d) >= tol) ? d : (d > 0 ? tol : -tol));
        double fu = f(u);

        // 这里是“最大化”的区间更新逻辑
        if (fu >= fx) {
            // u 更好：把区间向 u 靠拢
            if (u >= x) a = x; else b = x;
            v = w; fv = fw;
            w = x; fw = fx;
            x = u; fx = fu;
        } else {
            // u 更差：收缩但保留 x
            if (u < x) a = u; else b = u;
            if (fu >= fw) {
                v = w; fv = fw;
                w = u; fw = fu;
            } else if (fu >= fv) {
                v = u; fv = fu;
            }
        }

        // 额外：如果 fu 是 -inf（无解），上面的逻辑也会把区间往远离 u 的方向收缩
        // 相当于“无解点不可取”，自动把可行区域留下来
    }

    auto t1 = clock::now();
    res.elapsed_sec = std::chrono::duration<double>(t1 - t0).count();
    return res;
}

int main() {
    cpg_update_r0(0, 2400.0); //x
    cpg_update_r0(1, 450.0); //y
    cpg_update_r0(2, -330.0); //z
    cpg_update_v0(0, -10.0); //vx
    cpg_update_v0(1, -40.0); //vy
    cpg_update_v0(2, 10.0); //vz
    cpg_update_sin_y_gs(std::sin(30.0 * M_PI / 180.0));
    cpg_update_cos_theta_deg(std::cos(45.0 * M_PI / 180.0));
    const double L = 50.0;
    const double R = 90.0;
    const int iters = 20;

    auto ans = brent_max_feasible(L, R, iters);

    std::cout << "elapsed_sec: " << ans.elapsed_sec << "\n";
    std::cout << "best_tf:     " << ans.best_tf << "\n";
    std::cout << "best_m:      " << ans.best_m << "\n";
    std::cout << "solve_calls: " << ans.solve_calls << "\n";
    system("pause");
    return 0;

}
