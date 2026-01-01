extern "C" {
#include "cpg_workspace.h"
#include "cpg_solve.h"
}

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
