// Simple wrapper around the CVXPYgen-generated cpg_* API so callers can
// configure the landing problem in one place and run the solver.
#pragma once

#include <vector>

struct GFOLDConfig {
    // Discretization
    int steps = 100;           // number of shooting nodes (must match generated model)
    double tf = 57.29;         // final time (s)

    // Vehicle / environment
    double g0 = 3.71;          // gravity magnitude (m/s^2)
    double Isp = 2000.0;       // specific impulse (s)
    double T_max = 24000.0;    // maximum thrust (N)
    double throttle_min = 0.2; // minimum throttle fraction
    double throttle_max = 0.8; // maximum throttle fraction
    double m0 = 2000.0;        // initial mass (kg)

    // Initial state
    double r0[3] = {2400.0, 450.0, -330.0};   // x, y, z (m)
    double v0[3] = {-10.0, -40.0, 10.0};      // vx, vy, vz (m/s)

    // Constraints
    double glide_slope_deg = 30.0; // glide slope angle (deg)
    double max_angle_deg   = 45.0; // thrust cone half-angle (deg)
};

struct GFOLDThrustProfile {
    std::vector<double> t;          // time stamps (s)
    std::vector<double> thrust_N;   // thrust magnitude (N)
    std::vector<double> angle_deg;  // angle wrt x-axis (deg)
};

class GFOLDSolver {
public:
    explicit GFOLDSolver(const GFOLDConfig& cfg = {});

    void set_config(const GFOLDConfig& cfg);
    const GFOLDConfig& config() const { return cfg_; }

    bool solve();
    int status() const;

    GFOLDThrustProfile compute_thrust_profile() const;

private:
    void apply_initial_conditions() const;
    void update_state() const;

    GFOLDConfig cfg_;
};
