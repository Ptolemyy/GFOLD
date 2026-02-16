#include "GFOLD_solver.hpp"
#include "find_tf_2.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

extern "C" {
#include "cpg_workspace.h"
}

namespace fs = std::filesystem;
using namespace std::chrono_literals;

static std::string read_last_line(const fs::path& p) {
    std::ifstream in(p);
    if (!in) return "";
    std::string line, last;
    while (std::getline(in, line)) {
        if (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
            while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) line.pop_back();
        }
        if (!line.empty()) last.swap(line);
    }
    return last;
}

static std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ',')) out.push_back(item);
    return out;
}

// Read the newest line from send.txt and remove it from the file.
static std::string consume_last_line(const fs::path& p) {
    std::ifstream in(p);
    if (!in) return "";
    std::vector<std::string> lines;
    std::string l;
    while (std::getline(in, l)) {
        while (!l.empty() && (l.back() == '\n' || l.back() == '\r')) l.pop_back();
        if (!l.empty()) lines.push_back(l);
    }
    if (lines.empty()) return "";
    std::string last = lines.back();
    lines.pop_back();

    std::ofstream out(p, std::ios::trunc);
    for (size_t i = 0; i < lines.size(); ++i) {
        out << lines[i];
        if (i + 1 < lines.size()) out << "\n";
    }
    return last;
}

static bool atomic_write(const fs::path& p, const std::string& content) {
    fs::path tmp = p;
    tmp += ".tmp";

    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    if (!out) return false;
    out << content;
    out.close();

    std::error_code ec;
    fs::remove(p, ec);
    fs::rename(tmp, p, ec);
    return !ec;
}

static bool populate_cfg_from_tokens(const std::vector<std::string>& tok, GFOLDConfig& cfg, double* base_time_out) {
    // Expect at least: MODE,E,N,U,VE,VN,VU,SPEED,THRUST_MAX,ISP,MASS_WET
    if (tok.size() < 11) return false;
    // Optional extra fields:
    // [11]=THROT1, [12]=THROT2, [13]=THETA_DEG, [14]=YGS_DEG, [15]=ELAPSED_SEC
    try {
        // Solver assumes x-axis is altitude. Map ENU accordingly:
        cfg.r0[0] = std::stod(tok[1]); // U -> x (altitude)
        cfg.r0[1] = std::stod(tok[2]); // N -> y
        cfg.r0[2] = std::stod(tok[3]); // E -> z

        cfg.v0[0] = std::stod(tok[4]); // VU -> vx
        cfg.v0[1] = std::stod(tok[5]); // VN -> vy
        cfg.v0[2] = std::stod(tok[6]); // VE -> vz

        cfg.T_max = std::stod(tok[8]) * 1000.0;   // kN -> N
        cfg.Isp   = std::stod(tok[9]);            // s
        cfg.m0    = std::stod(tok[10]) * 1000.0;  // t -> kg

        if (tok.size() > 11) cfg.throttle_min = std::stod(tok[11]);
        if (tok.size() > 12) cfg.throttle_max = std::stod(tok[12]);
        if (tok.size() > 13) cfg.max_angle_deg = std::stod(tok[13]);
        if (tok.size() > 14) cfg.glide_slope_deg = std::stod(tok[14]);
        if (base_time_out) *base_time_out = 0.0;
        // Optional: elapsed time from kOS (index 15)
        if (tok.size() > 15) {
            if (base_time_out) *base_time_out = std::stod(tok[15]);
        }
    } catch (...) {
        return false;
    }
    return true;
}

struct TrajectoryCache {
    int steps = 0;
    double tf = 0.0;
    std::vector<double> rx;
    std::vector<double> ry;
    std::vector<double> rz;
    bool valid = false;
};

static int closest_prior_index(const TrajectoryCache& traj, const double r0[3]) {
    if (!traj.valid || traj.steps <= 0) return 0;
    int best_idx = 0;
    double best_d2 = (std::numeric_limits<double>::max)();
    for (int i = 0; i < traj.steps; ++i) {
        const double dx = traj.rx[i] - r0[0];
        const double dy = traj.ry[i] - r0[1];
        const double dz = traj.rz[i] - r0[2];
        const double d2 = dx * dx + dy * dy + dz * dz;
        if (d2 < best_d2) {
            best_d2 = d2;
            best_idx = i;
        }
    }
    return best_idx;
}

static std::vector<int> build_sample_indices(int steps, int gap, int max_count) {
    std::vector<int> indices;
    indices.reserve(max_count);
    const int stride = gap + 1;
    for (int i = 0; i < max_count; ++i) {
        const int idx = i * stride;
        if (idx >= steps) break;
        indices.push_back(idx);
    }
    return indices;
}

int main() {
    // Adjust this path to your KSP Scripts folder if needed.
    const fs::path send_path =
        R"(C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program\Ships\Script\send.txt)";
    const fs::path recv_path =
        R"(C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program\Ships\Script\receive.txt)";

    // Clear old content to avoid stale reads
    {
        std::error_code ec;
        fs::remove(send_path, ec);
        std::ofstream clear(send_path, std::ios::trunc);
    }

    bool has_best_tf = false;
    double best_tf = 0.0;
    TrajectoryCache last_traj;
    const std::vector<int> sample_gaps = {0, 1, 3, 4, 9};
    size_t sample_gap_mode = 0;
    constexpr int max_lines = 10;
    constexpr double recompute_time = 1.5;

    while (true) {
        std::this_thread::sleep_for(20ms);

        std::string line = consume_last_line(send_path);
        if (line.empty()) continue;

        auto tok = split_csv(line);
        if (tok.empty()) continue;

        int mode = 0;
        try { mode = std::stoi(tok[0]); } catch (...) { continue; }

        if (mode == 0) {
            GFOLDConfig cfg; // start with defaults
            cfg.tf = 60.0;   // placeholder; actual tf is searched over
            cfg.g0 = 9.81;
            if (!populate_cfg_from_tokens(tok, cfg, nullptr)) {
                std::cerr << "Parse error on line: " << line << "\n";
                continue;
            }

            std::cout << std::fixed << std::setprecision(6);
            std::cout << "[mode0] cfg tf=" << cfg.tf
                      << " g0=" << cfg.g0
                      << " Isp=" << cfg.Isp
                      << " T_max=" << cfg.T_max
                      << " throt=[" << cfg.throttle_min << "," << cfg.throttle_max << "]"
                      << " m0=" << cfg.m0
                      << " r0=[" << cfg.r0[0] << "," << cfg.r0[1] << "," << cfg.r0[2] << "]"
                      << " v0=[" << cfg.v0[0] << "," << cfg.v0[1] << "," << cfg.v0[2] << "]"
                      << " glide=" << cfg.glide_slope_deg
                      << " theta=" << cfg.max_angle_deg
                      << " steps=" << cfg.steps
                      << "\n";

            const double L = 10.0, R = 90.0;
            SearchResult res = find_best_tf(cfg, L, R, 20);

            if (!res.feasible) {
                std::cout << "[mode0] infeasible in [" << L << ", " << R << "]\n";
                continue;
            }

            has_best_tf = true;
            best_tf = res.best_tf;
            last_traj.valid = false;
            sample_gap_mode = 0;

            std::cout << "[mode0] best_tf=" << res.best_tf
                      << " best_m=" << res.best_m
                      << " solves=" << res.solve_calls
                      << " time=" << res.elapsed_sec << "s\n";

            // Request kOS to send fresh state as mode1 (fixed-format marker)
            {
                if (!atomic_write(recv_path, "COMPUTE_FINISH,1\n")) {
                    std::cerr << "Failed to write receive.txt\n";
                }
            }
        } else if (mode == 1 && has_best_tf) {
            // Populate cfg from current state, reuse best_tf
            GFOLDConfig cfg;
            cfg.tf = best_tf;
            cfg.g0 = 9.81;
            double base_time = 0.0;
            if (!populate_cfg_from_tokens(tok, cfg, &base_time)) {
                std::cerr << "Parse error on mode1 line: " << line << "\n";
                continue;
            }

            if (last_traj.valid && last_traj.steps == cfg.steps && last_traj.tf > 0.0) {
                const int idx = closest_prior_index(last_traj, cfg.r0);
                const double dt_prev = last_traj.tf / static_cast<double>(last_traj.steps);
                const double rem_tf = dt_prev * static_cast<double>(last_traj.steps - idx);
                if (rem_tf > 0.0) cfg.tf = rem_tf;
            }

            std::cout << std::fixed << std::setprecision(6);
            std::cout << "[mode1] cfg tf=" << cfg.tf
                      << " g0=" << cfg.g0
                      << " Isp=" << cfg.Isp
                      << " T_max=" << cfg.T_max
                      << " throt=[" << cfg.throttle_min << "," << cfg.throttle_max << "]"
                      << " m0=" << cfg.m0
                      << " r0=[" << cfg.r0[0] << "," << cfg.r0[1] << "," << cfg.r0[2] << "]"
                      << " v0=[" << cfg.v0[0] << "," << cfg.v0[1] << "," << cfg.v0[2] << "]"
                      << " glide=" << cfg.glide_slope_deg
                      << " theta=" << cfg.max_angle_deg
                      << " steps=" << cfg.steps
                      << "\n";

            GFOLDSolver solver(cfg);
            bool ok = solver.solve();
            if (!ok) {
                std::cerr << "[mode1] solve failed\n";
                continue;
            }

            const int steps = cfg.steps;
            const double dt = cfg.tf / static_cast<double>(steps);
            double* ux = CPG_Result.prim->u;
            double* uy = CPG_Result.prim->u + steps;
            double* uz = CPG_Result.prim->u + 2 * steps;
            double* rx = CPG_Result.prim->r;
            double* ry = CPG_Result.prim->r + steps;
            double* rz = CPG_Result.prim->r + 2 * steps;

            last_traj.steps = steps;
            last_traj.tf = cfg.tf;
            last_traj.rx.assign(rx, rx + steps);
            last_traj.ry.assign(ry, ry + steps);
            last_traj.rz.assign(rz, rz + steps);
            last_traj.valid = true;

            std::ostringstream oss;
            oss << std::setprecision(10) << std::fixed;
            int recv_lines = 0;
            oss << "COMPUTE_FINISH,1\n";
            recv_lines += 1;
            double last_t_abs = base_time;
            auto emit_indices = build_sample_indices(steps, sample_gaps[sample_gap_mode], max_lines);
            double tf_needed = 0.0;
            if (emit_indices.size() >= 2) {
                const int first_idx = emit_indices.front();
                const int last_idx = emit_indices.back();
                tf_needed = static_cast<double>(last_idx - first_idx) * dt;
            }

            while (tf_needed < recompute_time && (sample_gap_mode + 1) < sample_gaps.size()) {
                sample_gap_mode += 1;
                emit_indices = build_sample_indices(steps, sample_gaps[sample_gap_mode], max_lines);
                if (emit_indices.size() >= 2) {
                    const int first_idx = emit_indices.front();
                    const int last_idx = emit_indices.back();
                    tf_needed = static_cast<double>(last_idx - first_idx) * dt;
                } else {
                    tf_needed = 0.0;
                }
            }

            std::cout << "[mode1] sample_gap=" << sample_gaps[sample_gap_mode]
                      << " tf_needed=" << tf_needed
                      << " recompute_time=" << recompute_time << "\n";

            int lines_emitted = 0;
            for (int idx : emit_indices) {
                const double up = ux[idx];
                const double north = uy[idx];
                const double east = uz[idx];
                const double t_abs = base_time + (static_cast<double>(idx) * dt);
                last_t_abs = t_abs;
                const double mag = std::sqrt(up * up + north * north + east * east);
                std::cout << "[mode1] U[" << idx << "] up=" << up
                          << " north=" << north
                          << " east=" << east
                          << " mag=" << mag
                          << " t=" << t_abs << "\n";
                oss << "U," << up << "," << north << "," << east << "," << t_abs << "\n";
                recv_lines += 1;
                lines_emitted += 1;
            }
            // Append a zero-magnitude thrust command to signal end.
            if (lines_emitted == 0) {
                // ensure t_abs is defined even if no U lines were emitted
                last_t_abs = base_time;
            }
            const double end_t = last_t_abs + dt;
            oss << "U," << 0.0 << "," << 0.0 << "," << 0.0 << "," << end_t << "\n";
            recv_lines += 1;
            if (!atomic_write(recv_path, oss.str())) {
                std::cerr << "Failed to write receive.txt\n";
            }
            std::cout << "[mode1] profile written to receive.txt (lines=" << recv_lines << ")\n";
        }
    }

    return 0;
}
