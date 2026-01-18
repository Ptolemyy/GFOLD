#include "GFOLD_solver.hpp"
#include "find_tf_2.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
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

static bool populate_cfg_from_tokens(const std::vector<std::string>& tok, GFOLDConfig& cfg) {
    // Expect at least: MODE,E,N,U,VE,VN,VU,SPEED,THRUST_MAX,ISP,MASS_WET
    if (tok.size() < 11) return false;
    // Optional extra fields:
    // [11]=THROT1, [12]=THROT2, [13]=THETA_DEG, [14]=YGS_DEG, [15]=RECOMPUTE_TIME
    try {
        // Solver assumes x-axis is altitude. Map ENU accordingly:
        cfg.r0[0] = std::stod(tok[3]); // U -> x (altitude)
        cfg.r0[1] = std::stod(tok[2]); // N -> y
        cfg.r0[2] = std::stod(tok[1]); // E -> z

        cfg.v0[0] = std::stod(tok[6]); // VU -> vx
        cfg.v0[1] = std::stod(tok[5]); // VN -> vy
        cfg.v0[2] = std::stod(tok[4]); // VE -> vz

        cfg.T_max = std::stod(tok[8]) * 1000.0;   // kN -> N
        cfg.Isp   = std::stod(tok[9]);            // s
        cfg.m0    = std::stod(tok[10]) * 1000.0;  // t -> kg

        if (tok.size() > 11) cfg.throttle_min = std::stod(tok[11]);
        if (tok.size() > 12) cfg.throttle_max = std::stod(tok[12]);
        if (tok.size() > 13) cfg.max_angle_deg = std::stod(tok[13]);
        if (tok.size() > 14) cfg.glide_slope_deg = std::stod(tok[14]);
        // We parse recompute_time if provided (index 15) but do not use it yet.
        if (tok.size() > 15) {
            // Placeholder: parsed but unused for now
            double recompute_time = std::stod(tok[15]);
            (void)recompute_time;
        }
    } catch (...) {
        return false;
    }
    return true;
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
    bool mode1_handled = false;

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
            if (!populate_cfg_from_tokens(tok, cfg)) {
                std::cerr << "Parse error on line: " << line << "\n";
                continue;
            }

            const double L = 10.0, R = 90.0;
            SearchResult res = find_best_tf(cfg, L, R, 20);

            if (!res.feasible) {
                std::cout << "[mode0] infeasible in [" << L << ", " << R << "]\n";
                continue;
            }

            has_best_tf = true;
            best_tf = res.best_tf;
            mode1_handled = false;

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
        } else if (mode == 1 && has_best_tf && !mode1_handled) {
            // Populate cfg from current state, reuse best_tf
            GFOLDConfig cfg;
            cfg.tf = best_tf;
            cfg.g0 = 9.81;
            if (!populate_cfg_from_tokens(tok, cfg)) {
                std::cerr << "Parse error on mode1 line: " << line << "\n";
                continue;
            }

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

            std::ostringstream oss;
            oss << std::setprecision(10) << std::fixed;
            oss << "COMPUTE_FINISH,1\n";
            for (int i = 0; i < steps; ++i) {
                oss << "U," << ux[i] << "," << uy[i] << "," << uz[i] << "\n";
            }
            oss << "DT," << dt << "\n";
            if (!atomic_write(recv_path, oss.str())) {
                std::cerr << "Failed to write receive.txt\n";
            }
            mode1_handled = true;
            std::cout << "[mode1] profile written to receive.txt\n";
        }
    }

    return 0;
}
