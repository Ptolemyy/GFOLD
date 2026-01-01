#include "GFOLD_solver.hpp"
#include "find_tf_2.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

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

static bool populate_cfg_from_tokens(const std::vector<std::string>& tok, GFOLDConfig& cfg) {
    // Expect at least: MODE,E,N,U,VE,VN,VU,SPEED,THRUST_MAX,ISP,MASS_WET
    if (tok.size() < 11) return false;
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
    } catch (...) {
        return false;
    }
    return true;
}

int main() {
    // Adjust this path to your KSP Scripts folder if needed.
    const fs::path send_path =
        R"(C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program\Ships\Script\send.txt)";

    // Clear old content to avoid stale reads
    {
        std::error_code ec;
        fs::remove(send_path, ec);
        std::ofstream clear(send_path, std::ios::trunc);
    }

    std::string last_line;
    while (true) {
        std::this_thread::sleep_for(20ms);

        std::string line = read_last_line(send_path);
        if (line.empty() || line == last_line) continue;
        last_line = line;

        auto tok = split_csv(line);
        if (tok.empty()) continue;

        int mode = 0;
        try { mode = std::stoi(tok[0]); } catch (...) { continue; }

        if (mode != 0) continue; // only handle mode 0 for tf search

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

        std::cout << "[mode0] best_tf=" << res.best_tf
                  << " best_m=" << res.best_m
                  << " solves=" << res.solve_calls
                  << " time=" << res.elapsed_sec << "s\n";
    }

    return 0;
}
