#pragma once

#include "GFOLD_solver.hpp"

struct SearchResult {
    double best_tf = 0.0;
    double best_m  = -1.0;
    double elapsed_sec = 0.0;
    long long solve_calls = 0;
    bool feasible = false;
};

// Search for best tf in [a, b] (seconds) maximizing terminal mass.
SearchResult find_best_tf(const GFOLDConfig& cfg, double a, double b, int iters = 20);
