#define NOMINMAX
#include <matplotlibcpp17/mplot3d.h>
#include <matplotlibcpp17/pyplot.h>
#include <pybind11/embed.h>

#include <algorithm>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace py = pybind11;

struct XYZSeries {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
};

struct ENUVelSeries {
    std::vector<double> t;
    std::vector<double> vu;
    std::vector<double> vn;
    std::vector<double> ve;
};

static XYZSeries load_xyz_csv(const std::string& path) {
    XYZSeries s;
    std::ifstream in(path);
    if (!in) return s;

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string a, b, c;
        if (!std::getline(ss, a, ',')) continue;
        if (!std::getline(ss, b, ',')) continue;
        if (!std::getline(ss, c, ',')) continue;
        try {
            s.x.push_back(std::stod(a));
            s.y.push_back(std::stod(b));
            s.z.push_back(std::stod(c));
        } catch (...) {
            // Ignore malformed lines.
        }
    }
    return s;
}

static ENUVelSeries load_v_enu_t_csv(const std::string& path) {
    ENUVelSeries s;
    std::ifstream in(path);
    if (!in) return s;

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string a, b, c, d;
        if (!std::getline(ss, a, ',')) continue;
        if (!std::getline(ss, b, ',')) continue;
        if (!std::getline(ss, c, ',')) continue;
        if (!std::getline(ss, d, ',')) continue;
        try {
            s.t.push_back(std::stod(a));
            s.vu.push_back(std::stod(b));
            s.vn.push_back(std::stod(c));
            s.ve.push_back(std::stod(d));
        } catch (...) {
            // Ignore malformed lines.
        }
    }
    return s;
}

static void ensure_python_interpreter() {
    static std::unique_ptr<py::scoped_interpreter> guard;
    if (!guard) {
        guard = std::make_unique<py::scoped_interpreter>();
    }
}

void plot_recv_mode0_from_csv(
    const std::string& recv_csv,
    const std::string& mode0_csv,
    const std::string& recv_vel_csv) {
    const XYZSeries recv = load_xyz_csv(recv_csv);
    const XYZSeries mode0 = load_xyz_csv(mode0_csv);
    const ENUVelSeries vel = load_v_enu_t_csv(recv_vel_csv);

    ensure_python_interpreter();
    py::module_::import("matplotlib.pyplot").attr("close")("all");
    auto plt = matplotlibcpp17::pyplot::import();
    auto fig = plt.figure(Args(), Kwargs("figsize"_a = py::make_tuple(11, 10)));
    auto ax = fig.add_subplot(Args(2, 1, 1), Kwargs("projection"_a = "3d"));

    if (!mode0.x.empty()) {
        ax.plot(Args(mode0.x, mode0.y, mode0.z),
                Kwargs("label"_a = "best traj", "linewidth"_a = 2.0, "color"_a = "tab:blue"));
    }
    if (!recv.x.empty()) {
        ax.plot(Args(recv.x, recv.y, recv.z),
                Kwargs("label"_a = "real traj", "linewidth"_a = 2.0, "color"_a = "tab:orange"));
    }

    bool has_any = !mode0.x.empty() || !recv.x.empty();
    if (has_any) {
        double xmin = 0.0, xmax = 0.0;
        double ymin = 0.0, ymax = 0.0;
        double zmin = 0.0, zmax = 0.0;
        bool inited = false;

        auto update_bounds = [&](const XYZSeries& s) {
            for (size_t i = 0; i < s.x.size(); ++i) {
                if (!inited) {
                    xmin = xmax = s.x[i];
                    ymin = ymax = s.y[i];
                    zmin = zmax = s.z[i];
                    inited = true;
                } else {
                    xmin = std::min(xmin, s.x[i]);
                    xmax = std::max(xmax, s.x[i]);
                    ymin = std::min(ymin, s.y[i]);
                    ymax = std::max(ymax, s.y[i]);
                    zmin = std::min(zmin, s.z[i]);
                    zmax = std::max(zmax, s.z[i]);
                }
            }
        };

        update_bounds(mode0);
        update_bounds(recv);

        if (inited) {
            const double xmid = 0.5 * (xmin + xmax);
            const double ymid = 0.5 * (ymin + ymax);
            const double zmid = 0.5 * (zmin + zmax);
            const double xspan = xmax - xmin;
            const double yspan = ymax - ymin;
            const double zspan = zmax - zmin;
            const double half = 0.5 * std::max({xspan, yspan, zspan, 1e-6});

            // Fix limits/aspect so matplotlib does not auto-rescale axes differently.
            ax.set_xlim(Args(xmid - half, xmid + half));
            ax.set_ylim(Args(ymid - half, ymid + half));
            ax.set_zlim(Args(zmid - half, zmid + half));
        }
    }

    ax.set_xlabel(Args("Up"));
    ax.set_ylabel(Args("North"));
    ax.set_zlabel(Args("East"));
    ax.set_title(Args("Received R and Mode0 Trajectory"));
    ax.grid(Args(true));
    ax.legend();

    auto ax2 = fig.add_subplot(Args(2, 1, 2));
    if (!vel.t.empty()) {
        ax2.plot(Args(vel.t, vel.vu), Kwargs("label"_a = "v_up", "linewidth"_a = 1.8, "color"_a = "tab:red"));
        ax2.plot(Args(vel.t, vel.vn), Kwargs("label"_a = "v_north", "linewidth"_a = 1.8, "color"_a = "tab:green"));
        ax2.plot(Args(vel.t, vel.ve), Kwargs("label"_a = "v_east", "linewidth"_a = 1.8, "color"_a = "tab:purple"));
    }
    ax2.set_title(Args("Recv ENU Velocity vs Time"));
    ax2.set_xlabel(Args("t (s)"));
    ax2.set_ylabel(Args("v (m/s)"));
    ax2.grid(Args(true));
    ax2.legend();

    plt.show();
}
