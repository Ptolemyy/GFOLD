#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

namespace fs = std::filesystem;

// 读整个文件
std::string read_all(const fs::path& p) {
    std::ifstream in(p);
    if (!in) return "";
    std::string s((std::istreambuf_iterator<char>(in)),
                  std::istreambuf_iterator<char>());
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r'))
        s.pop_back();
    return s;
}

// 原子写：tmp → rename
bool atomic_write(const fs::path& p, const std::string& content) {
    fs::path tmp = p;
    tmp += ".tmp";

    std::ofstream out(tmp, std::ios::trunc);
    if (!out) return false;
    out << content << "\n";
    out.close();

    std::error_code ec;
    fs::remove(p, ec);
    fs::rename(tmp, p, ec);
    return !ec;
}

long long now_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count();
}

int main() {
    // ★ 你的真实路径 ★
    fs::path comm =
        R"(C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program\Ships\Script\comm.txt)";

    // 生成唯一 ID
    std::string id = std::to_string(now_ms());
    long long t_send = now_ms();

    std::string ping = "PING," + id + "," + std::to_string(t_send);

    // 写 PING
    if (!atomic_write(comm, ping)) {
        std::cerr << "Write failed. Try run as Administrator.\n";
        return 1;
    }

    std::cout << "Sent: " << ping << std::endl;

    // 等待 PONG
    const int timeout_ms = 5000;
    long long deadline = now_ms() + timeout_ms;

    while (now_ms() < deadline) {
        std::string s = read_all(comm);

        std::string expect = "PONG," + id + ",";
        if (s.rfind(expect, 0) == 0) {
            long long t_recv = now_ms();
            std::cout << "Recv: " << s << std::endl;
            std::cout << "RTT(ms): " << (t_recv - t_send) << std::endl;
                system("pause");
            return 0;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cerr << "Timeout waiting for PONG\n";
    system("pause");
    return 2;
}
