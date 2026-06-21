// main.cpp - Entry point cho multi-robot exploration simulator.
//
// Cách dùng:
//   simulator <map_json_path> <output_dir> [robot_radius] [safety_margin] [max_timesteps]
//
// Ví dụ:
//   simulator ../config/maps/map_random_01.json ../python/run_output 0 0 5000
//
// C++ giữ TOÀN BỘ logic mô phỏng (TruthMap, SharedMap, 5 robot, A*, frontier,
// collision, thread). Python chỉ đọc các file step_XXXXXX.json trong output_dir
// để vẽ UI, đúng kiến trúc tách biệt trong báo cáo (mục 9).

#include <iostream>
#include <string>
#include <chrono>
#include "map/TruthMap.hpp"
#include "coordination/Coordinator.hpp"
#include "core/Types.hpp"

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Cach dung: " << argv[0]
                  << " <map_json_path> <output_dir> [robot_radius=0] [safety_margin=0] [max_timesteps=200000]\n";
        return 1;
    }

    std::string mapPath = argv[1];
    std::string outDir = argv[2];
    int robotRadius = (argc > 3) ? std::stoi(argv[3]) : 0;
    int safetyMargin = (argc > 4) ? std::stoi(argv[4]) : 0;
    int maxTimesteps = (argc > 5) ? std::stoi(argv[5]) : mre::Constants::kMaxTimesteps;

    try {
        std::cout << "[simulator] Dang doc map tu: " << mapPath << "\n";
        mre::TruthMap truth = mre::TruthMap::loadFromFile(mapPath);
        std::cout << "[simulator] Map kich thuoc " << truth.width() << "x" << truth.height()
                  << ", so vi tri xuat phat: " << truth.startPositions().size()
                  << ", co dich that: " << (truth.hasTarget() ? "co" : "khong") << "\n";

        mre::Coordinator coordinator(truth, outDir, robotRadius, safetyMargin);
        std::cout << "[simulator] So robot khoi tao: " << coordinator.numRobots() << "\n";

        auto t0 = std::chrono::steady_clock::now();
        mre::SimStatus finalStatus = coordinator.runAll(maxTimesteps);
        auto t1 = std::chrono::steady_clock::now();
        double seconds = std::chrono::duration<double>(t1 - t0).count();

        std::cout << "[simulator] Ket thuc sau " << coordinator.currentTimestep()
                  << " timesteps (" << seconds << "s). Trang thai: "
                  << mre::UiProtocol::statusStr(finalStatus) << "\n";

    } catch (const std::exception& e) {
        std::cerr << "[simulator][LOI] " << e.what() << "\n";
        return 1;
    }

    return 0;
}
