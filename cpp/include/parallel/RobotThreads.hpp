#pragma once
#include <vector>
#include <thread>
#include <functional>
#include "robot/Robot.hpp"
#include "map/SharedMap.hpp"

namespace mre {

// RobotThreads hiện thực đúng mô hình mục 6 báo cáo: mỗi robot là một luồng
// lập kế hoạch riêng (sense + propose path), chạy song song; ghi vào
// SharedMap được bảo vệ bởi mutex bên trong chính SharedMap. Việc cấp quyền
// di chuyển và cập nhật cuối cùng do Coordinator (chạy tuần tự) đảm nhiệm,
// đúng nguyên tắc "không để 5 thread tự ý ghi map và tự di chuyển cùng lúc".
class RobotThreads {
public:
    // Chạy song song bước sense + cập nhật SharedMap cho tất cả robot.
    static void runSensePhase(std::vector<Robot>& robots, SharedMap& shared) {
        std::vector<std::thread> threads;
        threads.reserve(robots.size());
        for (auto& r : robots) {
            threads.emplace_back([&r, &shared]() { r.senseAndUpdate(shared); });
        }
        for (auto& t : threads) t.join();
    }

    // Chạy song song bước lập kế hoạch A* + đề xuất di chuyển cho tất cả robot.
    // SharedMap chỉ bị ĐỌC ở bước này (qua các hàm canTraverse* có mutex riêng),
    // không có ghi xung đột nên an toàn để chạy song song.
    static void runPlanPhase(std::vector<Robot>& robots, SharedMap& shared) {
        std::vector<std::thread> threads;
        threads.reserve(robots.size());
        for (auto& r : robots) {
            threads.emplace_back([&r, &shared]() { r.planAndPropose(shared); });
        }
        for (auto& t : threads) t.join();
    }
};

} // namespace mre
