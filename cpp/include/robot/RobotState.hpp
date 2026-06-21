#pragma once
#include <vector>
#include "core/Types.hpp"

namespace mre {

enum class RobotPhase : uint8_t {
    EXPLORING = 0,      // đang đi tới frontier được phân công
    GOING_TO_TARGET = 1, // đã biết đích thật, đang đi tới đích
    ARRIVED = 2,         // đã tới đích thật hoặc hết việc để làm
    WAITING = 3,          // đang chờ 1 nhịp để tránh va chạm
    UNREACHABLE = 4        // đã biết đích nhưng robot này bị cô lập, không có đường tới (không chặn việc dừng)
};

struct RobotState {
    int id = -1;
    Cell pos{};
    RobotPhase phase = RobotPhase::EXPLORING;

    Cell currentGoal{};         // frontier hoặc đích thật đang nhắm tới
    bool hasGoal = false;
    std::vector<Cell> plannedPath; // đường A* hiện tại, plannedPath[0] = pos hiện tại
    size_t pathIndex = 0;            // chỉ số bước tiếp theo trong plannedPath

    Cell proposedNextCell{};    // ô robot đề xuất di chuyển tới trong timestep này
    bool hasProposal = false;

    int waitTicks = 0;           // số nhịp còn phải chờ do nhường đường
    long long totalSteps = 0;    // tổng số bước đã di chuyển (thống kê)
    int pathFailCount = 0;        // số lần A* thất bại liên tiếp khi đi tới đích thật (phát hiện bị cô lập)
    int rejectStreak = 0;          // số lần liên tiếp bị ReservationTable từ chối di chuyển (chống đói tài nguyên / livelock)
    Cell avoidCell{-1, -1};       // ô tạm thời bị tránh (do vừa gây livelock) - không gán làm goal trong vài vòng tới
    int avoidTicks = 0;            // số timestep còn lại áp dụng avoidCell
};

} // namespace mre
