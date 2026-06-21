#pragma once
#include "robot/RobotState.hpp"
#include "sensing/SensorModel.hpp"
#include "map/SharedMap.hpp"
#include "planning/AStar.hpp"
#include "core/Types.hpp"

namespace mre {

// Robot đại diện cho một thread lập kế hoạch độc lập (theo mục 6 báo cáo:
// "Thread robot i: sense + propose path"). Robot KHÔNG tự quyết định việc
// di chuyển cuối cùng — chỉ đề xuất ô tiếp theo, Coordinator mới là nơi
// cấp quyền di chuyển thật sự, sau khi giải quyết xung đột.
class Robot {
public:
    Robot(int id, const Cell& startPos, const SensorModel& sensor)
        : sensor_(sensor) {
        state_.id = id;
        state_.pos = startPos;
    }

    const RobotState& state() const { return state_; }
    RobotState& mutableState() { return state_; }

    // Bước 1+2: cảm biến tại vị trí hiện tại và cập nhật SharedMap.
    void senseAndUpdate(SharedMap& shared) {
        auto result = sensor_.sense(state_.pos);
        lastSenseDetectedTarget_ = result.isTargetDetected;
        sensor_.applyToSharedMap(result, shared);
    }

    bool lastSenseDetectedTarget() const { return lastSenseDetectedTarget_; }

    // Gán mục tiêu mới (frontier hoặc đích thật) cho robot.
    void assignGoal(const Cell& goal, RobotPhase phase) {
        state_.currentGoal = goal;
        state_.hasGoal = true;
        state_.phase = phase;
        state_.plannedPath.clear();
        state_.pathIndex = 0;
    }

    void clearGoal() {
        state_.hasGoal = false;
        state_.plannedPath.clear();
        state_.pathIndex = 0;
    }

    // Bước 6: chạy A* tới mục tiêu hiện tại, đề xuất ô tiếp theo để di chuyển.
    // Trả về false nếu không tìm được đường (mục tiêu tạm thời không khả thi nữa).
    bool planAndPropose(SharedMap& shared) {
        state_.hasProposal = false;

        if (state_.waitTicks > 0) {
            // Đang phải chờ do nhường đường - không đề xuất di chuyển.
            return true;
        }

        if (!state_.hasGoal) return true;

        if (state_.phase == RobotPhase::UNREACHABLE) {
            // Robot đã xác định không thể tới đích (bị cô lập) - không cần lập kế hoạch nữa.
            return true;
        }

        if (state_.pos == state_.currentGoal) {
            state_.phase = (state_.phase == RobotPhase::GOING_TO_TARGET)
                                ? RobotPhase::ARRIVED
                                : state_.phase;
            state_.pathFailCount = 0;
            return true;
        }

        // Nếu mục tiêu hiện tại (frontier hoặc đích) đã bị nơi khác xác nhận là không tới được
        // (do robot khác từng thất bại đủ ngưỡng), bỏ ngay không cần thử lại.
        if (shared.isMarkedUnreachable(state_.currentGoal)) {
            if (state_.phase == RobotPhase::GOING_TO_TARGET) {
                state_.phase = RobotPhase::UNREACHABLE;
            }
            return false;
        }

        bool allowUnknown = (state_.phase == RobotPhase::EXPLORING);
        auto result = AStar::search(shared, state_.pos, state_.currentGoal, allowUnknown);
        if (!result.found || result.path.size() < 2) {
            state_.pathFailCount++;
            // Ngưỡng đủ lớn để loại trừ khả năng map đang cập nhật tạm thời (vd: robot khác
            // vừa che mất 1 đoạn đường) trước khi kết luận mục tiêu này thực sự không tới được.
            if (state_.pathFailCount >= kUnreachableThreshold) {
                shared.markUnreachable(state_.currentGoal);
                if (state_.phase == RobotPhase::GOING_TO_TARGET) {
                    state_.phase = RobotPhase::UNREACHABLE;
                }
                state_.pathFailCount = 0;
            }
            return false; // mục tiêu không còn khả thi (đã bị robot khác chặn/đổi trạng thái)
        }

        state_.pathFailCount = 0;
        state_.plannedPath = result.path;
        state_.pathIndex = 1; // path[0] = vị trí hiện tại
        state_.proposedNextCell = result.path[1];
        state_.hasProposal = true;
        return true;
    }

    // Bước 8 (chỉ thực thi khi Coordinator đã cấp quyền): di chuyển robot sang ô đã đề xuất.
    void commitMove(const Cell& to) {
        state_.pos = to;
        state_.totalSteps++;
        if (state_.pathIndex < state_.plannedPath.size()) {
            state_.pathIndex++;
        }
    }

    void stayInPlace() {
        // không di chuyển nhịp này (do nhường đường hoặc không có đề xuất)
    }

    void setWait(int ticks) { state_.waitTicks = ticks; }
    void tickWait() { if (state_.waitTicks > 0) state_.waitTicks--; }

private:
    static constexpr int kUnreachableThreshold = 5;
    RobotState state_;
    const SensorModel& sensor_;
    bool lastSenseDetectedTarget_ = false;
};

} // namespace mre
