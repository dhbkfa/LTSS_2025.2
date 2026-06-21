#pragma once
#include <vector>
#include <memory>
#include "robot/Robot.hpp"
#include "map/SharedMap.hpp"
#include "map/TruthMap.hpp"
#include "sensing/SensorModel.hpp"
#include "planning/Frontier.hpp"
#include "coordination/TaskAllocator.hpp"
#include "coordination/ReservationTable.hpp"
#include "parallel/RobotThreads.hpp"
#include "io/JsonLogger.hpp"
#include "io/UiProtocol.hpp"
#include "core/Types.hpp"
#include "core/Constants.hpp"

namespace mre {

// Coordinator hiện thực đúng vòng lặp chính mục 5.2 báo cáo:
//   1. sense  2. update SharedMap  3. check đứng trên đích
//   4. tìm frontier  5. phân công  6. A*  7. collision  8. move  9. UI update
// và điều kiện dừng Mode 3 (mục 5.3): có đích thì dừng khi tìm thấy & robot
// khác hội tụ về đích; không có đích thì dừng khi phủ hết map.
class Coordinator {
public:
    Coordinator(const TruthMap& truth, const std::string& outDir,
                int robotRadius = Constants::kDefaultRobotRadiusCells,
                int safetyMargin = Constants::kDefaultSafetyMarginCells)
        : truth_(truth),
          shared_(truth.width(), truth.height()),
          sensor_(truth, Constants::kSensorRange),
          logger_(outDir),
          robotRadius_(robotRadius),
          safetyMargin_(safetyMargin) {
        const auto& starts = truth.startPositions();
        int n = std::min<int>(Constants::kNumRobots, static_cast<int>(starts.size()));
        for (int i = 0; i < n; ++i) {
            robots_.emplace_back(i, starts[i], sensor_);
        }
    }

    int numRobots() const { return static_cast<int>(robots_.size()); }

    // Chạy toàn bộ mô phỏng đến khi đạt điều kiện dừng hoặc chạm giới hạn an toàn.
    SimStatus runAll(int maxTimesteps = Constants::kMaxTimesteps) {
        SimStatus status = SimStatus::RUNNING;
        for (timestep_ = 0; timestep_ < maxTimesteps; ++timestep_) {
            status = stepOnce();
            writeCurrentState(status);
            if (status != SimStatus::RUNNING) break;
        }
        logger_.writeMeta(timestep_, true);
        return status;
    }

    // Chạy đúng 1 timestep theo 9 bước, trả về trạng thái mô phỏng sau bước đó.
    SimStatus stepOnce() {
        // Bước 1+2: 5 robot cảm biến song song + cập nhật SharedMap (có mutex bên trong).
        RobotThreads::runSensePhase(robots_, shared_);

        // Inflate vật cản mới phát hiện theo bán kính robot + khoảng an toàn (mục 2 báo cáo).
        // Vị trí robot hiện tại luôn được loại trừ để tránh tự chặn đường chính mình.
        std::vector<Cell> robotPositions;
        robotPositions.reserve(robots_.size());
        for (auto& r : robots_) robotPositions.push_back(r.state().pos);
        shared_.applyInflation(robotRadius_, safetyMargin_, robotPositions);

        // Bước 3: kiểm tra robot nào vừa phát hiện đích thật.
        bool targetKnown = shared_.isTargetKnown();

        // Giảm dần thời gian "tránh ô vừa gây livelock" của từng robot.
        for (auto& r : robots_) {
            if (r.mutableState().avoidTicks > 0) r.mutableState().avoidTicks--;
        }

        if (targetKnown) {
            // Mode 1 trong Mode 3: mọi robot chuyển sang hội tụ về đích thật,
            // trừ robot đã ARRIVED hoặc đã xác định UNREACHABLE (bị cô lập, không có đường tới).
            Cell target = shared_.knownTarget();
            for (auto& r : robots_) {
                if (r.state().phase != RobotPhase::ARRIVED &&
                    r.state().phase != RobotPhase::UNREACHABLE) {
                    if (!r.state().hasGoal || r.state().currentGoal != target ||
                        r.state().phase == RobotPhase::EXPLORING) {
                        r.assignGoal(target, RobotPhase::GOING_TO_TARGET);
                    }
                }
            }
        } else {
            // Bước 4: tìm frontier.
            auto frontiers = Frontier::findAll(shared_);

            // Bước 5: phân công frontier cho từng robot đang ở pha EXPLORING.
            std::vector<RobotState> snapshotStates;
            snapshotStates.reserve(robots_.size());
            for (auto& r : robots_) snapshotStates.push_back(r.state());

            std::vector<Cell> assigned;
            TaskAllocator::assignFrontiers(snapshotStates, frontiers, assigned);

            for (size_t i = 0; i < robots_.size(); ++i) {
                if (assigned[i].x >= 0) {
                    auto& curState = robots_[i].mutableState();
                    if (!curState.hasGoal || curState.currentGoal != assigned[i]) {
                        robots_[i].assignGoal(assigned[i], RobotPhase::EXPLORING);
                    }
                } else if (robots_[i].state().phase == RobotPhase::EXPLORING) {
                    // không còn frontier phù hợp để giao cho robot này lúc này
                    robots_[i].clearGoal();
                }
            }
        }

        // Bước 6: mỗi robot chạy A* tới mục tiêu hiện tại (song song).
        RobotThreads::runPlanPhase(robots_, shared_);

        // Robot không tìm được đường tới mục tiêu hiện tại -> bỏ mục tiêu để vòng sau gán lại.
        for (auto& r : robots_) {
            if (r.state().hasGoal && !r.state().hasProposal &&
                r.state().pos != r.state().currentGoal && r.state().waitTicks == 0) {
                r.clearGoal();
            }
        }

        // Bước 7: kiểm tra & giải quyết va chạm.
        std::vector<RobotState> states;
        states.reserve(robots_.size());
        for (auto& r : robots_) states.push_back(r.state());
        bool tgtKnown = shared_.isTargetKnown();
        Cell tgtCell = tgtKnown ? shared_.knownTarget() : Cell{};
        auto decision = ReservationTable::resolve(states, tgtKnown, tgtCell);

        // Bước 8: robot di chuyển 1 ô nếu được cấp quyền, ngược lại chờ 1 nhịp.
        for (size_t i = 0; i < robots_.size(); ++i) {
            auto& r = robots_[i];
            if (r.state().waitTicks > 0) {
                r.tickWait();
                continue;
            }
            if (r.state().hasProposal) {
                if (decision.allowedToMove[i]) {
                    r.commitMove(r.state().proposedNextCell);
                    r.mutableState().rejectStreak = 0;
                } else {
                    // Nhường đường: chờ đúng 1 nhịp rồi A* sẽ tự tính lại đường ở bước sau.
                    r.setWait(1);
                    r.mutableState().rejectStreak++;
                    // Chống livelock (vd 2 robot kẹt trong hành lang hẹp, cứ luân phiên
                    // nhường nhau mãi mà không robot nào tiến triển): nếu 1 robot bị từ
                    // chối di chuyển quá nhiều lần liên tiếp, buộc nó bỏ mục tiêu hiện tại
                    // để vòng sau được gán một mục tiêu/đường đi khác.
                    if (r.mutableState().rejectStreak >= kLivelockThreshold) {
                        Cell stuckGoal = r.state().currentGoal;
                        r.clearGoal();
                        r.mutableState().rejectStreak = 0;
                        r.mutableState().avoidCell = stuckGoal;
                        r.mutableState().avoidTicks = kLivelockExtraWait;
                        // Chờ thêm vài nhịp (thay vì chỉ 1) để robot đang chắn đường có đủ
                        // thời gian tự di chuyển đi nơi khác trước khi robot này thử lại.
                        r.setWait(kLivelockExtraWait);
                    }
                }
            }
        }

        // Kiểm tra điều kiện dừng Mode 3.
        if (shared_.isTargetKnown()) {
            bool allDone = true;
            Cell target = shared_.knownTarget();
            for (auto& r : robots_) {
                bool atTarget = (r.state().pos == target);
                bool unreachable = (r.state().phase == RobotPhase::UNREACHABLE);
                if (!atTarget && !unreachable) { allDone = false; break; }
            }
            if (allDone) return SimStatus::TARGET_FOUND;
        } else {
            if (!Frontier::anyFrontierLeft(shared_)) {
                return SimStatus::MAP_COVERED;
            }
        }

        return SimStatus::RUNNING;
    }

    void writeCurrentState(SimStatus status) {
        std::vector<RobotState> states;
        states.reserve(robots_.size());
        for (auto& r : robots_) states.push_back(r.state());
        auto j = UiProtocol::buildStateJson(timestep_, status, shared_, states);
        logger_.writeStep(timestep_, j);
    }

    int currentTimestep() const { return timestep_; }

private:
    static constexpr int kLivelockThreshold = 4;
    static constexpr int kLivelockExtraWait = 3;
    const TruthMap& truth_;
    SharedMap shared_;
    SensorModel sensor_;
    std::vector<Robot> robots_;
    JsonLogger logger_;
    int robotRadius_;
    int safetyMargin_;
    int timestep_ = 0;
};

} // namespace mre
