#pragma once
#include <vector>
#include <unordered_set>
#include <algorithm>
#include <limits>
#include "core/Types.hpp"
#include "planning/Frontier.hpp"
#include "robot/RobotState.hpp"

namespace mre {

// Phân công mỗi robot đang ở pha EXPLORING một frontier riêng (đích ảo),
// ưu tiên gần robot đó nhất (mục 4.1: "có thể hướng về hướng robot để tìm
// kiếm nhanh hơn") và tránh nhiều robot cùng nhắm 1 frontier khi còn lựa chọn khác.
class TaskAllocator {
public:
    static void assignFrontiers(std::vector<RobotState>& robots,
                                 const std::vector<Frontier::FrontierCell>& frontiers,
                                 std::vector<Cell>& outAssigned) {
        outAssigned.assign(robots.size(), Cell{-1, -1});
        if (frontiers.empty()) return;

        std::unordered_set<long long> takenFrontierIdx;

        // Sắp xếp robot theo id để có thứ tự xác định (deterministic) khi phân công.
        std::vector<int> order(robots.size());
        for (size_t i = 0; i < robots.size(); ++i) order[i] = static_cast<int>(i);

        for (int ri : order) {
            RobotState& r = robots[ri];
            if (r.phase != RobotPhase::EXPLORING) continue;

            int bestIdx = -1;
            int bestScore = std::numeric_limits<int>::max();
            bool avoiding = (r.avoidTicks > 0);

            for (size_t fi = 0; fi < frontiers.size(); ++fi) {
                if (takenFrontierIdx.count(static_cast<long long>(fi)) && frontiers.size() > robots.size()) {
                    // Nếu còn đủ frontier chưa ai nhận thì tránh trùng; nếu frontier ít hơn số robot,
                    // cho phép trùng (robot khác nhau cùng hướng tới các phần khác nhau của cùng vùng mở).
                    continue;
                }
                const auto& f = frontiers[fi];

                // Tránh tạm thời ô vừa gây livelock cho robot này (chống đói tài nguyên):
                // robot vừa bị kẹt quá lâu khi nhắm ô này sẽ thử hướng khác trong vài vòng tới.
                if (avoiding && f.knownCell == r.avoidCell) continue;

                int dist = std::abs(f.knownCell.x - r.pos.x) + std::abs(f.knownCell.y - r.pos.y);
                // Ưu tiên gần; phần thưởng nhỏ cho frontier có nhiều ô UNKNOWN xung quanh (mở rộng nhanh hơn).
                int score = dist * 10 - f.unknownNeighborCount;
                if (score < bestScore) {
                    bestScore = score;
                    bestIdx = static_cast<int>(fi);
                }
            }

            // Nếu vì đang tránh avoidCell mà không còn lựa chọn nào khác, đành chấp nhận lại
            // avoidCell (tốt hơn là đứng im không làm gì) — thử lại toàn bộ danh sách không loại trừ.
            if (bestIdx < 0 && avoiding) {
                for (size_t fi = 0; fi < frontiers.size(); ++fi) {
                    const auto& f = frontiers[fi];
                    int dist = std::abs(f.knownCell.x - r.pos.x) + std::abs(f.knownCell.y - r.pos.y);
                    int score = dist * 10 - f.unknownNeighborCount;
                    if (score < bestScore) {
                        bestScore = score;
                        bestIdx = static_cast<int>(fi);
                    }
                }
            }

            if (bestIdx >= 0) {
                takenFrontierIdx.insert(bestIdx);
                outAssigned[ri] = frontiers[bestIdx].knownCell;
            }
        }
    }
};

} // namespace mre
