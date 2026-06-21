#pragma once
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include "core/Types.hpp"
#include "map/SharedMap.hpp"

namespace mre {

// A* giải đúng bài toán: "từ vị trí hiện tại đến MỘT mục tiêu đã chọn,
// đường nào ngắn nhất" (mục 4.3 báo cáo). Nó KHÔNG tự quyết định mục tiêu —
// mục tiêu (frontier hoặc đích thật) do TaskAllocator/Coordinator gán trước.
class AStar {
public:
    struct Result {
        bool found = false;
        std::vector<Cell> path; // bao gồm cả ô bắt đầu, không bao gồm reroute; path[0] = start
    };

    // allowUnknownGoal: cho phép ô đích là UNKNOWN (trường hợp đích là 1 frontier
    // chưa khám phá, ô đó về bản chất là biên giữa FREE và UNKNOWN).
    static Result search(SharedMap& map, const Cell& start, const Cell& goal,
                          bool allowUnknownGoal = true) {
        Result result;
        if (start == goal) {
            result.found = true;
            result.path = {start};
            return result;
        }

        auto heuristic = [&](const Cell& a, const Cell& b) {
            // Manhattan distance phù hợp với di chuyển 4 hướng trên lưới.
            return std::abs(a.x - b.x) + std::abs(a.y - b.y);
        };

        struct PQNode {
            int f;
            Cell c;
            bool operator>(const PQNode& o) const { return f > o.f; }
        };

        std::priority_queue<PQNode, std::vector<PQNode>, std::greater<PQNode>> open;
        std::unordered_map<Cell, int, CellHash> gScore;
        std::unordered_map<Cell, Cell, CellHash> cameFrom;
        std::unordered_set<Cell, CellHash> closed;

        gScore[start] = 0;
        open.push({heuristic(start, goal), start});

        int guardSteps = map.width() * map.height() * 4 + 16;

        while (!open.empty() && guardSteps-- > 0) {
            Cell current = open.top().c;
            open.pop();

            if (closed.count(current)) continue;
            closed.insert(current);

            if (current == goal) {
                // dựng lại đường đi
                std::vector<Cell> path;
                Cell c = current;
                while (!(c == start)) {
                    path.push_back(c);
                    c = cameFrom[c];
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                result.found = true;
                result.path = std::move(path);
                return result;
            }

            for (int d = 0; d < 4; ++d) {
                bool canGo = allowUnknownGoal
                                 ? map.canTraverseAllowUnknownGoal(current, d, goal)
                                 : map.canTraverse(current, d);
                if (!canGo) continue;
                Cell next{current.x + kDirX[d], current.y + kDirY[d]};
                if (closed.count(next)) continue;

                int tentativeG = gScore[current] + 1;
                auto it = gScore.find(next);
                if (it == gScore.end() || tentativeG < it->second) {
                    gScore[next] = tentativeG;
                    cameFrom[next] = current;
                    open.push({tentativeG + heuristic(next, goal), next});
                }
            }
        }

        result.found = false;
        return result;
    }
};

} // namespace mre
