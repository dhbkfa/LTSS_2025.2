#pragma once
#include "map/GridMap.hpp"
#include "core/Types.hpp"
#include <vector>
#include <queue>
#include <unordered_set>

namespace mre {

// Tính các ô INFLATED_OBSTACLE xung quanh mỗi ô OCCUPIED đã biết, theo
// Chebyshev distance <= (robotRadius + safetyMargin). Áp dụng cho SharedMap
// (map robot đã biết), không áp dụng cho TruthMap.
class Inflation {
public:
    // robotPositions: vị trí hiện tại của tất cả robot - KHÔNG BAO GIỜ bị inflate thành vật cản,
    // vì robot đang đứng ở đó nghĩa là ô đó chắc chắn đi được (tránh tự chặn đường chính mình
    // khi nhiều robot đứng gần vật cản hoặc gần nhau).
    static void apply(GridMap& map, int robotRadiusCells, int safetyMarginCells,
                       const std::vector<Cell>& robotPositions = {}) {
        int total = robotRadiusCells + safetyMarginCells;
        if (total <= 0) return;

        int w = map.width();
        int h = map.height();
        std::vector<Cell> occupiedCells;
        for (int x = 0; x < w; ++x) {
            for (int y = 0; y < h; ++y) {
                if (map.at(x, y) == CellState::OCCUPIED) {
                    occupiedCells.push_back({x, y});
                }
            }
        }

        std::unordered_set<long long> protectedCells;
        for (const auto& p : robotPositions) {
            protectedCells.insert((static_cast<long long>(p.x) << 32) ^ static_cast<uint32_t>(p.y));
        }

        for (const auto& oc : occupiedCells) {
            for (int dx = -total; dx <= total; ++dx) {
                for (int dy = -total; dy <= total; ++dy) {
                    if (dx == 0 && dy == 0) continue;
                    // Chebyshev distance để vùng cấm là hình vuông quanh vật cản
                    int cheb = std::max(std::abs(dx), std::abs(dy));
                    if (cheb > total) continue;
                    int nx = oc.x + dx;
                    int ny = oc.y + dy;
                    if (!map.inBounds(nx, ny) || !map.isActive(nx, ny)) continue;
                    long long key = (static_cast<long long>(nx) << 32) ^ static_cast<uint32_t>(ny);
                    if (protectedCells.count(key)) continue; // có robot đang đứng - không inflate
                    if (map.at(nx, ny) == CellState::FREE || map.at(nx, ny) == CellState::VISITED) {
                        map.setState(nx, ny, CellState::INFLATED_OBSTACLE);
                    }
                }
            }
        }
    }
};

} // namespace mre
