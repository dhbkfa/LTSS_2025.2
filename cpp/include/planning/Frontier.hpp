#pragma once
#include <vector>
#include <set>
#include "core/Types.hpp"
#include "map/SharedMap.hpp"
#include "map/GridMap.hpp"

namespace mre {

// Frontier = "đích ảo" tiếp theo robot nên đi tới để mở rộng tri thức.
// Có 2 loại frontier, theo đúng tinh thần mục 4.1 + 4.2 báo cáo:
//   (a) Frontier mở rộng: ô đã biết (FREE/VISITED) kề với ít nhất 1 ô UNKNOWN.
//   (b) Frontier xác nhận: ô đã biết là FREE nhưng CHƯA có robot nào từng đứng lên
//       (chưa VISITED) — vì đích thật chỉ lộ ra khi robot đứng đúng ô đó (mục 4.2),
//       nên những ô FREE "nhìn thấy từ xa" qua cảm biến vẫn phải được ghé thăm
//       trước khi kết luận đã phủ hết map.
class Frontier {
public:
    struct FrontierCell {
        Cell knownCell;     // ô A* sẽ dẫn robot tới (chính là ô mục tiêu)
        Cell unknownCell;   // với loại (a): ô UNKNOWN kề bên; với loại (b): bằng knownCell
        int unknownNeighborCount = 0; // càng nhiều ô UNKNOWN xung quanh càng "lợi" khi xếp hạng
    };

    // Lấy snapshot rồi quét toàn bộ (đơn giản, đủ nhanh cho map kích thước vừa và nhỏ
    // trong phạm vi đồ án; có thể tối ưu bằng incremental frontier nếu map rất lớn).
    static std::vector<FrontierCell> findAll(SharedMap& map) {
        std::vector<FrontierCell> frontiers;
        GridMap snap = map.snapshot();
        int w = snap.width(), h = snap.height();

        for (int x = 0; x < w; ++x) {
            for (int y = 0; y < h; ++y) {
                if (!snap.isActive(x, y)) continue;
                CellState s = snap.at(x, y);
                Cell here{x, y};
                if (map.isMarkedUnreachable(here)) continue; // đã xác nhận không tới được - bỏ qua

                // Loại (b): ô FREE nhưng chưa từng được robot đứng lên -> phải ghé thăm
                // để xác nhận có phải đích hay không.
                if (s == CellState::FREE) {
                    frontiers.push_back({here, here, 0});
                    continue;
                }

                if (s != CellState::VISITED && s != CellState::TARGET) continue;

                Cell known{x, y};
                int unknownCount = 0;
                Cell firstUnknown{};
                bool foundOne = false;

                for (int d = 0; d < 4; ++d) {
                    if (snap.hasWall(known, d)) continue; // không thể đi qua hướng này -> không tính là frontier hướng đó
                    int nx = x + kDirX[d];
                    int ny = y + kDirY[d];
                    if (!snap.inBounds(nx, ny) || !snap.isActive(nx, ny)) continue;
                    if (snap.at(nx, ny) == CellState::UNKNOWN) {
                        unknownCount++;
                        if (!foundOne) {
                            firstUnknown = Cell{nx, ny};
                            foundOne = true;
                        }
                    }
                }

                if (foundOne) {
                    frontiers.push_back({known, firstUnknown, unknownCount});
                }
            }
        }
        return frontiers;
    }

    static bool anyFrontierLeft(SharedMap& map) {
        // Dừng sớm ngay khi thấy 1 frontier — rẻ hơn so với findAll đầy đủ.
        GridMap snap = map.snapshot();
        int w = snap.width(), h = snap.height();
        for (int x = 0; x < w; ++x) {
            for (int y = 0; y < h; ++y) {
                if (!snap.isActive(x, y)) continue;
                CellState s = snap.at(x, y);
                Cell here{x, y};
                if (map.isMarkedUnreachable(here)) continue;

                if (s == CellState::FREE) return true; // chưa visited -> còn việc phải làm

                if (s != CellState::VISITED && s != CellState::TARGET) continue;
                for (int d = 0; d < 4; ++d) {
                    if (snap.hasWall(x, y, d)) continue;
                    int nx = x + kDirX[d], ny = y + kDirY[d];
                    if (!snap.inBounds(nx, ny) || !snap.isActive(nx, ny)) continue;
                    if (snap.at(nx, ny) == CellState::UNKNOWN) return true;
                }
            }
        }
        return false;
    }
};

} // namespace mre
