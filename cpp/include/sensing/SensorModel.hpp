#pragma once
#include "map/TruthMap.hpp"
#include "map/SharedMap.hpp"
#include "core/Types.hpp"
#include "core/Constants.hpp"

namespace mre {

// SensorModel là nơi DUY NHẤT được phép đọc TruthMap. Nó mô phỏng một cảm
// biến cục bộ: tại vị trí hiện tại của robot, trả về tường 4 hướng và
// trạng thái free/occupied của các ô lân cận trong bán kính cảm biến.
// Đồng thời đây là nơi duy nhất phát ra tín hiệu "isTargetDetected" — đúng
// theo mục 4.2 của báo cáo: không suy luận đích thật từ lịch sử di chuyển.
class SensorModel {
public:
    struct SenseResult {
        Cell position;
        bool wallDir[4] = {false, false, false, false}; // có tường ở từng hướng không
        std::vector<Cell> freeCellsNearby;                // các ô lân cận xác định là FREE
        std::vector<Cell> occupiedCellsNearby;             // các ô lân cận xác định là OCCUPIED
        bool isTargetDetected = false;                     // true nếu đích thật nằm trong tầm cảm biến
        Cell detectedTargetCell{};
    };

    explicit SensorModel(const TruthMap& truth, int sensorRange = Constants::kSensorRange)
        : truth_(truth), range_(sensorRange) {}

    // Quét xung quanh vị trí hiện tại. Mô phỏng cảm biến 4 hướng (giống wallFront/Left/Right
    // trong code micromouse gốc, mở rộng ra cả 4 hướng vì robot không có "hướng nhìn" cố định ở đây)
    // cộng thêm các ô trong bán kính Chebyshev `range_`.
    SenseResult sense(const Cell& pos) const {
        SenseResult r;
        r.position = pos;

        // Tường 4 hướng ngay tại vị trí hiện tại
        for (int d = 0; d < 4; ++d) {
            bool canMove = truth_.canMove(pos, d);
            r.wallDir[d] = !canMove;
        }

        // Quét vùng lân cận trong bán kính cảm biến (Chebyshev)
        for (int dx = -range_; dx <= range_; ++dx) {
            for (int dy = -range_; dy <= range_; ++dy) {
                int nx = pos.x + dx;
                int ny = pos.y + dy;
                if (!truth_.grid().inBounds(nx, ny)) continue;
                if (!truth_.grid().isActive(nx, ny)) continue;
                Cell c{nx, ny};
                if (truth_.isFree(c)) {
                    r.freeCellsNearby.push_back(c);
                } else {
                    r.occupiedCellsNearby.push_back(c);
                }
            }
        }

        // Phát hiện đích thật: CHỈ khi robot đứng đúng ô đích (theo mục 4.2 báo cáo).
        if (truth_.hasTarget() && truth_.target() == pos) {
            r.isTargetDetected = true;
            r.detectedTargetCell = pos;
        }

        return r;
    }

    // Áp dụng kết quả cảm biến vào SharedMap (ghi có khoá mutex bên trong SharedMap).
    void applyToSharedMap(const SenseResult& r, SharedMap& shared) const {
        shared.markFree(r.position);
        shared.markVisited(r.position);
        for (int d = 0; d < 4; ++d) {
            if (r.wallDir[d]) {
                shared.setWall(r.position.x, r.position.y, d, true);
            }
        }
        for (const auto& c : r.freeCellsNearby) shared.markFree(c);
        for (const auto& c : r.occupiedCellsNearby) shared.markOccupied(c);
        if (r.isTargetDetected) shared.markTarget(r.detectedTargetCell);
    }

private:
    const TruthMap& truth_;
    int range_;
};

} // namespace mre
