#pragma once

namespace mre {

struct Constants {
    static constexpr int kNumRobots = 5;
    static constexpr int kDefaultRobotRadiusCells = 0;   // bán kính robot tính theo số ô (0 = robot chiếm đúng 1 ô)
    static constexpr int kDefaultSafetyMarginCells = 0;  // khoảng an toàn cộng thêm khi inflate vật cản
    static constexpr int kSensorRange = 1;                // bán kính cảm biến quanh robot (tính bằng ô, Chebyshev)
    static constexpr int kMaxTimesteps = 200000;          // chặn an toàn tránh vòng lặp vô hạn
};

} // namespace mre
