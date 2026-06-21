#pragma once
#include <cstdint>
#include <vector>
#include <string>

namespace mre {

// Trạng thái của một ô trong lưới, theo đúng quy ước trong báo cáo thiết kế.
enum class CellState : uint8_t {
    UNKNOWN = 0,           // robot chưa biết
    FREE = 1,              // robot biết có thể đi
    OCCUPIED = 2,           // vật cản hoặc tường
    INFLATED_OBSTACLE = 3,  // vùng cấm sau khi cộng bán kính robot + khoảng an toàn
    VISITED = 4,            // đã có robot đi qua
    TARGET = 5               // đích thật (chỉ lộ ra khi phát hiện)
};

// Toạ độ lưới nguyên, dùng (x = cột, y = hàng).
struct Cell {
    int x = 0;
    int y = 0;

    bool operator==(const Cell& o) const { return x == o.x && y == o.y; }
    bool operator!=(const Cell& o) const { return !(*this == o); }
    bool operator<(const Cell& o) const { return (y != o.y) ? (y < o.y) : (x < o.x); }
};

struct CellHash {
    std::size_t operator()(const Cell& c) const noexcept {
        // 32-bit pack đủ cho map kích thước thực tế (<65536 mỗi chiều)
        return (static_cast<std::size_t>(static_cast<uint32_t>(c.x)) << 32) ^
               static_cast<uint32_t>(c.y);
    }
};

// 4 hướng di chuyển: 0=Bắc(N, +y), 1=Đông(E, +x), 2=Nam(S, -y), 3=Tây(W, -x)
// Quy ước này khớp với DIR_X/DIR_Y trong code Python gốc (y hướng lên).
constexpr int kDirX[4] = {0, 1, 0, -1};
constexpr int kDirY[4] = {1, 0, -1, 0};
constexpr int kOpposite[4] = {2, 3, 0, 1};
constexpr const char* kDirName[4] = {"N", "E", "S", "W"};

enum class RunMode : uint8_t {
    FIND_TARGET = 0,   // Mode 1: dừng khi có robot tìm thấy đích thật, các robot khác hội tụ về đích
    COVERAGE = 1,       // Mode 2: chạy đến khi phủ hết map (không có đích thật)
    COMBINED = 2        // Mode 3 (mặc định theo báo cáo): có đích thì dừng khi tìm thấy, không có đích thì phủ hết map
};

enum class SimStatus : uint8_t {
    RUNNING = 0,
    TARGET_FOUND = 1,
    MAP_COVERED = 2,
    STUCK = 3          // không còn frontier nào khả thi và chưa phủ hết (map có vùng không thể tới)
};

} // namespace mre
