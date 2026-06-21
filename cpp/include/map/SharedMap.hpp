#pragma once
#include <mutex>
#include <vector>
#include <unordered_set>
#include "map/GridMap.hpp"
#include "map/Inflation.hpp"
#include "core/Types.hpp"
#include "core/Constants.hpp"

namespace mre {

// SharedMap là bản đồ DUY NHẤT mà tất cả robot cùng đọc/ghi. Đây là tri thức
// tích luỹ được từ cảm biến, KHÔNG phải bản đồ thật. Mọi truy cập ghi phải
// qua mutex để tránh race condition khi 5 thread robot chạy song song
// (theo đúng cảnh báo trong báo cáo mục 6).
class SharedMap {
public:
    SharedMap() = default;
    explicit SharedMap(int width, int height) : grid_(width, height) {}

    int width() const { return grid_.width(); }
    int height() const { return grid_.height(); }

    // Đọc trạng thái 1 ô - an toàn cho nhiều thread đọc đồng thời nhờ shared lock kiểu đơn giản (mutex thường,
    // đủ dùng vì tần suất ghi không quá cao so với số ô đọc mỗi bước).
    CellState getState(int x, int y) {
        std::lock_guard<std::mutex> lk(mtx_);
        return grid_.at(x, y);
    }
    CellState getState(const Cell& c) { return getState(c.x, c.y); }

    bool isActive(int x, int y) {
        std::lock_guard<std::mutex> lk(mtx_);
        return grid_.isActive(x, y);
    }

    bool inBounds(int x, int y) const { return grid_.inBounds(x, y); }
    bool inBounds(const Cell& c) const { return grid_.inBounds(c); }

    // Cập nhật một ô là FREE (đã biết, đi được) - dùng khi sensor phát hiện ô trống.
    void markFree(const Cell& c) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!grid_.inBounds(c)) return;
        if (grid_.at(c) == CellState::UNKNOWN) {
            grid_.setState(c, CellState::FREE);
        }
    }

    void markOccupied(const Cell& c) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!grid_.inBounds(c)) return;
        grid_.setState(c, CellState::OCCUPIED);
    }

    void markVisited(const Cell& c) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!grid_.inBounds(c)) return;
        if (grid_.at(c) != CellState::OCCUPIED && grid_.at(c) != CellState::INFLATED_OBSTACLE) {
            grid_.setState(c, CellState::VISITED);
        }
    }

    void setWall(int x, int y, int dir, bool value) {
        std::lock_guard<std::mutex> lk(mtx_);
        grid_.setWall(x, y, dir, value);
    }

    bool hasWall(const Cell& c, int dir) {
        std::lock_guard<std::mutex> lk(mtx_);
        return grid_.hasWall(c, dir);
    }

    void markTarget(const Cell& c) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!grid_.inBounds(c)) return;
        grid_.setState(c, CellState::TARGET);
        targetKnown_ = true;
        knownTarget_ = c;
    }

    bool isTargetKnown() {
        std::lock_guard<std::mutex> lk(mtx_);
        return targetKnown_;
    }

    Cell knownTarget() {
        std::lock_guard<std::mutex> lk(mtx_);
        return knownTarget_;
    }

    // Đánh dấu 1 ô là "đã xác nhận không thể tới được" theo tri thức hiện tại (vd: bị
    // vùng INFLATED_OBSTACLE bao kín hoàn toàn). Dùng chung cho mọi robot để tránh
    // các robot khác nhau lặp đi lặp lại cùng 1 nỗ lực thất bại vô ích.
    void markUnreachable(const Cell& c) {
        std::lock_guard<std::mutex> lk(mtx_);
        unreachable_.insert(packCell(c));
    }

    bool isMarkedUnreachable(const Cell& c) {
        std::lock_guard<std::mutex> lk(mtx_);
        return unreachable_.count(packCell(c)) > 0;
    }

    // Xoá toàn bộ blacklist - gọi khi map vừa có cập nhật mới đáng kể (ô UNKNOWN -> FREE)
    // vì khi đó các ô từng "không tới được" có thể đã có đường đi mới mở ra.
    void clearUnreachableIfMapChanged(bool mapChanged) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (mapChanged) unreachable_.clear();
    }

    // Một ô đi được theo tri thức hiện có nếu: trong biên, active, đã biết là FREE/VISITED/TARGET
    // (KHÔNG đi vào UNKNOWN trực tiếp bằng A* — UNKNOWN chỉ tiếp cận được khi nó là biên của frontier),
    // và không bị tường chặn theo hướng đi từ 'from'.
    bool canTraverse(const Cell& from, int dir) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!grid_.inBounds(from) || !grid_.isActive(from.x, from.y)) return false;
        if (grid_.hasWall(from, dir)) return false;
        Cell to{from.x + kDirX[dir], from.y + kDirY[dir]};
        if (!grid_.inBounds(to) || !grid_.isActive(to.x, to.y)) return false;
        CellState s = grid_.at(to);
        return s == CellState::FREE || s == CellState::VISITED || s == CellState::TARGET;
    }

    // Cho phép A* đi tới đúng 1 ô UNKNOWN là đích (dùng khi đích là frontier mục tiêu).
    bool canTraverseAllowUnknownGoal(const Cell& from, int dir, const Cell& allowedGoal) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!grid_.inBounds(from) || !grid_.isActive(from.x, from.y)) return false;
        if (grid_.hasWall(from, dir)) return false;
        Cell to{from.x + kDirX[dir], from.y + kDirY[dir]};
        if (!grid_.inBounds(to) || !grid_.isActive(to.x, to.y)) return false;
        CellState s = grid_.at(to);
        if (s == CellState::FREE || s == CellState::VISITED || s == CellState::TARGET) return true;
        if (s == CellState::UNKNOWN && to == allowedGoal) return true;
        return false;
    }

    void applyInflation(int robotRadius, int safetyMargin, const std::vector<Cell>& robotPositions = {}) {
        std::lock_guard<std::mutex> lk(mtx_);
        Inflation::apply(grid_, robotRadius, safetyMargin, robotPositions);
    }

    // Snapshot toàn bộ map để ghi log/JSON hoặc tính frontier (đọc nhất quán tại 1 thời điểm).
    GridMap snapshot() {
        std::lock_guard<std::mutex> lk(mtx_);
        return grid_;
    }

private:
    static long long packCell(const Cell& c) {
        return (static_cast<long long>(c.x) << 32) ^ static_cast<uint32_t>(c.y);
    }

    std::mutex mtx_;
    GridMap grid_;
    bool targetKnown_ = false;
    Cell knownTarget_{};
    std::unordered_set<long long> unreachable_;
};

} // namespace mre
