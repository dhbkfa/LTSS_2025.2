#pragma once
#include <vector>
#include <array>
#include <fstream>
#include <stdexcept>
#include "core/Types.hpp"

namespace mre {

// GridMap là cấu trúc lưu trữ thuần: kích thước, trạng thái từng ô,
// và tường giữa các ô theo 4 hướng. Không chứa logic thuật toán.
class GridMap {
public:
    GridMap() = default;
    GridMap(int width, int height)
        : width_(width), height_(height),
          cells_(static_cast<size_t>(width) * height, CellState::UNKNOWN),
          // wall_[idx][d] = true nghĩa là có tường giữa ô idx và hàng xóm theo hướng d
          walls_(static_cast<size_t>(width) * height, {false, false, false, false}),
          active_(static_cast<size_t>(width) * height, true) {}

    int width() const { return width_; }
    int height() const { return height_; }

    bool inBounds(int x, int y) const {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }
    bool inBounds(const Cell& c) const { return inBounds(c.x, c.y); }

    size_t idx(int x, int y) const { return static_cast<size_t>(y) * width_ + x; }

    CellState at(int x, int y) const { return cells_[idx(x, y)]; }
    CellState at(const Cell& c) const { return at(c.x, c.y); }
    void setState(int x, int y, CellState s) { cells_[idx(x, y)] = s; }
    void setState(const Cell& c, CellState s) { setState(c.x, c.y, s); }

    // active = ô có tồn tại trong hình dạng bản đồ hay không (dùng khi map không phải hình chữ nhật đầy đủ)
    bool isActive(int x, int y) const { return active_[idx(x, y)]; }
    bool isActive(const Cell& c) const { return isActive(c.x, c.y); }
    void setActive(int x, int y, bool v) { active_[idx(x, y)] = v; }

    bool hasWall(int x, int y, int dir) const { return walls_[idx(x, y)][dir]; }
    bool hasWall(const Cell& c, int dir) const { return hasWall(c.x, c.y, dir); }

    void setWall(int x, int y, int dir, bool value) {
        walls_[idx(x, y)][dir] = value;
        int nx = x + kDirX[dir];
        int ny = y + kDirY[dir];
        if (inBounds(nx, ny)) {
            walls_[idx(nx, ny)][kOpposite[dir]] = value;
        }
    }

    // Một ô có thể đi được nếu: nằm trong biên, active, không phải vật cản/inflated, và không có tường chặn hướng đi.
    bool canMove(const Cell& from, int dir) const {
        if (!inBounds(from) || !isActive(from.x, from.y)) return false;
        if (hasWall(from, dir)) return false;
        Cell to{from.x + kDirX[dir], from.y + kDirY[dir]};
        if (!inBounds(to) || !isActive(to.x, to.y)) return false;
        CellState s = at(to);
        if (s == CellState::OCCUPIED || s == CellState::INFLATED_OBSTACLE) return false;
        return true;
    }

    bool isBlocked(const Cell& c) const {
        if (!inBounds(c) || !isActive(c.x, c.y)) return true;
        CellState s = at(c);
        return s == CellState::OCCUPIED || s == CellState::INFLATED_OBSTACLE;
    }

private:
    int width_ = 0;
    int height_ = 0;
    std::vector<CellState> cells_;
    std::vector<std::array<bool, 4>> walls_;
    std::vector<bool> active_;
};

} // namespace mre
