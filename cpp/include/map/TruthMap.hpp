#pragma once
#include <string>
#include <fstream>
#include <stdexcept>
#include <optional>
#include "map/GridMap.hpp"
#include "core/Types.hpp"
#include "nlohmann/json.hpp"

namespace mre {

// TruthMap nắm giữ sự thật tuyệt đối về môi trường: hình dạng map, vật cản,
// và vị trí đích thật (nếu có). CHỈ ĐƯỢC sensor model sử dụng để trả lời câu hỏi
// "tại ô (x,y), xung quanh có gì". Coordinator/Robot/Planner không bao giờ
// được đọc trực tiếp TruthMap.
class TruthMap {
public:
    // Load map JSON được xuất từ Python UI builder. Định dạng:
    // {
    //   "width": W, "height": H,
    //   "active": [[bool x H] x W],          // ô có tồn tại trong hình dạng map không
    //   "occupied": [[bool x H] x W],        // ô là vật cản
    //   "walls": { "N":[[bool]], "E":[[bool]], "S":[[bool]], "W":[[bool]] }, // tường nội bộ
    //   "start_positions": [[x,y], ...]      // tối đa 5 vị trí xuất phát
    //   "target": [x, y] | null              // đích thật, có thể không tồn tại
    // }
    static TruthMap loadFromFile(const std::string& path) {
        std::ifstream f(path);
        if (!f.is_open()) {
            throw std::runtime_error("Khong the mo file map: " + path);
        }
        nlohmann::json j;
        f >> j;
        return loadFromJson(j);
    }

    static TruthMap loadFromJson(const nlohmann::json& j) {
        TruthMap tm;
        int w = j.at("width").get<int>();
        int h = j.at("height").get<int>();
        tm.grid_ = GridMap(w, h);

        const auto& active = j.at("active");
        const auto& occupied = j.at("occupied");
        for (int x = 0; x < w; ++x) {
            for (int y = 0; y < h; ++y) {
                bool isActive = active.at(x).at(y).get<bool>();
                bool isOcc = occupied.at(x).at(y).get<bool>();
                tm.grid_.setActive(x, y, isActive);
                if (!isActive) {
                    tm.grid_.setState(x, y, CellState::OCCUPIED);
                } else if (isOcc) {
                    tm.grid_.setState(x, y, CellState::OCCUPIED);
                } else {
                    tm.grid_.setState(x, y, CellState::FREE);
                }
            }
        }

        if (j.contains("walls")) {
            const auto& walls = j.at("walls");
            static const char* dirKeys[4] = {"N", "E", "S", "W"};
            for (int d = 0; d < 4; ++d) {
                if (!walls.contains(dirKeys[d])) continue;
                const auto& wd = walls.at(dirKeys[d]);
                for (int x = 0; x < w; ++x) {
                    for (int y = 0; y < h; ++y) {
                        bool hasW = wd.at(x).at(y).get<bool>();
                        if (hasW) tm.grid_.setWall(x, y, d, true);
                    }
                }
            }
        }

        if (j.contains("start_positions")) {
            for (const auto& p : j.at("start_positions")) {
                tm.startPositions_.push_back(Cell{p.at(0).get<int>(), p.at(1).get<int>()});
            }
        }

        if (j.contains("target") && !j.at("target").is_null()) {
            const auto& t = j.at("target");
            tm.target_ = Cell{t.at(0).get<int>(), t.at(1).get<int>()};
            tm.hasTarget_ = true;
        } else {
            tm.hasTarget_ = false;
        }

        return tm;
    }

    int width() const { return grid_.width(); }
    int height() const { return grid_.height(); }

    bool isFree(int x, int y) const {
        if (!grid_.inBounds(x, y) || !grid_.isActive(x, y)) return false;
        return grid_.at(x, y) != CellState::OCCUPIED;
    }
    bool isFree(const Cell& c) const { return isFree(c.x, c.y); }

    bool canMove(const Cell& from, int dir) const { return grid_.canMove(from, dir); }

    bool hasTarget() const { return hasTarget_; }
    Cell target() const { return target_; }

    const std::vector<Cell>& startPositions() const { return startPositions_; }

    const GridMap& grid() const { return grid_; }

private:
    GridMap grid_;
    std::vector<Cell> startPositions_;
    Cell target_{};
    bool hasTarget_ = false;
};

} // namespace mre
