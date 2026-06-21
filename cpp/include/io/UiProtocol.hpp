#pragma once
#include <string>
#include <vector>
#include "core/Types.hpp"
#include "robot/RobotState.hpp"
#include "map/SharedMap.hpp"
#include "nlohmann/json.hpp"

namespace mre {

// UiProtocol chuyển trạng thái mô phỏng (SharedMap + robots) thành JSON theo
// định dạng cố định để Python UI vẽ lại, KHÔNG bao giờ lộ TruthMap (đúng
// nguyên tắc "robot chỉ thấy phần đã khám phá" trong mục 3.1 báo cáo).
class UiProtocol {
public:
    static const char* cellStateStr(CellState s) {
        switch (s) {
            case CellState::UNKNOWN: return "UNKNOWN";
            case CellState::FREE: return "FREE";
            case CellState::OCCUPIED: return "OCCUPIED";
            case CellState::INFLATED_OBSTACLE: return "INFLATED_OBSTACLE";
            case CellState::VISITED: return "VISITED";
            case CellState::TARGET: return "TARGET";
        }
        return "UNKNOWN";
    }

    static const char* phaseStr(RobotPhase p) {
        switch (p) {
            case RobotPhase::EXPLORING: return "EXPLORING";
            case RobotPhase::GOING_TO_TARGET: return "GOING_TO_TARGET";
            case RobotPhase::ARRIVED: return "ARRIVED";
            case RobotPhase::WAITING: return "WAITING";
            case RobotPhase::UNREACHABLE: return "UNREACHABLE";
        }
        return "EXPLORING";
    }

    static const char* statusStr(SimStatus s) {
        switch (s) {
            case SimStatus::RUNNING: return "RUNNING";
            case SimStatus::TARGET_FOUND: return "TARGET_FOUND";
            case SimStatus::MAP_COVERED: return "MAP_COVERED";
            case SimStatus::STUCK: return "STUCK";
        }
        return "RUNNING";
    }

    static nlohmann::json buildStateJson(int timestep,
                                          SimStatus status,
                                          SharedMap& shared,
                                          const std::vector<RobotState>& robots) {
        nlohmann::json j;
        j["timestep"] = timestep;
        j["status"] = statusStr(status);

        GridMap snap = shared.snapshot();
        int w = snap.width(), h = snap.height();
        j["width"] = w;
        j["height"] = h;

        // map dạng [x][y] -> string state, khớp chiều dữ liệu với map builder Python
        nlohmann::json cells = nlohmann::json::array();
        for (int x = 0; x < w; ++x) {
            nlohmann::json col = nlohmann::json::array();
            for (int y = 0; y < h; ++y) {
                if (!snap.isActive(x, y)) {
                    col.push_back("INACTIVE");
                } else {
                    col.push_back(cellStateStr(snap.at(x, y)));
                }
            }
            cells.push_back(col);
        }
        j["cells"] = cells;

        // Xuất tường — chỉ hiện tường tại các ô đã được khám phá (không UNKNOWN).
        // Đảm bảo "fog of war": ô chưa đến thăm không lộ thông tin tường.
        // Thứ tự hướng: N=0, E=1, S=2, W=3 (khớp với kDirX/kDirY trong C++).
        static const char* kWallDirKeys[4] = {"N", "E", "S", "W"};
        nlohmann::json wallsJson;
        for (int d = 0; d < 4; ++d) {
            nlohmann::json wd = nlohmann::json::array();
            for (int x = 0; x < w; ++x) {
                nlohmann::json col = nlohmann::json::array();
                for (int y = 0; y < h; ++y) {
                    // Chỉ hiển thị tường khi ô đã được biết (không UNKNOWN, phải active)
                    bool known = snap.isActive(x, y) && (snap.at(x, y) != CellState::UNKNOWN);
                    col.push_back(known && snap.hasWall(x, y, d));
                }
                wd.push_back(col);
            }
            wallsJson[kWallDirKeys[d]] = wd;
        }
        j["walls"] = wallsJson;

        nlohmann::json robotsJson = nlohmann::json::array();
        for (const auto& r : robots) {
            nlohmann::json rj;
            rj["id"] = r.id;
            rj["x"] = r.pos.x;
            rj["y"] = r.pos.y;
            rj["phase"] = phaseStr(r.phase);
            rj["has_goal"] = r.hasGoal;
            if (r.hasGoal) {
                rj["goal_x"] = r.currentGoal.x;
                rj["goal_y"] = r.currentGoal.y;
            }
            nlohmann::json pathJson = nlohmann::json::array();
            for (const auto& c : r.plannedPath) {
                pathJson.push_back({c.x, c.y});
            }
            rj["path"] = pathJson;
            rj["total_steps"] = r.totalSteps;
            robotsJson.push_back(rj);
        }
        j["robots"] = robotsJson;

        return j;
    }
};

} // namespace mre
