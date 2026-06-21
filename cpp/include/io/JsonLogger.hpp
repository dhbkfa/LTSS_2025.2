#pragma once
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include "nlohmann/json.hpp"

namespace mre {

// JsonLogger ghi mỗi timestep ra MỘT file JSON riêng trong thư mục output
// (đơn giản, dễ debug, đúng lựa chọn cơ chế giao tiếp C++ <-> Python qua file
// đã thống nhất). Tên file đệm số 0 để Python dễ sort theo thứ tự.
class JsonLogger {
public:
    explicit JsonLogger(const std::string& outDir) : outDir_(outDir) {
        std::filesystem::create_directories(outDir_);
    }

    void writeStep(int timestep, const nlohmann::json& stateJson) {
        std::ostringstream name;
        name << "step_" << std::setfill('0') << std::setw(6) << timestep << ".json";
        std::filesystem::path p = std::filesystem::path(outDir_) / name.str();
        std::ofstream f(p);
        f << stateJson.dump();
        f.close();
        lastStep_ = timestep;
    }

    // Ghi file "meta.json" cho biết tổng số bước đã ghi và trạng thái kết thúc —
    // giúp Python UI biết khi nào dữ liệu mới đã sẵn sàng / mô phỏng đã xong.
    void writeMeta(int totalSteps, bool finished) {
        nlohmann::json m;
        m["total_steps"] = totalSteps;
        m["finished"] = finished;
        std::filesystem::path p = std::filesystem::path(outDir_) / "meta.json";
        std::ofstream f(p);
        f << m.dump();
    }

private:
    std::string outDir_;
    int lastStep_ = -1;
};

} // namespace mre
