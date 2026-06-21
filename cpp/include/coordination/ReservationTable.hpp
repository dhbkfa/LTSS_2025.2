#pragma once
#include <vector>
#include <unordered_map>
#include <algorithm>
#include "core/Types.hpp"
#include "robot/RobotState.hpp"

namespace mre {

// ReservationTable giải quyết đúng 2 loại xung đột nêu trong mục 7 báo cáo:
//   7.1 Trùng ô: hai robot cùng đề xuất đi tới cùng 1 ô trong cùng timestep.
//   7.2 Đổi chỗ đối đầu: robot A đi B->A trong khi robot B đi A->B cùng lúc.
// Quy tắc xử lý: robot ưu tiên cao hơn (id nhỏ hơn) được đi; robot ưu tiên
// thấp hơn phải chờ 1 nhịp (waitTicks = 1) thay vì di chuyển.
//
// NGOẠI LỆ quan trọng (mục 5.3 báo cáo): khi đã xác nhận đích thật, nhiều
// robot ĐƯỢC PHÉP đứng chung 1 ô đích — ô đích không áp dụng luật "trùng ô".
//
// THIẾT KẾ QUAN TRỌNG: một ô đang có robot X đứng KHÔNG được coi là "trống"
// cho robot khác đi vào cho đến khi X đã được XÁC NHẬN CHẮC CHẮN sẽ di chuyển
// đi nơi khác trong chính timestep này. Nếu không, có thể xảy ra trường hợp:
// robot X bị từ chối di chuyển (phải đứng yên) NHƯNG robot Y đã được duyệt đi
// vào đúng ô X đang đứng vì lúc xét Y, ô đó tưởng là "sẽ trống". Để tránh điều
// này, thuật toán chạy theo kiểu lặp ổn định (fixed-point): bắt đầu giả định
// MỌI robot có proposal đều "có thể chiếm ô đích của nó", sau đó loại bỏ dần
// các đề xuất xung đột cho đến khi không còn xung đột nào thay đổi nữa.
//
// LƯU Ý QUAN TRỌNG VỀ SWAP: khi robot j thua robot i trong xung đột đổi-chỗ-đối-đầu
// (7.2) và phải đứng yên, "việc j đứng yên" KHÔNG được dùng ngược lại để hạ bệ chính
// i ở vòng lặp tiếp theo (nếu không, không robot nào trong cặp swap được đi - đã từng
// là bug thực tế khi phát triển). Ta ghi nhớ ai-thua-vì-ai (lostTo) để loại trừ đúng
// trường hợp này khỏi rule "robot đứng yên luôn thắng".
class ReservationTable {
public:
    struct Decision {
        std::vector<bool> allowedToMove; // theo index robot, true = được phép thực hiện đề xuất
    };

    // targetCell: ô đích thật đã biết (nếu có). hasTarget = false nếu chưa biết đích.
    static Decision resolve(const std::vector<RobotState>& robots,
                             bool hasTarget = false, const Cell& targetCell = Cell{}) {
        size_t n = robots.size();
        Decision decision;
        decision.allowedToMove.assign(n, false);

        auto packCell = [](const Cell& c) -> long long {
            return (static_cast<long long>(c.x) << 32) ^ static_cast<uint32_t>(c.y);
        };
        auto isTargetCell = [&](const Cell& c) {
            return hasTarget && c.x == targetCell.x && c.y == targetCell.y;
        };

        std::vector<int> activeIdx;
        for (size_t i = 0; i < n; ++i) {
            if (robots[i].hasProposal && robots[i].waitTicks == 0) {
                activeIdx.push_back(static_cast<int>(i));
            }
        }
        // Ưu tiên theo thứ tự id tăng dần (id nhỏ hơn = ưu tiên cao hơn) — đơn giản, xác định
        // (deterministic), tránh đói tài nguyên nghiêm trọng vì frontier được phân công lại mỗi timestep.
        std::sort(activeIdx.begin(), activeIdx.end());

        // Trạng thái "đang chờ quyết định" ban đầu = mọi robot active đều coi là SẼ di chuyển.
        std::vector<bool> tentativeMove(n, false);
        for (int idx : activeIdx) tentativeMove[idx] = true;

        // lostTo[i] = id của robot đã khiến i bị từ chối (qua xung đột 7.1 hoặc 7.2), -1 nếu chưa thua ai.
        std::vector<int> lostTo(n, -1);

        bool changed = true;
        int guard = static_cast<int>(n) * static_cast<int>(n) + 4; // đủ lớn cho chuỗi xung đột dây chuyền dài nhất có thể (n robot)
        while (changed && guard-- > 0) {
            changed = false;

            // Vị trí "occupied chắc chắn" tại cuối timestep theo giả định hiện tại:
            // - robot không active (không proposal/đang wait) -> luôn chiếm pos của nó
            // - robot active với tentativeMove=true -> chiếm proposedNextCell
            // - robot active với tentativeMove=false (đã bị từ chối ở vòng trước) -> chiếm pos của nó
            std::unordered_map<long long, std::vector<int>> occupants;
            for (size_t i = 0; i < n; ++i) {
                bool isActive = robots[i].hasProposal && robots[i].waitTicks == 0;
                Cell finalCell = (isActive && tentativeMove[i]) ? robots[i].proposedNextCell : robots[i].pos;
                if (isTargetCell(finalCell)) continue; // ô đích thật miễn trừ luật trùng ô
                occupants[packCell(finalCell)].push_back(static_cast<int>(i));
            }

            // 7.1 Trùng ô: ô có >1 robot chiếm cùng lúc.
            // Quy tắc ưu tiên:
            //   - Nếu trong nhóm có robot ĐỨNG YÊN (không active, hoặc active nhưng đã bị từ chối
            //     ở vòng trước nên tentativeMove=false) và robot đó KHÔNG đứng yên chỉ vì vừa thua
            //     chính robot đang được xét (tránh vòng lặp tự mâu thuẫn của swap): robot đứng yên
            //     thắng, mọi robot active+moving khác trong nhóm phải bị từ chối.
            //   - Nếu mọi robot trong nhóm đều active+moving (trường hợp trùng đích đến thật sự):
            //     robot id nhỏ nhất thắng, còn lại bị từ chối.
            for (auto& kv : occupants) {
                auto& occ = kv.second;
                if (occ.size() <= 1) continue;

                int stationaryBlocker = -1; // robot đứng yên "hợp lệ" để chặn người khác
                for (int i : occ) {
                    bool isActive = robots[i].hasProposal && robots[i].waitTicks == 0;
                    bool isMoving = isActive && tentativeMove[i];
                    if (!isMoving) { stationaryBlocker = i; break; }
                }

                if (stationaryBlocker != -1) {
                    for (int i : occ) {
                        bool isActive = robots[i].hasProposal && robots[i].waitTicks == 0;
                        if (!isActive || !tentativeMove[i]) continue; // chỉ xét robot đang active+moving
                        // Không cho robot đứng yên (lostTo == i, tức là nó đứng yên vì vừa thua i)
                        // quay lại hạ bệ chính i - tránh vòng lặp tự mâu thuẫn của swap.
                        if (lostTo[stationaryBlocker] == i) continue;
                        tentativeMove[i] = false;
                        lostTo[i] = stationaryBlocker;
                        changed = true;
                    }
                } else {
                    // Mọi robot trong nhóm đều đang active+moving và cùng tranh 1 ô đích ->
                    // chỉ id nhỏ nhất được giữ, còn lại bị từ chối.
                    int keepIdx = occ[0];
                    for (int i : occ) if (i < keepIdx) keepIdx = i;
                    for (int i : occ) {
                        if (i != keepIdx) {
                            tentativeMove[i] = false;
                            lostTo[i] = keepIdx;
                            changed = true;
                        }
                    }
                }
            }

            // 7.2 Đổi chỗ đối đầu: robot A (active, moving) đi tới pos hiện tại của robot B,
            // đồng thời robot B (active, moving) đi tới pos hiện tại của robot A.
            // QUAN TRỌNG: đây không đơn thuần là "1 trong 2 được đi" - nếu B bị từ chối và phải
            // đứng yên, B vẫn đứng đúng tại ô mà A đang muốn đi vào, nên A KHÔNG THỂ được phép đi
            // (nếu không 2 robot sẽ cùng ở 1 ô). Do đó khi phát hiện swap, ta hạ ĐỒNG THỜI cả 2
            // xuống "tạm thời từ chối", rồi để vòng lặp fixed-point tiếp theo (rule 7.1) tự xác
            // định lại: với cả 2 đứng yên, không còn ai tranh chấp ô của nhau nữa nên ổn định ngay;
            // nếu có robot thứ 3 muốn vào 1 trong 2 ô đó, rule 7.1 sẽ xử lý tiếp bình thường.
            // Sau khi cả 2 bị hạ vì swap, robot id nhỏ hơn được "thử lại" làm active+moving ở vòng
            // sau NẾU sau đó ô đích của nó không còn ai khác đứng - nhưng vì B (id lớn hơn) đã chắc
            // chắn đứng yên tại đúng ô đó, A vẫn sẽ bị rule 7.1 chặn lại đúng đắn ở vòng kế tiếp.
            for (int i : activeIdx) {
                if (!tentativeMove[i]) continue;
                for (int j : activeIdx) {
                    if (j <= i || !tentativeMove[j]) continue;
                    bool swap = (robots[i].proposedNextCell == robots[j].pos) &&
                                (robots[j].proposedNextCell == robots[i].pos);
                    if (swap) {
                        tentativeMove[i] = false;
                        tentativeMove[j] = false;
                        lostTo[i] = j; // đánh dấu chéo để rule 7.1 không hiểu lầm là "tự mâu thuẫn" lẫn nhau
                        lostTo[j] = i;
                        changed = true;
                    }
                }
            }
        }

        for (size_t i = 0; i < n; ++i) {
            decision.allowedToMove[i] = tentativeMove[i];
        }
        return decision;
    }
};

} // namespace mre
