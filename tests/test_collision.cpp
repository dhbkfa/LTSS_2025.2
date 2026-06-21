// test_collision.cpp - Unit test cho module ReservationTable (cpp/include/coordination/ReservationTable.hpp)
//
// Bao gồm các case thực tế đã từng gây bug khi phát triển (xem báo cáo mục 7):
// trùng ô, đổi chỗ đối đầu (cả 2 phải dừng vì đi đồng thời vào ô đối tác là không an toàn),
// và đặc biệt là tình huống "dây chuyền" - robot A muốn vào ô của robot B, nhưng B lại thua
// trong tranh chấp với robot C nên B đứng yên, khi đó A phải bị từ chối (không được đi vào ô
// của B nữa), không chỉ xét xung đột trực tiếp ở vòng đầu tiên.
//
// Build & chạy độc lập:
//   g++ -std=c++17 -I cpp/include -I cpp/third_party tests/test_collision.cpp -lpthread -o /tmp/test_collision
//   /tmp/test_collision

#include <iostream>
#include "coordination/ReservationTable.hpp"

using namespace mre;

static int g_failed = 0;

#define CHECK(cond, msg)                                                     \
    do {                                                                     \
        if (!(cond)) {                                                      \
            std::cerr << "[FAIL] " << msg << " (line " << __LINE__ << ")\n"; \
            g_failed++;                                                      \
        } else {                                                            \
            std::cout << "[ OK ] " << msg << "\n";                          \
        }                                                                    \
    } while (0)

static RobotState makeRobot(int id, Cell pos, Cell proposed) {
    RobotState r;
    r.id = id;
    r.pos = pos;
    r.hasProposal = true;
    r.proposedNextCell = proposed;
    r.waitTicks = 0;
    return r;
}

// Test 1 (mục 7.1 báo cáo): hai robot cùng đề xuất đi vào cùng 1 ô -> chỉ 1 được đi.
void test_same_cell_conflict() {
    std::vector<RobotState> robots;
    robots.push_back(makeRobot(0, {0, 0}, {1, 0}));
    robots.push_back(makeRobot(1, {2, 0}, {1, 0})); // cùng đích (1,0)

    auto decision = ReservationTable::resolve(robots, false, Cell{});
    int allowedCount = 0;
    for (bool b : decision.allowedToMove) if (b) allowedCount++;
    CHECK(allowedCount == 1, "test_same_cell_conflict: chi dung 1 robot duoc di");
    CHECK(decision.allowedToMove[0], "test_same_cell_conflict: robot id nho hon (0) duoc uu tien");
}

// Test 2 (mục 7.2 báo cáo): đổi chỗ đối đầu A<->B.
// LƯU Ý VẬT LÝ: trong 1 timestep, 2 robot di chuyển "đồng thời" - nếu chỉ cho 1 robot đi,
// nó sẽ đi vào đúng ô robot kia đang đứng yên (vì robot kia bị từ chối, không hề rời đi).
// Do đó đáp án an toàn duy nhất là CẢ HAI đều phải chờ (không ai được đi), rồi thử lại ở
// timestep sau khi đã thoát khỏi tình huống đối xứng (thường nhờ rejectStreak/giá trị ngẫu
// nhiên ở A* hoặc thay đổi mục tiêu tạm thời tại Coordinator).
void test_head_on_swap_conflict() {
    std::vector<RobotState> robots;
    robots.push_back(makeRobot(0, {0, 0}, {1, 0}));
    robots.push_back(makeRobot(1, {1, 0}, {0, 0})); // đổi chỗ trực tiếp với robot 0

    auto decision = ReservationTable::resolve(robots, false, Cell{});
    int allowedCount = 0;
    for (bool b : decision.allowedToMove) if (b) allowedCount++;
    CHECK(allowedCount == 0,
          "test_head_on_swap_conflict: ca 2 robot deu phai cho (khong ai duoc di vao o doi tac dang dung)");
}

// Test 3: không xung đột -> cả 2 robot đều được đi.
void test_no_conflict() {
    std::vector<RobotState> robots;
    robots.push_back(makeRobot(0, {0, 0}, {1, 0}));
    robots.push_back(makeRobot(1, {5, 5}, {5, 6}));

    auto decision = ReservationTable::resolve(robots, false, Cell{});
    CHECK(decision.allowedToMove[0] && decision.allowedToMove[1],
          "test_no_conflict: ca 2 robot deu duoc di khi khong xung dot");
}

// Test 4 (ngoại lệ mục 5.3 báo cáo): nhiều robot được phép cùng đứng ở ô đích thật.
void test_target_cell_allows_multiple_robots() {
    std::vector<RobotState> robots;
    Cell target{5, 5};
    robots.push_back(makeRobot(0, {4, 5}, target));
    robots.push_back(makeRobot(1, {6, 5}, target));
    robots.push_back(makeRobot(2, {5, 4}, target));

    auto decision = ReservationTable::resolve(robots, true, target);
    for (size_t i = 0; i < robots.size(); ++i) {
        CHECK(decision.allowedToMove[i], "test_target_cell_allows_multiple_robots: robot " + std::to_string(i) + " duoc phep vao chung o dich");
    }
}

// Test 5: robot đứng yên (không có proposal) "chiếm chỗ" - robot khác không được đi vào.
void test_stationary_robot_blocks() {
    std::vector<RobotState> robots;
    RobotState stationary;
    stationary.id = 0;
    stationary.pos = {3, 3};
    stationary.hasProposal = false; // không di chuyển

    robots.push_back(stationary);
    robots.push_back(makeRobot(1, {2, 3}, {3, 3})); // muốn đi vào đúng ô robot 0 đang đứng

    auto decision = ReservationTable::resolve(robots, false, Cell{});
    CHECK(!decision.allowedToMove[1], "test_stationary_robot_blocks: robot dang dung yen luon chan duong, robot khac khong duoc vao");
}

// Test 6 (case thực tế từng gây bug - xung đột "dây chuyền"):
// Robot 3 và 4 cùng tranh ô (2,3) -> robot 3 (id nhỏ hơn) thắng, robot 4 phải đứng yên tại (1,3).
// Robot 1 muốn đi vào (1,3) - đúng ô mà robot 4 VỪA bị buộc phải đứng yên ở đó.
// Robot 1 PHẢI bị từ chối, dù ở vòng xét đầu tiên (1,3) chưa có ai "đứng yên" tại đó
// (robot 4 ban đầu định di chuyển đi, chỉ bị từ chối SAU khi thua robot 3).
void test_chain_conflict_resolves_correctly() {
    std::vector<RobotState> robots;
    robots.push_back(makeRobot(0, {2, 0}, {3, 0}));   // không liên quan, không xung đột
    robots.push_back(makeRobot(1, {1, 2}, {1, 3}));   // muốn vào (1,3)
    robots.push_back(makeRobot(2, {0, 3}, {0, 4}));   // không liên quan
    robots.push_back(makeRobot(3, {2, 2}, {2, 3}));   // tranh (2,3) với robot 4, id nhỏ hơn -> thắng
    robots.push_back(makeRobot(4, {1, 3}, {2, 3}));   // đang đứng ở (1,3), muốn đi (2,3) nhưng thua robot 3

    auto decision = ReservationTable::resolve(robots, false, Cell{});

    CHECK(decision.allowedToMove[3], "test_chain_conflict: robot 3 thang tranh chap o (2,3)");
    CHECK(!decision.allowedToMove[4], "test_chain_conflict: robot 4 thua, phai dung yen tai (1,3)");
    CHECK(!decision.allowedToMove[1],
          "test_chain_conflict: robot 1 PHAI bi tu choi vi robot 4 gio dung yen dung tai (1,3) - day la bug da tung xay ra thuc te");
}

int main() {
    test_same_cell_conflict();
    test_head_on_swap_conflict();
    test_no_conflict();
    test_target_cell_allows_multiple_robots();
    test_stationary_robot_blocks();
    test_chain_conflict_resolves_correctly();

    std::cout << "\n=== Collision tests: " << (g_failed == 0 ? "TAT CA PASS" : "CO LOI") << " ===\n";
    return g_failed == 0 ? 0 : 1;
}
