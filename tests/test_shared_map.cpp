// test_shared_map.cpp - Unit test cho module SharedMap (cpp/include/map/SharedMap.hpp)
//
// Build & chạy độc lập:
//   g++ -std=c++17 -I cpp/include -I cpp/third_party tests/test_shared_map.cpp -lpthread -o /tmp/test_shared_map
//   /tmp/test_shared_map

#include <iostream>
#include "map/SharedMap.hpp"

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

// Test 1: ô mới khởi tạo phải là UNKNOWN.
void test_initial_state_unknown() {
    SharedMap map(5, 5);
    CHECK(map.getState(2, 2) == CellState::UNKNOWN, "test_initial_state_unknown: o moi la UNKNOWN");
}

// Test 2: markFree chỉ chuyển UNKNOWN -> FREE, không ghi đè trạng thái khác (vd VISITED).
void test_mark_free_does_not_overwrite() {
    SharedMap map(3, 3);
    map.markFree({1, 1});
    CHECK(map.getState(1, 1) == CellState::FREE, "test_mark_free_does_not_overwrite: UNKNOWN -> FREE");
    map.markVisited({1, 1});
    CHECK(map.getState(1, 1) == CellState::VISITED, "test_mark_free_does_not_overwrite: FREE -> VISITED qua markVisited");
    map.markFree({1, 1}); // gọi lại markFree không được phép hạ cấp VISITED về FREE
    CHECK(map.getState(1, 1) == CellState::VISITED, "test_mark_free_does_not_overwrite: markFree khong ghi de VISITED");
}

// Test 3: markOccupied luôn ghi đè (vật cản phát hiện được phải ưu tiên tuyệt đối).
void test_mark_occupied_overwrites() {
    SharedMap map(3, 3);
    map.markFree({1, 1});
    map.markVisited({1, 1});
    map.markOccupied({1, 1});
    CHECK(map.getState(1, 1) == CellState::OCCUPIED, "test_mark_occupied_overwrites: OCCUPIED ghi de moi trang thai truoc do");
}

// Test 4: setWall đối xứng - đặt tường ở 1 ô tự động phản ánh sang ô kề.
void test_wall_symmetry() {
    SharedMap map(2, 1);
    map.setWall(0, 0, 1, true); // hướng Đông từ (0,0) -> (1,0)
    CHECK(map.hasWall({0, 0}, 1), "test_wall_symmetry: tuong duoc dat o (0,0) huong Dong");
    CHECK(map.hasWall({1, 0}, 3), "test_wall_symmetry: tuong tu dong phan anh sang (1,0) huong Tay (doi xung)");
}

// Test 5: target được đánh dấu đúng và truy vấn lại đúng.
void test_mark_target() {
    SharedMap map(5, 5);
    CHECK(!map.isTargetKnown(), "test_mark_target: ban dau chua biet dich");
    map.markTarget({3, 3});
    CHECK(map.isTargetKnown(), "test_mark_target: sau khi markTarget thi isTargetKnown=true");
    Cell t = map.knownTarget();
    CHECK(t.x == 3 && t.y == 3, "test_mark_target: knownTarget tra ve dung vi tri");
    CHECK(map.getState(3, 3) == CellState::TARGET, "test_mark_target: trang thai o duoc cap nhat thanh TARGET");
}

// Test 6: applyInflation tạo vùng cấm quanh vật cản nhưng KHÔNG đè lên vị trí robot đang đứng
// (đây là bug thực tế đã phát hiện: robot đứng sát nhau + sát vật cản có thể tự bị "khoá" nếu
// không loại trừ vị trí robot khỏi vùng inflate).
void test_inflation_excludes_robot_positions() {
    SharedMap map(5, 5);
    for (int x = 0; x < 5; ++x)
        for (int y = 0; y < 5; ++y) {
            map.markFree({x, y});
            map.markVisited({x, y});
        }
    map.markOccupied({2, 2});

    std::vector<Cell> robotPositions = {{1, 2}, {2, 1}}; // 2 robot đứng sát vật cản (2,2)
    map.applyInflation(1, 0, robotPositions);

    CHECK(map.getState(1, 2) != CellState::INFLATED_OBSTACLE,
          "test_inflation_excludes_robot_positions: vi tri robot (1,2) khong bi inflate du sat vat can");
    CHECK(map.getState(2, 1) != CellState::INFLATED_OBSTACLE,
          "test_inflation_excludes_robot_positions: vi tri robot (2,1) khong bi inflate du sat vat can");
    // Ô (3,2) không có robot đứng và cũng sát vật cản -> phải bị inflate bình thường.
    CHECK(map.getState(3, 2) == CellState::INFLATED_OBSTACLE,
          "test_inflation_excludes_robot_positions: o khac khong co robot van bi inflate dung");
}

// Test 7: canTraverse chỉ cho phép đi tới ô FREE/VISITED/TARGET, không cho UNKNOWN/OCCUPIED.
void test_can_traverse_rules() {
    SharedMap map(3, 1);
    map.markFree({0, 0});
    map.markVisited({0, 0});
    map.markFree({1, 0});
    map.markVisited({1, 0});
    map.markOccupied({2, 0});

    CHECK(map.canTraverse({0, 0}, 1), "test_can_traverse_rules: di toi o FREE/VISITED duoc phep");

    SharedMap map2(3, 1);
    map2.markFree({0, 0});
    map2.markVisited({0, 0});
    // (1,0) vẫn UNKNOWN
    CHECK(!map2.canTraverse({0, 0}, 1), "test_can_traverse_rules: khong duoc di toi UNKNOWN qua canTraverse thuong");
}

// Test 8: canTraverseAllowUnknownGoal chỉ mở ngoại lệ cho đúng 1 ô đích được chỉ định, không phải mọi UNKNOWN.
void test_can_traverse_allow_unknown_goal_specific() {
    SharedMap map(3, 1);
    map.markFree({0, 0});
    map.markVisited({0, 0});
    // (1,0) và (2,0) đều UNKNOWN

    CHECK(map.canTraverseAllowUnknownGoal({0, 0}, 1, Cell{1, 0}),
          "test_can_traverse_allow_unknown_goal_specific: cho phep di toi UNKNOWN dung la allowedGoal");
}

int main() {
    test_initial_state_unknown();
    test_mark_free_does_not_overwrite();
    test_mark_occupied_overwrites();
    test_wall_symmetry();
    test_mark_target();
    test_inflation_excludes_robot_positions();
    test_can_traverse_rules();
    test_can_traverse_allow_unknown_goal_specific();

    std::cout << "\n=== SharedMap tests: " << (g_failed == 0 ? "TAT CA PASS" : "CO LOI") << " ===\n";
    return g_failed == 0 ? 0 : 1;
}
