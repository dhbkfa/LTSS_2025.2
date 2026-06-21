// test_frontier.cpp - Unit test cho module Frontier (cpp/include/planning/Frontier.hpp)
//
// Build & chạy độc lập:
//   g++ -std=c++17 -I cpp/include -I cpp/third_party tests/test_frontier.cpp -lpthread -o /tmp/test_frontier
//   /tmp/test_frontier

#include <iostream>
#include "map/SharedMap.hpp"
#include "planning/Frontier.hpp"

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

// Test 1: map toàn UNKNOWN -> không có frontier nào (chưa robot nào đứng lên đâu cả).
void test_all_unknown_no_frontier() {
    SharedMap map(5, 5);
    CHECK(!Frontier::anyFrontierLeft(map), "test_all_unknown_no_frontier: map toan UNKNOWN khong co frontier");
    auto fl = Frontier::findAll(map);
    CHECK(fl.empty(), "test_all_unknown_no_frontier: findAll tra ve rong");
}

// Test 2: 1 ô VISITED kề UNKNOWN -> phải nhận diện là frontier.
void test_visited_next_to_unknown_is_frontier() {
    SharedMap map(5, 5);
    map.markFree({2, 2});
    map.markVisited({2, 2});
    // (3,2) (1,2) (2,3) (2,1) đều UNKNOWN -> 4 hướng mở

    CHECK(Frontier::anyFrontierLeft(map), "test_visited_next_to_unknown_is_frontier: phat hien co frontier");
    auto fl = Frontier::findAll(map);
    CHECK(fl.size() == 1, "test_visited_next_to_unknown_is_frontier: dung 1 frontier cell");
    if (!fl.empty()) {
        CHECK(fl[0].knownCell == (Cell{2, 2}), "test_visited_next_to_unknown_is_frontier: knownCell dung vi tri");
        CHECK(fl[0].unknownNeighborCount == 4, "test_visited_next_to_unknown_is_frontier: 4 huong UNKNOWN xung quanh");
    }
}

// Test 3: ô FREE nhưng CHƯA VISITED phải được coi là mục tiêu cần ghé thăm
// (vì đích thật chỉ lộ ra khi robot đứng lên đúng ô - mục 4.2 báo cáo).
void test_free_unvisited_is_frontier() {
    SharedMap map(3, 3);
    map.markFree({1, 1}); // chỉ markFree, KHÔNG markVisited

    CHECK(Frontier::anyFrontierLeft(map), "test_free_unvisited_is_frontier: o FREE chua visited van la frontier");
    auto fl = Frontier::findAll(map);
    bool foundFreeCell = false;
    for (auto& f : fl) {
        if (f.knownCell == (Cell{1, 1})) foundFreeCell = true;
    }
    CHECK(foundFreeCell, "test_free_unvisited_is_frontier: findAll tra ve dung o FREE chua visited");
}

// Test 4: tường chặn hướng tới UNKNOWN -> không tính là frontier theo hướng đó.
void test_wall_blocks_frontier() {
    SharedMap map(3, 3);
    map.markFree({1, 1});
    map.markVisited({1, 1});
    // chặn cả 4 hướng bằng tường nhân tạo (mô phỏng đã xác nhận không đi được dù bên kia UNKNOWN)
    for (int d = 0; d < 4; ++d) map.setWall(1, 1, d, true);

    CHECK(!Frontier::anyFrontierLeft(map), "test_wall_blocks_frontier: tuong chan het 4 huong -> khong con frontier");
}

// Test 5: vùng đã phủ hết hoàn toàn (mọi ô VISITED, không còn UNKNOWN kề được) -> không còn frontier.
void test_fully_covered_no_frontier() {
    SharedMap map(2, 2);
    for (int x = 0; x < 2; ++x)
        for (int y = 0; y < 2; ++y) {
            map.markFree({x, y});
            map.markVisited({x, y});
        }
    CHECK(!Frontier::anyFrontierLeft(map), "test_fully_covered_no_frontier: map 2x2 da phu het -> khong con frontier");
}

// Test 6: ô OCCUPIED hoặc INFLATED_OBSTACLE không bao giờ được coi là frontier hay knownCell.
void test_occupied_not_frontier() {
    SharedMap map(3, 1);
    map.markFree({0, 0});
    map.markVisited({0, 0});
    map.markOccupied({1, 0});
    map.markFree({2, 0});
    map.markVisited({2, 0});

    auto fl = Frontier::findAll(map);
    for (auto& f : fl) {
        CHECK(!(f.knownCell == (Cell{1, 0})), "test_occupied_not_frontier: o OCCUPIED khong bao gio la knownCell");
    }
}

int main() {
    test_all_unknown_no_frontier();
    test_visited_next_to_unknown_is_frontier();
    test_free_unvisited_is_frontier();
    test_wall_blocks_frontier();
    test_fully_covered_no_frontier();
    test_occupied_not_frontier();

    std::cout << "\n=== Frontier tests: " << (g_failed == 0 ? "TAT CA PASS" : "CO LOI") << " ===\n";
    return g_failed == 0 ? 0 : 1;
}
