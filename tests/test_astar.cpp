// test_astar.cpp - Unit test cho module A* (cpp/include/planning/AStar.hpp)
//
// Build & chạy độc lập (không cần CMake/CTest):
//   g++ -std=c++17 -I cpp/include -I cpp/third_party tests/test_astar.cpp -lpthread -o /tmp/test_astar
//   /tmp/test_astar
//
// Hoặc qua CTest: cmake -DBUILD_TESTS=ON .. && make && ctest

#include <iostream>
#include <cassert>
#include "map/SharedMap.hpp"
#include "planning/AStar.hpp"

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

// Test 1: đường thẳng không vật cản phải tìm được đường ngắn nhất theo Manhattan.
void test_basic_path() {
    SharedMap map(5, 5);
    for (int x = 0; x < 5; ++x)
        for (int y = 0; y < 5; ++y) {
            map.markFree({x, y});
            map.markVisited({x, y});
        }

    auto result = AStar::search(map, Cell{0, 0}, Cell{4, 4}, false);
    CHECK(result.found, "test_basic_path: tim duoc duong");
    CHECK(result.path.size() == 9, "test_basic_path: do dai duong = 9 (Manhattan dist 8 + 1 diem dau)");
    CHECK(result.path.front() == (Cell{0, 0}), "test_basic_path: diem dau dung");
    CHECK(result.path.back() == (Cell{4, 4}), "test_basic_path: diem cuoi dung");
}

// Test 2: có vật cản chặn đường thẳng, A* phải đi vòng.
void test_path_around_obstacle() {
    SharedMap map(5, 5);
    for (int x = 0; x < 5; ++x)
        for (int y = 0; y < 5; ++y) {
            map.markFree({x, y});
            map.markVisited({x, y});
        }
    // tường dọc tại x=2, trừ lỗ ở y=4
    for (int y = 0; y < 4; ++y) map.markOccupied({2, y});

    auto result = AStar::search(map, Cell{0, 0}, Cell{4, 0}, false);
    CHECK(result.found, "test_path_around_obstacle: tim duoc duong vong qua vat can");
    if (result.found) {
        bool hitsWall = false;
        for (auto& c : result.path) {
            if (c.x == 2 && c.y < 4) hitsWall = true;
        }
        CHECK(!hitsWall, "test_path_around_obstacle: duong di khong xuyen qua vat can");
    }
}

// Test 3: không có đường đi khả thi -> A* phải trả về found=false, không treo.
void test_no_path_fully_blocked() {
    SharedMap map(3, 3);
    for (int x = 0; x < 3; ++x)
        for (int y = 0; y < 3; ++y) {
            map.markFree({x, y});
            map.markVisited({x, y});
        }
    // chặn kín ô (1,1) bằng tường 4 hướng (mô phỏng bị cô lập hoàn toàn dù không OCCUPIED)
    for (int d = 0; d < 4; ++d) map.setWall(1, 1, d, true);

    auto result = AStar::search(map, Cell{0, 0}, Cell{1, 1}, false);
    CHECK(!result.found, "test_no_path_fully_blocked: dung tra ve khong tim duoc duong");
}

// Test 4: đích là ô UNKNOWN (frontier) - chỉ được phép khi allowUnknownGoal=true.
void test_unknown_goal_allowed() {
    SharedMap map(3, 3);
    map.markFree({0, 0});
    map.markVisited({0, 0});
    map.markFree({1, 0});
    map.markVisited({1, 0});
    // (2,0) vẫn là UNKNOWN - đóng vai trò frontier

    auto resultAllow = AStar::search(map, Cell{0, 0}, Cell{2, 0}, true);
    CHECK(resultAllow.found, "test_unknown_goal_allowed: cho phep di toi UNKNOWN khi allowUnknownGoal=true");

    auto resultDeny = AStar::search(map, Cell{0, 0}, Cell{2, 0}, false);
    CHECK(!resultDeny.found, "test_unknown_goal_allowed: khong cho phep di toi UNKNOWN khi allowUnknownGoal=false");
}

// Test 5: điểm đầu = điểm cuối -> trả về đường đi 1 phần tử ngay lập tức.
void test_start_equals_goal() {
    SharedMap map(3, 3);
    map.markFree({1, 1});
    map.markVisited({1, 1});
    auto result = AStar::search(map, Cell{1, 1}, Cell{1, 1}, false);
    CHECK(result.found, "test_start_equals_goal: tim duoc (tam thuong)");
    CHECK(result.path.size() == 1, "test_start_equals_goal: duong di chi co 1 o");
}

// Test 6: A* tôn trọng tường nội bộ (không chỉ vật cản OCCUPIED).
void test_respects_walls() {
    SharedMap map(2, 1);
    map.markFree({0, 0});
    map.markVisited({0, 0});
    map.markFree({1, 0});
    map.markVisited({1, 0});
    map.setWall(0, 0, 1, true); // tường hướng Đông giữa (0,0) và (1,0)

    auto result = AStar::search(map, Cell{0, 0}, Cell{1, 0}, false);
    CHECK(!result.found, "test_respects_walls: khong di xuyen qua tuong noi bo du khong co vat can");
}

int main() {
    test_basic_path();
    test_path_around_obstacle();
    test_no_path_fully_blocked();
    test_unknown_goal_allowed();
    test_start_equals_goal();
    test_respects_walls();

    std::cout << "\n=== A* tests: " << (g_failed == 0 ? "TAT CA PASS" : "CO LOI") << " ===\n";
    return g_failed == 0 ? 0 : 1;
}
