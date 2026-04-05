/*
 * Comprehensive test suite for maze_solver.cpp
 *
 * Tests cover:
 *   - Trivial / edge cases  (start == goal, all walls, 1x1 grid)
 *   - No-path scenarios     (goal enclosed by walls, maze split by wall)
 *   - Corridor mazes        (straight horizontal and vertical)
 *   - Open grids            (BFS & A* both reach optimal Manhattan-distance path)
 *   - Sample mazes          (small 7x10, large 20x20 from maze_solver.cpp)
 *   - Path validity         (contiguous, on open cells, correct endpoints)
 *   - Algorithm comparisons (BFS vs A* path length; A* vs BFS node count)
 *   - Dead-end mazes        (many branches, single valid path)
 *   - Forced-detour maze    (unique long route, no shortcuts)
 *
 * Build & run:
 *   make test
 *   -- or --
 *   g++ -std=c++17 -O2 -Wall -Wextra -o maze_solver_tests maze_solver_tests.cpp
 *   ./maze_solver_tests
 */

#define MAZE_SOLVER_TESTING
#include "maze_solver.cpp"

#include <cstdlib>
#include <iostream>
#include <string>

// ---------------------------------------------------------------------------
// Minimal test framework
// ---------------------------------------------------------------------------

static int g_pass = 0;
static int g_fail = 0;

static void check(bool condition, const std::string& description) {
    if (condition) {
        ++g_pass;
        std::cout << "  PASS: " << description << "\n";
    } else {
        ++g_fail;
        std::cout << "  FAIL: " << description << "\n";
    }
}

static void begin_test(const std::string& name) {
    std::cout << "\n[TEST] " << name << "\n";
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Returns true when 'path' is a valid solution in 'grid' from 'start' to 'goal':
//   1. Non-empty and endpoints match.
//   2. Every cell in the path is passable (not a wall).
//   3. Consecutive cells are exactly 1 step apart (4-directional).
static bool is_valid_path(const Grid& grid, const std::vector<Point>& path,
                          const Point& start, const Point& goal) {
    if (path.empty()) return false;
    if (path.front() != start || path.back() != goal) return false;
    for (const auto& p : path) {
        if (!passable(grid, p)) return false;
    }
    for (std::size_t i = 1; i < path.size(); ++i) {
        int dr = std::abs(path[i].row - path[i - 1].row);
        int dc = std::abs(path[i].col - path[i - 1].col);
        if (dr + dc != 1) return false;
    }
    return true;
}

// Run BFS, DFS, A*(Manhattan), A*(Euclidean) and A*(Zero) in one call.
struct AllResults {
    Result bfs_r, dfs_r, astar_m, astar_e, astar_z;
};

static AllResults run_all(const Grid& grid, const Point& start, const Point& goal) {
    return {
        bfs  (grid, start, goal),
        dfs  (grid, start, goal),
        astar(grid, start, goal, manhattan),
        astar(grid, start, goal, euclidean),
        astar(grid, start, goal, zero_heuristic)
    };
}

// ---------------------------------------------------------------------------
// Test 1 – Start equals goal
// ---------------------------------------------------------------------------
static void test_start_equals_goal() {
    begin_test("Start equals goal");
    Grid grid = {
        {1,1,1,1,1},
        {1,0,0,0,1},
        {1,0,0,0,1},
        {1,0,0,0,1},
        {1,1,1,1,1}
    };
    Point start{2, 2}, goal{2, 2};
    AllResults r = run_all(grid, start, goal);

    check(r.bfs_r.found,             "BFS: path found");
    check(r.bfs_r.path_length == 1,  "BFS: path length == 1");
    check(r.dfs_r.found,             "DFS: path found");
    check(r.dfs_r.path_length == 1,  "DFS: path length == 1");
    check(r.astar_m.found,           "A*(Manhattan): path found");
    check(r.astar_m.path_length == 1,"A*(Manhattan): path length == 1");
    check(r.astar_e.found,           "A*(Euclidean): path found");
    check(r.astar_e.path_length == 1,"A*(Euclidean): path length == 1");
    check(r.astar_z.found,           "A*(Zero): path found");
    check(r.astar_z.path_length == 1,"A*(Zero): path length == 1");
}

// ---------------------------------------------------------------------------
// Test 2 – No path: goal enclosed by walls on all four sides
// ---------------------------------------------------------------------------
static void test_no_path_goal_enclosed() {
    begin_test("No path: goal enclosed by walls");
    //  . . . . .
    //  . . # . .
    //  . # G # .     G = open cell but all 4 neighbours are walls
    //  . . # . .
    //  . . . . .
    Grid grid = {
        {0,0,0,0,0},
        {0,0,1,0,0},
        {0,1,0,1,0},
        {0,0,1,0,0},
        {0,0,0,0,0}
    };
    Point start{0, 0}, goal{2, 2};
    AllResults r = run_all(grid, start, goal);

    check(!r.bfs_r.found,              "BFS: no path to enclosed goal");
    check(r.bfs_r.path_length == 0,    "BFS: path length == 0");
    check(!r.dfs_r.found,              "DFS: no path to enclosed goal");
    check(!r.astar_m.found,            "A*(Manhattan): no path to enclosed goal");
    check(r.astar_m.path_length == 0,  "A*(Manhattan): path length == 0");
    check(!r.astar_z.found,            "A*(Zero): no path to enclosed goal");
}

// ---------------------------------------------------------------------------
// Test 3 – No path: maze split by a vertical wall column
// ---------------------------------------------------------------------------
static void test_no_path_split_maze() {
    begin_test("No path: maze split by vertical wall");
    Grid grid = {
        {0,0,1,0,0},
        {0,0,1,0,0},
        {0,0,1,0,0},
        {0,0,1,0,0},
        {0,0,1,0,0}
    };
    Point start{0, 0}, goal{0, 4};
    AllResults r = run_all(grid, start, goal);

    check(!r.bfs_r.found,  "BFS: no path across wall");
    check(!r.dfs_r.found,  "DFS: no path across wall");
    check(!r.astar_m.found,"A*(Manhattan): no path across wall");
    check(!r.astar_e.found,"A*(Euclidean): no path across wall");
}

// ---------------------------------------------------------------------------
// Test 4 – Straight horizontal corridor
// ---------------------------------------------------------------------------
static void test_straight_corridor_horizontal() {
    begin_test("Straight horizontal corridor");
    Grid grid = {
        {1,1,1,1,1,1,1},
        {1,0,0,0,0,0,1},
        {1,1,1,1,1,1,1}
    };
    Point start{1, 1}, goal{1, 5};
    // Unique path: (1,1)→(1,2)→(1,3)→(1,4)→(1,5)  → 5 cells
    AllResults r = run_all(grid, start, goal);

    check(r.bfs_r.found,                                    "BFS: path found");
    check(r.bfs_r.path_length == 5,                         "BFS: path length == 5");
    check(is_valid_path(grid, r.bfs_r.path, start, goal),   "BFS: path is valid");
    check(r.dfs_r.found,                                    "DFS: path found");
    check(r.dfs_r.path_length == 5,                         "DFS: path length == 5");
    check(is_valid_path(grid, r.dfs_r.path, start, goal),   "DFS: path is valid");
    check(r.astar_m.found,                                  "A*(Manhattan): path found");
    check(r.astar_m.path_length == 5,                       "A*(Manhattan): path length == 5");
    check(is_valid_path(grid, r.astar_m.path, start, goal), "A*(Manhattan): path is valid");
}

// ---------------------------------------------------------------------------
// Test 5 – Straight vertical corridor
// ---------------------------------------------------------------------------
static void test_straight_corridor_vertical() {
    begin_test("Straight vertical corridor");
    Grid grid = {
        {1,1,1},
        {1,0,1},
        {1,0,1},
        {1,0,1},
        {1,0,1},
        {1,0,1},
        {1,1,1}
    };
    Point start{1, 1}, goal{5, 1};
    // Unique path: 5 cells
    AllResults r = run_all(grid, start, goal);

    check(r.bfs_r.found,                                    "BFS: path found");
    check(r.bfs_r.path_length == 5,                         "BFS: path length == 5");
    check(is_valid_path(grid, r.bfs_r.path, start, goal),   "BFS: path is valid");
    check(r.astar_m.found,                                  "A*(Manhattan): path found");
    check(r.astar_m.path_length == 5,                       "A*(Manhattan): path length == 5");
    check(is_valid_path(grid, r.astar_m.path, start, goal), "A*(Manhattan): path is valid");
}

// ---------------------------------------------------------------------------
// Test 6 – Minimal grid: 1×1 open cell (start == goal)
// ---------------------------------------------------------------------------
static void test_single_cell_grid() {
    begin_test("Minimal grid: 1x1 open cell (start == goal)");
    Grid grid{{OPEN}};
    Point start{0, 0}, goal{0, 0};
    AllResults r = run_all(grid, start, goal);

    check(r.bfs_r.found,             "BFS: found in 1x1 grid");
    check(r.bfs_r.path_length == 1,  "BFS: path length == 1");
    check(r.dfs_r.found,             "DFS: found in 1x1 grid");
    check(r.dfs_r.path_length == 1,  "DFS: path length == 1");
    check(r.astar_m.found,           "A*(Manhattan): found in 1x1 grid");
    check(r.astar_m.path_length == 1,"A*(Manhattan): path length == 1");
}

// ---------------------------------------------------------------------------
// Test 7 – Fully open grid: optimal path equals Manhattan distance
// ---------------------------------------------------------------------------
static void test_open_grid() {
    begin_test("Fully open grid (no internal walls)");
    // 7x7, start corner to opposite corner; Manhattan distance = 12 → path = 13 cells
    Grid grid(7, std::vector<int>(7, OPEN));
    Point start{0, 0}, goal{6, 6};

    AllResults r = run_all(grid, start, goal);

    check(r.bfs_r.found,                                    "BFS: path found");
    check(r.bfs_r.path_length == 13,                        "BFS: optimal path length == 13");
    check(is_valid_path(grid, r.bfs_r.path, start, goal),   "BFS: path is valid");
    check(r.astar_m.found,                                  "A*(Manhattan): path found");
    check(r.astar_m.path_length == 13,                      "A*(Manhattan): optimal path length == 13");
    check(is_valid_path(grid, r.astar_m.path, start, goal), "A*(Manhattan): path is valid");
    check(r.bfs_r.path_length == r.astar_m.path_length,
          "BFS and A*(Manhattan) agree on optimal path length");
    check(r.bfs_r.path_length == r.astar_e.path_length,
          "BFS and A*(Euclidean) agree on optimal path length");
}

// ---------------------------------------------------------------------------
// Test 8 – Small sample maze (7×10) from maze_solver.cpp
// ---------------------------------------------------------------------------
static void test_small_maze() {
    begin_test("Small maze 7x10 (from make_small_maze)");
    Grid grid = make_small_maze();
    Point start{1, 1}, goal{5, 8};

    AllResults r = run_all(grid, start, goal);

    check(r.bfs_r.found,   "BFS: path found");
    check(r.dfs_r.found,   "DFS: path found");
    check(r.astar_m.found, "A*(Manhattan): path found");
    check(r.astar_e.found, "A*(Euclidean): path found");
    check(r.astar_z.found, "A*(Zero/Dijkstra): path found");

    check(is_valid_path(grid, r.bfs_r.path,   start, goal), "BFS: path is valid");
    check(is_valid_path(grid, r.dfs_r.path,   start, goal), "DFS: path is valid");
    check(is_valid_path(grid, r.astar_m.path, start, goal), "A*(Manhattan): path is valid");

    // All optimal algorithms must agree on path length
    check(r.bfs_r.path_length == r.astar_m.path_length,
          "BFS and A*(Manhattan) have same optimal path length");
    check(r.bfs_r.path_length == r.astar_e.path_length,
          "BFS and A*(Euclidean) have same optimal path length");
    check(r.bfs_r.path_length == r.astar_z.path_length,
          "BFS and A*(Zero) have same optimal path length");

    // Tighter heuristic → fewer or equal nodes explored
    check(r.astar_m.nodes_explored <= r.astar_e.nodes_explored,
          "A*(Manhattan) explores fewer or equal nodes than A*(Euclidean)");
    check(r.astar_m.nodes_explored <= r.astar_z.nodes_explored,
          "A*(Manhattan) explores fewer or equal nodes than A*(Zero)");
}

// ---------------------------------------------------------------------------
// Test 9 – Large sample maze (20×20) from maze_solver.cpp
// ---------------------------------------------------------------------------
static void test_large_maze() {
    begin_test("Large maze 20x20 (from make_large_maze)");
    Grid grid = make_large_maze();
    Point start{1, 1}, goal{17, 18};

    AllResults r = run_all(grid, start, goal);

    check(r.bfs_r.found,   "BFS: path found");
    check(r.dfs_r.found,   "DFS: path found");
    check(r.astar_m.found, "A*(Manhattan): path found");
    check(r.astar_e.found, "A*(Euclidean): path found");

    check(is_valid_path(grid, r.bfs_r.path,   start, goal), "BFS: path is valid");
    check(is_valid_path(grid, r.dfs_r.path,   start, goal), "DFS: path is valid");
    check(is_valid_path(grid, r.astar_m.path, start, goal), "A*(Manhattan): path is valid");

    check(r.bfs_r.path_length == r.astar_m.path_length,
          "BFS and A*(Manhattan) have same optimal path length");
    check(r.astar_m.nodes_explored <= r.bfs_r.nodes_explored,
          "A*(Manhattan) explores <= nodes than BFS");
}

// ---------------------------------------------------------------------------
// Test 10 – nodes_explored is always at least 1
// ---------------------------------------------------------------------------
static void test_nodes_explored_nonzero() {
    begin_test("Nodes explored is always >= 1");
    // Even for start == goal, the start node itself is dequeued and counted.
    Grid grid = {
        {1,1,1},
        {1,0,1},
        {1,1,1}
    };
    Point start{1, 1}, goal{1, 1};

    Result r_bfs    = bfs  (grid, start, goal);
    Result r_astar  = astar(grid, start, goal, manhattan);

    check(r_bfs.nodes_explored >= 1,   "BFS explores at least 1 node");
    check(r_astar.nodes_explored >= 1, "A*(Manhattan) explores at least 1 node");
}

// ---------------------------------------------------------------------------
// Test 11 – Path contiguity: every consecutive step is distance 1
// ---------------------------------------------------------------------------
static void test_path_contiguity() {
    begin_test("Path contiguity (every step is exactly 1 Manhattan unit)");
    Grid grid = make_small_maze();
    Point start{1, 1}, goal{5, 8};

    Result r_bfs   = bfs  (grid, start, goal);
    Result r_dfs   = dfs  (grid, start, goal);
    Result r_astar = astar(grid, start, goal, manhattan);

    check(is_valid_path(grid, r_bfs.path,   start, goal), "BFS: path is contiguous");
    check(is_valid_path(grid, r_dfs.path,   start, goal), "DFS: path is contiguous");
    check(is_valid_path(grid, r_astar.path, start, goal), "A*(Manhattan): path is contiguous");
}

// ---------------------------------------------------------------------------
// Test 12 – All-wall grid: goal is a wall, no path possible
// ---------------------------------------------------------------------------
static void test_all_walls() {
    begin_test("All-wall grid (no open cells)");
    Grid grid = {
        {1,1,1},
        {1,1,1},
        {1,1,1}
    };
    Point start{0, 0}, goal{2, 2};
    // goal is a wall → passable() returns false → it is never enqueued → found = false
    AllResults r = run_all(grid, start, goal);

    check(!r.bfs_r.found,  "BFS: no path in all-wall grid");
    check(!r.dfs_r.found,  "DFS: no path in all-wall grid");
    check(!r.astar_m.found,"A*(Manhattan): no path in all-wall grid");
}

// ---------------------------------------------------------------------------
// Test 13 – Maze with many dead ends, single valid path
// ---------------------------------------------------------------------------
static void test_maze_with_dead_ends() {
    begin_test("Maze with dead ends (single valid path)");
    //  # # # # # # # # #
    //  # . # . # . # . #   ← dead-end stubs at cols 1,3,5 in rows 1-2
    //  # . # . # . # . #
    //  # . . . . . . . #   ← single connector row at row 3
    //  # # # # # # # # #
    Grid grid = {
        {1,1,1,1,1,1,1,1,1},
        {1,0,1,0,1,0,1,0,1},
        {1,0,1,0,1,0,1,0,1},
        {1,0,0,0,0,0,0,0,1},
        {1,1,1,1,1,1,1,1,1}
    };
    Point start{1, 1}, goal{1, 7};
    // Optimal path (length 11):
    //   (1,1)→(2,1)→(3,1)→(3,2)→(3,3)→(3,4)→(3,5)→(3,6)→(3,7)→(2,7)→(1,7)
    AllResults r = run_all(grid, start, goal);

    check(r.bfs_r.found,   "BFS: path found");
    check(r.dfs_r.found,   "DFS: path found");
    check(r.astar_m.found, "A*(Manhattan): path found");

    check(is_valid_path(grid, r.bfs_r.path,   start, goal), "BFS: path is valid");
    check(is_valid_path(grid, r.astar_m.path, start, goal), "A*(Manhattan): path is valid");

    check(r.bfs_r.path_length == 11,
          "BFS: correct optimal path length (11)");
    check(r.bfs_r.path_length == r.astar_m.path_length,
          "BFS and A*(Manhattan) agree on optimal path length");
}

// ---------------------------------------------------------------------------
// Test 14 – Goal adjacent to start (single step)
// ---------------------------------------------------------------------------
static void test_goal_adjacent_to_start() {
    begin_test("Goal adjacent to start (1 step away)");
    Grid grid = {
        {0,0,0},
        {0,0,0},
        {0,0,0}
    };
    Point start{1, 1}, goal{1, 2};
    AllResults r = run_all(grid, start, goal);

    check(r.bfs_r.found,             "BFS: path found");
    check(r.bfs_r.path_length == 2,  "BFS: path length == 2");
    check(r.astar_m.found,           "A*(Manhattan): path found");
    check(r.astar_m.path_length == 2,"A*(Manhattan): path length == 2");
    check(r.dfs_r.found,             "DFS: path found");
    check(r.dfs_r.path_length == 2,  "DFS: path length == 2");
}

// ---------------------------------------------------------------------------
// Test 15 – A*(Manhattan) efficiency: fewer nodes than BFS
// ---------------------------------------------------------------------------
static void test_astar_efficiency_vs_bfs() {
    begin_test("A*(Manhattan) explores fewer nodes than BFS");
    // Wide 5x20 grid: start is at the left-centre, goal at the right-centre.
    // BFS fans out in all four directions; A*(Manhattan) focuses rightward.
    // Expected: BFS ~94 nodes, A*(Manhattan) ~20 nodes (just the optimal path).
    Grid grid(5, std::vector<int>(20, OPEN));
    Point start{2, 0}, goal{2, 19};

    Result r_bfs     = bfs  (grid, start, goal);
    Result r_astar_m = astar(grid, start, goal, manhattan);

    check(r_bfs.found,    "BFS: path found");
    check(r_astar_m.found,"A*(Manhattan): path found");
    check(r_bfs.path_length == r_astar_m.path_length,
          "Same optimal path length");
    check(r_astar_m.nodes_explored < r_bfs.nodes_explored,
          "A*(Manhattan) explores strictly fewer nodes than BFS");
}

// ---------------------------------------------------------------------------
// Test 16 – Forced-detour maze: direct route blocked, must go around
// ---------------------------------------------------------------------------
static void test_forced_detour_maze() {
    begin_test("Forced-detour maze (direct shortcut blocked)");
    //  # # # # # # #
    //  # S # . . . #    S=(1,1)  wall at (1,2) blocks direct eastward route
    //  # . # . # . #
    //  # . . . # . #
    //  # # # # # # #    G=(1,5)
    Grid grid = {
        {1,1,1,1,1,1,1},
        {1,0,1,0,0,0,1},
        {1,0,1,0,1,0,1},
        {1,0,0,0,1,0,1},
        {1,1,1,1,1,1,1}
    };
    Point start{1, 1}, goal{1, 5};
    // Only path (length 9):
    //   (1,1)→(2,1)→(3,1)→(3,2)→(3,3)→(2,3)→(1,3)→(1,4)→(1,5)
    AllResults r = run_all(grid, start, goal);

    check(r.bfs_r.found,   "BFS: path found");
    check(r.dfs_r.found,   "DFS: path found");
    check(r.astar_m.found, "A*(Manhattan): path found");

    check(is_valid_path(grid, r.bfs_r.path,   start, goal), "BFS: path is valid");
    check(is_valid_path(grid, r.dfs_r.path,   start, goal), "DFS: path is valid");
    check(is_valid_path(grid, r.astar_m.path, start, goal), "A*(Manhattan): path is valid");

    check(r.bfs_r.path_length == 9,
          "BFS: correct forced-detour path length (9)");
    check(r.bfs_r.path_length == r.astar_m.path_length,
          "BFS and A*(Manhattan) agree on optimal path length");
}

// ---------------------------------------------------------------------------
// Main: run all tests and report summary
// ---------------------------------------------------------------------------

int main() {
    std::cout << "====================================================\n";
    std::cout << "  Maze Solver – Test Suite\n";
    std::cout << "====================================================\n";

    test_start_equals_goal();
    test_no_path_goal_enclosed();
    test_no_path_split_maze();
    test_straight_corridor_horizontal();
    test_straight_corridor_vertical();
    test_single_cell_grid();
    test_open_grid();
    test_small_maze();
    test_large_maze();
    test_nodes_explored_nonzero();
    test_path_contiguity();
    test_all_walls();
    test_maze_with_dead_ends();
    test_goal_adjacent_to_start();
    test_astar_efficiency_vs_bfs();
    test_forced_detour_maze();

    int total = g_pass + g_fail;
    std::cout << "\n====================================================\n";
    std::cout << "  Results: " << g_pass << " / " << total << " passed";
    if (g_fail == 0) {
        std::cout << "  ✓  ALL TESTS PASSED\n";
    } else {
        std::cout << "  ✗  " << g_fail << " FAILED\n";
    }
    std::cout << "====================================================\n";

    return g_fail == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
