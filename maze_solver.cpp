/*
 * Intelligent Maze Solver: BFS vs DFS vs A*
 *
 * Demonstrates and compares three search algorithms on a 2-D grid maze:
 *   - Breadth-First Search  (BFS)
 *   - Depth-First Search    (DFS)
 *   - A* with Manhattan distance heuristic
 *   - A* with Euclidean distance heuristic  (heuristic impact study)
 *
 * Metrics reported per algorithm:
 *   - Nodes explored (cells popped from the frontier)
 *   - Path length    (cells on the solution path, including start & goal)
 *   - Whether a path was found
 *
 * Build:  g++ -std=c++17 -O2 -o maze_solver maze_solver.cpp
 * Run:    ./maze_solver
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <queue>
#include <stack>
#include <string>
#include <unordered_map>
#include <vector>

// ---------------------------------------------------------------------------
// Maze representation
// ---------------------------------------------------------------------------

using Grid = std::vector<std::vector<int>>;

struct Point {
    int row, col;
    bool operator==(const Point& o) const { return row == o.row && col == o.col; }
    bool operator!=(const Point& o) const { return !(*this == o); }
    bool operator< (const Point& o) const {
        return row != o.row ? row < o.row : col < o.col;
    }
};

// Hash for Point so it can be used in unordered_map
struct PointHash {
    std::size_t operator()(const Point& p) const {
        return std::hash<int>()(p.row) * 10007 + std::hash<int>()(p.col);
    }
};

// Maze cell values
static constexpr int OPEN  = 0;
static constexpr int WALL  = 1;
static constexpr int START = 2;
static constexpr int GOAL  = 3;
static constexpr int PATH  = 4;

// Four-directional movement (Up, Down, Left, Right)
static const std::vector<Point> DIRECTIONS = {{-1,0},{1,0},{0,-1},{0,1}};

// ---------------------------------------------------------------------------
// Result of one algorithm run
// ---------------------------------------------------------------------------

struct Result {
    bool        found         = false;
    int         nodes_explored = 0;
    int         path_length   = 0;
    std::vector<Point> path;
    double      elapsed_ms    = 0.0;
};

// ---------------------------------------------------------------------------
// Utility helpers
// ---------------------------------------------------------------------------

static bool in_bounds(const Grid& grid, const Point& p) {
    return p.row >= 0 && p.row < (int)grid.size() &&
           p.col >= 0 && p.col < (int)grid[0].size();
}

static bool passable(const Grid& grid, const Point& p) {
    return in_bounds(grid, p) && grid[p.row][p.col] != WALL;
}

// Reconstruct path from parent map
static std::vector<Point> reconstruct(
    const std::unordered_map<Point, Point, PointHash>& parent,
    const Point& start,
    const Point& goal)
{
    std::vector<Point> path;
    Point cur = goal;
    while (cur != start) {
        path.push_back(cur);
        cur = parent.at(cur);
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

// ---------------------------------------------------------------------------
// BFS
// ---------------------------------------------------------------------------

Result bfs(const Grid& grid, const Point& start, const Point& goal) {
    auto t0 = std::chrono::high_resolution_clock::now();

    Result res;
    std::unordered_map<Point, Point, PointHash> parent;
    std::queue<Point> frontier;

    frontier.push(start);
    parent[start] = start;

    while (!frontier.empty()) {
        Point cur = frontier.front();
        frontier.pop();
        ++res.nodes_explored;

        if (cur == goal) {
            res.found = true;
            res.path  = reconstruct(parent, start, goal);
            res.path_length = (int)res.path.size();
            break;
        }

        for (const auto& dir : DIRECTIONS) {
            Point next{cur.row + dir.row, cur.col + dir.col};
            if (passable(grid, next) && parent.find(next) == parent.end()) {
                parent[next] = cur;
                frontier.push(next);
            }
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    res.elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return res;
}

// ---------------------------------------------------------------------------
// DFS (iterative, uses a stack; explores in LIFO order)
// ---------------------------------------------------------------------------

Result dfs(const Grid& grid, const Point& start, const Point& goal) {
    auto t0 = std::chrono::high_resolution_clock::now();

    Result res;
    std::unordered_map<Point, Point, PointHash> parent;
    std::stack<Point> frontier;

    frontier.push(start);
    parent[start] = start;

    while (!frontier.empty()) {
        Point cur = frontier.top();
        frontier.pop();
        ++res.nodes_explored;

        if (cur == goal) {
            res.found = true;
            res.path  = reconstruct(parent, start, goal);
            res.path_length = (int)res.path.size();
            break;
        }

        for (const auto& dir : DIRECTIONS) {
            Point next{cur.row + dir.row, cur.col + dir.col};
            if (passable(grid, next) && parent.find(next) == parent.end()) {
                parent[next] = cur;
                frontier.push(next);
            }
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    res.elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return res;
}

// ---------------------------------------------------------------------------
// A* (generic; accepts a heuristic function)
// ---------------------------------------------------------------------------

using Heuristic = std::function<double(const Point&, const Point&)>;

// Manhattan distance heuristic (admissible for 4-directional grids)
double manhattan(const Point& a, const Point& b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}

// Euclidean distance heuristic (also admissible; less informed than Manhattan
// for 4-directional movement, leading to more explored nodes)
double euclidean(const Point& a, const Point& b) {
    double dr = a.row - b.row;
    double dc = a.col - b.col;
    return std::sqrt(dr * dr + dc * dc);
}

// Zero heuristic – degenerates A* into Dijkstra / uniform-cost search
double zero_heuristic(const Point& /*a*/, const Point& /*b*/) {
    return 0.0;
}

Result astar(const Grid& grid, const Point& start, const Point& goal,
             const Heuristic& h) {
    auto t0 = std::chrono::high_resolution_clock::now();

    Result res;

    // (f-score, g-score, point)
    using PQItem = std::tuple<double, double, Point>;
    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> open;

    std::unordered_map<Point, double,    PointHash> g_cost;
    std::unordered_map<Point, Point,     PointHash> parent;

    g_cost[start] = 0.0;
    parent[start] = start;
    open.push({h(start, goal), 0.0, start});

    while (!open.empty()) {
        auto [f, g, cur] = open.top();
        open.pop();

        // Skip stale entries
        if (g > g_cost[cur] + 1e-9) continue;

        ++res.nodes_explored;

        if (cur == goal) {
            res.found = true;
            res.path  = reconstruct(parent, start, goal);
            res.path_length = (int)res.path.size();
            break;
        }

        for (const auto& dir : DIRECTIONS) {
            Point next{cur.row + dir.row, cur.col + dir.col};
            if (!passable(grid, next)) continue;

            double tentative_g = g_cost[cur] + 1.0;
            if (g_cost.find(next) == g_cost.end() || tentative_g < g_cost[next]) {
                g_cost[next] = tentative_g;
                parent[next] = cur;
                open.push({tentative_g + h(next, goal), tentative_g, next});
            }
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    res.elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return res;
}

// ---------------------------------------------------------------------------
// Maze printing
// ---------------------------------------------------------------------------

static void print_maze(const Grid& grid, const std::vector<Point>& path,
                       const Point& start, const Point& goal) {
    Grid display = grid;
    for (const auto& p : path) display[p.row][p.col] = PATH;
    display[start.row][start.col] = START;
    display[goal.row][goal.col]   = GOAL;

    std::cout << "  ";
    for (int c = 0; c < (int)grid[0].size(); ++c)
        std::cout << (c % 10);
    std::cout << "\n";

    for (int r = 0; r < (int)display.size(); ++r) {
        std::cout << (r % 10) << " ";
        for (int c = 0; c < (int)display[r].size(); ++c) {
            switch (display[r][c]) {
                case WALL:  std::cout << '#'; break;
                case OPEN:  std::cout << '.'; break;
                case START: std::cout << 'S'; break;
                case GOAL:  std::cout << 'G'; break;
                case PATH:  std::cout << '*'; break;
            }
        }
        std::cout << "\n";
    }
}

// ---------------------------------------------------------------------------
// Print comparison table
// ---------------------------------------------------------------------------

static void print_comparison(const std::vector<std::pair<std::string, Result>>& results) {
    const int w1 = 28, w2 = 10, w3 = 12, w4 = 12;
    std::string sep(w1 + w2 + w3 + w4 + 5, '-');

    std::cout << "\n" << sep << "\n";
    std::cout << std::left
              << std::setw(w1) << "Algorithm"
              << std::setw(w2) << "Found?"
              << std::setw(w3) << "Nodes Expl."
              << std::setw(w4) << "Path Length"
              << "\n" << sep << "\n";

    for (const auto& [name, res] : results) {
        std::cout << std::left
                  << std::setw(w1) << name
                  << std::setw(w2) << (res.found ? "YES" : "NO")
                  << std::setw(w3) << res.nodes_explored
                  << std::setw(w4) << (res.found ? res.path_length : 0)
                  << "\n";
    }
    std::cout << sep << "\n";
}

// ---------------------------------------------------------------------------
// Heuristic impact study
// ---------------------------------------------------------------------------

static void heuristic_impact_study(const Grid& grid,
                                   const Point& start,
                                   const Point& goal) {
    std::cout << "\n=== Heuristic Impact Study (A*) ===\n";

    struct HEntry { std::string name; Heuristic fn; };
    std::vector<HEntry> heuristics = {
        {"A* (Zero / Dijkstra)",   zero_heuristic},
        {"A* (Euclidean)",         euclidean},
        {"A* (Manhattan)",         manhattan},
    };

    std::vector<std::pair<std::string, Result>> results;
    for (const auto& h : heuristics) {
        Result r = astar(grid, start, goal, h.fn);
        results.emplace_back(h.name, r);
    }
    print_comparison(results);

    std::cout << "\nInterpretation:\n"
              << "  Manhattan heuristic is perfectly informed for 4-directional\n"
              << "  grids (no diagonal moves), so it directs the search most\n"
              << "  efficiently and explores the fewest nodes.\n"
              << "  Euclidean underestimates by allowing 'diagonal shortcuts' that\n"
              << "  don't exist, so it explores more nodes than Manhattan but still\n"
              << "  far fewer than the zero-heuristic (Dijkstra).\n";
}

// ---------------------------------------------------------------------------
// Sample mazes
// ---------------------------------------------------------------------------

// Maze layout key:  0=open  1=wall
// 'S' is injected at start / goal positions below
static Grid make_small_maze() {
    return {
        {1,1,1,1,1,1,1,1,1,1},
        {1,0,0,0,1,0,0,0,0,1},
        {1,0,1,0,1,0,1,1,0,1},
        {1,0,1,0,0,0,1,0,0,1},
        {1,0,1,1,1,1,1,0,1,1},
        {1,0,0,0,0,0,0,0,0,1},
        {1,1,1,1,1,1,1,1,1,1},
    };
}

static Grid make_large_maze() {
    return {
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
        {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,1,1,1,1,0,1,1,1,1,1,0,1,1,1,0,1,0,1},
        {1,0,1,0,0,0,0,1,0,0,0,1,0,0,0,1,0,1,0,1},
        {1,0,1,0,1,1,1,1,0,1,0,1,1,1,0,1,0,1,0,1},
        {1,0,0,0,1,0,0,0,0,1,0,0,0,0,0,1,0,0,0,1},
        {1,0,1,1,1,0,1,1,1,1,1,1,1,1,0,1,1,1,0,1},
        {1,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,1},
        {1,1,1,1,1,0,1,0,1,1,1,1,0,1,1,1,1,1,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,0,1},
        {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,1,1,0,1,1,1,1,1,1,0,1,1,1,1,0,1,0,1},
        {1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,1},
        {1,0,1,0,1,1,1,1,1,0,1,1,1,1,0,1,0,1,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,1,1,1,0,1,1,1,1,0,1,1,1,1,1,1,1,0,1},
        {1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1},
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    };
}

// ---------------------------------------------------------------------------
// Run all algorithms on a maze and display results
// ---------------------------------------------------------------------------

static void run_on_maze(const std::string& label,
                        Grid grid,
                        const Point& start,
                        const Point& goal) {
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "Maze: " << label << "\n";
    std::cout << "Start: (" << start.row << "," << start.col << ")  "
              << "Goal: ("  << goal.row  << "," << goal.col  << ")\n";
    std::cout << std::string(60, '=') << "\n";

    // Show the bare maze first
    std::cout << "\n[Empty maze]\n";
    print_maze(grid, {}, start, goal);

    // Run algorithms
    Result r_bfs  = bfs  (grid, start, goal);
    Result r_dfs  = dfs  (grid, start, goal);
    Result r_astar_m = astar(grid, start, goal, manhattan);
    Result r_astar_e = astar(grid, start, goal, euclidean);

    // Show paths
    std::cout << "\n[BFS path]\n";
    if (r_bfs.found) print_maze(grid, r_bfs.path, start, goal);
    else             std::cout << "  No path found.\n";

    std::cout << "\n[DFS path]\n";
    if (r_dfs.found) print_maze(grid, r_dfs.path, start, goal);
    else             std::cout << "  No path found.\n";

    std::cout << "\n[A* (Manhattan) path]\n";
    if (r_astar_m.found) print_maze(grid, r_astar_m.path, start, goal);
    else                 std::cout << "  No path found.\n";

    // Nodes explored comparison
    std::cout << "\n--- Nodes Explored Comparison ---\n";
    std::vector<std::pair<std::string, Result>> comparison = {
        {"BFS",              r_bfs},
        {"DFS",              r_dfs},
        {"A* (Manhattan)",   r_astar_m},
        {"A* (Euclidean)",   r_astar_e},
    };
    print_comparison(comparison);

    // Heuristic impact study
    heuristic_impact_study(grid, start, goal);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main() {
    std::cout << "====================================================\n";
    std::cout << "  Intelligent Maze Solver: BFS vs DFS vs A*\n";
    std::cout << "====================================================\n";

    // --- Small maze ---
    Grid small = make_small_maze();
    Point s_start{1, 1};
    Point s_goal {5, 8};
    run_on_maze("Small (7x10)", small, s_start, s_goal);

    // --- Large maze ---
    Grid large = make_large_maze();
    Point l_start{1, 1};
    Point l_goal {17, 18};
    run_on_maze("Large (20x20)", large, l_start, l_goal);

    std::cout << "\n====================================================\n";
    std::cout << "  Summary of Key Observations\n";
    std::cout << "====================================================\n";
    std::cout
        << "\n"
        << "  BFS guarantees the SHORTEST path (fewest steps) but may\n"
        << "  explore many nodes in open mazes.\n\n"
        << "  DFS finds A path quickly in some topologies, but the path\n"
        << "  is rarely optimal and node count varies widely.\n\n"
        << "  A* (Manhattan) is optimal and typically explores the\n"
        << "  fewest nodes by using a tight, admissible heuristic.\n\n"
        << "  A* (Euclidean) is also optimal but explores slightly more\n"
        << "  nodes than A*(Manhattan) on 4-directional grids because\n"
        << "  the Euclidean distance can underestimate more.\n\n"
        << "  Heuristic quality directly controls A* efficiency:\n"
        << "    Zero heuristic  -> explores most nodes (same as Dijkstra)\n"
        << "    Euclidean       -> moderate improvement\n"
        << "    Manhattan       -> best for 4-directional grids\n"
        << "\n";

    return 0;
}
