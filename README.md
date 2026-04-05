# Maze-search-algorithms

A comparative exploration of BFS, DFS, and A\* navigating mazes, measuring efficiency through nodes explored, path length, and heuristic intelligence.

## Overview

This C++ project implements three classic graph-search algorithms on 2-D grid mazes and produces a side-by-side comparison of:

| Metric | BFS | DFS | A\* (Manhattan) | A\* (Euclidean) |
|---|---|---|---|---|
| Guarantees shortest path? | ✅ | ❌ | ✅ | ✅ |
| Nodes explored (typical) | Medium | High/variable | **Lowest** | Low |
| Memory usage | Higher | Lower | Medium | Medium |

### Algorithms implemented

| Algorithm | Strategy | Data structure |
|---|---|---|
| **BFS** | Level-by-level expansion | Queue (FIFO) |
| **DFS** | Depth-first expansion | Stack (LIFO) |
| **A\* (Manhattan)** | f = g + \|Δrow\| + \|Δcol\| | Min-heap (priority queue) |
| **A\* (Euclidean)** | f = g + √(Δrow²+Δcol²) | Min-heap (priority queue) |
| **A\* (Zero / Dijkstra)** | f = g + 0 | Min-heap (priority queue) |

### Heuristic impact study

The heuristic impact study runs all three A\* variants on the same maze and shows how a tighter, more informed heuristic directly reduces the number of nodes explored while still guaranteeing an optimal path:

```
=== Heuristic Impact Study (A*) ===
Algorithm                   Found?    Nodes Expl. Path Length
-------------------------------------------------------------------
A* (Zero / Dijkstra)        YES       173         34
A* (Euclidean)              YES       92          34
A* (Manhattan)              YES       68          34
```

## Sample output (Large 20×20 maze)

```
--- Nodes Explored Comparison ---
Algorithm                   Found?    Nodes Expl. Path Length
-------------------------------------------------------------------
BFS                         YES       172         34
DFS                         YES       127         106
A* (Manhattan)              YES       68          34
A* (Euclidean)              YES       92          34
-------------------------------------------------------------------
```

## Build & Run

**Requirements:** a C++17-compatible compiler (g++ 7+, clang++ 5+, MSVC 2017+).

```bash
# Using make
make

# Or directly with g++
g++ -std=c++17 -O2 -o maze_solver maze_solver.cpp

# Run
./maze_solver
```

## Key observations

- **BFS** always finds the *shortest* path but explores many nodes in open areas.
- **DFS** explores fewer nodes on average but returns a sub-optimal (often very long) path.
- **A\* (Manhattan)** explores the *fewest* nodes among all variants because the Manhattan distance is a *perfectly tight admissible heuristic* for 4-directional grids.
- **A\* (Euclidean)** is also optimal but explores more nodes than Manhattan because the Euclidean metric can underestimate the true cost (it assumes diagonal moves that don't exist in a 4-directional grid).
- **A\* (Zero / Dijkstra)** degenerates to uniform-cost search and explores the most nodes of the three A\* variants, approaching BFS in behaviour.

## File structure

```
.
├── maze_solver.cpp   # Full implementation (BFS, DFS, A*, heuristic study)
├── Makefile          # Convenience build targets
└── README.md
```

