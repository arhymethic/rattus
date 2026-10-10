#include "FloodFill.h"
#include <iostream>
#include <algorithm>
#include <cstring>

#ifdef SIMULATION
#include "API.h"
#endif

namespace {
    char dirToChar(Direction d) {
        switch (d) {
            case NORTH: return 'n';
            case EAST:  return 'e';
            case SOUTH: return 's';
            case WEST:  return 'w';
            default:    return 'n';
        }
    }
}

// ============================================================================
// Constructor & Initialization
// ============================================================================

FloodFill::FloodFill()
    : width_(DEFAULT_MAZE_WIDTH), height_(DEFAULT_MAZE_HEIGHT),
      goalCount_(0), speedPathLen_(0) {
    init(DEFAULT_MAZE_WIDTH, DEFAULT_MAZE_HEIGHT);
}

void FloodFill::init(int width, int height) {
    width_ = std::clamp(width, 2, (int)MAX_MAZE_DIM);
    height_ = std::clamp(height, 2, (int)MAX_MAZE_DIM);

    std::memset(walls_, 0, sizeof(walls_));
    std::memset(visited_, 0, sizeof(visited_));
    speedPathLen_ = 0;

    for (int x = 0; x < width_; ++x) {
        for (int y = 0; y < height_; ++y) {
            dist_[x][y] = 999;
        }
    }

    // Outer perimeter boundary walls only
    for (int x = 0; x < width_; ++x) {
        walls_[x][0] |= WALL_SOUTH;
        walls_[x][height_ - 1] |= WALL_NORTH;
    }
    for (int y = 0; y < height_; ++y) {
        walls_[0][y] |= WALL_WEST;
        walls_[width_ - 1][y] |= WALL_EAST;
    }

    // BUG-3 FIX: No hardcoded East wall at (0,0).
    // The bot discovers all interior walls via sensors.

    setGoalsCenter();
}

// ============================================================================
// Goal Management
// ============================================================================

void FloodFill::setGoalsCenter() {
    goalCount_ = 0;
    int midX1 = (width_ - 1) / 2;
    int midX2 = width_ / 2;
    int midY1 = (height_ - 1) / 2;
    int midY2 = height_ / 2;

    goalCells_[goalCount_++] = {midX1, midY1};
    if (midX2 != midX1) goalCells_[goalCount_++] = {midX2, midY1};
    if (midY2 != midY1) goalCells_[goalCount_++] = {midX1, midY2};
    if (midX2 != midX1 && midY2 != midY1) goalCells_[goalCount_++] = {midX2, midY2};

    setGoalsAndFlood();
}

void FloodFill::setGoalsStart() {
    goalCount_ = 1;
    goalCells_[0] = {START_X, START_Y};
    setGoalsAndFlood();
}

void FloodFill::setGoalsAndFlood() {
    flood();
}

bool FloodFill::isGoal(int x, int y) const {
    for (int i = 0; i < goalCount_; ++i) {
        if (goalCells_[i].x == x && goalCells_[i].y == y) return true;
    }
    return false;
}

// ============================================================================
// MMS Visual Telemetry
// ============================================================================

void FloodFill::initMMSVisuals() {
#ifdef SIMULATION
    for (int x = 0; x < width_; ++x) {
        for (int y = 0; y < height_; ++y) {
            if (dist_[x][y] < 999) {
                API::setText(x, y, std::to_string(dist_[x][y]));
            }
            if (isGoal(x, y)) {
                API::setColor(x, y, 'G');
            }
        }
    }
    API::setColor(START_X, START_Y, 'Y');
#endif
}

// ============================================================================
// Wall Management
// ============================================================================

void FloodFill::setWallInternal(int x, int y, Direction dir) {
    if (!isValidCoord(x, y)) return;

    walls_[x][y] |= DIR_WALL_MASK[dir];

    int nx = x + DIR_DX[dir];
    int ny = y + DIR_DY[dir];
    if (isValidCoord(nx, ny)) {
        Direction opposite = static_cast<Direction>((dir + 2) % 4);
        walls_[nx][ny] |= DIR_WALL_MASK[opposite];
    }

#ifdef SIMULATION
    API::setWall(x, y, dirToChar(dir));
#endif
}

bool FloodFill::hasWall(int x, int y, Direction dir) const {
    if (!isValidCoord(x, y)) return true;
    return (walls_[x][y] & DIR_WALL_MASK[dir]) != 0;
}

bool FloodFill::updateWalls(int x, int y, Direction heading,
                            bool wallFront, bool wallLeft, bool wallRight) {
    bool newWallFound = false;

    Direction frontDir = heading;
    Direction rightDir = static_cast<Direction>((heading + 1) % 4);
    Direction leftDir  = static_cast<Direction>((heading + 3) % 4);

    if (wallFront && !hasWall(x, y, frontDir)) {
        setWallInternal(x, y, frontDir);
        newWallFound = true;
    }
    if (wallRight && !hasWall(x, y, rightDir)) {
        setWallInternal(x, y, rightDir);
        newWallFound = true;
    }
    if (wallLeft && !hasWall(x, y, leftDir)) {
        setWallInternal(x, y, leftDir);
        newWallFound = true;
    }

    if (newWallFound) {
        flood();
    }

    return newWallFound;
}

// ============================================================================
// IEEE UCI BFS Flood Fill — Static Ring Buffer, Zero Heap Allocation
// Reference: https://ieee.ics.uci.edu/micromouse/floodfill.html
//
//   Q = empty queue
//   distances = 2D array initialized to infinity
//   for cell in goalCells:
//       enqueue(Q, cell)
//       distances[cell] = 0
//   while Q is not empty:
//       currentCell = dequeue(Q)
//       for each neighbor of currentCell that is not blocked:
//           if distance[neighbor] > distance[currentCell] + 1:
//               distance[neighbor] = distance[currentCell] + 1
//               enqueue(Q, neighbor)
// ============================================================================

void FloodFill::flood() {
    // Reset all distances
    for (int x = 0; x < width_; ++x) {
        for (int y = 0; y < height_; ++y) {
            dist_[x][y] = 999;
        }
    }

    // Static ring buffer queue
    int qHead = 0, qTail = 0;

    // Seed goal cells
    for (int i = 0; i < goalCount_; ++i) {
        const auto& cell = goalCells_[i];
        if (isValidCoord(cell.x, cell.y)) {
            dist_[cell.x][cell.y] = 0;
            bfsQueue_[qTail++] = cell;
        }
    }

    // BFS expansion
    while (qHead != qTail) {
        CellCoord curr = bfsQueue_[qHead++];
        uint16_t currDist = dist_[curr.x][curr.y];

        for (int d = 0; d < DIR_COUNT; ++d) {
            // BUG-2 FIX: Single wall check is sufficient because
            // setWallInternal() maintains symmetric walls on both sides.
            if (!(walls_[curr.x][curr.y] & DIR_WALL_MASK[d])) {
                int nx = curr.x + DIR_DX[d];
                int ny = curr.y + DIR_DY[d];

                if (isValidCoord(nx, ny) && dist_[nx][ny] > currDist + 1) {
                    dist_[nx][ny] = currDist + 1;
                    bfsQueue_[qTail++] = {nx, ny};
                }
            }
        }
    }

#ifdef SIMULATION
    // Delta-only MMS updates (only changed cells)
    for (int x = 0; x < width_; ++x) {
        for (int y = 0; y < height_; ++y) {
            if (dist_[x][y] < 999) {
                API::setText(x, y, std::to_string(dist_[x][y]));
            }
        }
    }
#endif
}

// ============================================================================
// Speed-Run BFS — Strictly Through Verified Visited Cells
// Prevents speed-run path planner from shortcutting through unmapped walls.
// ============================================================================

void FloodFill::floodKnownOnly() {
    for (int x = 0; x < width_; ++x) {
        for (int y = 0; y < height_; ++y) {
            dist_[x][y] = 999;
        }
    }

    int qHead = 0, qTail = 0;

    // Seed goal cells that have actually been visited
    for (int i = 0; i < goalCount_; ++i) {
        const auto& cell = goalCells_[i];
        if (isValidCoord(cell.x, cell.y) && visited_[cell.x][cell.y]) {
            dist_[cell.x][cell.y] = 0;
            bfsQueue_[qTail++] = cell;
        }
    }

    // Fallback if no goal cell visited
    if (qTail == 0) {
        for (int i = 0; i < goalCount_; ++i) {
            const auto& cell = goalCells_[i];
            if (isValidCoord(cell.x, cell.y)) {
                dist_[cell.x][cell.y] = 0;
                bfsQueue_[qTail++] = cell;
            }
        }
    }

    // BFS expansion ONLY through visited cells
    while (qHead != qTail) {
        CellCoord curr = bfsQueue_[qHead++];
        uint16_t currDist = dist_[curr.x][curr.y];

        for (int d = 0; d < DIR_COUNT; ++d) {
            if (!(walls_[curr.x][curr.y] & DIR_WALL_MASK[d])) {
                int nx = curr.x + DIR_DX[d];
                int ny = curr.y + DIR_DY[d];

                if (isValidCoord(nx, ny) && visited_[nx][ny] && dist_[nx][ny] > currDist + 1) {
                    dist_[nx][ny] = currDist + 1;
                    bfsQueue_[qTail++] = {nx, ny};
                }
            }
        }
    }

#ifdef SIMULATION
    for (int x = 0; x < width_; ++x) {
        for (int y = 0; y < height_; ++y) {
            if (dist_[x][y] < 999) {
                API::setText(x, y, std::to_string(dist_[x][y]));
            }
        }
    }
#endif
}

// ============================================================================
// Next Move Decision
// ============================================================================

Direction FloodFill::getNextMove(int x, int y, Direction heading) const {
    Direction bestDir = heading;
    uint16_t minDist = 9999;

    // Tie-break: Forward > Left > Right > Back
    const Direction order[4] = {
        heading,
        static_cast<Direction>((heading + 3) % 4),
        static_cast<Direction>((heading + 1) % 4),
        static_cast<Direction>((heading + 2) % 4)
    };

    for (Direction dir : order) {
        if (!(walls_[x][y] & DIR_WALL_MASK[dir])) {
            int nx = x + DIR_DX[dir];
            int ny = y + DIR_DY[dir];
            if (isValidCoord(nx, ny) && dist_[nx][ny] < minDist) {
                minDist = dist_[nx][ny];
                bestDir = dir;
            }
        }
    }

    return bestDir;
}

// ============================================================================
// Speed-Run Path Computation (ALG-5 + OPT-6)
// Pre-computes optimal path and merges consecutive straight moves.
// ============================================================================

int FloodFill::computeSpeedPath(int startX, int startY, Direction startHeading) {
    floodKnownOnly();

    speedPathLen_ = 0;
    constexpr uint16_t INF_COST = 0xFFFF;
    static uint16_t cost[MAX_MAZE_DIM][MAX_MAZE_DIM][DIR_COUNT];
    static uint16_t parent[MAX_MAZE_DIM][MAX_MAZE_DIM][DIR_COUNT];

    for (int x = 0; x < width_; ++x) {
        for (int y = 0; y < height_; ++y) {
            for (int h = 0; h < DIR_COUNT; ++h) {
                cost[x][y][h] = INF_COST;
                parent[x][y][h] = 0;
            }
        }
    }

    struct HeapNode {
        uint16_t cost;
        uint8_t x, y, h;
    };

    static HeapNode heap[MAX_CELLS * DIR_COUNT];
    int heapSize = 0;

    auto heapPush = [&](uint16_t c, uint8_t x, uint8_t y, uint8_t h) {
        int idx = heapSize++;
        heap[idx] = {c, x, y, h};
        while (idx > 0) {
            int p = (idx - 1) / 2;
            if (heap[p].cost <= heap[idx].cost) break;
            std::swap(heap[p], heap[idx]);
            idx = p;
        }
    };

    auto heapPop = [&]() -> HeapNode {
        HeapNode top = heap[0];
        heap[0] = heap[--heapSize];
        int idx = 0;
        while (true) {
            int left = 2 * idx + 1;
            int right = 2 * idx + 2;
            int smallest = idx;
            if (left < heapSize && heap[left].cost < heap[smallest].cost) smallest = left;
            if (right < heapSize && heap[right].cost < heap[smallest].cost) smallest = right;
            if (smallest == idx) break;
            std::swap(heap[idx], heap[smallest]);
            idx = smallest;
        }
        return top;
    };

    cost[startX][startY][startHeading] = 0;
    heapPush(0, startX, startY, startHeading);

    uint8_t bestGoalX = 0, bestGoalY = 0, bestGoalH = 0;
    uint16_t bestGoalCost = INF_COST;

    while (heapSize > 0) {
        HeapNode curr = heapPop();
        if (curr.cost > cost[curr.x][curr.y][curr.h]) continue;

        if (isGoal(curr.x, curr.y)) {
            bestGoalX = curr.x;
            bestGoalY = curr.y;
            bestGoalH = curr.h;
            bestGoalCost = curr.cost;
            break; // Minimal turn-weighted cost goal reached
        }

        for (int nextDir = 0; nextDir < DIR_COUNT; ++nextDir) {
            if (!(walls_[curr.x][curr.y] & DIR_WALL_MASK[nextDir])) {
                int nx = curr.x + DIR_DX[nextDir];
                int ny = curr.y + DIR_DY[nextDir];

                if (isValidCoord(nx, ny) && visited_[nx][ny]) {
                    int turnDiff = (nextDir - curr.h + 4) % 4;
                    uint16_t turnCost = 0;
                    if (turnDiff == 1 || turnDiff == 3) turnCost = 2; // 90° turn penalty
                    else if (turnDiff == 2) turnCost = 4;              // 180° turn penalty

                    uint16_t newCost = curr.cost + 1 + turnCost;
                    if (newCost < cost[nx][ny][nextDir]) {
                        cost[nx][ny][nextDir] = newCost;
                        parent[nx][ny][nextDir] = (curr.y * MAX_MAZE_DIM + curr.x) * 4 + curr.h;
                        heapPush(newCost, nx, ny, nextDir);
                    }
                }
            }
        }
    }

    if (bestGoalCost == INF_COST) {
        return 0;
    }

    // Backtrack path of states from goal to start
    struct SimpleState {
        int x, y;
        Direction h;
    };
    static SimpleState revPath[MAX_CELLS];
    int revLen = 0;

    int bx = bestGoalX, by = bestGoalY;
    Direction bh = static_cast<Direction>(bestGoalH);

    while (bx != startX || by != startY || bh != startHeading) {
        revPath[revLen++] = {bx, by, bh};
        uint16_t p = parent[bx][by][bh];
        bh = static_cast<Direction>(p % 4);
        uint16_t cellIdx = p / 4;
        bx = cellIdx % MAX_MAZE_DIM;
        by = cellIdx / MAX_MAZE_DIM;
    }

    // Merge consecutive straight movements into multi-cell bursts
    for (int i = revLen - 1; i >= 0; ) {
        Direction dir = revPath[i].h;
        int straight = 0;
        while (i >= 0 && revPath[i].h == dir) {
            straight++;
            i--;
        }
        speedPath_[speedPathLen_++] = {dir, straight};
        if (speedPathLen_ >= MAX_CELLS) break;
    }

    return speedPathLen_;
}

// ============================================================================
// Static Utilities
// ============================================================================

TurnCommand FloodFill::getTurnCommand(Direction currentHeading, Direction targetHeading) {
    int diff = (static_cast<int>(targetHeading) - static_cast<int>(currentHeading) + 4) % 4;
    switch (diff) {
        case 0: return TURN_NONE;
        case 1: return TURN_RIGHT;
        case 2: return TURN_AROUND;
        case 3: return TURN_LEFT;
        default: return TURN_NONE;
    }
}

CellCoord FloodFill::advanceCoord(int x, int y, Direction dir) {
    return { x + DIR_DX[dir], y + DIR_DY[dir] };
}

uint16_t FloodFill::getDistance(int x, int y) const {
    if (!isValidCoord(x, y)) return 999;
    return dist_[x][y];
}

bool FloodFill::isVisited(int x, int y) const {
    if (!isValidCoord(x, y)) return false;
    return visited_[x][y];
}

void FloodFill::markVisited(int x, int y) {
    if (isValidCoord(x, y)) {
        visited_[x][y] = true;
    }
}
