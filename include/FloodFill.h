#pragma once

#include "Config.h"

/**
 * @brief Compact 2D Grid Coordinate.
 */
struct CellCoord {
    int x;
    int y;

    bool operator==(const CellCoord& other) const {
        return x == other.x && y == other.y;
    }
    bool operator!=(const CellCoord& other) const {
        return !(*this == other);
    }
};

/**
 * @brief A single step in a cached speed-run path.
 */
struct PathStep {
    Direction dir;        // Absolute direction to face
    int straightCells;    // Number of consecutive cells to traverse in this direction
};

/**
 * @brief Championship-Grade Flood Fill Maze Solver.
 *
 * Implements the IEEE UCI BFS Flood Fill algorithm with:
 * - Zero-heap-allocation static ring buffer BFS queue
 * - Fixed-size goal array (no std::vector)
 * - uint16_t distance field (half memory vs int)
 * - Multi-phase goal switching (center ↔ start)
 * - Speed-run path caching with multi-cell straight merging
 *
 * Reference: https://ieee.ics.uci.edu/micromouse/floodfill.html
 */
class FloodFill {
public:
    FloodFill();
    ~FloodFill() = default;

    /**
     * @brief Initialize maze: boundary walls, distance field, center goals.
     * No hardcoded interior walls — the bot discovers them via sensors.
     */
    void init(int width = DEFAULT_MAZE_WIDTH, int height = DEFAULT_MAZE_HEIGHT);

    /** @brief Render initial distance labels and goal markers in MMS. */
    void initMMSVisuals();

    /** @brief Switch goals to center cells and re-flood. */
    void setGoalsCenter();

    /** @brief Switch goals to start cell (0,0) and re-flood. */
    void setGoalsStart();

    bool isGoal(int x, int y) const;

    /**
     * @brief Register detected walls and trigger BFS re-flood if topology changed.
     * @return true if at least one new wall was discovered.
     */
    bool updateWalls(int x, int y, Direction heading,
                     bool wallFront, bool wallLeft, bool wallRight);

    /**
     * @brief IEEE UCI BFS Flood Fill.
     * Uses static ring buffer queue — zero heap allocations.
     */
    void flood();

    /**
     * @brief Speed-Run Flood Fill: BFS strictly through verified visited_ cells.
     * Ensures the pre-computed speed path never routes through unmapped walls.
     */
    void floodKnownOnly();

    /**
     * @brief Select next move: strictly lowest distance open neighbor.
     * Tie-break: Forward > Left > Right > Back.
     */
    Direction getNextMove(int x, int y, Direction heading) const;

    /**
     * @brief Pre-compute optimal speed-run path from (startX, startY) to nearest goal.
     * Merges consecutive same-direction moves into multi-cell straight bursts.
     * @return Number of steps in the cached path.
     */
    int computeSpeedPath(int startX, int startY, Direction startHeading);

    /** @brief Access the cached speed-run path. */
    const PathStep* getSpeedPath() const { return speedPath_; }
    int getSpeedPathLen() const { return speedPathLen_; }

    static TurnCommand getTurnCommand(Direction currentHeading, Direction targetHeading);
    static CellCoord advanceCoord(int x, int y, Direction dir);

    uint16_t getDistance(int x, int y) const;
    bool hasWall(int x, int y, Direction dir) const;
    bool isVisited(int x, int y) const;
    void markVisited(int x, int y);

    int getWidth() const { return width_; }
    int getHeight() const { return height_; }

private:
    int width_;
    int height_;
    uint8_t walls_[MAX_MAZE_DIM][MAX_MAZE_DIM];
    uint16_t dist_[MAX_MAZE_DIM][MAX_MAZE_DIM];
    bool visited_[MAX_MAZE_DIM][MAX_MAZE_DIM];

    CellCoord goalCells_[MAX_GOAL_CELLS];
    int goalCount_;

    // Speed-run cached path (worst case: every cell is a step)
    PathStep speedPath_[MAX_CELLS];
    int speedPathLen_;

    // Static BFS ring buffer (avoids std::queue heap allocation)
    CellCoord bfsQueue_[MAX_CELLS];

    bool isValidCoord(int x, int y) const {
        return (x >= 0 && x < width_ && y >= 0 && y < height_);
    }

    void setWallInternal(int x, int y, Direction dir);
    void setGoalsAndFlood();
};
