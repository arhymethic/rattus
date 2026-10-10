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
    int straightCells;    // Number of consecutive straight cells in this burst
};

/**
 * @brief Championship 10x10 Flood Fill & Turn-Weighted Dijkstra Solver.
 *
 * Designed with 100% static memory (zero heap allocation, < 2 KB total RAM).
 * Fits safely within ESP32 SRAM without task stack overflow risks.
 */
class FloodFill {
public:
    FloodFill();
    ~FloodFill() = default;

    /**
     * @brief Initialize maze: boundary walls, distance field, center goals.
     */
    void init(int width = DEFAULT_MAZE_WIDTH, int height = DEFAULT_MAZE_HEIGHT);

    /** @brief Switch goals to center cells (4,4), (4,5), (5,4), (5,5) and re-flood. */
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
     * @brief Standard IEEE UCI BFS Flood Fill (used during exploration).
     */
    void flood();

    /**
     * @brief Speed-Run Flood Fill: BFS strictly through verified visited_ cells.
     */
    void floodKnownOnly();

    /**
     * @brief Select next move: strictly lowest distance open neighbor with tie-breaking.
     */
    Direction getNextMove(int x, int y, Direction heading) const;

    /**
     * @brief Turn-Weighted Optimal Speed-Run Path Planner.
     * Computes the provably minimum-time path with turn penalties over verified cells.
     * Merges consecutive straight cells into multi-cell bursts.
     */
    int computeSpeedPath(int startX, int startY, Direction startHeading);

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

    // Speed-run cached path
    PathStep speedPath_[MAX_CELLS];
    int speedPathLen_;

    // Static BFS ring buffer
    CellCoord bfsQueue_[MAX_CELLS];

    bool isValidCoord(int x, int y) const {
        return (x >= 0 && x < width_ && y >= 0 && y < height_);
    }

    void setWallInternal(int x, int y, Direction dir);
    void setGoalsAndFlood();
};
