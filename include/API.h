#pragma once

#include <string>

/**
 * @brief Mackorone Micromouse Simulator (MMS) C++ Communication Bridge.
 * 
 * Interacts via standard standard I/O (stdin/stdout) with the MMS simulator process.
 * Standard error (stderr) is used for debugging logs.
 */
class API {
public:
    static int mazeWidth();
    static int mazeHeight();

    static bool wallFront();
    static bool wallRight();
    static bool wallLeft();

    static void moveForward(int distance = 1);
    static void turnRight();
    static void turnLeft();

    static void setWall(int x, int y, char direction);
    static void clearWall(int x, int y, char direction);

    static void setColor(int x, int y, char color);
    static void clearColor(int x, int y);
    static void clearAllColor();

    static void setText(int x, int y, const std::string& text);
    static void clearText(int x, int y);
    static void clearAllText();

    static bool wasReset();
    static void ackReset();
};
