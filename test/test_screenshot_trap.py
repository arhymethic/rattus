#!/usr/bin/env python3
"""
Test harness reproducing the exact dead-end wall trap from the user's screenshot:
- Boundary walls
- Wall between col 3 and col 4 from row 2 to row 9
- Wall between col 2 and col 3 from row 2 to row 8
- Dead end at (3, 9)
Verifies that mouse backtracks out of (3,9) and reaches (4,4) without looping or getting stuck.
"""
import subprocess
import sys

def main():
    proc = subprocess.Popen(
        ['./sim_mouse'],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1
    )

    x, y = 0, 0
    heading = 0 # 0=N, 1=E, 2=S, 3=W
    step_count = 0
    max_steps = 200

    # Vertical walls: set of ((x, y), (x+1, y))
    vert_walls = set()
    # Boundary
    for r in range(10):
        vert_walls.add((-1, r, 0, r))
        vert_walls.add((9, r, 10, r))
    # Wall between 3 and 4 from row 2 to 9
    for r in range(2, 10):
        vert_walls.add((3, r, 4, r))
    # Wall between 2 and 3 from row 2 to 8
    for r in range(2, 9):
        vert_walls.add((2, r, 3, r))
    # Wall between 0 and 1 from row 4 to 8 (as seen in screenshot)
    for r in range(4, 9):
        vert_walls.add((0, r, 1, r))

    # Horizontal walls: set of ((x, y), (x, y+1))
    horiz_walls = set()
    for c in range(10):
        horiz_walls.add((c, -1, c, 0))
        horiz_walls.add((c, 9, c, 10))
    # North wall of (3,9)
    horiz_walls.add((3, 9, 3, 10))
    # Horizontal wall under column 3: row 1 to 2
    horiz_walls.add((3, 1, 3, 2))

    def has_wall(cx, cy, d):
        if d == 0: # North
            return (cx, cy, cx, cy + 1) in horiz_walls or cy == 9
        elif d == 1: # East
            return (cx, cy, cx + 1, cy) in vert_walls or cx == 9
        elif d == 2: # South
            return (cx, cy - 1, cx, cy) in horiz_walls or cy == 0
        elif d == 3: # West
            return (cx - 1, cy, cx, cy) in vert_walls or cx == 0
        return False

    visited_cells = []

    while step_count < max_steps:
        line = proc.stdout.readline()
        if not line:
            break
        cmd = line.strip().split()
        if not cmd:
            continue

        action = cmd[0]
        if action == "mazeWidth":
            proc.stdin.write("10\n")
            proc.stdin.flush()
        elif action == "mazeHeight":
            proc.stdin.write("10\n")
            proc.stdin.flush()
        elif action == "wasReset":
            proc.stdin.write("false\n")
            proc.stdin.flush()
        elif action == "ackReset":
            proc.stdin.write("ack\n")
            proc.stdin.flush()
        elif action == "wallFront":
            w = has_wall(x, y, heading)
            proc.stdin.write("true\n" if w else "false\n")
            proc.stdin.flush()
        elif action == "wallLeft":
            w = has_wall(x, y, (heading + 3) % 4)
            proc.stdin.write("true\n" if w else "false\n")
            proc.stdin.flush()
        elif action == "wallRight":
            w = has_wall(x, y, (heading + 1) % 4)
            proc.stdin.write("true\n" if w else "false\n")
            proc.stdin.flush()
        elif action == "moveForward":
            if heading == 0: y += 1
            elif heading == 1: x += 1
            elif heading == 2: y -= 1
            elif heading == 3: x -= 1
            proc.stdin.write("ack\n")
            proc.stdin.flush()
            step_count += 1
            visited_cells.append((x, y))
            # Check if reached center (4,4), (4,5), (5,4), (5,5)
            if (x, y) in [(4,4), (4,5), (5,4), (5,5)]:
                print(f"Goal reached at ({x},{y}) in {step_count} steps!")
                break
        elif action in ["turnLeft", "turnRight"]:
            if action == "turnLeft": heading = (heading + 3) % 4
            else: heading = (heading + 1) % 4
            proc.stdin.write("ack\n")
            proc.stdin.flush()
        elif action in ["setWall", "clearWall", "setColor", "clearColor", "clearAllColor", "setText", "clearText", "clearAllText"]:
            pass

    proc.terminate()
    stderr_out = proc.stderr.read()
    print("Stderr log:")
    print(stderr_out)
    if (x, y) in [(4,4), (4,5), (5,4), (5,5)]:
        print("PASS: Robot successfully navigated out of dead ends and reached the goal!")
        sys.exit(0)
    else:
        print(f"Robot ended at ({x},{y}) after {step_count} steps.")
        sys.exit(1)

if __name__ == '__main__':
    main()
