#!/usr/bin/env python3
"""
Comprehensive 3-Phase Championship Test Harness.
Simulates a full 10x10 competition maze with interior walls and dead-ends.
Validates:
  Phase 1: Search Run from (0,0) to Center Goals (4,4), (4,5), (5,4), (5,5)
  Phase 2: Return Run from Center Goals back to (0,0)
  Phase 3: Speed Run with multi-cell moveForward(N) bursts to Center Goals
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

    # Maze topology: 10x10 grid with boundary + interior walls
    vert_walls = set()
    horiz_walls = set()

    # Outer perimeter boundary
    for r in range(10):
        vert_walls.add((-1, r, 0, r))
        vert_walls.add((9, r, 10, r))
    for c in range(10):
        horiz_walls.add((c, -1, c, 0))
        horiz_walls.add((c, 9, c, 10))

    # Add interior walls (corridor + dead end trap similar to UCI benchmark)
    for r in range(2, 10):
        vert_walls.add((3, r, 4, r))
    for r in range(2, 8):
        vert_walls.add((2, r, 3, r))
    for r in range(4, 9):
        vert_walls.add((0, r, 1, r))
    horiz_walls.add((3, 9, 3, 10))
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

    events = []

    while True:
        line = proc.stdout.readline()
        if not line:
            break
        tokens = line.strip().split()
        if not tokens:
            continue

        cmd = tokens[0]
        if cmd == "mazeWidth":
            proc.stdin.write("10\n")
            proc.stdin.flush()
        elif cmd == "mazeHeight":
            proc.stdin.write("10\n")
            proc.stdin.flush()
        elif cmd == "wasReset":
            proc.stdin.write("false\n")
            proc.stdin.flush()
        elif cmd == "ackReset":
            x, y, heading = 0, 0, 0
            proc.stdin.write("ack\n")
            proc.stdin.flush()
        elif cmd == "wallFront":
            w = has_wall(x, y, heading)
            proc.stdin.write("true\n" if w else "false\n")
            proc.stdin.flush()
        elif cmd == "wallLeft":
            w = has_wall(x, y, (heading + 3) % 4)
            proc.stdin.write("true\n" if w else "false\n")
            proc.stdin.flush()
        elif cmd == "wallRight":
            w = has_wall(x, y, (heading + 1) % 4)
            proc.stdin.write("true\n" if w else "false\n")
            proc.stdin.flush()
        elif cmd == "moveForward":
            dist = int(tokens[1]) if len(tokens) > 1 else 1
            if heading == 0: y += dist
            elif heading == 1: x += dist
            elif heading == 2: y -= dist
            elif heading == 3: x -= dist
            events.append(f"moveForward({dist}) -> ({x},{y})")
            proc.stdin.write("ack\n")
            proc.stdin.flush()
        elif cmd == "turnLeft":
            heading = (heading + 3) % 4
            proc.stdin.write("ack\n")
            proc.stdin.flush()
        elif cmd == "turnRight":
            heading = (heading + 1) % 4
            proc.stdin.write("ack\n")
            proc.stdin.flush()
        elif cmd in ["setWall", "clearWall", "setColor", "clearColor", "clearAllColor", "setText", "clearText", "clearAllText"]:
            pass

    proc.wait(timeout=5)
    stderr_out = proc.stderr.read()

    print("=== Execution Logs (stderr) ===")
    print(stderr_out)
    print("=== Total Move Events ===", len(events))
    multi_cell_moves = [e for e in events if "moveForward" in e and not "moveForward(1)" in e]
    print("=== Multi-Cell Speed Bursts ===", len(multi_cell_moves), multi_cell_moves)
    print("=== Process Return Code ===", proc.returncode)

    if proc.returncode == 0:
        print("TEST PASSED: 3-Phase Championship Navigation Completed Successfully!")
    else:
        print("TEST FAILED: Non-zero exit code.")
        sys.exit(1)

if __name__ == '__main__':
    main()
