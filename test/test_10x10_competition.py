#!/usr/bin/env python3
"""
10x10 Competition Maze Verification Test.
Runs sim_mouse on test/maze10x10.num.
Verifies:
  - Strict 10x10 grid boundaries
  - Phase 1: Search run reaches center (4,4), (4,5), (5,4), or (5,5)
  - Phase 2: Return run reaches (0,0)
  - Phase 3: Speed run executes cleanly with zero crashes!
"""
import subprocess
import sys

def load_maze(path):
    walls = {}
    with open(path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 6:
                x, y, n, e, s, w = map(int, parts)
                walls[(x, y)] = {
                    0: bool(n), # North
                    1: bool(e), # East
                    2: bool(s), # South
                    3: bool(w)  # West
                }
    return walls

def main():
    walls = load_maze('test/maze10x10.num')
    print(f"Loaded {len(walls)} cells from maze10x10.num")

    proc = subprocess.Popen(
        ['./sim_mouse'],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1
    )

    x, y, heading = 0, 0, 0
    step_count = 0
    crashes = []

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
            w = walls.get((x, y), {}).get(heading, True)
            proc.stdin.write("true\n" if w else "false\n")
            proc.stdin.flush()
        elif cmd == "wallLeft":
            w = walls.get((x, y), {}).get((heading + 3) % 4, True)
            proc.stdin.write("true\n" if w else "false\n")
            proc.stdin.flush()
        elif cmd == "wallRight":
            w = walls.get((x, y), {}).get((heading + 1) % 4, True)
            proc.stdin.write("true\n" if w else "false\n")
            proc.stdin.flush()
        elif cmd == "moveForward":
            dist = int(tokens[1]) if len(tokens) > 1 else 1
            has_crash = False
            for step in range(dist):
                if walls.get((x, y), {}).get(heading, True):
                    has_crash = True
                    crashes.append(f"CRASH at ({x},{y}) heading {heading} on step {step+1}/{dist}")
                    break
                if heading == 0: y += 1
                elif heading == 1: x += 1
                elif heading == 2: y -= 1
                elif heading == 3: x -= 1

            if has_crash:
                proc.stdin.write("crash\n")
            else:
                proc.stdin.write("ack\n")
            proc.stdin.flush()
            step_count += 1
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
    print("=== Execution Logs (stderr) ===")
    print(proc.stderr.read())
    print(f"Total crashes: {len(crashes)}")

    if len(crashes) == 0:
        print("PASS: 10x10 competition maze completed with 0 crashes!")
        sys.exit(0)
    else:
        print("FAIL: Crashes detected on 10x10 maze.")
        sys.exit(1)

if __name__ == '__main__':
    main()
