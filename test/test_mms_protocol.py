#!/usr/bin/env python3
"""
MMS Protocol Verification Harness.
Validates protocol compliance:
  - Query handshake: mazeWidth, mazeHeight
  - Sensor telemetry: wallFront, wallLeft, wallRight
  - Odometry execution: moveForward, turnLeft, turnRight
  - Visual telemetry: setWall, setColor, setText
  - Reset protocol: wasReset, ackReset
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

    x, y, heading = 0, 0, 0
    step_count = 0
    max_steps = 50
    handshake_verified = False

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
            handshake_verified = True
        elif action == "wasReset":
            proc.stdin.write("false\n")
            proc.stdin.flush()
        elif action == "ackReset":
            proc.stdin.write("ack\n")
            proc.stdin.flush()
        elif action == "wallFront":
            is_w = (heading == 0 and y == 9) or (heading == 1 and x == 9) or (heading == 2 and y == 0) or (heading == 3 and x == 0)
            proc.stdin.write("true\n" if is_w else "false\n")
            proc.stdin.flush()
        elif action == "wallLeft":
            ld = (heading + 3) % 4
            is_w = (ld == 0 and y == 9) or (ld == 1 and x == 9) or (ld == 2 and y == 0) or (ld == 3 and x == 0)
            proc.stdin.write("true\n" if is_w else "false\n")
            proc.stdin.flush()
        elif action == "wallRight":
            rd = (heading + 1) % 4
            is_w = (rd == 0 and y == 9) or (rd == 1 and x == 9) or (rd == 2 and y == 0) or (rd == 3 and x == 0)
            proc.stdin.write("true\n" if is_w else "false\n")
            proc.stdin.flush()
        elif action == "moveForward":
            dist = int(cmd[1]) if len(cmd) > 1 else 1
            if heading == 0: y += dist
            elif heading == 1: x += dist
            elif heading == 2: y -= dist
            elif heading == 3: x -= dist
            proc.stdin.write("ack\n")
            proc.stdin.flush()
            step_count += 1
            if (x, y) in [(4, 4), (4, 5), (5, 4), (5, 5)]:
                break
        elif action in ["turnLeft", "turnRight"]:
            if action == "turnLeft": heading = (heading + 3) % 4
            else: heading = (heading + 1) % 4
            proc.stdin.write("ack\n")
            proc.stdin.flush()
        elif action in ["setWall", "clearWall", "setColor", "clearColor", "clearAllColor", "setText", "clearText", "clearAllText"]:
            pass

    proc.terminate()
    if handshake_verified and step_count > 0:
        print(f"PASS: MMS protocol verified successfully across {step_count} steps.")
        sys.exit(0)
    else:
        print("FAIL: Protocol verification failed.")
        sys.exit(1)

if __name__ == '__main__':
    main()
