#!/usr/bin/env python3
"""
Unified 10x10 Micromouse Test Suite Runner.
Executes all 10x10 unit and integration tests and summarizes results.
"""
import subprocess
import sys
import time

TESTS = [
    ("MMS 10x10 Protocol Compliance", ["python3", "test/test_mms_protocol.py"]),
    ("10x10 Dead-End Wall Trap Navigation", ["python3", "test/test_screenshot_trap.py"]),
    ("10x10 3-Phase Championship Sprint", ["python3", "test/test_championship.py"]),
    ("10x10 Competition Benchmark (maze10x10.num)", ["python3", "test/test_10x10_competition.py"])
]

def main():
    print("=" * 65)
    print("  10x10 MICROMOUSE AUTONOMOUS FIRMWARE — TEST SUITE RUNNER")
    print("=" * 65)

    all_passed = True
    results = []

    for name, cmd in TESTS:
        print(f"\n[RUNNING] {name}...")
        start = time.time()
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        elapsed = time.time() - start

        if res.returncode == 0:
            print(f"[PASS]    {name} ({elapsed:.2f}s)")
            results.append((name, "PASS", elapsed))
        else:
            print(f"[FAIL]    {name} ({elapsed:.2f}s)")
            print("--- STDOUT ---")
            print(res.stdout[-500:] if len(res.stdout) > 500 else res.stdout)
            print("--- STDERR ---")
            print(res.stderr[-500:] if len(res.stderr) > 500 else res.stderr)
            all_passed = False
            results.append((name, "FAIL", elapsed))

    print("\n" + "=" * 65)
    print("  10x10 TEST SUMMARY REPORT")
    print("=" * 65)
    for name, status, elapsed in results:
        status_str = "✅ PASS" if status == "PASS" else "❌ FAIL"
        print(f"  {status_str}  {name:<46} ({elapsed:.2f}s)")
    print("=" * 65)

    if all_passed:
        print("\n🎉 ALL 10x10 TESTS PASSED! Firmware is verified for 10x10 competition.\n")
        sys.exit(0)
    else:
        print("\n💥 SOME TESTS FAILED. See output above for details.\n")
        sys.exit(1)

if __name__ == '__main__':
    main()
