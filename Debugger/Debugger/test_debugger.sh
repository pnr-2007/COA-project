#!/usr/bin/env bash
set -euo pipefail

g++ -std=c++17 -O2 -Wall -Wextra cliinterface.cpp -o coa_debugger_test
trap 'rm -f coa_debugger_test' EXIT

out=$(printf 'demo\nstep\nregs\nstep\nstep\nregs\nrun\nregs\nquit\n' | ./coa_debugger_test)
grep -q 'R1 = 0x0000000000000011' <<< "$out"
grep -q 'R4 = 0x0000000000000018' <<< "$out"
grep -q 'R5 = 0x000000000000000e' <<< "$out"
grep -q 'Program halted; cycles=4' <<< "$out"

out=$(printf 'load sample.asm\nsetreg R2 10\nsetreg R3 7\nstep\nregs\nbreak 0x8\ncontinue\ncontinue\nreset\nregs\nload missing.asm\nsetreg R0 5\nbreak 3\nquit\n' | ./coa_debugger_test)
grep -q 'Loaded 4 instructions' <<< "$out"
grep -q 'Breakpoint hit at PC=0x8' <<< "$out"
grep -q 'Error: cannot open file: missing.asm' <<< "$out"
grep -q 'Error: R0 is the constant zero register' <<< "$out"
grep -q 'Error: breakpoint address must be 4-byte aligned' <<< "$out"
echo 'ALL DEBUGGER TESTS PASSED'
