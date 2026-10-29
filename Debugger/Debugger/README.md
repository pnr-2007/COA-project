# RISC201 Interactive CLI Debugger

A self-contained debugger module for the team's RISC201 non-pipelined simulator. This directory includes the simulator/ISA/ALU source files it depends on so it can be built independently for testing. When integrating into the main repository, coordinate with the team before maintaining duplicate copies of those shared source files.

## Requirements

- GCC/MinGW-w64 or Clang with C++17 support
- Run commands from this directory so `sample.asm` and `demo.asm` paths resolve

## Build (Windows PowerShell / VS Code terminal)

```powershell
g++ -std=c++17 -O2 cliinterface.cpp -o coa_debugger.exe
.\coa_debugger.exe
```

## Build (Linux/macOS)

```bash
g++ -std=c++17 -O2 cliinterface.cpp -o coa_debugger
./coa_debugger
```

## Quick working demo

At the `dbg>` prompt, enter these commands one at a time:

```text
demo
step
regs
step
step
regs
run
regs
```

Expected final register values: `R1 = 0x11` (17), `R4 = 0x18` (24), `R5 = 0x0e` (14). The program should halt after 4 instructions/cycles.

## Load an assembly file

```text
load sample.asm
setreg R2 10
setreg R3 7
step
regs
step
step
regs
run
regs
```

## Commands

| Command | Description |
|---|---|
| `load <file.asm>` | Assemble and load supported assembly subset |
| `demo` | Load arithmetic demo and initialize R2=10, R3=7 |
| `step` | Execute one instruction |
| `run` | Execute until HALT, breakpoint, error, or safety limit |
| `continue` | Resume execution, skipping the breakpoint at the current PC once |
| `break <address>` | Set a 4-byte-aligned byte-address breakpoint, e.g. `break 0x8` |
| `delete <address>` | Remove a breakpoint |
| `regs` | Show registers, PC, cycle count |
| `setreg <Rn> <value>` | Set a register; R0 remains constant zero |
| `mem <start> <count>` | Inspect memory words |
| `disasm` / `where` | Decode and display the instruction at current PC |
| `reset` | Reload current program and reset CPU state (also clears breakpoints) |
| `help` | Show commands |
| `quit` | Exit |

## Assembler/debugger scope

This CLI supports the assembly subset listed by `help`, including common register ALU operations, load/store, branches/jumps, `RET`, `NOP`, and `HALT`, with labels for branch/jump targets. It is not a complete implementation of every assignment requirement: full matrix assembly, a complete two-pass assembler/disassembler, pipelined execution visualization, microprogramming/microPC visualization, stack macros/protection/visualizer, and comprehensive integration tests remain outside this module.

## Test

Run `bash test_debugger.sh` in Linux/macOS/Git Bash. The test builds the debugger and checks arithmetic results, file loading, stepping, breakpoints, reset, and error handling.
