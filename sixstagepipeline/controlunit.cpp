"""
6-stage microprogrammed CONTROL UNIT (horizontal)
  1 IF    Instruction Fetch
  2 ID    Instruction Decode (+ register read)
  3 EX    Execute / ALU (also computes the effective address for LOAD/STORE)
  4 MEM1  Memory address / access preparation (MAR <- address, MDR <- store data)
  5 MEM2  Actual memory access
  6 WB    Write Back

Instruction (8 bit):  opcode(2) | Rd(3) | Rs(3)
  00 ADD   Rd <- Rd + Rs        10 LOAD  Rd <- DM[Rs]
  01 SUB   Rd <- Rd - Rs        11 STORE DM[Rd] <- Rs

Microword (22 bit): [21..17] NEXT(5) | [16..0] one bit per control signal
"""
import random

STAGES = ["IF", "ID", "EX", "MEM1", "MEM2", "WB"]

SIGNALS = ["PC_MAR",   # MAR <- PC
           "PC_INC",   # PC  <- PC + 1
           "IM_RD",    # MDR <- IM[MAR]
           "MDR_IR",   # IR  <- MDR
           "DECODE",   # uPC <- MappingROM[opcode]
           "A_LD",     # A   <- Reg[Rs]
           "B_LD",     # B   <- Reg[Rd]
           "ALU_ADD",  # ALUOUT <- B + A
           "ALU_SUB",  # ALUOUT <- B - A
           "ALU_PA",   # ALUOUT <- A      (effective address for LOAD)
           "ALU_PB",   # ALUOUT <- B      (effective address for STORE)
           "MAR_ALU",  # MAR <- ALUOUT    (MEM1: address ready)
           "MDR_A",    # MDR <- A         (MEM1: store data ready)
           "DM_RD",    # MDR <- DM[MAR]   (MEM2: read)
           "DM_WR",    # DM[MAR] <- MDR   (MEM2: write)
           "WB_ALU",   # Reg[Rd] <- ALUOUT
           "WB_MDR"]   # Reg[Rd] <- MDR
BIT = {s: i for i, s in enumerate(SIGNALS)}
NEXT_SHIFT = len(SIGNALS)
WIDTH = NEXT_SHIFT + 5

# addr: (stage, [signals], NEXT)
MICROCODE = {
    # ---- common ----
    0x00: ("IF",   ["PC_MAR", "PC_INC"],      0x01),
    0x01: ("IF",   ["IM_RD"],                 0x02),
    0x02: ("IF",   ["MDR_IR"],                0x03),
    0x03: ("ID",   ["DECODE", "A_LD", "B_LD"], 0x00),   # NEXT comes from mapping ROM
    # ---- ADD ----
    0x04: ("EX",   ["ALU_ADD"],               0x05),
    0x05: ("MEM1", [],                        0x06),
    0x06: ("MEM2", [],                        0x07),
    0x07: ("WB",   ["WB_ALU"],                0x00),
    # ---- SUB ----
    0x08: ("EX",   ["ALU_SUB"],               0x09),
    0x09: ("MEM1", [],                        0x0A),
    0x0A: ("MEM2", [],                        0x0B),
    0x0B: ("WB",   ["WB_ALU"],                0x00),
    # ---- LOAD ----
    0x0C: ("EX",   ["ALU_PA"],                0x0D),
    0x0D: ("MEM1", ["MAR_ALU"],               0x0E),
    0x0E: ("MEM2", ["DM_RD"],                 0x0F),
    0x0F: ("WB",   ["WB_MDR"],                0x00),
    # ---- STORE ----
    0x10: ("EX",   ["ALU_PB"],                0x11),
    0x11: ("MEM1", ["MAR_ALU", "MDR_A"],      0x12),
    0x12: ("MEM2", ["DM_WR"],                 0x13),
    0x13: ("WB",   [],                        0x00),
}
MAPPING_ROM = {0b00: 0x04, 0b01: 0x08, 0b10: 0x0C, 0b11: 0x10}
OPNAME = {0: "ADD", 1: "SUB", 2: "LOAD", 3: "STORE"}


def pack(sigs, nxt):
    w = nxt << NEXT_SHIFT
    for s in sigs:
        w |= 1 << BIT[s]
    return w

CONTROL_STORE = {a: pack(sg, n) for a, (_, sg, n) in MICROCODE.items()}
STAGE_OF = {a: st for a, (st, _, _) in MICROCODE.items()}

# signal groups that must never fire together (same destination / same unit)
EXCLUSIVE = [
    {"ALU_ADD", "ALU_SUB", "ALU_PA", "ALU_PB"},   # one ALU function
    {"IM_RD", "DM_RD", "MDR_A"},                  # one writer of MDR
    {"PC_MAR", "MAR_ALU"},                        # one writer of MAR
    {"WB_ALU", "WB_MDR"},                         # one writer of Reg[Rd]
    {"IM_RD", "DM_RD", "DM_WR"},                  # one memory op per cycle
]


# ------------------------------------------------------------------
# CHECK: static verification of the control unit
# ------------------------------------------------------------------
def verify_control_unit():
    errors = []

    # 1. every word fits the width and every NEXT target exists
    for a, w in CONTROL_STORE.items():
        if w >> WIDTH:
            errors.append(f"{a:02X}: word wider than {WIDTH} bits")
        nxt = w >> NEXT_SHIFT
        if nxt != 0 and nxt not in CONTROL_STORE:
            errors.append(f"{a:02X}: NEXT {nxt:02X} does not exist")
        if nxt >= 32:
            errors.append(f"{a:02X}: NEXT does not fit 5 bits")

    # 2. no conflicting signals inside one word
    for a, (_, sigs, _) in MICROCODE.items():
        for group in EXCLUSIVE:
            if len(set(sigs) & group) > 1:
                errors.append(f"{a:02X}: conflicting signals {set(sigs) & group}")

    # 3. each opcode path visits IF,ID,EX,MEM1,MEM2,WB in order and returns to 0
    for op, start in MAPPING_ROM.items():
        path = [0x00, 0x01, 0x02, 0x03]
        a, seen = start, set()
        while a != 0:
            if a in seen:
                errors.append(f"{OPNAME[op]}: loop at {a:02X}"); break
            seen.add(a); path.append(a)
            a = CONTROL_STORE[a] >> NEXT_SHIFT
        stages = []
        for p in path:
            if not stages or stages[-1] != STAGE_OF[p]:
                stages.append(STAGE_OF[p])
        if stages != STAGES:
            errors.append(f"{OPNAME[op]}: stage order {stages} != {STAGES}")

    # 4. every microword is reachable
    reach = {0x00, 0x01, 0x02, 0x03} | set(MAPPING_ROM.values())
    for start in MAPPING_ROM.values():
        a = start
        while a != 0:
            reach.add(a); a = CONTROL_STORE[a] >> NEXT_SHIFT
    for a in CONTROL_STORE:
        if a not in reach:
            errors.append(f"{a:02X}: unreachable")

    # 5. mapping ROM covers all 4 opcodes with valid targets
    if set(MAPPING_ROM) != {0, 1, 2, 3}:
        errors.append("mapping ROM must cover opcodes 0..3")

    return errors


# ------------------------------------------------------------------
# Datapath + control unit simulator
# ------------------------------------------------------------------
def run(imem, regs, dmem, n_instr, trace=True):
    pc = MAR = MDR = IR = A = B = ALUOUT = 0
    upc, done, cycles = 0x00, 0, 0
    while done < n_instr:
        w = CONTROL_STORE[upc]
        on = {s for s in SIGNALS if (w >> BIT[s]) & 1}
        nxt = w >> NEXT_SHIFT
        rd, rs, op = (IR >> 3) & 7, IR & 7, (IR >> 6) & 3

        n = dict(pc=pc, MAR=MAR, MDR=MDR, IR=IR, A=A, B=B, ALUOUT=ALUOUT)
        if "PC_MAR"  in on: n["MAR"] = pc
        if "PC_INC"  in on: n["pc"] = (pc + 1) & 0xFF
        if "IM_RD"   in on: n["MDR"] = imem[MAR]
        if "MDR_IR"  in on: n["IR"] = MDR
        if "A_LD"    in on: n["A"] = regs[rs]
        if "B_LD"    in on: n["B"] = regs[rd]
        if "ALU_ADD" in on: n["ALUOUT"] = (B + A) & 0xFF
        if "ALU_SUB" in on: n["ALUOUT"] = (B - A) & 0xFF
        if "ALU_PA"  in on: n["ALUOUT"] = A
        if "ALU_PB"  in on: n["ALUOUT"] = B
        if "MAR_ALU" in on: n["MAR"] = ALUOUT
        if "MDR_A"   in on: n["MDR"] = A
        if "DM_RD"   in on: n["MDR"] = dmem[MAR]
        if "DM_WR"   in on: dmem[MAR] = MDR
        if "WB_ALU"  in on: regs[rd] = ALUOUT
        if "WB_MDR"  in on: regs[rd] = MDR
        pc, MAR, MDR, IR, A, B, ALUOUT = (n[k] for k in
            ("pc", "MAR", "MDR", "IR", "A", "B", "ALUOUT"))

        cycles += 1
        if trace:
            names = "+".join(s for s in SIGNALS if s in on) or "(idle)"
            print(f"uPC={upc:02X} {STAGE_OF[upc]:<4} word={w:06X}  {names}")
        if "DECODE" in on:
            upc = MAPPING_ROM[op]
        else:
            upc = nxt
            if upc == 0x00:
                done += 1
                if trace: print("-" * 62)
    return cycles


def reference(imem, regs, dmem, n):
    """Golden ISA model (no microcode) used to cross-check the control unit."""
    for i in range(n):
        ins = imem[i]; op, rd, rs = (ins >> 6) & 3, (ins >> 3) & 7, ins & 7
        if   op == 0: regs[rd] = (regs[rd] + regs[rs]) & 0xFF
        elif op == 1: regs[rd] = (regs[rd] - regs[rs]) & 0xFF
        elif op == 2: regs[rd] = dmem[regs[rs]]
        else:         dmem[regs[rd]] = regs[rs]


if __name__ == "__main__":
    # ---- step 1: check the control unit ----
    errs = verify_control_unit()
    print("CONTROL UNIT CHECK:", "PASS" if not errs else "FAIL")
    for e in errs: print("  -", e)
    assert not errs
    print(f"  {len(CONTROL_STORE)} microwords, {WIDTH}-bit words, "
          f"all 4 opcodes traverse {' > '.join(STAGES)}\n")

    # ---- step 2: run demo program ----
    prog = [0b00_001_010,   # ADD   R1, R2    R1 = 10+5 = 15
            0b01_001_011,   # SUB   R1, R3    R1 = 15-3 = 12
            0b11_100_001,   # STORE [R4], R1  DM[2] = 12
            0b10_101_100]   # LOAD  R5, [R4]  R5 = 12
    regs, dmem = [0, 10, 5, 3, 2, 0, 0, 0], [0] * 8
    cyc = run(prog, regs, dmem, len(prog))
    print("Registers:", regs, "| Data mem:", dmem, "| cycles:", cyc)
    assert regs[1] == 12 and dmem[2] == 12 and regs[5] == 12

    # ---- step 3: random cross-check against the golden model ----
    random.seed(1)
    compared = 0
    for _ in range(500):
        p = [random.randrange(256) for _ in range(random.randint(1, 12))]
        r0 = [random.randrange(8) for _ in range(8)]   # keep addresses in range
        d0 = [random.randrange(256) for _ in range(8)]
        r1, d1 = r0[:], d0[:]; r2, d2 = r0[:], d0[:]
        # make LOAD/STORE addresses always valid: registers hold 0..7 only for address use
        try:
            reference(p, r1, d1, len(p))
        except IndexError:
            continue
        run(p, r2, d2, len(p), trace=False)
        assert r1 == r2 and d1 == d2
        compared += 1
    print(f"Random cross-check vs golden model: {compared} valid programs OK")
    print("All checks passed")
