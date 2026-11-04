"""
6-stage HORIZONTAL microprogrammed control unit
Stages: IF -> ID -> OF -> EX -> MEM -> WB

Instruction (8 bit):  opcode(2) | Rd(3) | Rs(3)
  00 ADD   Rd <- Rd + Rs
  01 SUB   Rd <- Rd - Rs
  10 LOAD  Rd <- DM[Rs]
  11 STORE DM[Rd] <- Rs

Horizontal microword (21 bit):
  [20..16] NEXT (5)  |  [15..0] one bit per control signal (no decoder)
Several signals may be 1 in the same word -> they fire in parallel.
"""

# ---- control signals: one bit each ----
SIGNALS = ["PC_MAR",   # MAR   <- PC
           "PC_INC",   # PC    <- PC + 1
           "IM_RD",    # MDR   <- IM[MAR]
           "MDR_IR",   # IR    <- MDR
           "DECODE",   # uPC   <- MappingROM[opcode]
           "A_LD",     # A     <- Reg[Rs]
           "B_LD",     # B     <- Reg[Rd]
           "ALU_ADD",  # ALUOUT <- B + A
           "ALU_SUB",  # ALUOUT <- B - A
           "ALU_PA",   # ALUOUT <- A
           "ALU_PB",   # ALUOUT <- B
           "MAR_ALU",  # MAR   <- ALUOUT
           "DM_RD",    # MDR   <- DM[MAR]
           "DM_WR",    # DM[MAR] <- A
           "WB_ALU",   # Reg[Rd] <- ALUOUT
           "WB_MDR"]   # Reg[Rd] <- MDR
BIT = {name: i for i, name in enumerate(SIGNALS)}
NEXT_SHIFT = len(SIGNALS)          # NEXT field sits above the signal bits


def word(signals, nxt):
    w = nxt << NEXT_SHIFT
    for s in signals:
        w |= 1 << BIT[s]
    return w


# addr: (stage, [signals active this cycle], NEXT)
MICROCODE = {
    # ---- IF  (PC_INC runs in parallel with PC_MAR) ----
    0x00: ("IF",  ["PC_MAR", "PC_INC"], 0x01),
    0x01: ("IF",  ["IM_RD"],            0x02),
    0x02: ("IF",  ["MDR_IR"],           0x03),
    # ---- ID ----
    0x03: ("ID",  ["DECODE"],           0x00),   # NEXT replaced by mapping ROM

    # ---- ADD ----
    0x04: ("OF",  ["A_LD", "B_LD"],     0x05),   # both operands in ONE cycle
    0x05: ("EX",  ["ALU_ADD"],          0x06),
    0x06: ("MEM", [],                   0x07),
    0x07: ("WB",  ["WB_ALU"],           0x00),

    # ---- SUB ----
    0x08: ("OF",  ["A_LD", "B_LD"],     0x09),
    0x09: ("EX",  ["ALU_SUB"],          0x0A),
    0x0A: ("MEM", [],                   0x0B),
    0x0B: ("WB",  ["WB_ALU"],           0x00),

    # ---- LOAD  Rd <- DM[Rs] ----
    0x0C: ("OF",  ["A_LD"],             0x0D),
    0x0D: ("EX",  ["ALU_PA"],           0x0E),
    0x0E: ("MEM", ["MAR_ALU"],          0x0F),
    0x0F: ("MEM", ["DM_RD"],            0x10),
    0x10: ("WB",  ["WB_MDR"],           0x00),

    # ---- STORE  DM[Rd] <- Rs ----
    0x11: ("OF",  ["A_LD", "B_LD"],     0x12),
    0x12: ("EX",  ["ALU_PB"],           0x13),
    0x13: ("MEM", ["MAR_ALU"],          0x14),
    0x14: ("MEM", ["DM_WR"],            0x15),
    0x15: ("WB",  [],                   0x00),
}

CONTROL_STORE = {a: word(sig, n) for a, (_, sig, n) in MICROCODE.items()}
STAGE_OF = {a: s for a, (s, _, _) in MICROCODE.items()}
MAPPING_ROM = {0b00: 0x04, 0b01: 0x08, 0b10: 0x0C, 0b11: 0x11}
ALU_SIGS = ("ALU_ADD", "ALU_SUB", "ALU_PA", "ALU_PB")


def run(imem, regs, dmem, n_instr, trace=True):
    pc = MAR = MDR = IR = A = B = ALUOUT = 0
    upc, done, cycles = 0x00, 0, 0

    while done < n_instr:
        w = CONTROL_STORE[upc]
        on = {s for s in SIGNALS if (w >> BIT[s]) & 1}   # no decoder: read bits directly
        nxt = w >> NEXT_SHIFT

        # sanity: only one ALU function and one writer per register
        assert len(on & set(ALU_SIGS)) <= 1
        assert len(on & {"PC_MAR", "MAR_ALU"}) <= 1
        assert len(on & {"IM_RD", "DM_RD"}) <= 1

        rd, rs, op = (IR >> 3) & 7, IR & 7, (IR >> 6) & 3

        # all signals read the OLD register values, then everything latches
        n_pc, n_MAR, n_MDR, n_IR, n_A, n_B, n_ALU = pc, MAR, MDR, IR, A, B, ALUOUT
        if "PC_MAR"  in on: n_MAR = pc
        if "PC_INC"  in on: n_pc = pc + 1
        if "IM_RD"   in on: n_MDR = imem[MAR]
        if "MDR_IR"  in on: n_IR = MDR
        if "A_LD"    in on: n_A = regs[rs]
        if "B_LD"    in on: n_B = regs[rd]
        if "ALU_ADD" in on: n_ALU = (B + A) & 0xFF
        if "ALU_SUB" in on: n_ALU = (B - A) & 0xFF
        if "ALU_PA"  in on: n_ALU = A
        if "ALU_PB"  in on: n_ALU = B
        if "MAR_ALU" in on: n_MAR = ALUOUT
        if "DM_RD"   in on: n_MDR = dmem[MAR]
        if "DM_WR"   in on: dmem[MAR] = A
        if "WB_ALU"  in on: regs[rd] = ALUOUT
        if "WB_MDR"  in on: regs[rd] = MDR
        pc, MAR, MDR, IR, A, B, ALUOUT = n_pc, n_MAR, n_MDR, n_IR, n_A, n_B, n_ALU

        cycles += 1
        if trace:
            names = "+".join(s for s in SIGNALS if s in on) or "(idle)"
            print(f"uPC={upc:02X} {STAGE_OF[upc]:<3} word={w:06X}  {names}")

        if "DECODE" in on:
            upc = MAPPING_ROM[op]
        else:
            upc = nxt
            if upc == 0x00:
                done += 1
                if trace: print("-" * 60)
    return cycles


if __name__ == "__main__":
    prog = [
        0b00_001_010,   # ADD   R1, R2     R1 = 10 + 5 = 15
        0b01_001_011,   # SUB   R1, R3     R1 = 15 - 3 = 12
        0b11_100_001,   # STORE [R4], R1   DM[2] = 12
        0b10_101_100,   # LOAD  R5, [R4]   R5 = DM[2] = 12
    ]
    regs = [0, 10, 5, 3, 2, 0, 0, 0]
    dmem = [0] * 8
    cycles = run(prog, regs, dmem, len(prog))
    print("Registers:", regs)
    print("Data mem :", dmem)
    print("Microword width:", NEXT_SHIFT + 5, "bits | Control store size:",
          len(CONTROL_STORE), "words | Total cycles:", cycles)
    assert regs[1] == 12 and dmem[2] == 12 and regs[5] == 12
    print("All checks passed")
