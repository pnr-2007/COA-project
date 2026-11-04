"""
6-stage VERTICAL microprogrammed control unit
Stages: IF -> ID -> OF -> EX -> MEM -> WB

Instruction (8 bit):  opcode(2) | Rd(3) | Rs(3)
  00 ADD   Rd <- Rd + Rs
  01 SUB   Rd <- Rd - Rs
  10 LOAD  Rd <- DM[Rs]
  11 STORE DM[Rd] <- Rs

Vertical microword (13 bit):  MOP(4) | ARG(4) | NEXT(5)
"""

# ---- micro-operation codes (MOP) ----
NOP, PC_MAR, IMEM_RD, MDR_IR, PC_INC, DECODE, A_LOAD, B_LOAD, \
    ALU, DMAR, DMEM_RD, DMEM_WR, WB_ALU, WB_MDR = range(14)

# ---- ALU function codes (ARG for ALU) ----
F_ADD, F_SUB, F_PASS_A, F_PASS_B = range(4)

MOP_NAMES = ["NOP", "PC->MAR", "MDR<-IM[MAR]", "IR<-MDR", "PC<-PC+1",
             "DECODE", "A<-Reg[Rs]", "B<-Reg[Rd]", "ALU", "MAR<-ALUOUT",
             "MDR<-DM[MAR]", "DM[MAR]<-A", "Reg[Rd]<-ALUOUT", "Reg[Rd]<-MDR"]

def word(mop, arg=0, nxt=0):
    return (mop << 9) | (arg << 5) | nxt

# addr: (stage, MOP, ARG, NEXT)
MICROCODE = {
    # ---- common: fetch + decode ----
    0x00: ("IF",  PC_MAR,  0, 0x01),
    0x01: ("IF",  IMEM_RD, 0, 0x02),
    0x02: ("IF",  MDR_IR,  0, 0x03),
    0x03: ("IF",  PC_INC,  0, 0x04),
    0x04: ("ID",  DECODE,  0, 0x00),     # NEXT replaced by mapping ROM

    # ---- ADD ----
    0x05: ("OF",  A_LOAD,  0, 0x06),
    0x06: ("OF",  B_LOAD,  0, 0x07),
    0x07: ("EX",  ALU, F_ADD, 0x08),
    0x08: ("MEM", NOP,     0, 0x09),
    0x09: ("WB",  WB_ALU,  0, 0x00),

    # ---- SUB ----
    0x0A: ("OF",  A_LOAD,  0, 0x0B),
    0x0B: ("OF",  B_LOAD,  0, 0x0C),
    0x0C: ("EX",  ALU, F_SUB, 0x0D),
    0x0D: ("MEM", NOP,     0, 0x0E),
    0x0E: ("WB",  WB_ALU,  0, 0x00),

    # ---- LOAD  Rd <- DM[Rs] ----
    0x0F: ("OF",  A_LOAD,  0, 0x10),
    0x10: ("EX",  ALU, F_PASS_A, 0x11),
    0x11: ("MEM", DMAR,    0, 0x12),
    0x12: ("MEM", DMEM_RD, 0, 0x13),
    0x13: ("WB",  WB_MDR,  0, 0x00),

    # ---- STORE  DM[Rd] <- Rs ----
    0x14: ("OF",  A_LOAD,  0, 0x15),
    0x15: ("OF",  B_LOAD,  0, 0x16),
    0x16: ("EX",  ALU, F_PASS_B, 0x17),
    0x17: ("MEM", DMAR,    0, 0x18),
    0x18: ("MEM", DMEM_WR, 0, 0x19),
    0x19: ("WB",  NOP,     0, 0x00),
}

# Control store holds only the packed 13-bit words
CONTROL_STORE = {a: word(m, g, n) for a, (_, m, g, n) in MICROCODE.items()}
STAGE_OF = {a: s for a, (s, *_ ) in MICROCODE.items()}

# opcode -> start address of its micro-routine
MAPPING_ROM = {0b00: 0x05, 0b01: 0x0A, 0b10: 0x0F, 0b11: 0x14}


def alu(fn, a, b):
    if fn == F_ADD:    return (b + a) & 0xFF
    if fn == F_SUB:    return (b - a) & 0xFF
    if fn == F_PASS_A: return a
    if fn == F_PASS_B: return b
    raise ValueError(fn)


def run(imem, regs, dmem, n_instr, trace=True):
    pc = MAR = MDR = IR = A = B = ALUOUT = 0
    upc, done = 0x00, 0

    while done < n_instr:
        w = CONTROL_STORE[upc]                  # read microword
        mop, arg, nxt = (w >> 9) & 0xF, (w >> 5) & 0xF, w & 0x1F
        rd, rs, op = (IR >> 3) & 7, IR & 7, (IR >> 6) & 3

        if   mop == PC_MAR:   MAR = pc
        elif mop == IMEM_RD:  MDR = imem[MAR]
        elif mop == MDR_IR:   IR = MDR
        elif mop == PC_INC:   pc += 1
        elif mop == A_LOAD:   A = regs[rs]
        elif mop == B_LOAD:   B = regs[rd]
        elif mop == ALU:      ALUOUT = alu(arg, A, B)
        elif mop == DMAR:     MAR = ALUOUT
        elif mop == DMEM_RD:  MDR = dmem[MAR]
        elif mop == DMEM_WR:  dmem[MAR] = A
        elif mop == WB_ALU:   regs[rd] = ALUOUT
        elif mop == WB_MDR:   regs[rd] = MDR

        if trace:
            print(f"uPC={upc:02X} {STAGE_OF[upc]:<3} word={w:04X}  {MOP_NAMES[mop]}")

        if mop == DECODE:
            upc = MAPPING_ROM[op]
        else:
            upc = nxt
            if upc == 0x00:
                done += 1
                if trace: print("-" * 50)


if __name__ == "__main__":
    prog = [
        0b00_001_010,   # ADD   R1, R2     R1 = 10 + 5 = 15
        0b01_001_011,   # SUB   R1, R3     R1 = 15 - 3 = 12
        0b11_100_001,   # STORE [R4], R1   DM[2] = 12
        0b10_101_100,   # LOAD  R5, [R4]   R5 = DM[2] = 12
    ]
    regs = [0, 10, 5, 3, 2, 0, 0, 0]
    dmem = [0] * 8
    run(prog, regs, dmem, len(prog))
    print("Registers:", regs)
    print("Data mem :", dmem)
    assert regs[1] == 12 and dmem[2] == 12 and regs[5] == 12
    print("All checks passed")
