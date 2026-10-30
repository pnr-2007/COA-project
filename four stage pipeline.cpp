#pragma once

#include "exec.cpp"
#include <vector>

// Four-stage in-order pipeline, cycle accurate:
//
//     IF  |  DE+EX  |  MA  |  RW
//
//   IF     fetch the instruction at PC, PC += 4
//   DE+EX  decode, read registers, execute (ALU op, branch decision, address calculation)
//   MA     data-memory access for LOAD / STORE
//   RW     write the register file, retire, take faults and HALT
//
// THERE IS NO HAZARD UNIT YET. That means:
//   * No forwarding. A register written by an instruction is visible to the instruction
//     two places later (the register file is written in the first half of the cycle and
//     read in the second), but NOT to the very next instruction. Put one instruction
//     (a NOP) between a producer and its consumer. `Stats::rawHazards` counts the times
//     this was violated, so a test program can assert that it padded correctly.
//   * No load-use stall; LOAD follows the same one-instruction gap rule.
//   * No branch flush. A taken branch redirects the PC, but the instruction already fetched
//     behind it still executes (an exposed delay slot). Put a NOP after every branch.
//     CALL saves PC+4, which is that slot instruction.
//
// The only stall is structural: a long operation (MUL, DIV, ...) keeps DE+EX busy for
// `Config::longOpCycles` cycles, holding the IF/DE register and feeding bubbles to MA.
//
// Faults (SPEC.md section 6) travel with the instruction and take effect at RW, so every
// older instruction has committed and no younger one has. HALT behaves the same way.
class Pipeline {
public:
    using Word  = ISA::Word;
    using Fault = Exec::Fault;

    struct Config {
        size_t   imemBytes    = 1 << 18;   // 256 KiB
        size_t   dmemBytes    = 1 << 19;   // 512 KiB
        uint32_t longOpCycles = 64;        // cycles a MUL/DIV/MOD/SIN/COS/LOG spends in DE+EX (placeholder)
    };

    struct Stats {
        uint64_t cycles         = 0;
        uint64_t retired        = 0;   // instructions that committed (HALT included, faulting one not)
        uint64_t takenBranches  = 0;
        uint64_t execBusyCycles = 0;   // cycles lost waiting for a long operation
        uint64_t rawHazards     = 0;   // reads of a register the previous instruction is about to write
    };

    Pipeline() : Pipeline(Config()) {}
    explicit Pipeline(const Config& cfg)
        : cfg_(cfg), imem_(cfg.imemBytes / 4, 0xFFFFFFFFu), dmem_(cfg.dmemBytes / 8, 0) {}

    void loadProgram(const std::vector<ISA::InstrWord>& words, uint64_t baseBytes = 0) {
        if ((baseBytes & 3) != 0 || baseBytes + words.size() * 4 > cfg_.imemBytes)
            throw std::out_of_range("Pipeline::loadProgram: program does not fit");
        for (size_t i = 0; i < words.size(); ++i) imem_[(baseBytes >> 2) + i] = words[i];
    }

    // Advance one clock cycle.
    void step() {
        if (halted_) return;
        ++stats_.cycles;

        // ---- RW: runs first, so DE+EX below reads the freshly written register file ----
        if (maRw_.valid) {
            if (maRw_.fault != Fault::None) { fault_ = maRw_.fault; halted_ = true; return; }
            if (maRw_.regWrite && maRw_.rd != ISA::ZERO) regs_[maRw_.rd] = maRw_.value;
            ++stats_.retired;
            if (maRw_.halt) { halted_ = true; return; }
        }

        // ---- MA ----
        bool squash = false;                      // a memory fault kills everything younger
        MaRw nextMaRw;
        if (exMa_.valid) {
            nextMaRw.valid    = true;
            nextMaRw.pc       = exMa_.pc;
            nextMaRw.rd       = exMa_.rd;
            nextMaRw.regWrite = exMa_.regWrite;
            nextMaRw.halt     = exMa_.halt;
            nextMaRw.fault    = exMa_.fault;
            nextMaRw.value    = exMa_.value;
            if (exMa_.fault == Fault::None && (exMa_.memRead || exMa_.memWrite)) {
                const Word addr = exMa_.value;
                if ((addr & 7) != 0 || addr >= cfg_.dmemBytes) {
                    nextMaRw.fault    = Fault::MemoryFault;
                    nextMaRw.regWrite = false;
                    squash = true;
                } else if (exMa_.memRead) {
                    nextMaRw.value = dmem_[addr >> 3];
                } else {
                    dmem_[addr >> 3] = exMa_.storeData;
                }
            }
        }

        // ---- DE+EX ----
        ExMa nextExMa;                            // bubble unless something completes
        bool stallFetch = false, stopFetch = false, redirect = false;
        uint64_t target = 0;

        if (busy_.active) {
            if (--busy_.remaining == 0) { nextExMa = busy_.result; busy_.active = false; }
            else                        { stallFetch = true; ++stats_.execBusyCycles; }
        } else if (ifDe_.valid) {
            decodeExecute(nextExMa, stopFetch, redirect, target, stallFetch);
        }

        // ---- IF ----
        IfDe nextIfDe;
        if (squash || stopFetch) {
            fetchEnabled_ = false;
            if (squash) { nextExMa = ExMa(); busy_.active = false; }
        } else if (stallFetch) {
            nextIfDe = ifDe_;
        } else if (fetchEnabled_) {
            nextIfDe = fetch();                   // fetches the sequential instruction, PC += 4
        }
        if (redirect && !squash && !stopFetch) {  // taken branch: next fetch is the target
            pc_ = target;
            ++stats_.takenBranches;
        }

        maRw_ = nextMaRw;
        exMa_ = nextExMa;
        ifDe_ = nextIfDe;
    }

    // Run until HALT, a fault, or maxCycles. Returns true if the machine stopped.
    bool run(uint64_t maxCycles) {
        while (!halted_ && stats_.cycles < maxCycles) step();
        return halted_;
    }

    bool         halted() const { return halted_; }
    Fault        fault()  const { return fault_; }
    const Stats& stats()  const { return stats_; }
    uint64_t     pc()     const { return pc_; }
    Word         reg(unsigned i) const { return regs_[i]; }
    const std::vector<Word>& dmem() const { return dmem_; }

private:
    struct IfDe {
        bool valid = false, fetchFault = false;
        uint64_t pc = 0;
        ISA::InstrWord word = 0;
    };
    struct ExMa {
        bool valid = false, regWrite = false, memRead = false, memWrite = false, halt = false;
        uint64_t pc = 0;
        uint8_t  rd = 0;
        Word     value = 0;       // ALU result, link address, or memory address
        Word     storeData = 0;
        Fault    fault = Fault::None;
    };
    struct MaRw {
        bool valid = false, regWrite = false, halt = false;
        uint64_t pc = 0;
        uint8_t  rd = 0;
        Word     value = 0;
        Fault    fault = Fault::None;
    };
    struct Busy {
        bool active = false;
        uint32_t remaining = 0;
        ExMa result;
    };

    IfDe fetch() {
        IfDe r;
        r.valid = true;
        r.pc    = pc_;
        if ((pc_ & 3) != 0 || pc_ >= cfg_.imemBytes) {
            r.fetchFault  = true;
            fetchEnabled_ = false;
        } else {
            r.word = imem_[pc_ >> 2];
            pc_ += 4;
        }
        return r;
    }

    void decodeExecute(ExMa& out, bool& stopFetch, bool& redirect, uint64_t& target, bool& stallFetch) {
        out.valid = true;
        out.pc    = ifDe_.pc;

        ISA::InstructionFields f;
        Fault bad = Fault::None;
        if (ifDe_.fetchFault) {
            bad = Fault::FetchFault;
        } else {
            try { f = ISA::decode(ifDe_.word); bad = Exec::validate(f); }
            catch (const std::invalid_argument&) { bad = Fault::IllegalInstruction; }
        }
        if (bad != Fault::None) { out.fault = bad; stopFetch = true; return; }
        if (f.opcode == ISA::Opcode::HALT) { out.halt = true; stopFetch = true; return; }

        const Exec::RegUse u = Exec::regUse(f);

        // No hazard unit: count the reads that will see a stale value.
        if (exMa_.valid && exMa_.regWrite && exMa_.rd != ISA::ZERO &&
            ((u.readsA && u.a == exMa_.rd) || (u.readsB && u.b == exMa_.rd)))
            ++stats_.rawHazards;

        auto readReg = [&](uint8_t r) -> Word {
            if (r == ISA::ZERO) return 0;
            if (r == ISA::PC)   return ifDe_.pc;     // R15 reads as the instruction's own address
            return regs_[r];
        };
        const Word a = u.readsA ? readReg(u.a) : 0;
        const Word b = u.readsB ? readReg(u.b) : 0;
        const Word imm = static_cast<Word>(static_cast<int64_t>(f.immediate));

        out.regWrite = u.writes;
        out.rd       = u.dest;

        switch (f.type()) {
            case ISA::InstructionType::ALU: {
                Exec::alu(f.opcode, a, f.modifier ? imm : b, out.value);
                if (Exec::isLongOp(f.opcode) && cfg_.longOpCycles > 1) {
                    busy_.active    = true;
                    busy_.remaining = cfg_.longOpCycles - 1;
                    busy_.result    = out;
                    out             = ExMa();         // nothing reaches MA until it finishes
                    stallFetch      = true;
                    ++stats_.execBusyCycles;
                }
                break;
            }
            case ISA::InstructionType::MEMORY:
                out.value = a + imm;                  // byte address
                if (f.opcode == ISA::Opcode::LOAD) out.memRead = true;
                else { out.memWrite = true; out.storeData = b; }
                break;
            case ISA::InstructionType::BRANCH:
                if (Exec::branchTaken(f.opcode, a, b)) {
                    redirect = true;
                    target = (f.opcode == ISA::Opcode::RET)
                           ? a
                           : ifDe_.pc + static_cast<Word>(static_cast<int64_t>(f.immediate) * 4);
                }
                if (f.opcode == ISA::Opcode::CALL) out.value = ifDe_.pc + 4;
                break;
            default: break;                           // NOP
        }
    }

    Config cfg_;
    std::vector<ISA::InstrWord> imem_;
    std::vector<Word>           dmem_;
    ISA::RegisterFile           regs_{};
    uint64_t pc_ = 0;
    bool     fetchEnabled_ = true;
    bool     halted_ = false;
    Fault    fault_ = Fault::None;

    IfDe ifDe_;
    ExMa exMa_;
    MaRw maRw_;
    Busy busy_;
    Stats stats_;
};
