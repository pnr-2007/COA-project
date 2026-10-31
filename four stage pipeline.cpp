#pragma once

#include "exec.cpp"
#include "matrix_unit.cpp"
#include <vector>

class Pipeline {
public:
    using Word  = ISA::Word;
    using Fault = Exec::Fault;

    struct Config {
        size_t   imemBytes    = 1 << 18;
        size_t   dmemBytes    = 1 << 19;
        uint32_t longOpCycles = 64;
        MatrixUnit::Config matrix;
    };

    struct Stats {
        uint64_t cycles            = 0;
        uint64_t retired           = 0;
        uint64_t takenBranches     = 0;
        uint64_t execBusyCycles    = 0;
        uint64_t rawHazards        = 0;
        uint64_t matrixOps         = 0;
        uint64_t matrixStallCycles = 0;
        uint64_t drainCycles       = 0;
    };

    Pipeline() : Pipeline(Config()) {}
    explicit Pipeline(const Config& cfg)
        : cfg_(cfg), imem_(cfg.imemBytes / 4, 0xFFFFFFFFu), dmem_(cfg.dmemBytes / 8, 0), mu_(dmem_, cfg.matrix) {}
    Pipeline(const Pipeline&) = delete;
    Pipeline& operator=(const Pipeline&) = delete;

    void loadProgram(const std::vector<ISA::InstrWord>& words, uint64_t baseBytes = 0) {
        if ((baseBytes & 3) != 0 || baseBytes + words.size() * 4 > cfg_.imemBytes)
            throw std::out_of_range("Pipeline::loadProgram: program does not fit");
        for (size_t i = 0; i < words.size(); ++i) imem_[(baseBytes >> 2) + i] = words[i];
    }

    void step() {
        if (halted_) return;
        ++stats_.cycles;

        if (draining_) {
            ++stats_.drainCycles;
            mu_.step();
            if (!mu_.busy()) halted_ = true;
            return;
        }

        if (maRw_.valid) {
            if (maRw_.fault != Fault::None) { fault_ = maRw_.fault; beginDrain(); return; }
            if (maRw_.regWrite && maRw_.rd != ISA::ZERO) regs_[maRw_.rd] = maRw_.value;
            ++stats_.retired;
            if (maRw_.halt) { beginDrain(); return; }
        }

        if (mu_.step()) matBusy_ = false;

        bool squash = false;
        MaRw nextMaRw;
        if (exMa_.valid) {
            nextMaRw.valid    = true;
            nextMaRw.pc       = exMa_.pc;
            nextMaRw.rd       = exMa_.rd;
            nextMaRw.regWrite = exMa_.regWrite;
            nextMaRw.halt     = exMa_.halt;
            nextMaRw.fault    = exMa_.fault;
            nextMaRw.value    = exMa_.value;
            if (exMa_.fault == Fault::None) {
                if (exMa_.matrixOp) {
                    const MatrixUnit::StartResult r =
                        mu_.start(exMa_.matFields, exMa_.matPtrD, exMa_.matPtrA, exMa_.matPtrB);
                    if (r == MatrixUnit::StartResult::Started) {
                        ++stats_.matrixOps;
                    } else if (r == MatrixUnit::StartResult::Busy) {
                        throw std::logic_error("Pipeline: matrix unit started while busy");
                    } else {
                        nextMaRw.fault = (r == MatrixUnit::StartResult::MemoryFault) ? Fault::MemoryFault
                                                                                      : Fault::IllegalInstruction;
                        squash   = true;
                        matBusy_ = false;
                    }
                } else if (exMa_.memRead || exMa_.memWrite) {
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
        }

        ExMa nextExMa;
        bool stallFetch = false, stopFetch = false, redirect = false;
        uint64_t target = 0;

        if (busy_.active) {
            if (--busy_.remaining == 0) { nextExMa = busy_.result; busy_.active = false; }
            else                        { stallFetch = true; ++stats_.execBusyCycles; }
        } else if (ifDe_.valid) {
            decodeExecute(nextExMa, stopFetch, redirect, target, stallFetch);
        }

        IfDe nextIfDe;
        if (squash || stopFetch) {
            fetchEnabled_ = false;
            if (squash) { nextExMa = ExMa(); busy_.active = false; }
        } else if (stallFetch) {
            nextIfDe = ifDe_;
        } else if (fetchEnabled_) {
            nextIfDe = fetch();
        }
        if (redirect && !squash && !stopFetch) {
            pc_ = target;
            ++stats_.takenBranches;
        }
        if (nextExMa.valid && nextExMa.matrixOp) matBusy_ = true;

        maRw_ = nextMaRw;
        exMa_ = nextExMa;
        ifDe_ = nextIfDe;
    }

    bool run(uint64_t maxCycles) {
        while (!halted_ && stats_.cycles < maxCycles) step();
        return halted_;
    }

    bool         halted() const { return halted_; }
    Fault        fault()  const { return fault_; }
    const Stats& stats()  const { return stats_; }
    const MatrixUnit::Stats& matrixStats() const { return mu_.stats(); }
    uint64_t     pc()     const { return pc_; }
    Word         reg(unsigned i) const { return regs_[i]; }
    const std::vector<Word>& dmem() const { return dmem_; }
    void pokeDmem(size_t wordIndex, Word value) { dmem_.at(wordIndex) = value; }

private:
    struct IfDe {
        bool valid = false, fetchFault = false;
        uint64_t pc = 0;
        ISA::InstrWord word = 0;
    };
    struct ExMa {
        bool valid = false, regWrite = false, memRead = false, memWrite = false, halt = false;
        bool matrixOp = false;
        uint64_t pc = 0;
        uint8_t  rd = 0;
        Word     value = 0;
        Word     storeData = 0;
        ISA::InstructionFields matFields;
        Word     matPtrD = 0, matPtrA = 0, matPtrB = 0;
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

    void beginDrain() {
        draining_ = true;
        if (!mu_.busy()) halted_ = true;
    }

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

        const bool isMatrix = f.type() == ISA::InstructionType::MATRIX;
        if ((isMatrix || f.opcode == ISA::Opcode::MATWAIT) && matBusy_) {
            out = ExMa();
            stallFetch = true;
            ++stats_.matrixStallCycles;
            return;
        }

        const Exec::RegUse u = Exec::regUse(f);

        if (exMa_.valid && exMa_.regWrite && exMa_.rd != ISA::ZERO &&
            ((u.readsA && u.a == exMa_.rd) || (u.readsB && u.b == exMa_.rd) || (u.readsC && u.c == exMa_.rd)))
            ++stats_.rawHazards;

        auto readReg = [&](uint8_t r) -> Word {
            if (r == ISA::ZERO) return 0;
            if (r == ISA::PC)   return ifDe_.pc;
            return regs_[r];
        };
        const Word a = u.readsA ? readReg(u.a) : 0;
        const Word b = u.readsB ? readReg(u.b) : 0;
        const Word c = u.readsC ? readReg(u.c) : 0;
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
                    out             = ExMa();
                    stallFetch      = true;
                    ++stats_.execBusyCycles;
                }
                break;
            }
            case ISA::InstructionType::MEMORY:
                out.value = a + imm;
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
            case ISA::InstructionType::MATRIX:
                out.matrixOp = true;
                out.matFields = f;
                out.matPtrD = c;
                out.matPtrA = a;
                out.matPtrB = b;
                break;
            default: break;
        }
    }

    Config cfg_;
    std::vector<ISA::InstrWord> imem_;
    std::vector<Word>           dmem_;
    MatrixUnit                  mu_;
    ISA::RegisterFile           regs_{};
    uint64_t pc_ = 0;
    bool     fetchEnabled_ = true;
    bool     halted_ = false;
    bool     draining_ = false;
    bool     matBusy_ = false;
    Fault    fault_ = Fault::None;

    IfDe ifDe_;
    ExMa exMa_;
    MaRw maRw_;
    Busy busy_;
    Stats stats_;
};
