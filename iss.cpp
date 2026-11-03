#pragma once

#include "exec.cpp"
#include "interrupt_handler.cpp"
#include <vector>

class ISS {
public:
    using Word  = ISA::Word;
    using Fault = Exec::Fault;

    ISS() : ISS(1 << 18, 1 << 19) {}
    ISS(size_t imemBytes, size_t dmemBytes)
        : imemBytes_(imemBytes), dmemBytes_(dmemBytes),
          imem_(imemBytes / 4, 0xFFFFFFFFu), dmem_(dmemBytes / 8, 0) {}

    void loadProgram(const std::vector<ISA::InstrWord>& words, uint64_t baseBytes = 0) {
        if ((baseBytes & 3) != 0 || baseBytes + words.size() * 4 > imemBytes_)
            throw std::out_of_range("ISS::loadProgram: program does not fit");
        for (size_t i = 0; i < words.size(); ++i) imem_[(baseBytes >> 2) + i] = words[i];
    }

    bool step() {
        if (halted_) return false;
        if ((pc_ & 3) != 0 || pc_ >= imemBytes_) return stop(Fault::FetchFault, pc_);

        ISA::InstructionFields f;
        try { f = ISA::decode(imem_[pc_ >> 2]); }
        catch (const std::invalid_argument&) { return stop(Fault::IllegalInstruction, imem_[pc_ >> 2]); }
        const Fault bad = Exec::validate(f);
        if (bad != Fault::None) return stop(bad, imem_[pc_ >> 2]);

        if (f.opcode == ISA::Opcode::HALT) { ++retired_; halted_ = true; return false; }

        if (f.opcode == ISA::Opcode::RETI && !trap_.inTrap()) return stop(Fault::IllegalInstruction, imem_[pc_ >> 2]);

        const Exec::RegUse u = Exec::regUse(f);
        auto readReg = [&](uint8_t r, bool fp) -> Word {
            if (fp)             return fregs_[r];
            if (r == ISA::ZERO) return 0;
            if (r == ISA::PC)   return pc_;
            return regs_[r];
        };
        const Word a = u.readsA ? readReg(u.a, u.aFp) : 0;
        const Word b = u.readsB ? readReg(u.b, u.bFp) : 0;
        const Word c = u.readsC ? readReg(u.c, false) : 0;

        uint64_t next  = pc_ + 4;
        Word     value = 0;
        bool     writes = u.writes;

        switch (f.type()) {
            case ISA::InstructionType::ALU: {
                const Word second = f.modifier ? static_cast<Word>(static_cast<int64_t>(f.immediate)) : b;
                Exec::alu(f.opcode, a, second, value);
                break;
            }
            case ISA::InstructionType::MEMORY: {
                const Word addr = a + static_cast<Word>(static_cast<int64_t>(f.immediate));
                if ((addr & 7) != 0 || addr >= dmemBytes_) return stop(Fault::MemoryFault, addr);
                if (f.opcode == ISA::Opcode::LOAD || f.opcode == ISA::Opcode::FLOAD) value = dmem_[addr >> 3];
                else                                                                 dmem_[addr >> 3] = b;
                break;
            }
            case ISA::InstructionType::FP: {
                const unsigned rm = trap_.roundingMode();
                if (Exec::fpRoundingIllegal(f.opcode, rm)) return stop(Fault::IllegalInstruction, imem_[pc_ >> 2]);
                uint8_t fl = 0;
                value = FPU::execute(Exec::fpOpOf(f.opcode), a, b, rm, fl);
                trap_.orFpFlags(fl);
                break;
            }
            case ISA::InstructionType::BRANCH: {
                if (Exec::branchTaken(f.opcode, a, b)) {
                    next = (f.opcode == ISA::Opcode::RET)
                         ? a
                         : pc_ + static_cast<Word>(static_cast<int64_t>(f.immediate) * 4);
                }
                if (f.opcode == ISA::Opcode::CALL) value = pc_ + 4;
                break;
            }
            case ISA::InstructionType::MATRIX: {
                unsigned aw = 0, bw = 0, ow = 0;
                Exec::matrixWords(f, aw, bw, ow);
                if (!Exec::regionOk(a, aw, dmemBytes_) || (bw && !Exec::regionOk(b, bw, dmemBytes_)) ||
                    !Exec::regionOk(c, ow, dmemBytes_))
                    return stop(Fault::MemoryFault, c);

                const bool mul = f.opcode == ISA::Opcode::MATMUL;
                ALU::Matrix A(static_cast<uint8_t>(f.matrixRows), static_cast<uint8_t>(mul ? f.matrixInner : f.matrixCols));
                ALU::Matrix B(static_cast<uint8_t>(mul ? f.matrixInner : f.matrixRows), static_cast<uint8_t>(f.matrixCols));
                for (unsigned e = 0; e < aw; ++e) A.data[e] = dmem_[(a >> 3) + e];
                for (unsigned e = 0; e < bw; ++e) B.data[e] = dmem_[(b >> 3) + e];

                const ALU::MatrixResult r = ALU::executeMatrixOp(f, A, B);
                for (unsigned e = 0; e < ow; ++e) dmem_[(c >> 3) + e] = r.hasScalar ? r.scalar : r.matrix.data[e];
                if (r.overflow) trap_.setStatusBits(ExceptionHandler::STATUS_OVERFLOW);
                break;
            }
            case ISA::InstructionType::SYSTEM:
                if (f.opcode == ISA::Opcode::RDSR)      value = trap_.read(static_cast<unsigned>(f.immediate));
                else if (f.opcode == ISA::Opcode::WRSR) trap_.write(static_cast<unsigned>(f.immediate), a);
                else if (f.opcode == ISA::Opcode::RETI) next = trap_.returnFromTrap();
                break;
            default: break;
        }

        if (writes) {
            if (u.destFp)                  fregs_[u.dest] = value;
            else if (u.dest != ISA::ZERO)  regs_[u.dest] = value;
        }
        ++retired_;
        pc_ = next;
        for (const Scheduled& e : schedule_)
            if (e.atRetired == retired_) trap_.raise(e.line);
        const int line = trap_.pendingInterrupt();
        if (line >= 0) pc_ = trap_.enterInterrupt(static_cast<unsigned>(line), pc_);
        return true;
    }

    void run(uint64_t maxInstructions) {
        for (uint64_t i = 0; i < maxInstructions && step(); ++i) {}
    }

    bool     halted()  const { return halted_; }
    Fault    fault()   const { return fault_; }
    uint64_t retired() const { return retired_; }
    uint64_t pc()      const { return pc_; }
    Word     reg(unsigned i) const { return regs_[i]; }
    Word     freg(unsigned i) const { return fregs_[i]; }
    const std::vector<Word>& dmem() const { return dmem_; }
    void pokeDmem(size_t wordIndex, Word value) { dmem_.at(wordIndex) = value; }
    void scheduleInterrupt(uint64_t atRetired, unsigned line) { schedule_.push_back({atRetired, line}); }
    const InterruptHandler& specialRegs() const { return trap_; }

private:
    bool stop(Fault f, Word badValue = 0) {
        if (trap_.canTakeException()) {
            pc_ = trap_.enterException(static_cast<unsigned>(f), pc_, badValue);
            return true;
        }
        fault_ = f;
        halted_ = true;
        return false;
    }

    struct Scheduled { uint64_t atRetired; unsigned line; };
    std::vector<Scheduled> schedule_;
    InterruptHandler trap_;

    size_t   imemBytes_, dmemBytes_;
    std::vector<ISA::InstrWord> imem_;
    std::vector<Word>           dmem_;
    ISA::RegisterFile           regs_{};
    ISA::RegisterFile           fregs_{};
    uint64_t pc_ = 0;
    bool     halted_ = false;
    Fault    fault_ = Fault::None;
    uint64_t retired_ = 0;
};
