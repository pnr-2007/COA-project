#pragma once

#include "exec.cpp"
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
        if ((pc_ & 3) != 0 || pc_ >= imemBytes_) return stop(Fault::FetchFault);

        ISA::InstructionFields f;
        try { f = ISA::decode(imem_[pc_ >> 2]); }
        catch (const std::invalid_argument&) { return stop(Fault::IllegalInstruction); }
        const Fault bad = Exec::validate(f);
        if (bad != Fault::None) return stop(bad);

        if (f.opcode == ISA::Opcode::HALT) { ++retired_; halted_ = true; return false; }

        const Exec::RegUse u = Exec::regUse(f);
        auto readReg = [&](uint8_t r) -> Word {
            if (r == ISA::ZERO) return 0;
            if (r == ISA::PC)   return pc_;
            return regs_[r];
        };
        const Word a = u.readsA ? readReg(u.a) : 0;
        const Word b = u.readsB ? readReg(u.b) : 0;
        const Word c = u.readsC ? readReg(u.c) : 0;

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
                if ((addr & 7) != 0 || addr >= dmemBytes_) return stop(Fault::MemoryFault);
                if (f.opcode == ISA::Opcode::LOAD) value = dmem_[addr >> 3];
                else                               dmem_[addr >> 3] = b;
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
                    return stop(Fault::MemoryFault);

                const bool mul = f.opcode == ISA::Opcode::MATMUL;
                ALU::Matrix A(static_cast<uint8_t>(f.matrixRows), static_cast<uint8_t>(mul ? f.matrixInner : f.matrixCols));
                ALU::Matrix B(static_cast<uint8_t>(mul ? f.matrixInner : f.matrixRows), static_cast<uint8_t>(f.matrixCols));
                for (unsigned e = 0; e < aw; ++e) A.data[e] = dmem_[(a >> 3) + e];
                for (unsigned e = 0; e < bw; ++e) B.data[e] = dmem_[(b >> 3) + e];

                const ALU::MatrixResult r = ALU::executeMatrixOp(f, A, B);
                for (unsigned e = 0; e < ow; ++e) dmem_[(c >> 3) + e] = r.hasScalar ? r.scalar : r.matrix.data[e];
                break;
            }
            default: break;
        }

        if (writes && u.dest != ISA::ZERO) regs_[u.dest] = value;
        ++retired_;
        pc_ = next;
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
    const std::vector<Word>& dmem() const { return dmem_; }
    void pokeDmem(size_t wordIndex, Word value) { dmem_.at(wordIndex) = value; }

private:
    bool stop(Fault f) { fault_ = f; halted_ = true; return false; }

    size_t   imemBytes_, dmemBytes_;
    std::vector<ISA::InstrWord> imem_;
    std::vector<Word>           dmem_;
    ISA::RegisterFile           regs_{};
    uint64_t pc_ = 0;
    bool     halted_ = false;
    Fault    fault_ = Fault::None;
    uint64_t retired_ = 0;
};
