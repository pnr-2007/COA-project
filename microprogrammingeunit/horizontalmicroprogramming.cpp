
#pragma once

#include "control_unit_signals.cpp"
#include <array>
#include <cstdint>
#include <sstream>
#include <string>

class HorizontalMicroprogrammedControlUnit {
public:
    using Fault = Exec::Fault;
    using Op = ISA::Opcode;

    static constexpr unsigned OP_COUNT = ISA::opcodeValue(Op::OPCODE_COUNT);

    struct Result {
        ControlWord word{};
        unsigned microCycles = 0;
    };

    struct Stats {
        uint64_t instructions = 0;
        uint64_t microCycles = 0;
        uint64_t faults = 0;
        std::array<uint64_t, OP_COUNT> opcodeCount{};
        std::array<uint64_t, OP_COUNT> opcodeCycles{};

        double avgCycles() const {
            return instructions == 0 ? 0.0 :
                static_cast<double>(microCycles) / instructions;
        }
    };

    HorizontalMicroprogrammedControlUnit() {
        reset();
    }

    void reset() {
        current_ = {};
        active_ = false;
        pc_ = 0;
        stats_ = {};
    }

    Result translateWord(ISA::InstrWord rawWord, uint64_t pc,
                         bool fetchFault = false) const {
        Result result;
        result.word = ControlUnit::decodeWord(rawWord, pc, fetchFault);
        result.microCycles = 1;
        return result;
    }

    Result translate(const ISA::InstructionFields& fields,
                     ISA::InstrWord rawWord = 0) const {
        Result result;

        Fault fault = ControlUnit::validate(fields);

        if (fault != Fault::None) {
            result.word.valid = true;
            result.word.opcode = fields.opcode;
            result.word.fault = fault;
            result.word.faultValue = rawWord;
        } else {
            result.word = ControlUnit::decode(fields);
        }

        result.microCycles = 1;
        return result;
    }

    void begin(ISA::InstrWord rawWord, uint64_t pc,
               bool fetchFault = false) {
        current_ = translateWord(rawWord, pc, fetchFault);
        active_ = true;
        pc_ = 0;
    }

    Result step() {
        if (!active_) return {};

        active_ = false;
        pc_ = 1;

        stats_.instructions++;
        stats_.microCycles += current_.microCycles;

        if (current_.word.fault != Fault::None) {
            stats_.faults++;
        } else {
            unsigned index = ISA::opcodeValue(current_.word.opcode);

            if (index < OP_COUNT) {
                stats_.opcodeCount[index]++;
                stats_.opcodeCycles[index] += current_.microCycles;
            }
        }

        return current_;
    }

    bool running() const {
        return active_;
    }

    const ControlWord& word() const {
        return current_.word;
    }

    unsigned microPc() const {
        return pc_;
    }

    unsigned entryPoint(Op opcode) const {
        return ISA::opcodeValue(opcode);
    }

    unsigned storeWords() const {
        return OP_COUNT;
    }

    unsigned storeBits() const {
        return sizeof(ControlWord) * 8;
    }

    const Stats& stats() const {
        return stats_;
    }

    std::string disassemble(const ControlWord& w) const {
        std::ostringstream out;

        out << "VALID=" << w.valid;
        out << " OPCODE=" << ISA::mnemonicOf(w.opcode);

        out << " READ_A=" << w.readsA;
        out << " READ_B=" << w.readsB;
        out << " READ_C=" << w.readsC;
        out << " IMM_B=" << w.immB;

        out << " SRC_A=" << unsigned(w.srcA);
        out << " SRC_B=" << unsigned(w.srcB);
        out << " SRC_C=" << unsigned(w.srcC);

        out << " SRC_A_FP=" << w.srcAFp;
        out << " SRC_B_FP=" << w.srcBFp;
        out << " DEST_FP=" << w.destFp;

        out << " ALU=" << unsigned(w.alu);
        out << " LONG_OP=" << w.longOp;

        out << " FP_OP=" << w.fpOp;
        out << " FP_ROUNDS=" << w.fpRounds;

        out << " REG_WRITE=" << w.regWrite;
        out << " DEST=" << unsigned(w.dest);
        out << " RESULT=" << unsigned(w.result);

        out << " MEM_READ=" << w.memRead;
        out << " MEM_WRITE=" << w.memWrite;

        out << " BRANCH=" << unsigned(w.branch);
        out << " CONDITIONAL=" << w.conditionalBranch;

        out << " MATRIX=" << w.matrixOp;
        out << " MAT_WAIT=" << w.matWait;

        out << " HALT=" << w.halt;
        out << " SERIALIZING=" << w.serializing;

        out << " READ_SR=" << w.readsSR;
        out << " WRITE_SR=" << w.writesSR;

        out << " FAULT=" << unsigned(w.fault);
        out << " FAULT_VALUE=" << w.faultValue;

        return out.str();
    }

    std::string disassemble() const {
        return disassemble(current_.word);
    }

    bool selfCheck() const {
        for (unsigned i = 0; i < OP_COUNT; i++) {
            Op opcode = static_cast<Op>(i);

            if (!ISA::isValidOpcode(opcode)) {
                return false;
            }
        }

        return true;
    }

private:
    Result current_{};
    bool active_ = false;
    unsigned pc_ = 0;
    Stats stats_{};
};

