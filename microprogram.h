#pragma once

#include <array>
#include <cstdint>
#include <iostream>
#include <string>
#include "isa.cpp"

enum class MicroALUOp : uint8_t {
    NONE,
    ADD,
    SUB,
    MUL,
    DIV,
    MOD,
    AND,
    OR,
    XOR,
    NOT,
    LSL,
    LSR,
    ASR,
    SIN,
    COS,
    LOG,
    EXP,
    POW,
    CMP
};

enum class MicroSequence : uint8_t {
    NEXT,
    DISPATCH,
    HALT
};

struct MicroInstruction {
    bool pcOut = false;
    bool pcIn = false;
    bool pcInc = false;

    bool irOut = false;
    bool irIn = false;

    bool regRead1 = false;
    bool regRead2 = false;
    bool regWrite = false;

    bool immOut = false;

    bool marIn = false;
    bool mdrIn = false;
    bool mdrOut = false;
    bool memRead = false;
    bool memWrite = false;

    bool aluEnable = false;
    bool aluOut = false;
    MicroALUOp aluOp = MicroALUOp::NONE;

    bool branch = false;
    bool branchEqual = false;
    bool branchGreater = false;
    bool branchZero = false;
    bool branchNotZero = false;

    bool linkWrite = false;
    bool linkOut = false;

    bool matrixEnable = false;

    MicroSequence sequence = MicroSequence::NEXT;
    uint16_t nextAddress = 0;

    std::string name;
};

class Microprogram {
public:
    static constexpr uint16_t FETCH_ADDRESS = 0;
    static constexpr uint16_t OPCODE_BASE = 16;
    static constexpr uint16_t OPCODE_BLOCK_SIZE = 8;
    static constexpr size_t CONTROL_MEMORY_SIZE = 512;

    Microprogram();

    const MicroInstruction& read(uint16_t address) const;

    uint16_t entryAddress(ISA::Opcode opcode) const;
    uint16_t fetchAddress() const;

    size_t size() const;

    void dump() const;

private:
    std::array<MicroInstruction, CONTROL_MEMORY_SIZE> controlMemory{};
    std::array<uint16_t, 64> opcodeEntry{};

    void buildFetch();
    void buildOpcode(ISA::Opcode opcode);
};

class MicroprogrammedControlUnit {
public:
    MicroprogrammedControlUnit();

    void reset();

    void loadInstruction(ISA::InstrWord instruction);
    void loadOpcode(ISA::Opcode opcode);

    bool step();
    void run(size_t maxCycles = 1000);

    uint16_t microPC() const;
    ISA::Opcode currentOpcode() const;
    const MicroInstruction& currentMicroInstruction() const;

    bool halted() const;

    const Microprogram& getMicroprogram() const;

    void printState() const;

private:
    Microprogram program;
    uint16_t microProgramCounter = Microprogram::FETCH_ADDRESS;
    ISA::Opcode opcode = ISA::Opcode::NOP;
    MicroInstruction current{};
    bool isHalted = false;
};
