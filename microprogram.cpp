#include "microprogram.h"

Microprogram::Microprogram() {
    buildFetch();

    for (uint8_t i = 0; i < ISA::opcodeValue(ISA::Opcode::OPCODE_COUNT); ++i) {
        ISA::Opcode opcode = static_cast<ISA::Opcode>(i);
        opcodeEntry[i] = OPCODE_BASE + i * OPCODE_BLOCK_SIZE;
        buildOpcode(opcode);
    }
}

const MicroInstruction& Microprogram::read(uint16_t address) const {
    if (address >= CONTROL_MEMORY_SIZE)
        throw std::out_of_range("Invalid microprogram address");

    return controlMemory[address];
}

uint16_t Microprogram::entryAddress(ISA::Opcode opcode) const {
    return opcodeEntry[ISA::opcodeValue(opcode)];
}

uint16_t Microprogram::fetchAddress() const {
    return FETCH_ADDRESS;
}

size_t Microprogram::size() const {
    return CONTROL_MEMORY_SIZE;
}

void Microprogram::buildFetch() {
    controlMemory[0].name = "FETCH: PC -> MAR";
    controlMemory[0].pcOut = true;
    controlMemory[0].marIn = true;
    controlMemory[0].nextAddress = 1;

    controlMemory[1].name = "FETCH: Memory Read";
    controlMemory[1].memRead = true;
    controlMemory[1].mdrIn = true;
    controlMemory[1].nextAddress = 2;

    controlMemory[2].name = "FETCH: MDR -> IR";
    controlMemory[2].mdrOut = true;
    controlMemory[2].irIn = true;
    controlMemory[2].nextAddress = 3;

    controlMemory[3].name = "FETCH: PC + 4";
    controlMemory[3].pcInc = true;
    controlMemory[3].nextAddress = 4;

    controlMemory[4].name = "FETCH: Dispatch";
    controlMemory[4].sequence = MicroSequence::DISPATCH;
}

void Microprogram::buildOpcode(ISA::Opcode opcode) {
    uint16_t base = entryAddress(opcode);
    auto& m = controlMemory;

    switch (opcode) {
        case ISA::Opcode::ADD:
        case ISA::Opcode::SUB:
        case ISA::Opcode::MUL:
        case ISA::Opcode::DIV:
        case ISA::Opcode::MOD:
        case ISA::Opcode::AND:
        case ISA::Opcode::OR:
        case ISA::Opcode::XOR:
        case ISA::Opcode::NOT:
        case ISA::Opcode::LSL:
        case ISA::Opcode::LSR:
        case ISA::Opcode::ASR:
        case ISA::Opcode::SIN:
        case ISA::Opcode::COS:
        case ISA::Opcode::LOG:
        case ISA::Opcode::EXP:
        case ISA::Opcode::POW:
        case ISA::Opcode::CMP: {
            m[base].name = std::string(ISA::mnemonicOf(opcode)) + ": Read operands";
            m[base].regRead1 = true;
            m[base].regRead2 = !ISA::isUnary(opcode);
            m[base].nextAddress = base + 1;

            m[base + 1].name = std::string(ISA::mnemonicOf(opcode)) + ": Execute ALU";
            m[base + 1].aluEnable = true;
            m[base + 1].aluOp = static_cast<MicroALUOp>(
                static_cast<uint8_t>(opcode) + 1
            );
            m[base + 1].nextAddress = base + 2;

            m[base + 2].name = std::string(ISA::mnemonicOf(opcode)) + ": Write result";
            m[base + 2].regWrite = true;
            m[base + 2].nextAddress = FETCH_ADDRESS;
            break;
        }

        case ISA::Opcode::LOAD:
            m[base].name = "LOAD: Read base register";
            m[base].regRead1 = true;
            m[base].nextAddress = base + 1;

            m[base + 1].name = "LOAD: Calculate address";
            m[base + 1].aluEnable = true;
            m[base + 1].aluOp = MicroALUOp::ADD;
            m[base + 1].immOut = true;
            m[base + 1].nextAddress = base + 2;

            m[base + 2].name = "LOAD: Memory Read";
            m[base + 2].memRead = true;
            m[base + 2].mdrIn = true;
            m[base + 2].nextAddress = base + 3;

            m[base + 3].name = "LOAD: Write register";
            m[base + 3].mdrOut = true;
            m[base + 3].regWrite = true;
            m[base + 3].nextAddress = FETCH_ADDRESS;
            break;

        case ISA::Opcode::STORE:
            m[base].name = "STORE: Read registers";
            m[base].regRead1 = true;
            m[base].regRead2 = true;
            m[base].nextAddress = base + 1;

            m[base + 1].name = "STORE: Calculate address";
            m[base + 1].aluEnable = true;
            m[base + 1].aluOp = MicroALUOp::ADD;
            m[base + 1].immOut = true;
            m[base + 1].nextAddress = base + 2;

            m[base + 2].name = "STORE: Memory Write";
            m[base + 2].memWrite = true;
            m[base + 2].nextAddress = FETCH_ADDRESS;
            break;

        case ISA::Opcode::JEQ:
            m[base].name = "JEQ: Read registers";
            m[base].regRead1 = true;
            m[base].regRead2 = true;
            m[base].nextAddress = base + 1;

            m[base + 1].name = "JEQ: Compare";
            m[base + 1].aluEnable = true;
            m[base + 1].aluOp = MicroALUOp::CMP;
            m[base + 1].nextAddress = base + 2;

            m[base + 2].name = "JEQ: Branch if equal";
            m[base + 2].branch = true;
            m[base + 2].branchEqual = true;
            m[base + 2].nextAddress = FETCH_ADDRESS;
            break;

        case ISA::Opcode::JGT:
            m[base].name = "JGT: Read registers";
            m[base].regRead1 = true;
            m[base].regRead2 = true;
            m[base].nextAddress = base + 1;

            m[base + 1].name = "JGT: Compare";
            m[base + 1].aluEnable = true;
            m[base + 1].aluOp = MicroALUOp::CMP;
            m[base + 1].nextAddress = base + 2;

            m[base + 2].name = "JGT: Branch if greater";
            m[base + 2].branch = true;
            m[base + 2].branchGreater = true;
            m[base + 2].nextAddress = FETCH_ADDRESS;
            break;

        case ISA::Opcode::JZ:
            m[base].name = "JZ: Read register";
            m[base].regRead1 = true;
            m[base].nextAddress = base + 1;

            m[base + 1].name = "JZ: Test zero";
            m[base + 1].aluEnable = true;
            m[base + 1].aluOp = MicroALUOp::CMP;
            m[base + 1].nextAddress = base + 2;

            m[base + 2].name = "JZ: Branch if zero";
            m[base + 2].branch = true;
            m[base + 2].branchZero = true;
            m[base + 2].nextAddress = FETCH_ADDRESS;
            break;

        case ISA::Opcode::JNZ:
            m[base].name = "JNZ: Read register";
            m[base].regRead1 = true;
            m[base].nextAddress = base + 1;

            m[base + 1].name = "JNZ: Test zero";
            m[base + 1].aluEnable = true;
            m[base + 1].aluOp = MicroALUOp::CMP;
            m[base + 1].nextAddress = base + 2;

            m[base + 2].name = "JNZ: Branch if non-zero";
            m[base + 2].branch = true;
            m[base + 2].branchNotZero = true;
            m[base + 2].nextAddress = FETCH_ADDRESS;
            break;

        case ISA::Opcode::JMP:
            m[base].name = "JMP: Immediate -> PC";
            m[base].immOut = true;
            m[base].pcIn = true;
            m[base].nextAddress = FETCH_ADDRESS;
            break;

        case ISA::Opcode::CALL:
            m[base].name = "CALL: Save return address";
            m[base].linkWrite = true;
            m[base].nextAddress = base + 1;

            m[base + 1].name = "CALL: Immediate -> PC";
            m[base + 1].immOut = true;
            m[base + 1].pcIn = true;
            m[base + 1].nextAddress = FETCH_ADDRESS;
            break;

        case ISA::Opcode::RET:
            m[base].name = "RET: LR -> PC";
            m[base].linkOut = true;
            m[base].pcIn = true;
            m[base].nextAddress = FETCH_ADDRESS;
            break;

        case ISA::Opcode::MATMUL:
        case ISA::Opcode::MATADD:
        case ISA::Opcode::MATTRANSPOSE:
        case ISA::Opcode::MATINVERSE:
        case ISA::Opcode::MATADJOINT:
        case ISA::Opcode::MATDETERMINANT:
            m[base].name = std::string(ISA::mnemonicOf(opcode)) + ": Read matrix operands";
            m[base].regRead1 = true;
            m[base].regRead2 = true;
            m[base].nextAddress = base + 1;

            m[base + 1].name = std::string(ISA::mnemonicOf(opcode)) + ": Execute matrix operation";
            m[base + 1].matrixEnable = true;
            m[base + 1].nextAddress = base + 2;

            m[base + 2].name = std::string(ISA::mnemonicOf(opcode)) + ": Write result";
            m[base + 2].regWrite = true;
            m[base + 2].nextAddress = FETCH_ADDRESS;
            break;

        case ISA::Opcode::NOP:
            m[base].name = "NOP";
            m[base].nextAddress = FETCH_ADDRESS;
            break;

        case ISA::Opcode::HALT:
            m[base].name = "HALT";
            m[base].sequence = MicroSequence::HALT;
            break;

        default:
            break;
    }
}

void Microprogram::dump() const {
    for (size_t i = 0; i < CONTROL_MEMORY_SIZE; ++i) {
        if (!controlMemory[i].name.empty()) {
            std::cout << "µ" << i << " : "
                      << controlMemory[i].name << '\n';
        }
    }
}

MicroprogrammedControlUnit::MicroprogrammedControlUnit() {
    reset();
}

void MicroprogrammedControlUnit::reset() {
    microProgramCounter = Microprogram::FETCH_ADDRESS;
    opcode = ISA::Opcode::NOP;
    current = program.read(microProgramCounter);
    isHalted = false;
}

void MicroprogrammedControlUnit::loadInstruction(ISA::InstrWord instruction) {
    ISA::InstructionFields fields = ISA::decode(instruction);
    loadOpcode(fields.opcode);
}

void MicroprogrammedControlUnit::loadOpcode(ISA::Opcode newOpcode) {
    opcode = newOpcode;
    microProgramCounter = program.entryAddress(opcode);
    current = program.read(microProgramCounter);
    isHalted = false;
}

bool MicroprogrammedControlUnit::step() {
    if (isHalted)
        return false;

    current = program.read(microProgramCounter);

    std::cout << "\n[micro-cycle]\n";
    std::cout << "microPC = " << microProgramCounter << '\n';
    std::cout << "microinstruction = " << current.name << '\n';

    if (current.sequence == MicroSequence::HALT) {
        isHalted = true;
        std::cout << "[MICROCONTROL] HALT\n";
        return false;
    }

    if (current.sequence == MicroSequence::DISPATCH) {
        microProgramCounter = program.entryAddress(opcode);
    } else {
        microProgramCounter = current.nextAddress;
    }

    return true;
}

void MicroprogrammedControlUnit::run(size_t maxCycles) {
    size_t cycles = 0;

    while (!isHalted && cycles < maxCycles) {
        if (!step())
            break;
        ++cycles;
    }
}

uint16_t MicroprogrammedControlUnit::microPC() const {
    return microProgramCounter;
}

ISA::Opcode MicroprogrammedControlUnit::currentOpcode() const {
    return opcode;
}

const MicroInstruction&
MicroprogrammedControlUnit::currentMicroInstruction() const {
    return current;
}

bool MicroprogrammedControlUnit::halted() const {
    return isHalted;
}

const Microprogram& MicroprogrammedControlUnit::getMicroprogram() const {
    return program;
}

void MicroprogrammedControlUnit::printState() const {
    std::cout << "microPC = " << microProgramCounter << '\n';
    std::cout << "opcode = "
              << ISA::mnemonicOf(opcode) << '\n';
    std::cout << "microinstruction = "
              << current.name << '\n';
}
