#include "matrix_unit.cpp"
#include <iostream>
#include <random>

using Word = ISA::Word;
using Op   = ISA::Opcode;
using SR   = MatrixUnit::StartResult;

static int fails = 0;
#define CHECK(c) do { if (!(c)) { ++fails; std::cout << "  FAIL line " << __LINE__ << ": " #c "\n"; } } while (0)

static const Word AP = 0x0000, BP = 0x2000, CP = 0x4000;
static const Word SENTINEL = 0xDEADBEEFDEADBEEFull;

static ISA::InstructionFields fields(Op op, unsigned rows, unsigned inner, unsigned cols) {
    ISA::InstructionFields f;
    f.opcode = op;
    f.matrixRows = rows;
    f.matrixInner = inner;
    f.matrixCols = cols;
    if (op == Op::MATADD || op == Op::MATDETERMINANT) f.matrixInner = 0;
    return f;
}

static ISA::InstructionFields viaEncoding(const ISA::InstructionFields& f) { return ISA::decode(ISA::encode(f)); }

static void put(std::vector<Word>& mem, Word base, const ALU::Matrix& m) {
    for (unsigned e = 0; e < unsigned(m.rows) * m.cols; ++e) mem[(base >> 3) + e] = m.data[e];
}

static void fillSentinel(std::vector<Word>& mem, Word base, unsigned words) {
    for (unsigned e = 0; e < words; ++e) mem[(base >> 3) + e] = SENTINEL;
}

static uint64_t runToDone(MatrixUnit& u) {
    uint64_t cycles = 0;
    while (u.busy() && cycles < 10'000'000) { ++cycles; u.step(); }
    return cycles;
}

static ALU::Matrix randomMatrix(std::mt19937& rng, unsigned r, unsigned c, bool wild) {
    ALU::Matrix m(r, c);
    std::uniform_real_distribution<double> d(-4.0, 4.0);
    for (unsigned i = 0; i < r * c; ++i) {
        if (wild && rng() % 6 == 0) m.data[i] = ALU::Matrix::fromInt(static_cast<int64_t>(rng() % 4000000000u) - 2000000000);
        else                        m.data[i] = ALU::Matrix::fromDouble(d(rng));
    }
    return m;
}

static bool sameWords(const std::vector<Word>& mem, Word base, const Word* expect, unsigned words) {
    for (unsigned e = 0; e < words; ++e) if (mem[(base >> 3) + e] != expect[e]) return false;
    return mem[(base >> 3) + words] == SENTINEL;
}

static void checkAdd(MatrixUnit& u, std::vector<Word>& mem, const ALU::Matrix& A, const ALU::Matrix& B, const char* tag) {
    put(mem, AP, A); put(mem, BP, B); fillSentinel(mem, CP, 401);
    const auto ref = ALU::matAdd(A, B);
    CHECK(u.start(viaEncoding(fields(Op::MATADD, A.rows, 0, A.cols)), CP, AP, BP) == SR::Started);
    runToDone(u);
    if (!sameWords(mem, CP, ref.matrix.data.data(), A.rows * A.cols) || u.lastOverflow() != ref.overflow) {
        ++fails; std::cout << "  FAIL matadd mismatch (" << tag << ") " << int(A.rows) << "x" << int(A.cols) << "\n";
    }
}

static void checkMul(MatrixUnit& u, std::vector<Word>& mem, const ALU::Matrix& A, const ALU::Matrix& B, const char* tag) {
    put(mem, AP, A); put(mem, BP, B); fillSentinel(mem, CP, 401);
    const auto ref = ALU::matMul(A, B);
    CHECK(u.start(viaEncoding(fields(Op::MATMUL, A.rows, A.cols, B.cols)), CP, AP, BP) == SR::Started);
    runToDone(u);
    if (!sameWords(mem, CP, ref.matrix.data.data(), A.rows * B.cols) || u.lastOverflow() != ref.overflow) {
        ++fails; std::cout << "  FAIL matmul mismatch (" << tag << ") " << int(A.rows) << "x" << int(A.cols) << "x" << int(B.cols) << "\n";
    }
}

static void checkDet(MatrixUnit& u, std::vector<Word>& mem, const ALU::Matrix& A, const char* tag) {
    put(mem, AP, A); fillSentinel(mem, CP, 2);
    const auto ref = ALU::matDeterminant(A);
    CHECK(u.start(viaEncoding(fields(Op::MATDETERMINANT, A.rows, 0, A.cols)), CP, AP, 0) == SR::Started);
    runToDone(u);
    if (!sameWords(mem, CP, &ref.scalar, 1) || u.lastOverflow() != ref.overflow) {
        ++fails; std::cout << "  FAIL determinant mismatch (" << tag << ") n=" << int(A.rows) << "\n";
    }
}

int main() {
    std::mt19937 rng(2024);
    auto pick = [&](unsigned lo, unsigned hi) { return std::uniform_int_distribution<unsigned>(lo, hi)(rng); };

    {
        int cases = 0;
        for (unsigned lanes : {1u, 2u, 3u, 4u, 5u, 8u, 16u, 20u, 32u}) {
            MatrixUnit::Config cfg;
            cfg.lanes = lanes;
            cfg.memWordsPerCycle = pick(1, 4);
            cfg.mulLatency = pick(1, 4);
            cfg.divLatency = pick(1, 20);
            cfg.dividers = pick(1, 3);
            std::vector<Word> mem(1 << 16, 0);
            MatrixUnit u(mem, cfg);
            for (int it = 0; it < 24; ++it) {
                const bool wild = it % 4 == 3;
                const unsigned r = pick(1, 7), k = pick(1, 7), c = pick(1, 7);
                checkAdd(u, mem, randomMatrix(rng, r, c, wild), randomMatrix(rng, r, c, wild), "random");
                checkMul(u, mem, randomMatrix(rng, r, k, wild), randomMatrix(rng, k, c, wild), "random");
                const unsigned n = pick(1, 8);
                checkDet(u, mem, randomMatrix(rng, n, n, wild), "random");
                cases += 3;
            }
        }
        std::cout << "random cases checked against ALU: " << cases << "\n";
    }

    {
        std::vector<Word> mem(1 << 16, 0);
        for (unsigned lanes : {4u, 7u, 20u}) {
            MatrixUnit::Config cfg; cfg.lanes = lanes;
            MatrixUnit u(mem, cfg);
            checkMul(u, mem, randomMatrix(rng, 20, 20, false), randomMatrix(rng, 20, 20, false), "20x20");
            checkAdd(u, mem, randomMatrix(rng, 20, 20, false), randomMatrix(rng, 20, 20, false), "20x20");
        }
        MatrixUnit::Config cfg; cfg.lanes = 4;
        MatrixUnit u(mem, cfg);
        checkDet(u, mem, randomMatrix(rng, 20, 20, false), "20x20");
        ALU::Matrix I(5, 5);
        for (int i = 0; i < 5; ++i) I.at(i, i) = ALU::FX_SCALE;
        checkDet(u, mem, I, "identity");
        ALU::Matrix S(4, 4);
        for (int i = 0; i < 16; ++i) S.data[i] = ALU::Matrix::fromInt(i + 1);
        checkDet(u, mem, S, "singular");
        ALU::Matrix Z(3, 3);
        checkDet(u, mem, Z, "zero");
    }

    {
        std::vector<Word> mem(1 << 16, 0);
        MatrixUnit u(mem);
        ALU::Matrix A = randomMatrix(rng, 3, 3, false), B = randomMatrix(rng, 3, 3, false);
        put(mem, AP, A); put(mem, BP, B);
        const auto ref = ALU::matAdd(A, B);
        CHECK(u.start(fields(Op::MATADD, 3, 0, 3), AP, AP, BP) == SR::Started);
        runToDone(u);
        bool same = true;
        for (int e = 0; e < 9; ++e) same = same && mem[(AP >> 3) + e] == ref.matrix.data[e];
        CHECK(same);
    }

    {
        std::vector<Word> mem(1 << 16, 0);
        MatrixUnit u(mem);
        CHECK(u.start(fields(Op::MATADD, 4, 0, 4), CP, AP, BP) == SR::Started);
        CHECK(u.start(fields(Op::MATADD, 4, 0, 4), CP, AP, BP) == SR::Busy);
        runToDone(u);
        CHECK(!u.busy());
        CHECK(u.start(fields(Op::MATADD, 0, 0, 4), CP, AP, BP) == SR::InvalidDimensions);
        CHECK(u.start(fields(Op::MATADD, 21, 0, 4), CP, AP, BP) == SR::InvalidDimensions);
        CHECK(u.start(fields(Op::MATMUL, 2, 21, 2), CP, AP, BP) == SR::InvalidDimensions);
        CHECK(u.start(fields(Op::MATDETERMINANT, 3, 0, 4), CP, AP, 0) == SR::InvalidDimensions);
        CHECK(u.start(fields(Op::ADD, 2, 0, 2), CP, AP, BP) == SR::InvalidDimensions);
        CHECK(u.start(fields(Op::MATADD, 2, 0, 2), CP, AP + 4, BP) == SR::MemoryFault);
        CHECK(u.start(fields(Op::MATADD, 2, 0, 2), CP + 1, AP, BP) == SR::MemoryFault);
        CHECK(u.start(fields(Op::MATADD, 2, 0, 2), CP, (1 << 19) - 16, BP) == SR::MemoryFault);
        CHECK(u.start(fields(Op::MATADD, 2, 0, 2), (1 << 19), AP, BP) == SR::MemoryFault);
        CHECK(!u.busy());
    }

    {
        std::vector<Word> mem(1 << 16, 0);
        MatrixUnit u(mem);
        ALU::Matrix A = randomMatrix(rng, 2, 2, false), B = randomMatrix(rng, 2, 2, false);
        put(mem, AP, A); put(mem, BP, B); fillSentinel(mem, CP, 5);
        CHECK(u.start(fields(Op::MATADD, 2, 0, 2), CP, AP, BP) == SR::Started);
        const uint64_t total = u.remainingCycles();
        for (uint64_t c = 1; c < total; ++c) { CHECK(!u.step()); CHECK(mem[CP >> 3] == SENTINEL); }
        CHECK(u.step());
        CHECK(mem[CP >> 3] != SENTINEL);
    }

    {
        std::vector<Word> mem(1 << 16, 0);
        MatrixUnit u(mem);
        ALU::Matrix A = randomMatrix(rng, 4, 4, false), B = randomMatrix(rng, 4, 4, false);
        put(mem, AP, A); put(mem, BP, B);
        u.start(fields(Op::MATADD, 4, 0, 4), CP, AP, BP);
        CHECK(runToDone(u) == 16 + 16 + 4 + 16);
        CHECK(u.stats().loadCycles == 32 && u.stats().computeCycles == 4 && u.stats().storeCycles == 16);
        CHECK(u.stats().busyCycles == 52 && u.stats().utilization() == 1.0);
    }
    {
        std::vector<Word> mem(1 << 16, 0);
        MatrixUnit u(mem);
        ALU::Matrix A = randomMatrix(rng, 2, 3, false), B = randomMatrix(rng, 3, 4, false);
        put(mem, AP, A); put(mem, BP, B);
        u.start(fields(Op::MATMUL, 2, 3, 4), CP, AP, BP);
        CHECK(runToDone(u) == 18 + (2 * 1 * 3 + 3) + 8);
        CHECK(u.stats().vectorSteps == 6 && u.stats().utilization() == 1.0);
    }
    {
        std::vector<Word> mem(1 << 16, 0);
        MatrixUnit u(mem);
        ALU::Matrix A = randomMatrix(rng, 2, 3, false), B = randomMatrix(rng, 3, 5, false);
        put(mem, AP, A); put(mem, BP, B);
        u.start(fields(Op::MATMUL, 2, 3, 5), CP, AP, BP);
        CHECK(runToDone(u) == 21 + (2 * 2 * 3 + 3) + 10);
        CHECK(u.stats().vectorSteps == 12 && u.stats().utilization() == 30.0 / 48.0);
    }
    {
        std::vector<Word> mem(1 << 16, 0);
        MatrixUnit::Config cfg; cfg.lanes = 1;
        MatrixUnit u(mem, cfg);
        ALU::Matrix A = randomMatrix(rng, 2, 3, false), B = randomMatrix(rng, 3, 4, false);
        put(mem, AP, A); put(mem, BP, B);
        u.start(fields(Op::MATMUL, 2, 3, 4), CP, AP, BP);
        CHECK(runToDone(u) == 18 + (2 * 4 * 3 + 3) + 8);
    }
    {
        std::vector<Word> mem(1 << 16, 0);
        MatrixUnit u(mem);
        ALU::Matrix I(3, 3);
        for (int i = 0; i < 3; ++i) I.at(i, i) = ALU::FX_SCALE;
        put(mem, AP, I);
        u.start(fields(Op::MATDETERMINANT, 3, 0, 3), CP, AP, 0);
        CHECK(runToDone(u) == 9 + 60 + 1);
        CHECK(mem[CP >> 3] == ALU::FX_SCALE);
    }

    std::cout << (fails ? "FAILURES: " + std::to_string(fails) : std::string("all matrix unit checks passed")) << "\n";
    return fails != 0;
}
