#include "fpu.cpp"
#include <cfenv>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>

static uint64_t bits(double d) { uint64_t u; std::memcpy(&u, &d, 8); return u; }
static double dbl(uint64_t u) { double d; std::memcpy(&d, &u, 8); return d; }

static std::mt19937_64 rng(12345);

static uint64_t genOne() {
    const unsigned cat = rng() % 10;
    const uint64_t sign = (rng() & 1) << 63;
    static const uint64_t exps[] = {0, 1, 2, 3, 0x3FD, 0x3FE, 0x3FF, 0x400, 0x401, 0x432, 0x433, 0x434, 0x7FD, 0x7FE, 0x7FF, 0x3CA, 0x3CB, 0x001, 0x400 + 30};
    static const uint64_t fracs[] = {0, 1, 2, (1ull << 52) - 1, (1ull << 51), (1ull << 51) - 1, (1ull << 51) + 1, 0x5555555555555ull, 0xAAAAAAAAAAAAAull};
    switch (cat) {
        case 0: return rng();
        case 1: return sign | (exps[rng() % 19] << 52) | (rng() & ((1ull << 52) - 1));
        case 2: return sign | (exps[rng() % 19] << 52) | fracs[rng() % 9];
        case 3: return sign | ((rng() % 2047) << 52) | fracs[rng() % 9];
        case 4: return sign | ((rng() % 40) << 52) | (rng() & ((1ull << 52) - 1));
        case 5: return sign | ((0x3E0 + rng() % 64) << 52) | (rng() & ((1ull << 52) - 1));
        case 6: return sign | ((0x7C0 + rng() % 64) << 52) | (rng() & ((1ull << 52) - 1));
        case 7: return sign | (rng() & ((1ull << 52) - 1));
        default: return sign | ((rng() % 2046 + 1) << 52) | (rng() & ((1ull << 52) - 1));
    }
}

static void genPair(uint64_t& a, uint64_t& b) {
    a = genOne();
    const unsigned m = rng() % 6;
    if (m == 0) b = a ^ (rng() % 8);
    else if (m == 1) b = (a ^ (1ull << 63)) + (rng() % 5) - 2;
    else if (m == 2) b = a + (rng() % 64) - 32;
    else if (m == 3) b = (a & ~((1ull << 52) - 1)) + ((rng() % 3) << 52) + (rng() & 0xFF);
    else b = genOne();
}

static int hostRm(unsigned rm) {
    switch (rm) { case 0: return FE_TONEAREST; case 1: return FE_TOWARDZERO; case 2: return FE_DOWNWARD; default: return FE_UPWARD; }
}

static uint8_t hostFlags() {
    uint8_t f = 0;
    if (fetestexcept(FE_INVALID)) f |= FPU::NV;
    if (fetestexcept(FE_DIVBYZERO)) f |= FPU::DZ;
    if (fetestexcept(FE_OVERFLOW)) f |= FPU::OF;
    if (fetestexcept(FE_UNDERFLOW)) f |= FPU::UF;
    if (fetestexcept(FE_INEXACT)) f |= FPU::NX;
    return f;
}

static uint64_t failures = 0;
static uint64_t checked = 0;

static void report(const char* name, unsigned rm, uint64_t a, uint64_t b, uint64_t got, uint8_t gf, uint64_t exp, uint8_t ef) {
    if (++failures <= 30)
        std::printf("FAIL %s rm=%u a=%016llx b=%016llx got=%016llx/%02x exp=%016llx/%02x\n", name, rm,
                    (unsigned long long)a, (unsigned long long)b, (unsigned long long)got, gf, (unsigned long long)exp, ef);
}

static void compare(const char* name, unsigned rm, uint64_t a, uint64_t b, uint64_t got, uint8_t gf, uint64_t exp, uint8_t ef) {
    ++checked;
    const bool nanBoth = FPU::isNaN(got) && FPU::isNaN(exp);
    bool valOk = nanBoth ? got == FPU::QNAN : got == exp;
    uint8_t g = gf, e = ef;
    const uint64_t mag = got & ~(1ull << 63);
    if (mag == 0x0010000000000000ull && (e & FPU::UF) && !(g & FPU::UF)) e &= ~FPU::UF;
    if (!valOk || g != e) report(name, rm, a, b, got, gf, exp, ef);
}

static void testBinary(FPU::Op op, const char* name, long iters) {
    for (long i = 0; i < iters; ++i) {
        uint64_t a, b;
        genPair(a, b);
        const unsigned rm = rng() % 4;
        volatile double va = dbl(a), vb = dbl(b), vr;
        std::fesetround(hostRm(rm));
        std::feclearexcept(FE_ALL_EXCEPT);
        switch (op) {
            case FPU::Op::ADD: vr = va + vb; break;
            case FPU::Op::SUB: vr = va - vb; break;
            case FPU::Op::MUL: vr = va * vb; break;
            default:           vr = va / vb; break;
        }
        const uint8_t hf = hostFlags();
        const uint64_t hr = bits(vr);
        std::fesetround(FE_TONEAREST);
        uint8_t f = 0;
        const uint64_t r = FPU::execute(op, a, b, rm, f);
        compare(name, rm, a, b, r, f, hr, hf);
    }
}

static void testSqrt(long iters) {
    for (long i = 0; i < iters; ++i) {
        uint64_t a = genOne();
        if (rng() % 3) a &= ~(1ull << 63);
        const unsigned rm = rng() % 4;
        volatile double va = dbl(a), vr;
        std::fesetround(hostRm(rm));
        std::feclearexcept(FE_ALL_EXCEPT);
        vr = std::sqrt(va);
        const uint8_t hf = hostFlags();
        const uint64_t hr = bits(vr);
        std::fesetround(FE_TONEAREST);
        uint8_t f = 0;
        const uint64_t r = FPU::execute(FPU::Op::SQRT, a, 0, rm, f);
        compare("sqrt", rm, a, 0, r, f, hr, hf);
    }
}

static void testCmp(long iters) {
    for (long i = 0; i < iters; ++i) {
        uint64_t a, b;
        genPair(a, b);
        const double x = dbl(a), y = dbl(b);
        uint8_t f = 0;
        const uint64_t r = FPU::execute(FPU::Op::CMP, a, b, 0, f);
        uint64_t exp;
        uint8_t ef = 0;
        if (std::isnan(x) || std::isnan(y)) {
            exp = 2;
            if (FPU::isSNaN(a) || FPU::isSNaN(b)) ef = FPU::NV;
        } else exp = x < y ? uint64_t(-1) : (x > y ? 1 : 0);
        ++checked;
        if (r != exp || f != ef) report("cmp", 0, a, b, r, f, exp, ef);
    }
}

static void testCvtDQ(long iters) {
    for (long i = 0; i < iters; ++i) {
        uint64_t a = rng();
        const unsigned sh = rng() % 64;
        if (rng() % 2) a >>= sh;
        if (rng() % 4 == 0) a = uint64_t(-int64_t(a));
        const unsigned rm = rng() % 4;
        volatile long long vi = (long long)a;
        volatile double vd;
        std::fesetround(hostRm(rm));
        std::feclearexcept(FE_ALL_EXCEPT);
        vd = (double)vi;
        const uint8_t hf = hostFlags();
        std::fesetround(FE_TONEAREST);
        const uint64_t hr = bits(std::ldexp(vd, -32));
        uint8_t f = 0;
        const uint64_t r = FPU::execute(FPU::Op::CVTDQ, a, 0, rm, f);
        compare("cvtdq", rm, a, 0, r, f, hr, hf & FPU::NX);
    }
}

static void testCvtQD(long iters) {
    for (long i = 0; i < iters; ++i) {
        uint64_t a = genOne();
        if (rng() % 2) a = (a & ~(0x7FFull << 52)) | ((0x3C0ull + rng() % 0x60) << 52);
        const unsigned rm = rng() % 4;
        const double x = dbl(a);
        uint8_t f = 0;
        const uint64_t r = FPU::execute(FPU::Op::CVTQD, a, 0, rm, f);
        uint64_t exp;
        uint8_t ef = 0;
        const double scaled = std::ldexp(x, 32);
        if (std::isnan(x)) { exp = 0x7FFFFFFFFFFFFFFFull; ef = FPU::NV; }
        else if (!(scaled < 9223372036854775808.0 && scaled >= -9223372036854775808.0)) {
            exp = std::signbit(x) ? 0x8000000000000000ull : 0x7FFFFFFFFFFFFFFFull;
            ef = FPU::NV;
        } else {
            volatile double vs = scaled;
            volatile long long vl;
            std::fesetround(hostRm(rm));
            std::feclearexcept(FE_ALL_EXCEPT);
            vl = std::llrint(vs);
            ef = hostFlags() & FPU::NX;
            std::fesetround(FE_TONEAREST);
            exp = (uint64_t)vl;
        }
        ++checked;
        if (r != exp || f != ef) report("cvtqd", rm, a, 0, r, f, exp, ef);
    }
}

static void testRmm(long iters) {
    static const FPU::Op ops[] = {FPU::Op::ADD, FPU::Op::SUB, FPU::Op::MUL, FPU::Op::DIV, FPU::Op::SQRT};
    for (long i = 0; i < iters; ++i) {
        uint64_t a, b;
        genPair(a, b);
        const FPU::Op op = ops[rng() % 5];
        uint8_t f0 = 0, f4 = 0, f1 = 0, fa = 0;
        const uint64_t rne = FPU::execute(op, a, b, FPU::RNE, f0);
        const uint64_t rmm = FPU::execute(op, a, b, FPU::RMM, f4);
        const uint64_t rtz = FPU::execute(op, a, b, FPU::RTZ, f1);
        const bool neg = (rmm >> 63) != 0 && !FPU::isNaN(rmm);
        const uint64_t away = FPU::execute(op, a, b, neg ? FPU::RDN : FPU::RUP, fa);
        ++checked;
        bool ok = (f0 == f4);
        if (FPU::isNaN(rne)) ok = ok && rmm == rne;
        else ok = ok && (rmm == rne || (rmm == away && rne == rtz));
        if (!ok) report("rmm", 4, a, b, rmm, f4, rne, f0);
    }
    uint8_t f = 0;
    const uint64_t one = bits(1.0), tiny = bits(std::ldexp(1.0, -53));
    ++checked;
    if (FPU::execute(FPU::Op::ADD, one, tiny, FPU::RMM, f) != bits(1.0 + std::ldexp(1.0, -52))) report("rmm-tie-up", 4, one, tiny, 0, f, 0, 0);
    f = 0;
    ++checked;
    if (FPU::execute(FPU::Op::ADD, one, tiny, FPU::RNE, f) != one) report("rne-tie-even", 0, one, tiny, 0, f, 0, 0);
    f = 0;
    ++checked;
    if (FPU::execute(FPU::Op::ADD, bits(-1.0), bits(-std::ldexp(1.0, -53)), FPU::RMM, f) != bits(-1.0 - std::ldexp(1.0, -52))) report("rmm-tie-neg", 4, 0, 0, 0, f, 0, 0);
}

static void testDirected() {
    uint8_t f;
    auto chk = [&](bool c, const char* n) { ++checked; if (!c) { ++failures; std::printf("FAIL directed %s\n", n); } };
    f = 0; chk(FPU::execute(FPU::Op::DIV, bits(1.0), bits(0.0), 0, f) == bits(INFINITY) && f == FPU::DZ, "1/0");
    f = 0; chk(FPU::execute(FPU::Op::DIV, bits(0.0), bits(0.0), 0, f) == FPU::QNAN && f == FPU::NV, "0/0");
    f = 0; chk(FPU::execute(FPU::Op::SQRT, bits(-4.0), 0, 0, f) == FPU::QNAN && f == FPU::NV, "sqrt(-4)");
    f = 0; chk(FPU::execute(FPU::Op::SQRT, bits(-0.0), 0, 0, f) == bits(-0.0) && f == 0, "sqrt(-0)");
    f = 0; chk(FPU::execute(FPU::Op::SQRT, bits(4.0), 0, 0, f) == bits(2.0) && f == 0, "sqrt(4)");
    f = 0; chk(FPU::execute(FPU::Op::ADD, bits(1.0), bits(-1.0), FPU::RDN, f) == bits(-0.0), "1-1 RDN");
    f = 0; chk(FPU::execute(FPU::Op::ADD, bits(1.0), bits(-1.0), FPU::RNE, f) == bits(0.0), "1-1 RNE");
    f = 0; chk(FPU::execute(FPU::Op::MUL, bits(1e300), bits(1e300), FPU::RTZ, f) == 0x7FEFFFFFFFFFFFFFull && f == (FPU::OF | FPU::NX), "ovf RTZ");
    f = 0; chk(FPU::execute(FPU::Op::MUL, bits(1e-300), bits(1e-300), FPU::RNE, f) == 0 && f == (FPU::UF | FPU::NX), "unf");
    f = 0; chk(FPU::execute(FPU::Op::CVTQD, bits(1.5), 0, 0, f) == (3ull << 31) && f == 0, "1.5->q");
    f = 0; chk(FPU::execute(FPU::Op::CVTQD, bits(1e30), 0, 0, f) == 0x7FFFFFFFFFFFFFFFull && f == FPU::NV, "sat");
    f = 0; chk(FPU::execute(FPU::Op::CVTDQ, 3ull << 31, 0, 0, f) == bits(1.5) && f == 0, "q->1.5");
    f = 0; chk(FPU::execute(FPU::Op::CMP, bits(0.0), bits(-0.0), 0, f) == 0, "cmp zeros");
    f = 0; chk(FPU::execute(FPU::Op::CMP, 0x7FF0000000000001ull, bits(1.0), 0, f) == 2 && f == FPU::NV, "cmp snan");
    f = 0; chk(FPU::execute(FPU::Op::CMP, FPU::QNAN, bits(1.0), 0, f) == 2 && f == 0, "cmp qnan");
}

int main() {
    const long N = 400000;
    testDirected();
    testBinary(FPU::Op::ADD, "add", N);
    testBinary(FPU::Op::SUB, "sub", N);
    testBinary(FPU::Op::MUL, "mul", N);
    testBinary(FPU::Op::DIV, "div", N);
    testSqrt(N);
    testCmp(N / 4);
    testCvtDQ(N / 2);
    testCvtQD(N / 2);
    testRmm(N);
    std::printf("checked %llu failures %llu\n", (unsigned long long)checked, (unsigned long long)failures);
    return failures ? 1 : 0;
}
