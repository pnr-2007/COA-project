#pragma once

#include "isa.cpp"
#include <vector>
#include <algorithm>
#include <cmath>

class ALU {
public:
    using Word = ISA::Word;

    static constexpr int WIDTH = 64;
    static constexpr int BLOCK_SIZE = 4;
    static constexpr int NUM_BLOCKS = WIDTH / BLOCK_SIZE;
    static constexpr int GROUPS = NUM_BLOCKS / BLOCK_SIZE;

    static constexpr int FX_FRAC_BITS = 32;
    static constexpr uint64_t FX_SCALE = 1ULL << FX_FRAC_BITS;
    static constexpr uint64_t FX_FRAC_MASK = FX_SCALE - 1;

    static constexpr int LUT_INDEX_BITS = 8;
    static constexpr uint32_t POLY_SEGMENTS = 1 << LUT_INDEX_BITS;
    static constexpr uint32_t POLY_LUT_SIZE = POLY_SEGMENTS + 1;

    struct Result {
        Word value = 0;
        bool carryOut = false;
        bool overflow = false;
        bool zero = false;
        bool negative = false;
        bool divideByZero = false;
    };

    struct MultiplyResult {
        Word low = 0;
        Word high = 0;
        bool overflow = false;
        bool zero = false;
        bool negative = false;
    };

    struct DivideResult {
        Word quotient = 0;
        Word remainder = 0;
        bool divideByZero = false;
        bool overflow = false;
    };

    static double targetPolynomialHost(double x) {
        return 2.0 * (x * x * x) - 3.0 * (x * x) + 5.0;
    }

    static void initTranscendentalLUTs() {
        polynomialLUT.resize(POLY_LUT_SIZE);
        sineLUT.resize(POLY_LUT_SIZE);
        cosineLUT.resize(POLY_LUT_SIZE);
        log2LUT.resize(POLY_LUT_SIZE);

        for (uint32_t i = 0; i < POLY_LUT_SIZE; ++i) {
            double normalized_x = static_cast<double>(i) / POLY_SEGMENTS;
            
            polynomialLUT[i] = static_cast<Word>(targetPolynomialHost(normalized_x) * FX_SCALE);
            
            double rad_x = normalized_x * (M_PI / 2.0);
            sineLUT[i] = static_cast<Word>(std::sin(rad_x) * FX_SCALE);
            cosineLUT[i] = static_cast<Word>(std::cos(rad_x) * FX_SCALE);
            
            double log_x = 1.0 + normalized_x;
            log2LUT[i] = static_cast<Word>(std::log2(log_x) * FX_SCALE);
        }
    }

    static Result evaluatePolynomial(Word x_fixed) {
        if (x_fixed >= FX_SCALE) {
            Result r;
            r.value = polynomialLUT[POLY_SEGMENTS];
            updateFlags(r);
            return r;
        }
        return interpolateFromLUT(x_fixed, polynomialLUT);
    }

    static Result evaluateSine(Word x_fixed) {
        constexpr uint64_t TWO_PI_FX = static_cast<uint64_t>(2.0 * M_PI * FX_SCALE);
        constexpr uint64_t PI_FX = static_cast<uint64_t>(M_PI * FX_SCALE);
        constexpr uint64_t HALF_PI_FX = static_cast<uint64_t>((M_PI / 2.0) * FX_SCALE);

        uint64_t scaled_x = x_fixed % TWO_PI_FX;
        bool is_negative = false;

        if (scaled_x >= PI_FX) {
            scaled_x -= PI_FX;
            is_negative = true;
        }

        if (scaled_x > HALF_PI_FX) {
            scaled_x = PI_FX - scaled_x;
        }

        uint64_t lut_input = (scaled_x * FX_SCALE) / HALF_PI_FX;
        Result r = interpolateFromLUT(lut_input, sineLUT);

        if (is_negative) {
            r = sub(0, r.value);
        }
        return r;
    }

    static Result evaluateCosine(Word x_fixed) {
        constexpr uint64_t TWO_PI_FX = static_cast<uint64_t>(2.0 * M_PI * FX_SCALE);
        constexpr uint64_t PI_FX = static_cast<uint64_t>(M_PI * FX_SCALE);
        constexpr uint64_t HALF_PI_FX = static_cast<uint64_t>((M_PI / 2.0) * FX_SCALE);

        uint64_t scaled_x = x_fixed % TWO_PI_FX;
        bool is_negative = false;

        if (scaled_x >= PI_FX) {
            scaled_x = TWO_PI_FX - scaled_x;
        }

        if (scaled_x > HALF_PI_FX) {
            scaled_x = PI_FX - scaled_x;
            is_negative = true;
        }

        uint64_t lut_input = (scaled_x * FX_SCALE) / HALF_PI_FX;
        Result r = interpolateFromLUT(lut_input, cosineLUT);

        if (is_negative) {
            r = sub(0, r.value);
        }
        return r;
    }

    static Result evaluateLog2(Word x_fixed) {
        if (x_fixed == 0) {
            Result r;
            r.divideByZero = true;
            r.overflow = true;
            return r;
        }

        int integer_part = 0;
        uint64_t significand = x_fixed;

        if (significand >= (2ULL << FX_FRAC_BITS)) {
            while (significand >= (2ULL << FX_FRAC_BITS)) {
                significand >>= 1;
                integer_part++;
            }
        } else if (significand < (1ULL << FX_FRAC_BITS)) {
            while (significand < (1ULL << FX_FRAC_BITS)) {
                significand <<= 1;
                integer_part--;
            }
        }

        uint64_t lut_input = significand - (1ULL << FX_FRAC_BITS);
        Result fractional_log = interpolateFromLUT(lut_input, log2LUT);

        int64_t integer_part_fx = static_cast<int64_t>(integer_part) << FX_FRAC_BITS;
        Result r = add(integer_part_fx, fractional_log.value);
        return r;
    }

    static Result add(Word a, Word b) { return claadd(a, b, 0); }
    static Result sub(Word a, Word b) { return claadd(a, ~b, 1); }
    static Result addWithCarry(Word a, Word b, bool cin) { return claadd(a, b, cin ? 1 : 0); }

    static Result bitwiseAnd(Word a, Word b) { Result r; r.value = a & b; updateFlags(r); return r; }
    static Result bitwiseOr(Word a, Word b)  { Result r; r.value = a | b; updateFlags(r); return r; }
    static Result bitwiseXor(Word a, Word b) { Result r; r.value = a ^ b; updateFlags(r); return r; }
    static Result bitwiseNot(Word a)         { Result r; r.value = ~a;    updateFlags(r); return r; }

    static Result lsl(Word a, Word shiftAmt) {
        Result r;
        r.value = a << (shiftAmt & (WIDTH - 1));
        updateFlags(r);
        return r;
    }

    static Result lsr(Word a, Word shiftAmt) {
        Result r;
        r.value = static_cast<uint64_t>(a) >> (shiftAmt & (WIDTH - 1));
        updateFlags(r);
        return r;
    }

    static Result asr(Word a, Word shiftAmt) {
        Result r;
        r.value = static_cast<Word>(static_cast<int64_t>(a) >> (shiftAmt & (WIDTH - 1)));
        updateFlags(r);
        return r;
    }

    static MultiplyResult multiply(Word multiplicand, Word multiplier) {
        Word A = 0, Q = multiplier;
        bool q_minus_1 = false;

        for (int i = 0; i < WIDTH; ++i) {
            bool q0 = Q & 1;
            if (q0 ^ q_minus_1) {
                A = (q0 ? sub(A, multiplicand) : add(A, multiplicand)).value;
            }
            q_minus_1 = q0;
            Q = (Q >> 1) | (A << (WIDTH - 1));
            A = (A >> 1) | ((A >> (WIDTH - 1)) << (WIDTH - 1));
        }

        MultiplyResult res{Q, A};
        res.zero = !(A | Q);
        res.negative = (A >> (WIDTH - 1)) & 1;
        res.overflow = static_cast<int64_t>(A) != (static_cast<int64_t>(Q) >> (WIDTH - 1));
        return res;
    }

    static DivideResult divide(Word dividend, Word divisor) {
        if (!divisor)  return {0, 0, true, false};
        if (!dividend) return {};

        int shift_D = __builtin_clzll(static_cast<uint64_t>(divisor));
        unsigned __int128 D = static_cast<unsigned __int128>(divisor) << shift_D;
        unsigned __int128 x = (((unsigned __int128)48 << 62) - 8 * D) / 17;

        for (int i = 0; i < 5; ++i) {
            unsigned __int128 term = (D * x) >> 64;
            unsigned __int128 two_minus_term = ((unsigned __int128)1 << 63) - term;
            x = (x * two_minus_term) >> 62;
        }

        Word q = static_cast<Word>(((unsigned __int128)dividend * x) >> (126 - shift_D));
        Word r = dividend - q * divisor;

        if (r >= divisor)               { q++; r -= divisor; }
        else if (q * divisor > dividend) { q--; r += divisor; }

        return {q, r, false, false};
    }

    static Result modulus(Word dividend, Word divisor) {
        DivideResult divRes = divide(dividend, divisor);
        Result r;
        r.value = divRes.remainder;
        r.zero = !r.value;
        r.negative = (r.value >> (WIDTH - 1)) & 1;
        r.overflow = divRes.divideByZero;
        return r;
    }

private:
    static inline std::vector<Word> polynomialLUT;
    static inline std::vector<Word> sineLUT;
    static inline std::vector<Word> cosineLUT;
    static inline std::vector<Word> log2LUT;

    static void updateFlags(Result& r) {
        r.zero = !r.value;
        r.negative = (r.value >> (WIDTH - 1)) & 1;
    }

    static Result interpolateFromLUT(Word x_fixed, const std::vector<Word>& lut) {
        if (x_fixed >= FX_SCALE) {
            Result r;
            r.value = lut[POLY_SEGMENTS];
            updateFlags(r);
            return r;
        }

        uint32_t idx = static_cast<uint32_t>(x_fixed >> (FX_FRAC_BITS - LUT_INDEX_BITS));
        Word frac_bits = x_fixed & ((1ULL << (FX_FRAC_BITS - LUT_INDEX_BITS)) - 1);
        Word frac = frac_bits << LUT_INDEX_BITS;

        Word y0 = lut[idx];
        Word y1 = lut[idx + 1];

        Word delta_y = sub(y1, y0).value;

        unsigned __int128 product = static_cast<unsigned __int128>(delta_y) * frac;
        Word offset = static_cast<Word>(product >> 32);

        return add(y0, offset);
    }

    static void groupGP(const bool* g, const bool* p, bool& G, bool& P) {
        G = g | (p & g) | (p & p & g) | (p & p & p & g);
        P = p & p & p & p;
    }

    static void carries4(const bool* g, const bool* p, bool cin, bool* c) {
        c = cin;
        c = g | (p & cin);
        c = g | (p & g) | (p & p & cin);
        c = g | (p & g) | (p & p & g) | (p & p & p & cin);
    }

    static Result claadd(Word a, Word b, int cin) {
        static_assert(sizeof(Word) * 8 == WIDTH, "Word size must equal 64 bits");

        bool g[WIDTH], p[WIDTH], g1[NUM_BLOCKS], p1[NUM_BLOCKS], g2[GROUPS], p2[GROUPS];
        bool C2[GROUPS], C1[NUM_BLOCKS], C0[WIDTH], G3, P3;

        for (int i = 0; i < WIDTH; ++i) {
            g[i] = (a >> i) & (b >> i) & 1;
            p[i] = ((a ^ b) >> i) & 1;
        }

        for (int i = 0; i < NUM_BLOCKS; ++i) groupGP(&g[i * BLOCK_SIZE], &p[i * BLOCK_SIZE], g1[i], p1[i]);
        for (int i = 0; i < GROUPS; ++i) groupGP(&g1[i * BLOCK_SIZE], &p1[i * BLOCK_SIZE], g2[i], p2[i]);
        groupGP(g2, p2, G3, P3);

        carries4(g2, p2, cin, C2);
        for (int j = 0; j < GROUPS; ++j) carries4(&g1[j * BLOCK_SIZE], &p1[j * BLOCK_SIZE], C2[j], &C1[j * BLOCK_SIZE]);
        for (int k = 0; k < NUM_BLOCKS; ++k) carries4(&g[k * BLOCK_SIZE], &p[k * BLOCK_SIZE], C1[k], &C0[k * BLOCK_SIZE]);

        Result r;
        for (int i = 0; i < WIDTH; ++i) r.value |= (Word(p[i] ^ C0[i]) << i);

        r.carryOut = G3 | (P3 & cin);
        r.overflow = C0[WIDTH - 1] != r.carryOut;
        r.zero = !r.value;
        r.negative = (r.value >> (WIDTH - 1)) & 1;
        return r;
    }
};

         

     
