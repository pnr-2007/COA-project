#pragma once

#include "isa.cpp"
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>

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

    static constexpr int MAT_MAX_ROWS  = ISA::MAX_MATRIX_ROWS;
    static constexpr int MAT_MAX_COLS  = ISA::MAX_MATRIX_COLS;
    static constexpr int MAT_MAX_ELEMS = MAT_MAX_ROWS * MAT_MAX_COLS;
    static constexpr uint64_t MAT_SINGULAR_EPS = FX_SCALE >> 24;

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

    struct Matrix {
        uint8_t rows = 0;
        uint8_t cols = 0;
        std::array<Word, MAT_MAX_ELEMS> data{};

        Matrix() = default;
        Matrix(uint8_t r, uint8_t c) : rows(r), cols(c) {}

        Word&       at(int r, int c)       { return data[r * cols + c]; }
        const Word& at(int r, int c) const { return data[r * cols + c]; }

        bool isSquare() const { return rows == cols; }

        static Word fromInt(int64_t v)   { return static_cast<Word>(v * static_cast<int64_t>(FX_SCALE)); }
        static Word fromDouble(double v) { return static_cast<Word>(static_cast<int64_t>(std::llround(v * static_cast<double>(FX_SCALE)))); }
        static double toDouble(Word w)   { return static_cast<double>(static_cast<int64_t>(w)) / static_cast<double>(FX_SCALE); }
    };

    struct MatrixResult {
        Matrix matrix;
        Word   scalar = 0;
        bool   hasScalar = false;

        bool   overflow = false;
        bool   dimensionMismatch = false;
        bool   invalidDimensions = false;
        bool   singular = false;
        bool   unsupported = false;

        bool   zero = false;
        bool   negative = false;

        bool ok() const {
            return !overflow && !dimensionMismatch && !invalidDimensions && !singular && !unsupported;
        }
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
        ensureLUTs();
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

        uint64_t lut_input = static_cast<uint64_t>((static_cast<unsigned __int128>(scaled_x) << FX_FRAC_BITS) / HALF_PI_FX);
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

        uint64_t lut_input = static_cast<uint64_t>((static_cast<unsigned __int128>(scaled_x) << FX_FRAC_BITS) / HALF_PI_FX);
        Result r = interpolateFromLUT(lut_input, cosineLUT);

        if (is_negative) {
            r = sub(0, r.value);
        }
        return r;
    }

    static Result evaluateSineSigned(Word x_fixed) {
        if (static_cast<int64_t>(x_fixed) < 0) {
            Result r = evaluateSine(Word(0) - x_fixed);
            return sub(0, r.value);
        }
        return evaluateSine(x_fixed);
    }

    static Result evaluateCosineSigned(Word x_fixed) {
        if (static_cast<int64_t>(x_fixed) < 0) {
            return evaluateCosine(Word(0) - x_fixed);
        }
        return evaluateCosine(x_fixed);
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

        Word integer_part_fx = static_cast<Word>(static_cast<int64_t>(integer_part) * static_cast<int64_t>(FX_SCALE));
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
        bool a_sign = false;
        const bool m_sign = (multiplicand >> (WIDTH - 1)) & 1;

        for (int i = 0; i < WIDTH; ++i) {
            bool q0 = Q & 1;
            if (q0 ^ q_minus_1) {
                Result r = q0 ? sub(A, multiplicand) : add(A, multiplicand);
                a_sign = a_sign ^ (q0 ? !m_sign : m_sign) ^ r.carryOut;
                A = r.value;
            }
            q_minus_1 = q0;
            Q = (Q >> 1) | (A << (WIDTH - 1));
            A = (A >> 1) | (static_cast<Word>(a_sign) << (WIDTH - 1));
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
        unsigned __int128 prod = static_cast<unsigned __int128>(q) * divisor;

        while (prod > dividend)  { q--; prod -= divisor; }
        while (dividend - prod >= divisor) { q++; prod += divisor; }

        Word r = static_cast<Word>(dividend - prod);

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


    static MatrixResult matAdd(const Matrix& A, const Matrix& B) {
        MatrixResult out;
        if (!validDims(A) || !validDims(B)) { out.invalidDimensions = true; return out; }
        if (A.rows != B.rows || A.cols != B.cols) { out.dimensionMismatch = true; return out; }

        out.matrix = Matrix(A.rows, A.cols);
        const int n = A.rows * A.cols;
        for (int i = 0; i < n; ++i) {
            out.matrix.data[i] = fxAdd(A.data[i], B.data[i], out.overflow);
        }
        return out;
    }

    static MatrixResult matMul(const Matrix& A, const Matrix& B) {
        MatrixResult out;
        if (!validDims(A) || !validDims(B)) { out.invalidDimensions = true; return out; }
        if (A.cols != B.rows) { out.dimensionMismatch = true; return out; }

        out.matrix = Matrix(A.rows, B.cols);
        for (int i = 0; i < A.rows; ++i) {
            for (int j = 0; j < B.cols; ++j) {
                Word acc = 0;
                for (int k = 0; k < A.cols; ++k) {
                    Word p = fxMul(A.at(i, k), B.at(k, j), out.overflow);
                    acc = fxAdd(acc, p, out.overflow);
                }
                out.matrix.at(i, j) = acc;
            }
        }
        return out;
    }

    static MatrixResult matTranspose(const Matrix& A) {
        MatrixResult out;
        if (!validDims(A)) { out.invalidDimensions = true; return out; }

        out.matrix = Matrix(A.cols, A.rows);
        for (int i = 0; i < A.rows; ++i)
            for (int j = 0; j < A.cols; ++j)
                out.matrix.at(j, i) = A.at(i, j);
        return out;
    }

    static MatrixResult matDeterminant(const Matrix& A) {
        MatrixResult out;
        if (!validDims(A)) { out.invalidDimensions = true; return out; }
        if (!A.isSquare()) { out.dimensionMismatch = true; return out; }

        out.hasScalar = true;
        out.scalar = determinantImpl(A.data, A.rows, out.overflow);
        out.zero = !out.scalar;
        out.negative = isNeg(out.scalar);
        return out;
    }

    static MatrixResult matInverse(const Matrix& A) {
        MatrixResult out;
        if (!validDims(A)) { out.invalidDimensions = true; return out; }
        if (!A.isSquare()) { out.dimensionMismatch = true; return out; }

        const int n = A.rows;
        const int w = 2 * n;
        std::array<Word, MAT_MAX_ROWS * 2 * MAT_MAX_COLS> aug{};

        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) aug[i * w + j] = A.at(i, j);
            aug[i * w + n + i] = FX_SCALE;
        }

        for (int k = 0; k < n; ++k) {
            int piv = k;
            Word best = absW(aug[k * w + k]);
            for (int i = k + 1; i < n; ++i) {
                Word v = absW(aug[i * w + k]);
                if (v > best) { best = v; piv = i; }
            }
            if (best < MAT_SINGULAR_EPS) { out.singular = true; return out; }
            if (piv != k)
                for (int j = 0; j < w; ++j) std::swap(aug[k * w + j], aug[piv * w + j]);

            Word pivot = aug[k * w + k];
            for (int j = 0; j < w; ++j)
                aug[k * w + j] = fxDiv(aug[k * w + j], pivot, out.overflow);

            for (int i = 0; i < n; ++i) {
                if (i == k) continue;
                Word factor = aug[i * w + k];
                if (!factor) continue;
                for (int j = 0; j < w; ++j) {
                    Word t = fxMul(factor, aug[k * w + j], out.overflow);
                    aug[i * w + j] = fxSub(aug[i * w + j], t, out.overflow);
                }
            }
        }

        out.matrix = Matrix(A.rows, A.cols);
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                out.matrix.at(i, j) = aug[i * w + n + j];
        return out;
    }

    static MatrixResult matAdjoint(const Matrix& A) {
        MatrixResult out;
        if (!validDims(A)) { out.invalidDimensions = true; return out; }
        if (!A.isSquare()) { out.dimensionMismatch = true; return out; }

        const int n = A.rows;
        out.matrix = Matrix(A.rows, A.cols);

        if (n == 1) {
            out.matrix.at(0, 0) = FX_SCALE;
            return out;
        }

        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                std::array<Word, MAT_MAX_ELEMS> minor{};
                int mr = 0;
                for (int r = 0; r < n; ++r) {
                    if (r == i) continue;
                    int mc = 0;
                    for (int c = 0; c < n; ++c) {
                        if (c == j) continue;
                        minor[mr * (n - 1) + mc++] = A.at(r, c);
                    }
                    ++mr;
                }
                Word d = determinantImpl(minor, n - 1, out.overflow);
                if ((i + j) & 1) d = Word(0) - d;
                out.matrix.at(j, i) = d;
            }
        }
        return out;
    }

    static MatrixResult executeMatrixOp(const ISA::InstructionFields& f,
                                        const Matrix& A, const Matrix& B) {
        MatrixResult bad;
        if (f.type() != ISA::InstructionType::MATRIX) { bad.unsupported = true; return bad; }
        if (f.matrixRows != A.rows || f.matrixCols != A.cols) { bad.dimensionMismatch = true; return bad; }

        switch (f.opcode) {
            case ISA::Opcode::MATADD:         return matAdd(A, B);
            case ISA::Opcode::MATMUL:         return matMul(A, B);
            case ISA::Opcode::MATTRANSPOSE:   return matTranspose(A);
            case ISA::Opcode::MATINVERSE:     return matInverse(A);
            case ISA::Opcode::MATADJOINT:     return matAdjoint(A);
            case ISA::Opcode::MATDETERMINANT: return matDeterminant(A);
            default:                          bad.unsupported = true; return bad;
        }
    }

private:
    static inline std::vector<Word> polynomialLUT;
    static inline std::vector<Word> sineLUT;
    static inline std::vector<Word> cosineLUT;
    static inline std::vector<Word> log2LUT;

    static void ensureLUTs() {
        if (sineLUT.empty()) initTranscendentalLUTs();
    }

    static void updateFlags(Result& r) {
        r.zero = !r.value;
        r.negative = (r.value >> (WIDTH - 1)) & 1;
    }


    static bool validDims(const Matrix& m) {
        return m.rows > 0 && m.cols > 0 && m.rows <= MAT_MAX_ROWS && m.cols <= MAT_MAX_COLS;
    }

    static bool isNeg(Word a) { return (a >> (WIDTH - 1)) & 1; }
    static Word absW(Word a)  { return isNeg(a) ? Word(0) - a : a; }

    static Word fxAdd(Word a, Word b, bool& ovf) {
        Result r = add(a, b);
        ovf |= r.overflow;
        return r.value;
    }

    static Word fxSub(Word a, Word b, bool& ovf) {
        Result r = sub(a, b);
        ovf |= r.overflow;
        return r.value;
    }

    static Word fxMul(Word a, Word b, bool& ovf) {
        MultiplyResult m = multiply(a, b);
        int64_t top = static_cast<int64_t>(m.high) >> (FX_FRAC_BITS - 1);
        if (top != 0 && top != -1) ovf = true;
        return (m.high << FX_FRAC_BITS) | (m.low >> FX_FRAC_BITS);
    }

    static Word fxDiv(Word a, Word b, bool& ovf) {
        if (!b) { ovf = true; return 0; }
        const bool neg = isNeg(a) != isNeg(b);
        unsigned __int128 num = static_cast<unsigned __int128>(absW(a)) << FX_FRAC_BITS;
        unsigned __int128 q = num / absW(b);
        const unsigned __int128 limit = neg ? (1ULL << 63) : ((1ULL << 63) - 1);
        if (q > limit) ovf = true;
        Word r = static_cast<Word>(q);
        return neg ? Word(0) - r : r;
    }

    static Word determinantImpl(std::array<Word, MAT_MAX_ELEMS> m, int n, bool& ovf) {
        Word det = FX_SCALE;
        for (int k = 0; k < n; ++k) {
            int piv = k;
            Word best = absW(m[k * n + k]);
            for (int i = k + 1; i < n; ++i) {
                Word v = absW(m[i * n + k]);
                if (v > best) { best = v; piv = i; }
            }
            if (!best) return 0;
            if (piv != k) {
                for (int j = 0; j < n; ++j) std::swap(m[k * n + j], m[piv * n + j]);
                det = Word(0) - det;
            }

            Word pivot = m[k * n + k];
            det = fxMul(det, pivot, ovf);

            for (int i = k + 1; i < n; ++i) {
                Word factor = fxDiv(m[i * n + k], pivot, ovf);
                if (!factor) continue;
                for (int j = k; j < n; ++j) {
                    Word t = fxMul(factor, m[k * n + j], ovf);
                    m[i * n + j] = fxSub(m[i * n + j], t, ovf);
                }
            }
        }
        return det;
    }

    static Result interpolateFromLUT(Word x_fixed, const std::vector<Word>& lut) {
        ensureLUTs();
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

        int64_t delta_y = static_cast<int64_t>(sub(y1, y0).value);

        __int128 product = static_cast<__int128>(delta_y) * static_cast<int64_t>(frac);
        Word offset = static_cast<Word>(static_cast<int64_t>(product >> FX_FRAC_BITS));

        return add(y0, offset);
    }

    static void groupGP(const bool* g, const bool* p, bool& G, bool& P) {
        G = g[3] | (p[3] & g[2]) | (p[3] & p[2] & g[1]) | (p[3] & p[2] & p[1] & g[0]);
        P = p[3] & p[2] & p[1] & p[0];
    }

    static void carries4(const bool* g, const bool* p, bool cin, bool* c) {
        c[0] = cin;
        c[1] = g[0] | (p[0] & cin);
        c[2] = g[1] | (p[1] & g[0]) | (p[1] & p[0] & cin);
        c[3] = g[2] | (p[2] & g[1]) | (p[2] & p[1] & g[0]) | (p[2] & p[1] & p[0] & cin);
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
