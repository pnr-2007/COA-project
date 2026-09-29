#pragma once

#include "isa.cpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

class ALU {
public:
    using Word = ISA::Word;
    using u128 = unsigned __int128;
    using i128 = __int128;

    static constexpr int WIDTH = 64;

    static constexpr int FX_FRAC_BITS = 32;
    static constexpr uint64_t FX_SCALE = 1ULL << FX_FRAC_BITS;

    static constexpr int LUT_INDEX_BITS = 8;
    static constexpr uint32_t POLY_SEGMENTS = 1u << LUT_INDEX_BITS;
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

    // Tables are built lazily and thread-safely on first use; calling this
    // is optional and only forces the build up front.
    static void initTranscendentalLUTs() { (void)luts(); }

    // ---- Transcendental functions -------------------------------------------------

    static Result evaluatePolynomial(Word x) { return interpolate(x, luts().poly); }

    static Result evaluateSine(Word x_fixed) {
        uint64_t s = x_fixed % TWO_PI_FX;
        bool neg = false;

        if (s >= PI_FX) { s -= PI_FX; neg = true; }
        if (s > HALF_PI_FX) s = PI_FX - s;

        Result r = interpolate(toLutInput(s), luts().sine);
        return neg ? sub(0, r.value) : r;
    }

    static Result evaluateCosine(Word x_fixed) {
        uint64_t s = x_fixed % TWO_PI_FX;
        bool neg = false;

        if (s >= PI_FX) s = TWO_PI_FX - s;
        if (s > HALF_PI_FX) { s = PI_FX - s; neg = true; }

        Result r = interpolate(toLutInput(s), luts().cosine);
        return neg ? sub(0, r.value) : r;
    }

    // Signed-angle variants: x_fixed is a two's-complement Q32 angle in radians.
    // sin is odd, cos is even, so reduce to |x| and fix up the sign.
    static Result evaluateSineSigned(Word x_fixed) {
        if (static_cast<int64_t>(x_fixed) < 0) {
            return sub(0, evaluateSine(Word(0) - x_fixed).value);
        }
        return evaluateSine(x_fixed);
    }

    static Result evaluateCosineSigned(Word x_fixed) {
        return evaluateCosine(static_cast<int64_t>(x_fixed) < 0 ? Word(0) - x_fixed : x_fixed);
    }

    static Result evaluateLog2(Word x_fixed) {
        if (x_fixed == 0) {
            Result r;
            r.divideByZero = true;
            r.overflow = true;
            return r;
        }

        // Normalise significand into [1, 2) in O(1) using the leading-zero count.
        const int msb = 63 - __builtin_clzll(x_fixed);
        const int integer_part = msb - FX_FRAC_BITS;
        const uint64_t significand = (integer_part >= 0)
            ? (x_fixed >> integer_part)
            : (x_fixed << -integer_part);

        Result frac = interpolate(significand - FX_SCALE, luts().log2);
        return add(static_cast<Word>(static_cast<int64_t>(integer_part) * static_cast<int64_t>(FX_SCALE)),
                   frac.value);
    }

    // ---- Integer arithmetic -------------------------------------------------------

    static Result add(Word a, Word b)                        { return addc(a, b, 0); }
    static Result sub(Word a, Word b)                        { return addc(a, ~b, 1); }
    static Result addWithCarry(Word a, Word b, bool cin)     { return addc(a, b, cin ? 1 : 0); }

    static Result bitwiseAnd(Word a, Word b) { return logic(a & b); }
    static Result bitwiseOr(Word a, Word b)  { return logic(a | b); }
    static Result bitwiseXor(Word a, Word b) { return logic(a ^ b); }
    static Result bitwiseNot(Word a)         { return logic(~a); }

    static Result lsl(Word a, Word n) { return logic(a << (n & (WIDTH - 1))); }
    static Result lsr(Word a, Word n) { return logic(static_cast<uint64_t>(a) >> (n & (WIDTH - 1))); }
    static Result asr(Word a, Word n) {
        return logic(static_cast<Word>(static_cast<int64_t>(a) >> (n & (WIDTH - 1))));
    }

    // Signed 64x64 -> 128 multiply (same result as the Booth loop, in one instruction).
    static MultiplyResult multiply(Word multiplicand, Word multiplier) {
        const i128 p = static_cast<i128>(static_cast<int64_t>(multiplicand)) *
                       static_cast<int64_t>(multiplier);

        MultiplyResult res;
        res.low  = static_cast<Word>(p);
        res.high = static_cast<Word>(static_cast<u128>(p) >> 64);
        res.zero = (p == 0);
        res.negative = (p < 0);
        res.overflow = static_cast<int64_t>(res.high) != (static_cast<int64_t>(res.low) >> 63);
        return res;
    }

    // Unsigned divide using the native (exact) divider.
    static DivideResult divide(Word dividend, Word divisor) {
        if (!divisor) return {0, 0, true, false};
        const uint64_t a = dividend, b = divisor;
        return {a / b, a % b, false, false};
    }

    static Result modulus(Word dividend, Word divisor) {
        if (!divisor) {
            Result r;
            r.zero = true;
            r.overflow = true;
            return r;
        }
        return logicFlagsOnly(static_cast<uint64_t>(dividend) % static_cast<uint64_t>(divisor));
    }

    // =================================================================================
    // Matrix operations. Elements are signed Q32 fixed-point values stored in Words,
    // row-major. All arithmetic goes through the ALU add/sub and 128-bit multiply/divide
    // with overflow tracking.
    // =================================================================================
    struct Matrix {
        size_t rows = 0, cols = 0;
        std::vector<Word> data;

        Matrix() = default;
        Matrix(size_t r, size_t c) : rows(r), cols(c), data(r * c, 0) {}

        Word&       at(size_t i, size_t j)       { return data[i * cols + j]; }
        const Word& at(size_t i, size_t j) const { return data[i * cols + j]; }

        static Matrix identity(size_t n) {
            Matrix m(n, n);
            for (size_t i = 0; i < n; ++i) m.at(i, i) = FX_SCALE;
            return m;
        }

        static Matrix fromDouble(const std::vector<std::vector<double>>& v) {
            Matrix m(v.size(), v.empty() ? 0 : v[0].size());
            for (size_t i = 0; i < m.rows; ++i)
                for (size_t j = 0; j < m.cols; ++j)
                    m.at(i, j) = static_cast<Word>(static_cast<int64_t>(std::llround(v[i][j] * 4294967296.0)));
            return m;
        }

        std::vector<std::vector<double>> toDouble() const {
            std::vector<std::vector<double>> v(rows, std::vector<double>(cols));
            for (size_t i = 0; i < rows; ++i)
                for (size_t j = 0; j < cols; ++j)
                    v[i][j] = static_cast<double>(static_cast<int64_t>(at(i, j))) / 4294967296.0;
            return v;
        }
    };

    struct MatrixResult {
        Matrix value;
        Word scalar = 0;              // determinant, where computed
        bool singular = false;
        bool dimensionError = false;
        bool overflow = false;        // some intermediate left the Q31.32 range
        bool ok() const { return !singular && !dimensionError && !overflow; }
    };

    // Below this size Strassen falls back to the classic triple loop.
    static constexpr size_t STRASSEN_CUTOFF = 16;
    // Pivots with |value| <= this many Q32 units are treated as zero.
    static constexpr uint64_t PIVOT_EPS = 16;

    // Strassen multiplication: 7 recursive products instead of 8 per level.
    // Non-power-of-two / non-square inputs are zero-padded to the next power of two.
    static MatrixResult matMultiplyStrassen(const Matrix& A, const Matrix& B) {
        MatrixResult res;
        if (A.cols != B.rows) { res.dimensionError = true; return res; }

        const size_t r = A.rows, k = A.cols, c = B.cols;
        const size_t dim = std::max({r, k, c});
        res.value = Matrix(r, c);
        if (dim == 0) return res;

        bool ovf = false;
        if (dim <= STRASSEN_CUTOFF) {
            naiveGeneral(A.data.data(), B.data.data(), res.value.data.data(), r, k, c, ovf);
            res.overflow = ovf;
            return res;
        }

        size_t m = 1;
        while (m < dim) m <<= 1;

        std::vector<Word> PA(m * m, 0), PB(m * m, 0), PC(m * m, 0);
        for (size_t i = 0; i < r; ++i) std::copy_n(&A.data[i * k], k, &PA[i * m]);
        for (size_t i = 0; i < k; ++i) std::copy_n(&B.data[i * c], c, &PB[i * m]);

        strassenRec(PA, PB, PC, m, ovf);

        for (size_t i = 0; i < r; ++i) std::copy_n(&PC[i * m], c, &res.value.data[i * c]);
        res.overflow = ovf;
        return res;
    }

    // Reference O(n^3) multiply (also useful for validating Strassen).
    static MatrixResult matMultiplyNaive(const Matrix& A, const Matrix& B) {
        MatrixResult res;
        if (A.cols != B.rows) { res.dimensionError = true; return res; }
        res.value = Matrix(A.rows, B.cols);
        bool ovf = false;
        naiveGeneral(A.data.data(), B.data.data(), res.value.data.data(), A.rows, A.cols, B.cols, ovf);
        res.overflow = ovf;
        return res;
    }

    // Transpose is a pure index permutation (no elimination involved); done cache-blocked.
    static MatrixResult matTranspose(const Matrix& A) {
        MatrixResult res;
        res.value = Matrix(A.cols, A.rows);
        constexpr size_t T = 16;
        for (size_t ii = 0; ii < A.rows; ii += T)
            for (size_t jj = 0; jj < A.cols; jj += T)
                for (size_t i = ii; i < std::min(ii + T, A.rows); ++i)
                    for (size_t j = jj; j < std::min(jj + T, A.cols); ++j)
                        res.value.data[j * A.rows + i] = A.data[i * A.cols + j];
        return res;
    }

    // Determinant via forward Gaussian elimination with partial pivoting.
    // A (near-)zero pivot gives scalar = 0 and singular = true.
    static MatrixResult matDeterminant(const Matrix& A) {
        MatrixResult res;
        if (A.rows != A.cols) { res.dimensionError = true; return res; }
        bool ovf = false, sing = false;
        res.scalar = detCore(A.data, A.rows, ovf, sing);
        res.singular = sing;
        res.overflow = ovf;
        return res;
    }

    // Inverse via Gauss-Jordan elimination on [A | I] with partial pivoting.
    static MatrixResult matInverse(const Matrix& A) {
        MatrixResult res;
        if (A.rows != A.cols) { res.dimensionError = true; return res; }
        gaussJordan(A, res.value, res.scalar, res.singular, res.overflow);
        if (res.singular) res.value = Matrix();
        return res;
    }

    // Adjugate (classical adjoint). If A is invertible: adj(A) = det(A) * A^-1, both
    // obtained from one Gauss-Jordan pass. If A is singular the inverse doesn't exist,
    // so each cofactor is computed as a minor determinant (also by Gaussian elimination);
    // that path is O(n^5) but valid for singular input, so `singular` stays false.
    static MatrixResult matAdjoint(const Matrix& A) {
        MatrixResult res;
        if (A.rows != A.cols) { res.dimensionError = true; return res; }

        const size_t n = A.rows;
        res.value = Matrix(n, n);
        if (n == 0) return res;
        if (n == 1) { res.value.at(0, 0) = FX_SCALE; return res; }

        Matrix inv;
        Word det = 0;
        bool sing = false, ovf = false;
        gaussJordan(A, inv, det, sing, ovf);

        if (!sing) {
            for (size_t i = 0; i < n * n; ++i) res.value.data[i] = fxMul(inv.data[i], det, ovf);
            res.scalar = det;
            res.overflow = ovf;
            return res;
        }

        ovf = false;
        std::vector<Word> minor((n - 1) * (n - 1));
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                size_t idx = 0;
                for (size_t r = 0; r < n; ++r) {
                    if (r == i) continue;
                    for (size_t c = 0; c < n; ++c)
                        if (c != j) minor[idx++] = A.at(r, c);
                }
                bool s = false;
                Word d = detCore(minor, n - 1, ovf, s);
                if ((i + j) & 1) d = sub(0, d).value;
                res.value.at(j, i) = d;   // adjugate is the transpose of the cofactor matrix
            }
        }
        res.scalar = 0;
        res.overflow = ovf;
        return res;
    }

private:
    // ---- Signed Q32 helpers -------------------------------------------------------
    static int64_t sx(Word w) { return static_cast<int64_t>(w); }

    static uint64_t fxMag(Word w) {
        const int64_t s = sx(w);
        return s < 0 ? Word(0) - w : w;
    }

    static Word fromRaw(i128 v, bool& ovf) {
        if (v > INT64_MAX || v < INT64_MIN) ovf = true;
        return static_cast<Word>(static_cast<int64_t>(v));
    }

    static Word fxMul(Word a, Word b, bool& ovf) {
        const i128 p = static_cast<i128>(sx(a)) * sx(b);
        return fromRaw((p + (i128(1) << (FX_FRAC_BITS - 1))) >> FX_FRAC_BITS, ovf);
    }

    // b must be non-zero.
    static Word fxDiv(Word a, Word b, bool& ovf) {
        return fromRaw((static_cast<i128>(sx(a)) << FX_FRAC_BITS) / sx(b), ovf);
    }

    static Word fxSub(Word a, Word b, bool& ovf) {
        const Result r = sub(a, b);
        ovf |= r.overflow;
        return r.value;
    }

    static Word fxAdd(Word a, Word b, bool& ovf) {
        const Result r = add(a, b);
        ovf |= r.overflow;
        return r.value;
    }

    // C(r x c) = A(r x k) * B(k x c). Products are accumulated in 128 bits and rounded
    // once per output element, so the only rounding error is the final shift.
    static void naiveGeneral(const Word* A, const Word* B, Word* C,
                             size_t r, size_t k, size_t c, bool& ovf) {
        std::vector<i128> acc(c);
        const i128 half = i128(1) << (FX_FRAC_BITS - 1);
        for (size_t i = 0; i < r; ++i) {
            std::fill(acc.begin(), acc.end(), i128(0));
            for (size_t kk = 0; kk < k; ++kk) {
                const int64_t a = sx(A[i * k + kk]);
                if (!a) continue;
                const Word* brow = B + kk * c;
                for (size_t j = 0; j < c; ++j) acc[j] += static_cast<i128>(a) * sx(brow[j]);
            }
            for (size_t j = 0; j < c; ++j) C[i * c + j] = fromRaw((acc[j] + half) >> FX_FRAC_BITS, ovf);
        }
    }

    static std::vector<Word> addFlat(const std::vector<Word>& X, const std::vector<Word>& Y, bool& ovf) {
        std::vector<Word> Z(X.size());
        for (size_t i = 0; i < X.size(); ++i) Z[i] = fxAdd(X[i], Y[i], ovf);
        return Z;
    }

    static std::vector<Word> subFlat(const std::vector<Word>& X, const std::vector<Word>& Y, bool& ovf) {
        std::vector<Word> Z(X.size());
        for (size_t i = 0; i < X.size(); ++i) Z[i] = fxSub(X[i], Y[i], ovf);
        return Z;
    }

    // n is a power of two. A, B, C are n x n row-major.
    static void strassenRec(const std::vector<Word>& A, const std::vector<Word>& B,
                            std::vector<Word>& C, size_t n, bool& ovf) {
        if (n <= STRASSEN_CUTOFF) {
            naiveGeneral(A.data(), B.data(), C.data(), n, n, n, ovf);
            return;
        }

        const size_t h = n / 2;
        auto quad = [&](const std::vector<Word>& M, size_t qr, size_t qc) {
            std::vector<Word> Q(h * h);
            for (size_t i = 0; i < h; ++i)
                std::copy_n(&M[(qr * h + i) * n + qc * h], h, &Q[i * h]);
            return Q;
        };
        auto mul = [&](const std::vector<Word>& X, const std::vector<Word>& Y) {
            std::vector<Word> Z(h * h);
            strassenRec(X, Y, Z, h, ovf);
            return Z;
        };

        const auto A11 = quad(A, 0, 0), A12 = quad(A, 0, 1), A21 = quad(A, 1, 0), A22 = quad(A, 1, 1);
        const auto B11 = quad(B, 0, 0), B12 = quad(B, 0, 1), B21 = quad(B, 1, 0), B22 = quad(B, 1, 1);

        const auto M1 = mul(addFlat(A11, A22, ovf), addFlat(B11, B22, ovf));
        const auto M2 = mul(addFlat(A21, A22, ovf), B11);
        const auto M3 = mul(A11, subFlat(B12, B22, ovf));
        const auto M4 = mul(A22, subFlat(B21, B11, ovf));
        const auto M5 = mul(addFlat(A11, A12, ovf), B22);
        const auto M6 = mul(subFlat(A21, A11, ovf), addFlat(B11, B12, ovf));
        const auto M7 = mul(subFlat(A12, A22, ovf), addFlat(B21, B22, ovf));

        const auto C11 = addFlat(subFlat(addFlat(M1, M4, ovf), M5, ovf), M7, ovf);
        const auto C12 = addFlat(M3, M5, ovf);
        const auto C21 = addFlat(M2, M4, ovf);
        const auto C22 = addFlat(addFlat(subFlat(M1, M2, ovf), M3, ovf), M6, ovf);

        for (size_t i = 0; i < h; ++i) {
            std::copy_n(&C11[i * h], h, &C[i * n]);
            std::copy_n(&C12[i * h], h, &C[i * n + h]);
            std::copy_n(&C21[i * h], h, &C[(h + i) * n + 0]);
            std::copy_n(&C22[i * h], h, &C[(h + i) * n + h]);
        }
    }

    // Forward elimination with partial pivoting; det = sign * product of pivots.
    static Word detCore(std::vector<Word> a, size_t n, bool& ovf, bool& singular) {
        Word det = FX_SCALE;
        for (size_t col = 0; col < n; ++col) {
            size_t piv = col;
            uint64_t best = fxMag(a[col * n + col]);
            for (size_t r = col + 1; r < n; ++r) {
                const uint64_t mag = fxMag(a[r * n + col]);
                if (mag > best) { best = mag; piv = r; }
            }
            if (best <= PIVOT_EPS) { singular = true; return 0; }

            if (piv != col) {
                for (size_t k = 0; k < n; ++k) std::swap(a[piv * n + k], a[col * n + k]);
                det = sub(0, det).value;
            }

            const Word pivot = a[col * n + col];
            det = fxMul(det, pivot, ovf);

            for (size_t r = col + 1; r < n; ++r) {
                if (!a[r * n + col]) continue;
                const Word f = fxDiv(a[r * n + col], pivot, ovf);
                a[r * n + col] = 0;
                for (size_t k = col + 1; k < n; ++k)
                    a[r * n + k] = fxSub(a[r * n + k], fxMul(f, a[col * n + k], ovf), ovf);
            }
        }
        return det;
    }

    // Gauss-Jordan on [A | I]. Produces the inverse and the determinant in one pass.
    static void gaussJordan(const Matrix& A, Matrix& inv, Word& det, bool& singular, bool& ovf) {
        const size_t n = A.rows, w = 2 * n;
        std::vector<Word> m(n * w, 0);
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) m[i * w + j] = A.at(i, j);
            m[i * w + n + i] = FX_SCALE;
        }

        det = FX_SCALE;
        singular = false;

        for (size_t col = 0; col < n; ++col) {
            size_t piv = col;
            uint64_t best = fxMag(m[col * w + col]);
            for (size_t r = col + 1; r < n; ++r) {
                const uint64_t mag = fxMag(m[r * w + col]);
                if (mag > best) { best = mag; piv = r; }
            }
            if (best <= PIVOT_EPS) { singular = true; det = 0; return; }

            if (piv != col) {
                for (size_t k = 0; k < w; ++k) std::swap(m[piv * w + k], m[col * w + k]);
                det = sub(0, det).value;
            }

            const Word pivot = m[col * w + col];
            det = fxMul(det, pivot, ovf);

            // Columns < col of the pivot row are already zero, so start at col.
            for (size_t k = col; k < w; ++k) m[col * w + k] = fxDiv(m[col * w + k], pivot, ovf);

            for (size_t r = 0; r < n; ++r) {
                if (r == col) continue;
                const Word f = m[r * w + col];
                if (!f) continue;
                for (size_t k = col; k < w; ++k)
                    m[r * w + k] = fxSub(m[r * w + k], fxMul(f, m[col * w + k], ovf), ovf);
            }
        }

        inv = Matrix(n, n);
        for (size_t i = 0; i < n; ++i)
            for (size_t j = 0; j < n; ++j) inv.at(i, j) = m[i * w + n + j];
    }

    // ---- Transcendental helpers ---------------------------------------------------
    static constexpr uint64_t TWO_PI_FX  = static_cast<uint64_t>(2.0 * M_PI * FX_SCALE);
    static constexpr uint64_t PI_FX      = static_cast<uint64_t>(M_PI * FX_SCALE);
    static constexpr uint64_t HALF_PI_FX = static_cast<uint64_t>((M_PI / 2.0) * FX_SCALE);

    using LUT = std::array<Word, POLY_LUT_SIZE>;

    struct Tables {
        LUT poly{}, sine{}, cosine{}, log2{};
        Tables() {
            for (uint32_t i = 0; i < POLY_LUT_SIZE; ++i) {
                const double x = static_cast<double>(i) / POLY_SEGMENTS;
                const double rad = x * (M_PI / 2.0);
                poly[i]   = static_cast<Word>(targetPolynomialHost(x) * FX_SCALE);
                sine[i]   = static_cast<Word>(std::sin(rad) * FX_SCALE);
                cosine[i] = static_cast<Word>(std::cos(rad) * FX_SCALE);
                log2[i]   = static_cast<Word>(std::log2(1.0 + x) * FX_SCALE);
            }
        }
    };

    static const Tables& luts() {
        static const Tables t;
        return t;
    }

    static void setFlags(Result& r) {
        r.zero = (r.value == 0);
        r.negative = (r.value >> (WIDTH - 1)) & 1;
    }

    static Result logic(Word v) {
        Result r;
        r.value = v;
        setFlags(r);
        return r;
    }

    static Result logicFlagsOnly(Word v) { return logic(v); }

    // Map a value in [0, pi/2] (Q32) to a LUT input in [0, 1] (Q32).
    // 128-bit intermediate: the original 64-bit product overflowed.
    static uint64_t toLutInput(uint64_t s) {
        return static_cast<uint64_t>((static_cast<u128>(s) << FX_FRAC_BITS) / HALF_PI_FX);
    }

    // Native add-with-carry producing identical flags to the carry-lookahead model.
    static Result addc(Word a, Word b, unsigned cin) {
        const u128 sum = static_cast<u128>(a) + b + cin;
        Result r;
        r.value = static_cast<Word>(sum);
        r.carryOut = static_cast<bool>(sum >> 64);
        r.overflow = static_cast<bool>((~(a ^ b) & (a ^ r.value)) >> (WIDTH - 1));
        setFlags(r);
        return r;
    }

    static Result interpolate(Word x, const LUT& lut) {
        if (x >= FX_SCALE) {
            Result r;
            r.value = lut[POLY_SEGMENTS];
            setFlags(r);
            return r;
        }

        constexpr int SHIFT = FX_FRAC_BITS - LUT_INDEX_BITS;
        const uint32_t idx = static_cast<uint32_t>(x >> SHIFT);
        const int64_t frac = static_cast<int64_t>((x & ((1ULL << SHIFT) - 1)) << LUT_INDEX_BITS);

        const Word y0 = lut[idx];
        // Signed slope so decreasing segments (cos, the polynomial) interpolate correctly.
        const int64_t dy = static_cast<int64_t>(lut[idx + 1] - y0);
        const int64_t offset = static_cast<int64_t>((static_cast<i128>(dy) * frac) >> FX_FRAC_BITS);

        return addc(y0, static_cast<Word>(offset), 0);
    }
};

         

     
