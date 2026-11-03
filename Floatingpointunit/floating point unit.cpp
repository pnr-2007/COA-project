#pragma once

#include <algorithm>
#include <cstdint>

class FPU {
public:
    using u64  = uint64_t;
    using u128 = unsigned __int128;

    enum Flag : uint8_t { NV = 1, DZ = 2, OF = 4, UF = 8, NX = 16 };
    enum Round : uint8_t { RNE = 0, RTZ = 1, RDN = 2, RUP = 3, RMM = 4 };
    enum class Op : uint8_t { ADD, SUB, MUL, DIV, SQRT, CMP, CVTDQ, CVTQD, MVDX, MVXD };

    static constexpr u64 QNAN       = 0x7FF8000000000000ull;
    static constexpr u64 CMP_UNORD  = 2;

    struct Latency {
        unsigned add     = 4;
        unsigned mul     = 5;
        unsigned div     = 20;
        unsigned sqrt    = 24;
        unsigned convert = 3;
        unsigned compare = 2;
        unsigned move    = 1;
    };

    static bool validRounding(unsigned rm) { return rm <= RMM; }

    static bool usesRounding(Op op) {
        return op == Op::ADD || op == Op::SUB || op == Op::MUL || op == Op::DIV ||
               op == Op::SQRT || op == Op::CVTDQ || op == Op::CVTQD;
    }

    static unsigned latency(Op op, const Latency& l) {
        switch (op) {
            case Op::ADD: case Op::SUB: return l.add;
            case Op::MUL:               return l.mul;
            case Op::DIV:               return l.div;
            case Op::SQRT:              return l.sqrt;
            case Op::CVTDQ: case Op::CVTQD: return l.convert;
            case Op::CMP:               return l.compare;
            default:                    return l.move;
        }
    }

    static u64 execute(Op op, u64 a, u64 b, unsigned rm, uint8_t& flags) {
        switch (op) {
            case Op::ADD:   return addSub(a, b, false, rm, flags);
            case Op::SUB:   return addSub(a, b, true, rm, flags);
            case Op::MUL:   return mul(a, b, rm, flags);
            case Op::DIV:   return div(a, b, rm, flags);
            case Op::SQRT:  return sqrt(a, rm, flags);
            case Op::CMP:   return cmp(a, b, flags);
            case Op::CVTDQ: return cvtDQ(a, rm, flags);
            case Op::CVTQD: return cvtQD(a, rm, flags);
            case Op::MVDX:
            case Op::MVXD:  return a;
        }
        return 0;
    }

    static bool isNaN(u64 x)  { return ((x >> 52) & 0x7FF) == 0x7FF && (x & FRAC_MASK) != 0; }
    static bool isSNaN(u64 x) { return isNaN(x) && !(x & QUIET_BIT); }

private:
    static constexpr u64 FRAC_MASK = (u64(1) << 52) - 1;
    static constexpr u64 QUIET_BIT = u64(1) << 51;
    static constexpr u64 HIDDEN    = u64(1) << 52;
    static constexpr u64 SIGN_BIT  = u64(1) << 63;
    static constexpr u64 INF_BITS  = 0x7FF0000000000000ull;
    static constexpr u64 MAX_FIN   = 0x7FEFFFFFFFFFFFFFull;

    enum Cls { ZERO, FINITE, INF, NAN_ };

    struct Unpacked {
        bool    sign = false;
        Cls     cls  = ZERO;
        int64_t e    = 0;
        u64     m    = 0;
    };

    static Unpacked unpack(u64 x) {
        Unpacked u;
        u.sign = (x >> 63) != 0;
        const unsigned ex = (x >> 52) & 0x7FF;
        const u64 fr = x & FRAC_MASK;
        if (ex == 0x7FF) { u.cls = fr ? NAN_ : INF; return u; }
        if (ex == 0) {
            if (!fr) { u.cls = ZERO; return u; }
            u.cls = FINITE;
            const int lz = __builtin_clzll(fr) - 11;
            u.m = fr << lz;
            u.e = -1074 - lz;
            return u;
        }
        u.cls = FINITE;
        u.m = fr | HIDDEN;
        u.e = int64_t(ex) - 1075;
        return u;
    }

    static int msb(u128 v) {
        const u64 hi = u64(v >> 64);
        if (hi) return 127 - __builtin_clzll(hi);
        return 63 - __builtin_clzll(u64(v));
    }

    static void split(u128 M, int64_t sh, bool sticky, u128& kept, bool& rnd, bool& st) {
        if (sh <= 0) {
            kept = M << (-sh);
            rnd = false;
            st = sticky;
        } else if (sh >= 124) {
            kept = 0;
            rnd = false;
            st = true;
        } else {
            kept = M >> sh;
            rnd = ((M >> (sh - 1)) & 1) != 0;
            st = sticky || (M & ((u128(1) << (sh - 1)) - 1)) != 0;
        }
    }

    static bool increment(bool sign, bool lsb, bool rnd, bool st, unsigned rm) {
        switch (rm) {
            case RNE: return rnd && (st || lsb);
            case RTZ: return false;
            case RDN: return sign && (rnd || st);
            case RUP: return !sign && (rnd || st);
            case RMM: return rnd;
        }
        return false;
    }

    static u64 overflowResult(bool sign, unsigned rm) {
        const u64 s = sign ? SIGN_BIT : 0;
        switch (rm) {
            case RTZ: return s | MAX_FIN;
            case RDN: return sign ? (s | INF_BITS) : MAX_FIN;
            case RUP: return sign ? (s | MAX_FIN) : INF_BITS;
            default:  return s | INF_BITS;
        }
    }

    static u64 roundPack(bool sign, u128 M, int64_t E, bool sticky, unsigned rm, uint8_t& fl) {
        const int p = msb(M);
        const int64_t s = std::max<int64_t>(p - 52, -1074 - E);
        const bool tinyPre = (E + p) < -1022;

        u128 kept; bool rnd, st;
        split(M, s, sticky, kept, rnd, st);
        const bool inexact = rnd || st;

        bool tiny = false;
        if (tinyPre && inexact) {
            u128 k2; bool r2, t2;
            split(M, p - 52, sticky, k2, r2, t2);
            const bool up = increment(sign, (k2 & 1) != 0, r2, t2, rm);
            const bool reachesNormal = up && (k2 + 1) == (u128(1) << 53) && (E + p) == -1023;
            tiny = !reachesNormal;
        }

        if (increment(sign, (kept & 1) != 0, rnd, st, rm)) ++kept;
        int64_t Eres = E + s;
        if (kept == (u128(1) << 53)) { kept >>= 1; ++Eres; }

        if (inexact) fl |= NX;
        if (tiny) fl |= UF;

        const u64 sb = sign ? SIGN_BIT : 0;
        if (kept >= u128(HIDDEN)) {
            const int64_t ex = Eres + 1075;
            if (ex >= 2047) {
                fl |= OF | NX;
                return overflowResult(sign, rm);
            }
            return sb | (u64(ex) << 52) | (u64(kept) & FRAC_MASK);
        }
        return sb | u64(kept);
    }

    static u64 nanResult(u64 a, u64 b, uint8_t& fl) {
        if (isSNaN(a) || isSNaN(b)) fl |= NV;
        return QNAN;
    }

    static u64 addSub(u64 a, u64 b, bool sub, unsigned rm, uint8_t& fl) {
        if (isNaN(a) || isNaN(b)) return nanResult(a, b, fl);
        Unpacked x = unpack(a), y = unpack(b);
        y.sign ^= sub;

        if (x.cls == INF || y.cls == INF) {
            if (x.cls == INF && y.cls == INF && x.sign != y.sign) { fl |= NV; return QNAN; }
            const bool sg = x.cls == INF ? x.sign : y.sign;
            return (sg ? SIGN_BIT : 0) | INF_BITS;
        }
        if (x.cls == ZERO && y.cls == ZERO) {
            const bool sg = x.sign == y.sign ? x.sign : (rm == RDN);
            return sg ? SIGN_BIT : 0;
        }
        if (x.cls == ZERO) return (b ^ (sub ? SIGN_BIT : 0));
        if (y.cls == ZERO) return a;

        if (x.e < y.e) std::swap(x, y);
        const int64_t d = x.e - y.e;

        if (d <= 64) {
            const u128 A = u128(x.m) << d;
            const u128 B = y.m;
            u128 M; bool sign;
            if (x.sign == y.sign) { M = A + B; sign = x.sign; }
            else if (A >= B)      { M = A - B; sign = x.sign; }
            else                  { M = B - A; sign = y.sign; }
            if (M == 0) return rm == RDN ? SIGN_BIT : 0;
            return roundPack(sign, M, y.e, false, rm, fl);
        }

        const u128 A = u128(x.m) << 64;
        const u128 M = (x.sign == y.sign) ? A : A - 1;
        return roundPack(x.sign, M, x.e - 64, true, rm, fl);
    }

    static u64 mul(u64 a, u64 b, unsigned rm, uint8_t& fl) {
        if (isNaN(a) || isNaN(b)) return nanResult(a, b, fl);
        const Unpacked x = unpack(a), y = unpack(b);
        const bool sign = x.sign != y.sign;
        const u64 sb = sign ? SIGN_BIT : 0;
        if ((x.cls == INF && y.cls == ZERO) || (x.cls == ZERO && y.cls == INF)) { fl |= NV; return QNAN; }
        if (x.cls == INF || y.cls == INF) return sb | INF_BITS;
        if (x.cls == ZERO || y.cls == ZERO) return sb;
        return roundPack(sign, u128(x.m) * u128(y.m), x.e + y.e, false, rm, fl);
    }

    static u64 div(u64 a, u64 b, unsigned rm, uint8_t& fl) {
        if (isNaN(a) || isNaN(b)) return nanResult(a, b, fl);
        const Unpacked x = unpack(a), y = unpack(b);
        const bool sign = x.sign != y.sign;
        const u64 sb = sign ? SIGN_BIT : 0;
        if ((x.cls == INF && y.cls == INF) || (x.cls == ZERO && y.cls == ZERO)) { fl |= NV; return QNAN; }
        if (x.cls == INF) return sb | INF_BITS;
        if (y.cls == INF) return sb;
        if (y.cls == ZERO) { fl |= DZ; return sb | INF_BITS; }
        if (x.cls == ZERO) return sb;
        const u128 N = u128(x.m) << 64;
        const u128 q = N / y.m;
        const u128 r = N % y.m;
        return roundPack(sign, q, x.e - y.e - 64, r != 0, rm, fl);
    }

    static u128 isqrt(u128 n, u128& rem) {
        u128 r = 0;
        u128 bit = u128(1) << 126;
        while (bit > n) bit >>= 2;
        while (bit) {
            if (n >= r + bit) { n -= r + bit; r = (r >> 1) + bit; }
            else r >>= 1;
            bit >>= 2;
        }
        rem = n;
        return r;
    }

    static u64 sqrt(u64 a, unsigned rm, uint8_t& fl) {
        if (isNaN(a)) return nanResult(a, a, fl);
        const Unpacked x = unpack(a);
        if (x.cls == ZERO) return a;
        if (x.sign) { fl |= NV; return QNAN; }
        if (x.cls == INF) return a;
        u64 m = x.m;
        int64_t e = x.e;
        if (e & 1) { m <<= 1; e -= 1; }
        const u128 n = u128(m) << 62;
        u128 rem;
        const u128 r = isqrt(n, rem);
        return roundPack(false, r, (e - 62) / 2, rem != 0, rm, fl);
    }

    static u64 cmp(u64 a, u64 b, uint8_t& fl) {
        if (isNaN(a) || isNaN(b)) {
            if (isSNaN(a) || isSNaN(b)) fl |= NV;
            return CMP_UNORD;
        }
        const bool za = (a << 1) == 0, zb = (b << 1) == 0;
        if (za && zb) return 0;
        auto key = [](u64 v) { return (v >> 63) ? ~v : (v | SIGN_BIT); };
        const u64 ka = key(a), kb = key(b);
        if (ka == kb) return 0;
        return ka < kb ? u64(-1) : u64(1);
    }

    static u64 cvtDQ(u64 a, unsigned rm, uint8_t& fl) {
        const int64_t x = int64_t(a);
        if (x == 0) return 0;
        const bool sign = x < 0;
        const u64 mag = sign ? u64(0) - u64(x) : u64(x);
        return roundPack(sign, mag, -32, false, rm, fl);
    }

    static u64 cvtQD(u64 a, unsigned rm, uint8_t& fl) {
        constexpr u64 IMAX = 0x7FFFFFFFFFFFFFFFull;
        constexpr u64 IMIN = 0x8000000000000000ull;
        if (isNaN(a)) { fl |= NV; return IMAX; }
        const Unpacked x = unpack(a);
        if (x.cls == ZERO) return 0;
        if (x.cls == INF) { fl |= NV; return x.sign ? IMIN : IMAX; }

        const int64_t sh = x.e + 32;
        u128 kept; bool rnd = false, st = false;
        if (sh >= 0) {
            if (sh > 70) { fl |= NV; return x.sign ? IMIN : IMAX; }
            kept = u128(x.m) << sh;
        } else {
            split(u128(x.m), -sh, false, kept, rnd, st);
        }
        if (increment(x.sign, (kept & 1) != 0, rnd, st, rm)) ++kept;

        const u128 limit = x.sign ? (u128(1) << 63) : (u128(1) << 63) - 1;
        if (kept > limit) { fl |= NV; return x.sign ? IMIN : IMAX; }
        if (rnd || st) fl |= NX;
        return x.sign ? u64(0) - u64(kept) : u64(kept);
    }
};
