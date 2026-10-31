#pragma once

#include "exec.cpp"
#include <cstdint>

enum class ExceptionCode : uint8_t {
    None               = 0,
    IllegalInstruction = 1,
    MemoryFault        = 2,
    FetchFault         = 3,
    DivideByZero       = 4,
    Overflow           = 5
};

class ExceptionHandler {
public:
    static constexpr uint64_t EXCEPTION_VECTOR_BASE = 0x00000080;

    struct ExceptionRecord {
        bool active           = false;
        ExceptionCode code    = ExceptionCode::None;
        uint64_t epc          = 0;
        uint64_t faultAddress = 0;
    };

    inline void raiseException(Exec::Fault fault, uint64_t faultPC, uint64_t faultAddr = 0) {
        if (record_.active) return;
        record_.active = true;
        record_.epc = faultPC;
        record_.faultAddress = faultAddr;

        switch (fault) {
            case Exec::Fault::IllegalInstruction: record_.code = ExceptionCode::IllegalInstruction; break;
            case Exec::Fault::MemoryFault:        record_.code = ExceptionCode::MemoryFault;        break;
            case Exec::Fault::FetchFault:         record_.code = ExceptionCode::FetchFault;         break;
            default:                               record_.code = ExceptionCode::None;               break;
        }
    }

    inline void raiseALUException(ExceptionCode code, uint64_t faultPC) {
        if (record_.active) return;
        record_.active = true;
        record_.code   = code;
        record_.epc    = faultPC;
    }

    inline bool hasException() const { return record_.active; }
    inline uint64_t trapVector() const { return EXCEPTION_VECTOR_BASE; }
    inline void clear() { record_ = ExceptionRecord(); }

    const ExceptionRecord& record() const { return record_; }

private:
    ExceptionRecord record_;
};
