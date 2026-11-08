#pragma once

// Step-by-step CPU execution simulator with an interactive debugger (assignment 2c/2d).
// Wraps either pipeline (4- or 6-stage) and adds: cycle/instruction stepping, breakpoints,
// watchpoints, register/memory/pipeline/stack inspection, and stack-exception reports.

#include "pipeline_four_stage.cpp"
#include "pipeline_six_stage.cpp"
#include "assembler.cpp"
#include "disassembler.cpp"
#include "stack_viz.cpp"
#include <functional>
#include <iostream>
#include <memory>
#include <set>

class Debugger {
public:
    using Word = ISA::Word;
    static constexpr uint64_t DEFAULT_STACK_BYTES = 4096;

    Debugger() = default;

    // ---------- loading ----------
    // Preprocess (stack macros) -> assemble -> build pipeline.  Throws on errors.
    void loadSource(const std::string& source) {
        src_ = source;
        expanded_ = StackPreprocessor::expand(source);
        prog_ = Assembler::assemble(expanded_);
        loaded_ = true;
        rebuild();
    }
    void loadProgram(const Program& p) { prog_ = p; loaded_ = true; rebuild(); }

    void setDepth(int d) { depth_ = d; if (loaded_) rebuild(); }
    void setCache(bool on) { cache_ = on; if (loaded_) rebuild(); }
    int  depth() const { return depth_; }
    PipelineCore* cpu() { return cpu_.get(); }
    const Program& program() const { return prog_; }
    const std::vector<SourceLine>& expanded() const { return expanded_; }

    // ---------- command interpreter ----------
    // Returns false when the user asked to quit.
    bool execute(const std::string& line, std::ostream& out) {
        std::istringstream in(line);
        std::string cmd;
        if (!(in >> cmd)) return true;
        std::vector<std::string> a; std::string t;
        while (in >> t) a.push_back(t);
        try { return dispatch(cmd, a, out); }
        catch (const std::exception& e) { out << "error: " << e.what() << "\n"; return true; }
    }

    // Run to completion (batch mode). Returns true if the machine halted.
    bool runToEnd(uint64_t maxCycles, std::ostream& out) {
        need();
        const Stop s = runUntil([](const Ev&) { return false; }, maxCycles, out, false);
        report(s, out);
        return cpu_->halted();
    }

private:
    struct Ev { bool retired = false; uint64_t pc = 0; ISA::InstrWord word = 0; bool trap = false; };
    enum class Stop { Done, Breakpoint, Watch, Halted, Exception, MaxCycles };

    // ---------- state ----------
    std::string src_;
    std::vector<SourceLine> expanded_;
    Program prog_;
    bool loaded_ = false, cache_ = true;
    int depth_ = 4;
    std::unique_ptr<PipelineCore> cpu_;
    std::set<uint64_t> breaks_;
    struct RegWatch { bool fp; unsigned r; Word last; };
    struct MemWatch { uint64_t addr; Word last; };
    std::vector<RegWatch> regWatch_;
    std::vector<MemWatch> memWatch_;
    std::string watchHit_;
    bool dispRegs_ = false, dispPipe_ = false, dispStack_ = false, trace_ = false, dispMicro_ = false;
    HorizontalMicrocodeUnit6 mu_;          // stand-alone copy used to step through control memory
    ISA::RegisterFile prevRegs_{};
    std::vector<Word> prevStack_;
    Ev lastRetire_;
    bool haveRetire_ = false;
    uint64_t lastExceptions_ = 0;

    void need() const { if (!cpu_) throw std::runtime_error("no program loaded (use `load <file>`)"); }

    uint64_t stackHigh() const { return cpu_ ? cpu_->stackGuard().high() : 0; }

    void rebuild() {
        PipelineCore::Config cfg;
        cfg.memory.timing = cache_;
        if (depth_ == 6) cpu_ = std::make_unique<SixStagePipeline>(cfg);
        else             cpu_ = std::make_unique<FourStagePipeline>(cfg);
        cpu_->loadProgram(prog_.text);
        for (size_t i = 0; i < prog_.data.size(); ++i) cpu_->pokeDmem(i, prog_.data[i]);
        const uint64_t dbytes = cfg.dmemBytes;
        const uint64_t sz = prog_.stackBytes ? prog_.stackBytes : DEFAULT_STACK_BYTES;
        if (prog_.data.size() * 8 + sz > dbytes) throw std::runtime_error("data + stack do not fit in data memory");
        cpu_->stackGuard().configure(dbytes - sz, dbytes, 64);
        cpu_->pokeReg(ISA::SP, dbytes);
        breaks_.clear(); regWatch_.clear(); memWatch_.clear();
        haveRetire_ = false; lastExceptions_ = 0;
        prevRegs_ = ISA::RegisterFile{};
        for (unsigned i = 0; i < 16; ++i) prevRegs_[i] = cpu_->reg(i);
        prevStack_.clear();
    }

    // ---------- value parsing ----------
    uint64_t parseAddr(const std::string& s) const {
        auto it = prog_.symbols.find(s);
        if (it != prog_.symbols.end()) return it->second;
        if (s.rfind("line:", 0) == 0) {
            const int ln = std::stoi(s.substr(5));
            for (size_t i = 0; i < prog_.lines.size(); ++i) if (prog_.lines[i].srcLine == ln) return i * 4;
            throw std::runtime_error("no instruction on source line " + std::to_string(ln));
        }
        char* end = nullptr;
        const unsigned long long v = std::strtoull(s.c_str(), &end, 0);
        if (s.empty() || *end) throw std::runtime_error("bad address or symbol '" + s + "'");
        return v;
    }
    static bool regOf(const std::string& s, bool& fp, unsigned& r) {
        const std::string t = upperStr(s);
        fp = false;
        if (t == "SP") { r = 13; return true; }
        if (t == "LR") { r = 14; return true; }
        if (t == "PC") { r = 15; return true; }
        if (t.size() >= 2 && (t[0] == 'R' || t[0] == 'F') && t.find_first_not_of("0123456789", 1) == std::string::npos) {
            r = static_cast<unsigned>(std::stoi(t.substr(1)));
            fp = t[0] == 'F';
            return r < 16;
        }
        return false;
    }

    // ---------- formatting ----------
    static std::string hex(uint64_t v, int w = 0) {
        char b[32]; std::snprintf(b, sizeof b, "0x%0*llX", w, (unsigned long long)v); return b;
    }
    std::string disasm(ISA::InstrWord w, uint64_t pc) const { return Disassembler::format(w, pc, &prog_.textLabels); }
    std::string srcOf(uint64_t pc) const {
        const size_t i = pc / 4;
        if (pc % 4 || i >= prog_.lines.size()) return "";
        const Program::Line& l = prog_.lines[i];
        std::string s = "line " + std::to_string(l.srcLine);
        if (!l.origin.empty()) s += ": " + l.origin;
        return s;
    }
    static const char* faultName(Exec::Fault f) {
        switch (f) {
            case Exec::Fault::IllegalInstruction: return "ILLEGAL_INSTRUCTION";
            case Exec::Fault::MemoryFault:        return "MEMORY_FAULT";
            case Exec::Fault::FetchFault:         return "FETCH_FAULT";
            default:                              return "NONE";
        }
    }

    // ---------- one clock cycle ----------
    Ev cycleOnce(std::ostream& out) {
        const std::vector<PipelineCore::StageView> v = cpu_->stageView();
        const PipelineCore::Stats before = cpu_->stats();
        cpu_->step();
        Ev e;
        e.retired = cpu_->stats().retired > before.retired;
        if (e.retired) { e.pc = v.back().pc; e.word = v.back().word; lastRetire_ = e; haveRetire_ = true; }
        e.trap = cpu_->stats().exceptionsTaken > before.exceptionsTaken;
        cpu_->stackGuard().noteSp(cpu_->reg(ISA::SP));
        if (trace_) out << traceLine(v);
        return e;
    }

    std::string traceLine(const std::vector<PipelineCore::StageView>& v) const {
        std::ostringstream os;
        os << "cyc " << cpu_->stats().cycles << " |";
        for (const auto& s : v) {
            os << " " << s.name << ":";
            if (!s.valid) os << "--";
            else if (!s.hasWord) os << hex(s.pc);
            else { std::string d = disasm(s.word, s.pc); os << d.substr(0, d.find(' ')) << "@" << std::hex << s.pc << std::dec; }
            if (!s.note.empty()) os << "(" << s.note << ")";
        }
        os << "\n";
        return os.str();
    }

    bool breakHit() const {
        if (breaks_.empty()) return false;
        const auto v = cpu_->stageView();
        return v.back().valid && breaks_.count(v.back().pc);
    }

    bool watchHit() {
        const PipelineCore& c = *cpu_;
        for (RegWatch& w : regWatch_) {
            const Word now = w.fp ? c.freg(w.r) : c.reg(w.r);
            if (now != w.last) {
                watchHit_ = std::string(w.fp ? "F" : "R") + std::to_string(w.r) + ": " + hex(w.last) + " -> " + hex(now);
                w.last = now; return true;
            }
        }
        if (!memWatch_.empty()) {
            const std::vector<Word> m = c.dmem();
            for (MemWatch& w : memWatch_) {
                const Word now = w.addr / 8 < m.size() ? m[w.addr / 8] : 0;
                if (now != w.last) { watchHit_ = "mem[" + hex(w.addr) + "]: " + hex(w.last) + " -> " + hex(now); w.last = now; return true; }
            }
        }
        return false;
    }

    Stop runUntil(const std::function<bool(const Ev&)>& done, uint64_t maxCycles, std::ostream& out, bool checkBreak = true) {
        const uint64_t start = cpu_->stats().cycles;
        for (;;) {
            if (cpu_->halted()) return Stop::Halted;
            if (cpu_->stats().cycles - start >= maxCycles) return Stop::MaxCycles;
            const Ev e = cycleOnce(out);
            if (e.trap) return Stop::Exception;
            if (cpu_->halted()) return Stop::Halted;
            if (watchHit()) return Stop::Watch;
            if (done(e)) return Stop::Done;
            if (checkBreak && breakHit()) return Stop::Breakpoint;
        }
    }

    // ---------- reporting ----------
    void stackExceptionReport(std::ostream& out, bool handled) const {
        const StackGuard& g = cpu_->stackGuard();
        const StackGuard::Violation& v = g.last();
        out << "\n*** " << StackGuard::name(v.kind) << " exception ***\n";
        out << "  instruction : " << hex(v.pc, 5) << "  "
            << (v.pc / 4 < prog_.text.size() ? disasm(prog_.text[v.pc / 4], v.pc) : "?") << "   [" << srcOf(v.pc) << "]\n";
        out << "  access      : " << (v.isStore ? "store" : "load") << " at " << hex(v.addr) << "\n";
        if (v.kind == StackGuard::Kind::Overflow)
            out << "  reason      : address is " << (g.low() - v.addr) << " bytes below the stack limit " << hex(g.low()) << " (too many pushes?)\n";
        else if (v.kind == StackGuard::Kind::Underflow)
            out << "  reason      : address is at/above the stack base " << hex(g.high()) << " (pop of an empty stack?)\n";
        else
            out << "  reason      : non-SP store into the red zone just below the stack limit " << hex(g.low()) << "\n";
        out << "  SP at fault : " << hex(v.sp) << "   region [" << hex(g.low()) << ", " << hex(g.high()) << ")\n";
        out << "  delivered as: MEMORY_FAULT (code 2), BADADDR=" << hex(v.addr)
            << (handled ? "; trap handler entered via TVEC\n" : "; no handler installed, machine halted\n");
        out << "  older instructions committed; this one and all younger ones did not (precise).\n";
    }

    void report(Stop s, std::ostream& out) {
        const PipelineCore::Stats& st = cpu_->stats();
        switch (s) {
            case Stop::Halted:
                if (cpu_->fault() != Exec::Fault::None) {
                    out << "machine halted by fault " << faultName(cpu_->fault()) << "\n";
                    if (cpu_->fault() == Exec::Fault::MemoryFault && cpu_->stackGuard().last().kind != StackGuard::Kind::None)
                        stackExceptionReport(out, false);
                } else out << "HALT reached";
                out << "  [" << st.cycles << " cycles, " << st.retired << " instructions retired, CPI "
                    << (st.retired ? double(st.cycles) / double(st.retired) : 0.0) << "]\n";
                break;
            case Stop::Exception: {
                const auto& sr = cpu_->specialRegs();
                const unsigned cause = static_cast<unsigned>(sr.read(ExceptionHandler::CAUSE));
                out << "exception taken: " << (cause >= 16 ? "interrupt line " + std::to_string(cause - 16) : faultName(static_cast<Exec::Fault>(cause)))
                    << "  EPC=" << hex(sr.read(ExceptionHandler::EPC)) << "  BADADDR=" << hex(sr.read(ExceptionHandler::BADADDR))
                    << "  -> vector " << hex(cpu_->pc()) << "\n";
                const StackGuard::Violation& v = cpu_->stackGuard().last();
                if (cause == 2 && v.kind != StackGuard::Kind::None && v.addr == sr.read(ExceptionHandler::BADADDR))
                    stackExceptionReport(out, true);
                break;
            }
            case Stop::Breakpoint: {
                const auto v = cpu_->stageView().back();
                out << "breakpoint at " << hex(v.pc, 5) << " " << disasm(v.word, v.pc) << " (about to commit)\n";
                break;
            }
            case Stop::Watch: out << "watchpoint: " << watchHit_ << "\n"; break;
            case Stop::MaxCycles: out << "stopped: cycle limit reached (use `continue` to go on)\n"; break;
            case Stop::Done: break;
        }
        status(out);
    }

    void status(std::ostream& out) {
        const PipelineCore::Stats& st = cpu_->stats();
        if (haveRetire_)
            out << "last retired: " << hex(lastRetire_.pc, 5) << "  " << disasm(lastRetire_.word, lastRetire_.pc)
                << "   [" << srcOf(lastRetire_.pc) << "]\n";
        out << "cycle " << st.cycles << "  retired " << st.retired << "  fetch pc " << hex(cpu_->pc(), 5)
            << "  (" << depth_ << "-stage" << (cpu_->halted() ? ", halted" : "") << ")\n";
        if (dispRegs_) printRegs(out, false);
        if (dispPipe_) printPipe(out);
        if (dispStack_) printStack(out);
        if (dispMicro_ && haveRetire_) microTrace(out, lastRetire_.word, lastRetire_.pc);
    }

    void printRegs(std::ostream& out, bool fp) {
        char b[96];
        for (unsigned i = 0; i < 16; ++i) {
            const Word v = fp ? cpu_->freg(i) : cpu_->reg(i);
            const char* nm = fp ? "" : (i == 13 ? "SP" : i == 14 ? "LR" : i == 15 ? "PC" : "");
            const bool ch = !fp && v != prevRegs_[i];
            std::snprintf(b, sizeof b, " %c%s%-2u%-3s %016llX", ch ? '*' : ' ', fp ? "F" : "R", i, nm, (unsigned long long)(!fp && i == 15 ? cpu_->pc() : v));
            out << b << ((i % 4 == 3) ? "\n" : "");
        }
        if (!fp) for (unsigned i = 0; i < 16; ++i) prevRegs_[i] = cpu_->reg(i);
    }

    void printPipe(std::ostream& out) const {
        out << "pipeline (instruction in each stage at the start of the next cycle):\n";
        for (const auto& s : cpu_->stageView()) {
            char b[64]; std::snprintf(b, sizeof b, "  %-6s ", s.name); out << b;
            if (!s.valid) out << "-- bubble";
            else if (!s.hasWord) out << "fetch pc=" << hex(s.pc, 5);
            else out << hex(s.pc, 5) << "  " << disasm(s.word, s.pc);
            if (s.fault != Exec::Fault::None) out << "  [" << faultName(s.fault) << "]";
            if (!s.note.empty()) out << "  (" << s.note << ")";
            out << "\n";
        }
    }

    void printStack(std::ostream& out) {
        const std::vector<Word> m = cpu_->dmem();
        out << StackViz::render(*cpu_, m, prevStack_.empty() ? nullptr : &prevStack_);
        prevStack_ = m;
    }

    // ---------- microprogram view (control memory, one microinstruction per step) ----------
    void microTrace(std::ostream& out, ISA::InstrWord word, uint64_t pc) {
        out << "microprogram for " << hex(pc, 5) << "  " << disasm(word, pc) << "   [word " << hex(word, 8) << "]\n";
        mu_.begin(word, pc);
        if (mu_.running() && mu_.microPc() >= HorizontalMicrocodeUnit6::STORE_ROWS) { out << "  bad entry\n"; return; }
        unsigned n = 0;
        while (mu_.running()) {
            const unsigned upc = mu_.microPc();
            if (!mu_.result().done && upc < mu_.storeWords() && !direct(word)) {
                char b[32]; std::snprintf(b, sizeof b, "  uPC=%-2u ", upc); out << b;
                out << mu_.disassembleRow(upc) << "\n";
            }
            const HorizontalMicrocodeUnit6::Result r = mu_.step();
            ++n;
            if (r.done) {
                if (r.word.fault != Exec::Fault::None) {
                    out << "  control unit raised " << faultName(r.word.fault) << " directly (no control-store row), value "
                        << hex(r.word.faultValue) << " -> exception reaches the pipeline at commit\n";
                } else {
                    out << "  -> last row reached after " << r.microCycles << " micro-cycle(s); control word:\n    "
                        << mu_.disassemble(r.word) << "\n";
                }
            } else out << "         next uPC = " << mu_.microPc() << "\n";
        }
        (void)n;
    }
    static bool direct(ISA::InstrWord w) {
        try { return Exec::validate(ISA::decode(w)) != Exec::Fault::None; } catch (const std::invalid_argument&) { return true; }
    }

    void microRom(std::ostream& out) const {
        out << "control store: " << mu_.storeWords() << " rows x " << mu_.storeBits() << " bits (horizontal), "
            << HorizontalMicrocodeUnit6::OP_COUNT << " entry points (uPC = opcode value), continuation rows after that\n";
        for (unsigned a = 0; a < mu_.storeWords(); ++a) {
            const unsigned op = a;
            if (a < HorizontalMicrocodeUnit6::OP_COUNT)
                out << "  [" << std::string(ISA::mnemonicOf(static_cast<ISA::Opcode>(op))) << "] ";
            else out << "  [continuation] ";
            out << mu_.disassembleRow(a) << "\n";
        }
    }

    void microStats(std::ostream& out) const {
        out << "micro-cycles per opcode (control-store walk length):\n";
        for (unsigned op = 0; op < HorizontalMicrocodeUnit6::OP_COUNT; ++op) {
            ISA::InstructionFields f; f.opcode = static_cast<ISA::Opcode>(op);
            const unsigned c = mu_.translate(f).microCycles;
            if (c > 1) out << "  " << std::string(ISA::mnemonicOf(f.opcode)) << ": " << c << "\n";
        }
        out << "  (all other opcodes: 1)\n";
    }

    void printStats(std::ostream& out) const {
        const PipelineCore::Stats& s = cpu_->stats();
        const PipelineCore::InterlockStats il = cpu_->interlockStats();
        const PipelineCore::ForwardStats fw = cpu_->forwardStats();
        out << depth_ << "-stage pipeline, caches " << (cache_ ? "on" : "off") << "\n"
            << "  cycles " << s.cycles << "   retired " << s.retired << "   CPI "
            << (s.retired ? double(s.cycles) / double(s.retired) : 0.0) << "\n"
            << "  branches " << s.branches << " (taken " << s.takenBranches << ", mispredicted " << s.mispredicts << ")\n"
            << "  stalls: load-use " << il.loadUse << ", serialize " << il.serialize << ", multi-cycle exec " << s.execBusyCycles
            << ", fetch(cache) " << s.ifStallCycles << ", memory(cache) " << s.memStallCycles << "\n"
            << "  forwarding: ";
        if (depth_ == 4) out << "EX/MA " << fw.fromExMa << ", MA/RW " << fw.fromMaRw << "\n";
        else out << "MEM1 " << fw.fromMem1 << ", MEM2 " << fw.fromMem2 << ", WB " << fw.fromWb << "\n";
        out << "  exceptions " << s.exceptionsTaken << ", interrupts " << s.interruptsTaken << "\n";
        const StackGuard& g = cpu_->stackGuard();
        if (g.enabled())
            out << "  stack: peak " << g.peakBytes() << " B of " << g.capacity() << ", " << g.stats().checks
                << " checks, " << g.stats().violations << " violations\n";
    }

    void printMem(std::ostream& out, uint64_t addr, unsigned n) const {
        const std::vector<Word> m = cpu_->dmem();
        addr &= ~uint64_t(7);
        for (unsigned i = 0; i < n; ++i) {
            const uint64_t a = addr + 8ull * i;
            if (a / 8 >= m.size()) { out << "  " << hex(a, 6) << ": (out of range)\n"; break; }
            std::string lab;
            auto it = prog_.dataLabels.find(a);
            if (it != prog_.dataLabels.end()) lab = "  <" + it->second + ">";
            out << "  " << hex(a, 6) << ": " << hex(m[a / 8], 16) << "  " << (long long)m[a / 8] << lab << "\n";
        }
    }

    void listCode(std::ostream& out, uint64_t addr, unsigned n) const {
        const uint64_t cur = cpu_ ? (cpu_->stageView().back().valid ? cpu_->stageView().back().pc : cpu_->pc()) : ~0ull;
        for (unsigned i = 0; i < n; ++i) {
            const uint64_t a = addr + 4ull * i;
            if (a / 4 >= prog_.text.size()) break;
            auto lab = prog_.textLabels.find(a);
            if (lab != prog_.textLabels.end()) out << "        " << lab->second << ":\n";
            out << (a == cur ? " =>" : "   ") << (breaks_.count(a) ? "B " : "  ") << Disassembler::line(prog_.text[a / 4], a, &prog_.textLabels)
                << "    ; " << srcOf(a) << "\n";
        }
    }

    static const char* helpText() {
        return
"Execution\n"
"  step [n]        run until n instructions retire (default 1)      (alias: s)\n"
"  cycle [n]       advance n clock cycles (default 1)               (alias: cy)\n"
"  continue        run until breakpoint/watchpoint/exception/halt   (alias: c, run)\n"
"  reset           reload the program and restart\n"
"  break <addr|label|line:N>   stop when that instruction is about to commit (alias: b)\n"
"  delete [addr|label|all]     remove breakpoints (alias: d);   breaks: list them\n"
"  watch <R5|F2|mem ADDR>      stop when the value changes;     unwatch: clear all\n"
"Inspection\n"
"  regs / fregs / sr           integer registers, FP registers, special registers\n"
"  mem <addr|label> [n]        data memory words      list [addr|label] [n]   code listing\n"
"  pipe                        what each pipeline stage holds   stats   performance counters\n"
"  stack                       ASCII stack frame     stack on|off   redraw after every stop\n"
"  display regs|pipe|stack on|off     auto-show after each stop;   trace on|off  one line per cycle\n"
"Modify\n"
"  set R5 <value> | set mem <addr> <value> | set depth 4|6 | set cache on|off\n"
"  int <line 0..7>             raise an interrupt line\n"
"Microcode (control memory)\n"
"  micro [addr|label]          step through the microprogram of the instruction in decode (or at addr)\n"
"  micro word <hex>            same for a raw instruction word (shows illegal-instruction faults)\n"
"  micro rom | micro stats     dump the control store / micro-cycles per opcode;  display micro on|off\n"
"  help, quit\n";
    }

    bool dispatch(const std::string& cmd, const std::vector<std::string>& a, std::ostream& out) {
        auto isOn = [&](size_t i) { return a.size() > i && (a[i] == "on" || a[i] == "1"); };
        if (cmd == "quit" || cmd == "q" || cmd == "exit") return false;
        if (cmd == "help" || cmd == "h" || cmd == "?") { out << helpText(); return true; }
        if (cmd == "set" && a.size() >= 2 && (a[0] == "depth" || a[0] == "cache")) {
            if (a[0] == "depth") { const int d = std::stoi(a[1]); if (d != 4 && d != 6) throw std::runtime_error("depth must be 4 or 6"); setDepth(d); }
            else setCache(a[1] == "on");
            out << (loaded_ ? "pipeline rebuilt; program reloaded\n" : "ok\n");
            return true;
        }
        if (cmd == "reset") { if (!loaded_) throw std::runtime_error("no program loaded"); rebuild(); out << "reset\n"; return true; }
        need();
        if (cmd == "step" || cmd == "s" || cmd == "next") {
            const unsigned n = a.empty() ? 1 : static_cast<unsigned>(std::stoul(a[0]));
            unsigned got = 0;
            report(runUntil([&](const Ev& e) { return e.retired && ++got >= n; }, 10'000'000, out), out);
        } else if (cmd == "cycle" || cmd == "cy") {
            const unsigned n = a.empty() ? 1 : static_cast<unsigned>(std::stoul(a[0]));
            unsigned got = 0;
            report(runUntil([&](const Ev&) { return ++got >= n; }, 10'000'000, out), out);
        } else if (cmd == "continue" || cmd == "c" || cmd == "run") {
            report(runUntil([](const Ev&) { return false; }, 50'000'000, out), out);
        } else if (cmd == "break" || cmd == "b") {
            if (a.empty()) throw std::runtime_error("break <addr|label|line:N>");
            const uint64_t ad = parseAddr(a[0]);
            if (ad / 4 >= prog_.text.size()) throw std::runtime_error("address outside the program");
            breaks_.insert(ad); out << "breakpoint at " << hex(ad, 5) << "\n";
        } else if (cmd == "breaks") {
            for (uint64_t b : breaks_) out << "  " << hex(b, 5) << "  " << disasm(prog_.text[b / 4], b) << "\n";
            if (breaks_.empty()) out << "  (none)\n";
        } else if (cmd == "delete" || cmd == "d") {
            if (a.empty() || a[0] == "all") breaks_.clear(); else breaks_.erase(parseAddr(a[0]));
            out << "ok\n";
        } else if (cmd == "watch") {
            bool fp; unsigned r;
            if (a.size() >= 2 && a[0] == "mem") {
                const uint64_t ad = parseAddr(a[1]) & ~uint64_t(7);
                const auto m = cpu_->dmem();
                memWatch_.push_back({ad, ad / 8 < m.size() ? m[ad / 8] : 0});
            } else if (!a.empty() && regOf(a[0], fp, r)) regWatch_.push_back({fp, r, fp ? cpu_->freg(r) : cpu_->reg(r)});
            else throw std::runtime_error("watch <R5|F2|mem ADDR>");
            out << "watching\n";
        } else if (cmd == "unwatch") { regWatch_.clear(); memWatch_.clear(); out << "ok\n"; }
        else if (cmd == "regs" || cmd == "r") printRegs(out, false);
        else if (cmd == "fregs") printRegs(out, true);
        else if (cmd == "sr") {
            for (unsigned i = 0; i < ExceptionHandler::SR_COUNT; ++i)
                out << "  " << specialRegName(i) << "\t" << hex(cpu_->specialRegs().read(i)) << "\n";
        } else if (cmd == "mem" || cmd == "x") {
            if (a.empty()) throw std::runtime_error("mem <addr|label> [n]");
            printMem(out, parseAddr(a[0]), a.size() > 1 ? static_cast<unsigned>(std::stoul(a[1])) : 8);
        } else if (cmd == "list" || cmd == "dis" || cmd == "l") {
            uint64_t ad = 0; unsigned n = 20;
            if (!a.empty()) ad = parseAddr(a[0]);
            else { const auto v = cpu_->stageView().back(); ad = v.valid && v.pc >= 16 ? v.pc - 16 : 0; }
            if (a.size() > 1) n = static_cast<unsigned>(std::stoul(a[1]));
            listCode(out, ad, n);
        } else if (cmd == "pipe" || cmd == "p") printPipe(out);
        else if (cmd == "stats") printStats(out);
        else if (cmd == "stack") {
            if (!a.empty() && (a[0] == "on" || a[0] == "off")) { dispStack_ = a[0] == "on"; out << "stack display " << a[0] << "\n"; }
            else printStack(out);
        } else if (cmd == "display") {
            if (a.size() < 2) throw std::runtime_error("display regs|pipe|stack|micro on|off");
            const bool on = isOn(1);
            if (a[0] == "regs") dispRegs_ = on; else if (a[0] == "pipe") dispPipe_ = on; else if (a[0] == "stack") dispStack_ = on; else if (a[0] == "micro") dispMicro_ = on;
            else throw std::runtime_error("display regs|pipe|stack|micro on|off");
            out << "ok\n";
        } else if (cmd == "micro") {
            if (!a.empty() && a[0] == "rom") microRom(out);
            else if (!a.empty() && a[0] == "stats") microStats(out);
            else if (a.size() == 2 && a[0] == "word") microTrace(out, Disassembler::parseWord(a[1]), 0);
            else if (!a.empty()) {
                const uint64_t ad = parseAddr(a[0]);
                if (ad / 4 >= prog_.text.size()) throw std::runtime_error("address outside the program");
                microTrace(out, prog_.text[ad / 4], ad);
            } else {
                const auto v = cpu_->stageView()[1];          // decode stage (ID / DE+EX)
                if (!v.valid || !v.hasWord) throw std::runtime_error("decode stage is empty (bubble); give an address: micro <addr|label>");
                microTrace(out, v.word, v.pc);
            }
        } else if (cmd == "trace") { trace_ = isOn(0); out << "trace " << (trace_ ? "on" : "off") << "\n"; }
        else if (cmd == "set") {
            bool fp; unsigned r;
            if (a.size() == 3 && a[0] == "mem") { cpu_->pokeDmem(parseAddr(a[1]) / 8, parseAddr(a[2])); out << "ok\n"; }
            else if (a.size() == 2 && regOf(a[0], fp, r)) {
                const Word v = static_cast<Word>(std::stoll(a[1], nullptr, 0));
                if (fp) cpu_->pokeFreg(r, v); else cpu_->pokeReg(r, v);
                out << "ok\n";
            } else throw std::runtime_error("set R5 <v> | set mem <addr> <v> | set depth 4|6 | set cache on|off");
        } else if (cmd == "int") {
            if (a.empty()) throw std::runtime_error("int <0..7>");
            cpu_->raiseInterrupt(static_cast<unsigned>(std::stoul(a[0]))); out << "raised\n";
        } else out << "unknown command '" << cmd << "' (try `help`)\n";
        return true;
    }
};
