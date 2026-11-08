// RISC201 interactive simulator.
//   risc201 [--stages 4|6] [--no-cache] [--run] [--expand] [--list] program.asm
// Reads debugger commands from stdin (prompt only when interactive); `help` lists them.

#include "debugger.cpp"
#include <fstream>
#include <unistd.h>

int main(int argc, char** argv) {
    Debugger dbg;
    std::string file;
    bool runOnly = false, expandOnly = false, listOnly = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--stages" && i + 1 < argc) dbg.setDepth(std::atoi(argv[++i]));
        else if (a == "--no-cache") dbg.setCache(false);
        else if (a == "--run") runOnly = true;
        else if (a == "--expand") expandOnly = true;
        else if (a == "--list") listOnly = true;
        else file = a;
    }
    if (!file.empty()) {
        std::ifstream f(file);
        if (!f) { std::cerr << "cannot open " << file << "\n"; return 1; }
        std::stringstream ss; ss << f.rdbuf();
        try {
            if (expandOnly) { std::cout << StackPreprocessor::toText(StackPreprocessor::expand(ss.str())); return 0; }
            dbg.loadSource(ss.str());
        } catch (const std::exception& e) { std::cerr << e.what() << "\n"; return 1; }
        if (listOnly) { dbg.execute("list 0 100000", std::cout); return 0; }
        if (runOnly) return dbg.runToEnd(50'000'000, std::cout) ? 0 : 2;
    }
    const bool tty = isatty(0);
    std::cout << "RISC201 simulator - " << (file.empty() ? "no program (use `load`... or start with a file)" : file) << " - type `help`\n";
    std::string line;
    for (;;) {
        if (tty) std::cout << "risc201> " << std::flush;
        if (!std::getline(std::cin, line)) break;
        if (!tty) std::cout << "risc201> " << line << "\n";
        if (line.rfind("load ", 0) == 0) {
            try {
                std::ifstream f(line.substr(5));
                if (!f) { std::cout << "cannot open file\n"; continue; }
                std::stringstream ss; ss << f.rdbuf();
                dbg.loadSource(ss.str()); std::cout << "loaded\n";
            } catch (const std::exception& e) { std::cout << e.what() << "\n"; }
            continue;
        }
        if (!dbg.execute(line, std::cout)) break;
    }
    return 0;
}
