// Week 1 placeholder for the headless benchmark (Week 6, P12).
// The target is already wired so that "physics2d_bench" is a stable deliverable
// name; the real broad-phase benchmark replaces this file in Week 6.

#include <cstdio>

int main() {
    std::printf("[physics2d_bench] week 1 placeholder\n");
    std::printf(
        "  planned: --broadphase brute|sap|all --bodies 50,200,500,2000 --frames 1000 --seed 42 --json\n");
    std::printf("  scheduled: week 6 (Sweep and Prune + Tracy numbers)\n");
    return 0;
}
