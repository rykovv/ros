// Expected: static_assert failure — cannot read non-readable field (WO)
#include "test_registers.hpp"

#include <ros/eval.hpp>

int main() {
    constexpr mixed_reg r{};
    eval(r.command); // command is WO
}
