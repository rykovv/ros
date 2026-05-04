// Expected: static_assert failure — cannot read non-readable field (WO)
#include "test_registers.hpp"
#include <ros/apply.hpp>

int main() {
    constexpr mixed_reg r{};
    apply(r.command.read()); // command is WO
}
