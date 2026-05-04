// Expected: static_assert failure — cannot read a write-only register
#include "test_registers.hpp"
#include <ros/apply.hpp>

int main() {
    constexpr wo_reg r{};
    apply(r.read());
}
