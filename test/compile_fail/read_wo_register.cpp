// Expected: static_assert failure — cannot read a write-only register
#include "test_registers.hpp"
#include <ros/eval.hpp>

int main() {
    constexpr wo_reg r{};
    eval(r.read());
}
