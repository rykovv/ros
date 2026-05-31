// Expected: static_assert failure — cannot write a read-only field
#include "test_registers.hpp"
#include <ros/eval.hpp>

int main() {
    constexpr mixed_reg r{};
    eval(r.status = 1_f); // status is RO
}
