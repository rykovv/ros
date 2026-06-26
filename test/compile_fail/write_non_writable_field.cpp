// Expected: static_assert failure — cannot write non-writable field (RO)
#include "test_registers.hpp"

#include <ros/eval.hpp>

int main() {
    constexpr mixed_reg r{};
    eval(r.status = 5_f); // status is RO
}
