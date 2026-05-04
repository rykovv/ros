// Expected: static_assert failure — cannot write a read-only field
#include "test_registers.hpp"
#include <ros/apply.hpp>

int main() {
    constexpr mixed_reg r{};
    apply(r.status = 1_f); // status is RO
}
