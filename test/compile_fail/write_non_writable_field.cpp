// Expected: static_assert failure — cannot write non-writable field (RO)
#include "test_registers.hpp"
#include <ros/apply.hpp>

int main() {
    constexpr mixed_reg r{};
    apply(r.status = 5_f); // status is RO
}
