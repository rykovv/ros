// Expected: static_assert failure — attempt to assign read-only bits at compile time
#include "test_registers.hpp"
#include <ros/apply.hpp>

int main() {
    constexpr ro_reg r{};
    apply(r = 0xFF_r); // all bits are RO
}
