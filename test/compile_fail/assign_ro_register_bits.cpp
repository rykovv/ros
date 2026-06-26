// Expected: static_assert failure — attempt to assign read-only bits at compile
// time
#include "test_registers.hpp"

#include <ros/eval.hpp>

int main() {
    constexpr ro_reg r{};
    eval(r = 0xFF_r); // all bits are RO
}
