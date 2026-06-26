// Expected: static_assert failure — assigned value type is too wide
#include "test_registers.hpp"

#include <ros/eval.hpp>

int main() {
    constexpr rw_reg r{}; // uint8_t register
    uint32_t wide = 0x1234;
    eval(r = wide); // uint32_t is wider than uint8_t
}
