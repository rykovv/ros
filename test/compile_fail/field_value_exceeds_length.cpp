// Expected: static_assert failure — assigned value greater than the field length
#include "test_registers.hpp"
#include <ros/apply.hpp>

int main() {
    constexpr rw_reg r{};
    apply(r.low = 0xFF_f); // low is [3:0], max value = 0x0F
}
