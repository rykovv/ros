// Expected: constraint failure — one_assignment_per_field violated
#include "test_registers.hpp"
#include <ros/apply.hpp>

int main() {
    constexpr rw_reg r{};
    apply(r.low = 1_f, r.low = 2_f); // duplicate assignment to same field
}
