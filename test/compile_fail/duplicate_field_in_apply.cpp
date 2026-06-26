// Expected: constraint failure — one_assignment_per_field violated
#include "test_registers.hpp"

#include <ros/eval.hpp>

int main() {
    constexpr rw_reg r{};
    eval(r.low = 1_f, r.low = 2_f); // duplicate assignment to same field
}
