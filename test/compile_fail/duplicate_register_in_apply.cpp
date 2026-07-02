// Expected: constraint failure — one_assignment_per_field violated
#include "test_registers.hpp"

#include <ros/eval.hpp>

int main() {
    constexpr rw_reg r{};
    eval(r = 1_r, r = 2); // duplicate assignment to same register
}
