// Python-identical number formatting for the planner's reason strings: printf-style fixed precision (Python's
// f"{v:.3f}" rounds the exact binary value like printf does) and repr() of a float (shortest round-trip digits).
#pragma once

#include <string>

namespace grasp {

// printf-style formatting into a std::string.
std::string format(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Python repr(float): shortest digits that round-trip, fixed notation for exponents -4 < e <= 16, else 1e-05 style.
std::string py_repr(double value);

}  // namespace grasp
