#include "py_format.hpp"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace grasp {
namespace {

constexpr int MAX_DIGITS = 17;  // a double always round-trips with 17 significant digits
constexpr int FIXED_MIN_DECPT = -4;  // repr uses fixed notation for -4 < decpt <= 16
constexpr int FIXED_MAX_DECPT = 16;

}  // namespace

std::string format(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  va_list copy;
  va_copy(copy, args);
  const int size = std::vsnprintf(nullptr, 0, fmt, copy);
  va_end(copy);
  std::string out(static_cast<std::size_t>(size), '\0');
  std::vsnprintf(out.data(), out.size() + 1, fmt, args);
  va_end(args);
  return out;
}

std::string py_repr(double value) {
  if (std::isnan(value)) return "nan";
  if (std::isinf(value)) return value > 0 ? "inf" : "-inf";
  char buf[64];
  for (int digits = 1; digits <= MAX_DIGITS; ++digits) {
    std::snprintf(buf, sizeof(buf), "%.*e", digits - 1, value);
    if (std::strtod(buf, nullptr) == value) break;
  }
  // buf is [-]d[.ddd]e[+-]xx
  std::string text(buf);
  const bool negative = text[0] == '-';
  if (negative) text.erase(0, 1);
  const std::size_t e = text.find('e');
  std::string mantissa = text.substr(0, e);
  const int exponent = std::atoi(text.c_str() + e + 1);
  std::string digits;
  for (char c : mantissa)
    if (c != '.') digits += c;
  while (digits.size() > 1 && digits.back() == '0') digits.pop_back();
  const int decpt = exponent + 1;
  std::string out = negative ? "-" : "";
  if (decpt > FIXED_MIN_DECPT && decpt <= FIXED_MAX_DECPT) {
    if (decpt <= 0) {
      out += "0." + std::string(static_cast<std::size_t>(-decpt), '0') + digits;
    } else if (static_cast<std::size_t>(decpt) >= digits.size()) {
      out += digits + std::string(static_cast<std::size_t>(decpt) - digits.size(), '0') + ".0";
    } else {
      out += digits.substr(0, static_cast<std::size_t>(decpt)) + "." + digits.substr(static_cast<std::size_t>(decpt));
    }
    return out;
  }
  out += digits.substr(0, 1);
  if (digits.size() > 1) out += "." + digits.substr(1);
  out += format("e%c%02d", exponent < 0 ? '-' : '+', std::abs(exponent));
  return out;
}

}  // namespace grasp
