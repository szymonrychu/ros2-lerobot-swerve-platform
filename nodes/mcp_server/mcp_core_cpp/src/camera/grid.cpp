// Occupancy grid kernels (see grid.hpp). Cells are read in place: a C-contiguous buffer through its format code, any
// other object through the sequence protocol, one item at a time, so a crop reads only the cells it shows.
#include "camera/grid.hpp"

#include <Python.h>

#include <cstring>
#include <limits>
#include <string>

namespace nb = nanobind;

namespace mcp_core::camera
{
namespace
{

constexpr int64_t INT16_MIN_VALUE = std::numeric_limits<int16_t>::min();
constexpr int64_t INT16_MAX_VALUE = std::numeric_limits<int16_t>::max();

uint8_t grey_of(int16_t cell)
{
  if (cell >= OCCUPIED_THRESHOLD) {
    return OCCUPIED_GREY;
  }
  if (cell >= 0 && cell <= FREE_THRESHOLD) {
    return FREE_GREY;
  }
  return UNKNOWN_GREY;
}

// A Python sequence item as numpy's int16 conversion of a list reads it: ints range-checked, floats truncated,
// objects with __index__ (numpy integers) through it.
int16_t item_to_int16(PyObject * item)
{
  if (PyFloat_Check(item)) {
    return static_cast<int16_t>(static_cast<int64_t>(PyFloat_AS_DOUBLE(item)));
  }
  PyObject * index = PyLong_Check(item) ? (Py_INCREF(item), item) : PyNumber_Index(item);
  if (index == nullptr) {
    PyErr_Clear();
    throw nb::type_error(("occupancy cell is not an integer: " + std::string(Py_TYPE(item)->tp_name)).c_str());
  }
  int overflow = 0;
  const long long value = PyLong_AsLongLongAndOverflow(index, &overflow);
  Py_DECREF(index);
  if (overflow != 0 || value < INT16_MIN_VALUE || value > INT16_MAX_VALUE) {
    PyErr_SetString(PyExc_OverflowError, "Python integer out of bounds for int16");
    throw nb::python_error();
  }
  return static_cast<int16_t>(value);
}

// Element reader of a buffer format code (native byte order); nullptr for formats read as a sequence instead.
using BufferReader = int16_t (*)(const char *);

template <typename T>
int16_t read_as_int16(const char * p)
{
  T value;
  std::memcpy(&value, p, sizeof(T));
  // numpy's unsafe cast: integers wrap to int16, floats truncate.
  return static_cast<int16_t>(static_cast<int64_t>(value));
}

BufferReader buffer_reader(const char * format, Py_ssize_t itemsize)
{
  if (format == nullptr) {
    return itemsize == 1 ? read_as_int16<uint8_t> : nullptr;
  }
  if (*format == '@' || *format == '=' || (*format == '<' && PY_LITTLE_ENDIAN)) {
    ++format;
  }
  if (format[0] == '\0' || format[1] != '\0') {
    return nullptr;
  }
  BufferReader reader = nullptr;
  switch (format[0]) {
    case 'b': reader = read_as_int16<int8_t>; break;
    case 'B': reader = read_as_int16<uint8_t>; break;
    case '?': reader = read_as_int16<bool>; break;
    case 'h': reader = read_as_int16<int16_t>; break;
    case 'H': reader = read_as_int16<uint16_t>; break;
    case 'i': reader = read_as_int16<int32_t>; break;
    case 'I': reader = read_as_int16<uint32_t>; break;
    case 'l': reader = itemsize == 8 ? read_as_int16<int64_t> : read_as_int16<int32_t>; break;
    case 'L': reader = itemsize == 8 ? read_as_int16<uint64_t> : read_as_int16<uint32_t>; break;
    case 'q': reader = read_as_int16<int64_t>; break;
    case 'Q': reader = read_as_int16<uint64_t>; break;
    case 'f': reader = read_as_int16<float>; break;
    case 'd': reader = read_as_int16<double>; break;
    default: return nullptr;
  }
  return reader;
}

// Random access to the cells of a grid object, whatever Python type it is.
class Cells
{
public:
  explicit Cells(nb::handle data)
  {
    if (PyObject_GetBuffer(data.ptr(), &view_, PyBUF_C_CONTIGUOUS | PyBUF_FORMAT) == 0) {
      reader_ = buffer_reader(view_.format, view_.itemsize);
      if (reader_ != nullptr) {
        has_view_ = true;
        size_ = view_.len / view_.itemsize;
        return;
      }
      PyBuffer_Release(&view_);
    } else {
      PyErr_Clear();
    }
    seq_ = PySequence_Fast(data.ptr(), "occupancy grid data must be a sequence or a buffer of integers");
    if (seq_ == nullptr) {
      throw nb::python_error();
    }
    size_ = PySequence_Fast_GET_SIZE(seq_);
    items_ = PySequence_Fast_ITEMS(seq_);
  }

  Cells(const Cells &) = delete;
  Cells & operator=(const Cells &) = delete;

  ~Cells()
  {
    if (has_view_) {
      PyBuffer_Release(&view_);
    }
    Py_XDECREF(seq_);
  }

  int64_t size() const {return size_;}

  int16_t operator[](int64_t i) const
  {
    if (has_view_) {
      return reader_(static_cast<const char *>(view_.buf) + i * view_.itemsize);
    }
    return item_to_int16(items_[i]);
  }

  // Calls visit(cell) for every cell in order.
  template <typename Visit>
  void for_each(Visit && visit) const
  {
    if (has_view_) {
      const char * p = static_cast<const char *>(view_.buf);
      for (int64_t i = 0; i < size_; ++i, p += view_.itemsize) {
        visit(reader_(p));
      }
      return;
    }
    for (int64_t i = 0; i < size_; ++i) {
      PyObject * item = items_[i];
#if PY_VERSION_HEX >= 0x030C0000
      // Fast path of a small int (every occupancy value): its value without a call.
      if (PyLong_CheckExact(item) && PyUnstable_Long_IsCompact(reinterpret_cast<PyLongObject *>(item))) {
        const Py_ssize_t value = PyUnstable_Long_CompactValue(reinterpret_cast<PyLongObject *>(item));
        if (value >= INT16_MIN_VALUE && value <= INT16_MAX_VALUE) {
          visit(static_cast<int16_t>(value));
          continue;
        }
      }
#endif
      visit(item_to_int16(item));
    }
  }

private:
  Py_buffer view_{};
  bool has_view_ = false;
  BufferReader reader_ = nullptr;
  PyObject * seq_ = nullptr;
  PyObject ** items_ = nullptr;
  int64_t size_ = 0;
};

}  // namespace

MapCounts map_counts(nb::handle data)
{
  const Cells cells(data);
  MapCounts counts;
  cells.for_each(
    [&counts](int16_t cell) {
      counts.known += cell >= 0;
      counts.occupied += cell >= OCCUPIED_THRESHOLD;
      counts.free += cell >= 0 && cell <= FREE_THRESHOLD;
    });
  return counts;
}

std::vector<uint8_t> map_crop(nb::handle data, int64_t width, int64_t height, int64_t x0, int64_t y0, int64_t size)
{
  const Cells cells(data);
  if (width < 0 || height < 0 || cells.size() != width * height) {
    throw nb::value_error(
      ("cannot reshape array of size " + std::to_string(cells.size()) + " into shape (" + std::to_string(height) +
      "," + std::to_string(width) + ")")
        .c_str());
  }
  if (size < 0) {
    throw nb::value_error("negative crop size");
  }
  std::vector<uint8_t> crop(static_cast<size_t>(size * size), UNKNOWN_GREY);
  for (int64_t row = 0; row < size; ++row) {
    const int64_t y = y0 + size - 1 - row;  // flipped: the first output row is the crop's top (largest y)
    if (y < 0 || y >= height) {
      continue;
    }
    uint8_t * out = crop.data() + row * size;
    for (int64_t col = 0; col < size; ++col) {
      const int64_t x = x0 + col;
      if (x >= 0 && x < width) {
        out[col] = grey_of(cells[y * width + x]);
      }
    }
  }
  return crop;
}

}  // namespace mcp_core::camera
