#ifndef STRING_VIEW_HPP
#define STRING_VIEW_HPP

#include <stdint.h>

namespace nonstd{

template < typename T> class VectorView{

public:

  typedef T	value_type;

  VectorView(const T* data, size_t size): data_(data), size_(size) { }

  VectorView(const VectorView& other): data_(other.data()), size_(other.size()) { }

  const T& operator[](size_t index) const { return data_[index]; }

  const T* data() const { return data_; }

  size_t size() const { return size_; }

private:
  const T* data_;
  const size_t size_;
};

}

#endif // STRING_VIEW_HPP
