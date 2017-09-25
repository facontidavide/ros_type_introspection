#ifndef VECTOR_VIEW_HPP
#define VECTOR_VIEW_HPP

#include <exception>
#include <vector>
#include <array>

namespace nonstd{

template < typename T, typename S = size_t>
class VectorViewMutable
{
public:

  typedef T	  value_type;
  typedef S 	size_type;

  VectorViewMutable(T* data, size_type size): data_(data), size_(size) { }

  VectorViewMutable(const VectorViewMutable& other): data_(other.data()), size_(other.size()) { }

  VectorViewMutable(std::vector<T>& other): data_(other.data()), size_(other.size()) { }

  template<size_t SIZE>
  VectorViewMutable(std::array<T, SIZE>& other): data_(other.data()), size_(other.size()) { }

  const value_type& operator[](size_type index) const { return data_[index]; }
  value_type& operator[](size_type index)             { return data_[index]; }

  const value_type& at(size_type index) const
  {
    if( index >= size_ ) {
      throw std::range_error("buffer overrun");
    }
    return data_[index];
  }

  value_type& at(size_type index)
  {
    if( index >= size_ ) {
      throw std::range_error("buffer overrun");
    }
    return data_[index];
  }

  const value_type* data() const { return data_; }
  value_type* data()             { return data_; }

  size_type size() const { return size_; }

protected:
  value_type* data_;
  const size_type   size_;
};

//-----------------------------------------------------------------------------------


// helper container to store the pointer to a contiguous array
template < typename T, typename S = size_t>
class VectorView
{
public:

  typedef T	  value_type;
  typedef S 	size_type;

  VectorView(const T* data, size_type size): data_(data), size_(size) { }

  VectorView(const VectorView& other): data_(other.data()), size_(other.size()) { }

  VectorView(const VectorViewMutable<T>& other): data_(other.data()), size_(other.size()) { }

  VectorView(const std::vector<T>& other): data_(other.data()), size_(other.size()) { }

  template<size_t SIZE>
  VectorView(const std::array<T, SIZE>& other): data_(other.data()), size_(other.size()) { }

  const value_type& operator[](size_type index) const { return data_[index]; }

  const value_type& at(size_type index) const
  {
    if( index >= size_ )
    {
      throw std::range_error("buffer overrun");
    }
    return data_[index];
  }

  const value_type* data() const { return data_; }
  //value_type* data()             { return data_; }

  size_type size() const { return size_; }

protected:
  const value_type* data_;
  const size_type   size_;
};



} //end namspace

#endif // VECTOR_VIEW_HPP
