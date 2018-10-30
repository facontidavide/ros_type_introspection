#ifndef GROWING_VECTOR_H
#define GROWING_VECTOR_H

#include <stdlib.h>
#include <vector>

/**
 * The goal of this wrapper is to create a vector-like
 * container that can only grow.
 * This allows to reuse instances of T.
 */
template <typename T>
class RecyclingVector
{
public:
  typedef typename std::vector<T>::iterator Iterator;
  RecyclingVector() : _size(0) {}

  const T& operator[](size_t index) const
  {
    return _storage[index];
  }
  T& operator[](size_t index)
  {
    return _storage[index];
  }
  void resize(size_t new_size)
  {
    if( new_size > _storage.size() )
    {
      _storage.resize(new_size);
    }
    _size = new_size;
  }
  void clear() { _size = 0; }

  size_t size() const { return _size; }

  Iterator begin() { return _storage.begin(); }

  Iterator end() { return _storage.begin() + _size; }

private:

  std::vector<T> _storage;
  size_t _size;
};

#endif // GROWING_VECTOR_H
