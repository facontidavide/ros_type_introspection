
#ifndef VARIANT_NUMBER_EXCEPTIONS_H
#define VARIANT_NUMBER_EXCEPTIONS_H

#include <exception>
#include <string>

namespace RosIntrospection
{

class RangeException: public std::exception
{
public:

    explicit RangeException(const char* message): msg_(message)  {}
    explicit RangeException(const std::string& message):  msg_(message)  {}
    ~RangeException() throw () {}
    const char* what() const throw ()
    {
        return msg_.c_str();
    }

protected:
    std::string msg_;
};

class TypeException: public std::exception
{
public:

    explicit TypeException(const char* message): msg_(message)  {}
    explicit TypeException(const std::string& message):  msg_(message)  {}
    ~TypeException() throw () {}
    const char* what() const throw ()
    {
        return msg_.c_str();
    }

protected:
    std::string msg_;
};

} //end namespace

#endif
