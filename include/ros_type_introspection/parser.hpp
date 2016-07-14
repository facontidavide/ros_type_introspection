#ifndef ROSTypePARSER_H
#define ROSTypePARSER_H

#include <vector>
#include <map>
#include <boost/utility/string_ref.hpp>
#include <ros_type_introspection/string.hpp>

namespace ROSTypeParser{


#if 0
typedef ssoX::basic_string<31,char> ShortString;
typedef ssoX::basic_string<63,char> LongString;
#else
typedef std::string ShortString;
typedef std::string LongString;
#endif


enum BuiltinType {
    BOOL , BYTE, CHAR,
    UINT8, UINT16, UINT32, UINT64,
    INT8, INT16, INT32, INT64,
    FLOAT32, FLOAT64,
    TIME, DURATION,
    STRING, OTHER
};




class ROSType {
public:

    ROSType(){}

    ROSType(const std::string& name);

    /// Concatenation of msg_name and pkg_name.
    /// ex.: geometry_msgs/Pose[40] -> "geometry_msgs/Pose"
    const std::string& baseName() const;

    /// ex.: geometry_msgs/Pose[40] -> "Pose"
    const ShortString& msgName()  const;

    /// ex.: geometry_msgs/Pose[40] -> "geometry_msgs"
    const ShortString& pkgName()  const;

    /// True if the type is an array
    bool isArray() const;

    /// True if the type is ROS builtin
    bool isBuiltin() const;

    /// 1 if !is_array, -1 if is_array and array is
    /// variable length, otherwise length in name
    int  arraySize() const;

    /// If builtin, size of builtin, -1 means variable or undefined
    int typeSize() const;

    /// If type is builtin, type id.  OTHER otherwise.
    BuiltinType typeID() const;

    bool operator==(const ROSType& other) const  {
        return this->baseName() == other.baseName();
    }

    bool operator<(const ROSType& other) const {
        return this->baseName() < other.baseName();
    }

protected:

    BuiltinType _id;
    int         _array_size;
    std::string _base_name;
    ShortString _msg_name;
    ShortString _pkg_name;

};

class ROSField {
public:
    ROSField(const std::string& name, const ROSType& type ):
        _name( name ), _type( type ) {}

    ROSField(const std::string& definition );

    const ShortString&  name() const { return _name; }

    const ROSType&      type() const { return _type; }

    /// True if field is a constant in message definition
    bool isConstant() const {
        return _value.size() != 0;
    }

    /// If constant, value of field, else undefined
    const ShortString& value() const   { return _value; }

    friend class ROSMessage;

protected:
    ShortString _name;
    ROSType     _type;
    ShortString _value;
};

class ROSMessage{
public:
    ROSMessage() {}
    ROSMessage(const std::string& msg_def );

    void updateTypes(std::vector<ROSType> all_types);

    ROSType type;
    std::vector<ROSField> fields;
};

typedef std::vector<ROSMessage> ROSTypeList;


//------------------------------


ROSTypeList buildROSTypeMapFromDefinition(
        const std::string& type_name,
        const std::string& msg_definition);

std::ostream& operator<<(std::ostream& s, const ROSTypeList& c);


} // end namespace

#endif // ROSTypePARSER_H
