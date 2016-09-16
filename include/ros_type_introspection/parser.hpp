#ifndef ROSTypePARSER_H
#define ROSTypePARSER_H

#include <vector>
#include <map>
#include <boost/utility/string_ref.hpp>
#include <ros_type_introspection/string.hpp>
#include <ros_type_introspection/stringtree.h>

namespace RosIntrospection{


#if 1
typedef ssoX::basic_string< 63, char> SString;
#else
typedef std::string SString;
#endif

typedef details::TreeElement<SString> StringElement;
typedef details::Tree<SString> StringTree;

enum BuiltinType {
    BOOL , BYTE, CHAR,
    UINT8, UINT16, UINT32, UINT64,
    INT8, INT16, INT32, INT64,
    FLOAT32, FLOAT64,
    TIME, DURATION,
    STRING, OTHER
};

const int BuiltinTypeSize[OTHER] = {
    1, 1, 1,
    1, 2, 4, 8,
    1, 2, 4, 8,
    4, 8,
    8, 8,
    -1
};

class ROSType {
public:

    ROSType(){}

    ROSType(const std::string& name);

    /// Concatenation of msg_name and pkg_name.
    /// ex.: geometry_msgs/Pose[40]"
    const std::string& baseName() const;

    /// ex.: geometry_msgs/Pose[40] -> "Pose"
    const SString& msgName()  const;

    /// ex.: geometry_msgs/Pose[40] -> "geometry_msgs"
    const SString& pkgName()  const;

    void setPkgName(const SString& new_pkg);

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
    SString _msg_name;
    SString _pkg_name;

};

class ROSField {
public:
    ROSField(const std::string& name, const ROSType& type ):
        _name( name ), _type( type ) {}

    ROSField(const std::string& definition );

    const SString&  name() const { return _name; }

    const ROSType&      type() const { return _type; }

    /// True if field is a constant in message definition
    bool isConstant() const {
        return _value.size() != 0;
    }

    /// If constant, value of field, else undefined
    const SString& value() const   { return _value; }

    friend class ROSMessage;

protected:
    SString _name;
    ROSType     _type;
    SString _value;
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
