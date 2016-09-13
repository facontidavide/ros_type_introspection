#ifndef STRINGTREE_H
#define STRINGTREE_H


#include <vector>
#include <iostream>
#include <boost/container/stable_vector.hpp>
#include <ros_type_introspection/string.hpp>

namespace details{

/**
 * @brief Element of the tree. it has a single parent and N >= 0 children.
 */
template <typename T> class TreeElement{

public:

    TreeElement(TreeElement<T> *parent, const T& value );

    const TreeElement* parent() const               { return _parent; }
    const T& value() const                          { return _value; }
    T& value()                                      { return _value; }
    const std::vector<TreeElement>& children()const { return _children; }
    std::vector<TreeElement>& children()            { return _children; }

    TreeElement& addChild(const T& child );

    std::string toStr() const;

private:
    TreeElement*             _parent;
    T                        _value;
    boost::container::stable_vector<TreeElement> _children;
};


template <typename T> class Tree
{
public:
    Tree(): _root(nullptr,"root") {}

    /**
     * Add the elements to the tree and return the pointer to the leaf.
     * The leaf correspnds to the last element of concatenated_values in the Tree.
     */
    template<typename Vect> const TreeElement<T>* append(const Vect& concatenated_values);

    /**
     * Find a set of elements in the tree and return the pointer to the leaf.
     * The first element of the concatenated_values should be a root of the Tree.
     * The leaf corresponds to the last element of concatenated_values in the Tree.
     */
    template<typename Vect> const TreeElement<T>* find( const Vect& concatenated_values, bool partial_allowed = false);

    TreeElement<T>* root() { return &_root; }

private:

    friend std::ostream& operator<<(std::ostream& os, const Tree& _this)
    {
        _this.print_impl(os, &(_this._root.children()), 0);
        return os;
    }


    void print_impl(std::ostream& os, const std::vector<TreeElement<T>> *children, int indent ) const;

    TreeElement<T> _root;
};

//-----------------------------------------


template <typename T> inline
void Tree<T>::print_impl(std::ostream &os, const std::vector<TreeElement<T>> *children, int indent) const
{
    for (const auto& child: (*children))
    {
        for (int i=0; i<indent; i++)
        {
            os << " ";
        }
        os << child.value() << std::endl;
        print_impl(os, &(child.children()), indent+3);
    }
}

template <typename T> inline
TreeElement<T>::TreeElement(TreeElement<T> *parent, const T& value):
    _parent(parent), _value(value)
{

}

template <typename T> inline
std::string TreeElement<T>::toStr() const
{
    const TreeElement<T>* node = this;
    std::vector<const TreeElement<T>*> vect;
    vect.push_back( node );

    while( node->parent() != nullptr )
    {
        node = node->parent();
        vect.push_back( node );
    }

    std::string out;
    if( vect.size() > 0)
    {
        for (int i = vect.size() -1; i >= 0 ; i--)
        {
            const auto& name = vect[i]->value();
            out.append( name.data(), name.size() );
            if( i != 0) out.append(".");
        }
    }
    return out;
}

template <typename T> inline
TreeElement<T>& TreeElement<T>::addChild(const T& value)
{
    _children.push_back( TreeElement<T>(this, value));
    return _children.back();
}


template <typename T> template<typename Vect> inline
const TreeElement<T>* Tree<T>::append(const Vect &concatenated_values)
{
    TreeElement<T>* node = &_root;

    for (const auto& value: concatenated_values)
    {
        bool found = false;
        for (auto& child: (node->children() ) )
        {
            if( child.value() == value)
            {
                node = &(child);
                found = true;
                break;
            }
        }
        if(!found){
            node = &( node->addChild( value ) );
        }
    }
    return node;
}

template <typename T> template<typename Vect> inline
const TreeElement<T> *Tree<T>::find(const Vect& concatenated_values, bool partial_allowed )
{
    TreeElement<T>* node = &_root;

    for (const auto& value: concatenated_values)
    {
        bool found = false;
        for (auto& child: (node->children() ) )
        {
            if( child.value() == value)
            {
                node = &(child);
                found = true;
                break;
            }
        }
        if( !found ) return nullptr;
    }

    if( partial_allowed || node->children().empty() )
    {
        return  node;
    }
    return nullptr;
}

}

typedef details::TreeElement<ssoX::basic_string<char>> StringElement;
typedef details::Tree<ssoX::basic_string<char>> StringTree;


#endif // STRINGTREE_H
