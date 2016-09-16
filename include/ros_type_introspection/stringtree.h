#ifndef STRINGTREE_H
#define STRINGTREE_H

#include <vector>
#include <deque>
#include <iostream>
#include <boost/container/stable_vector.hpp>
#include <ros_type_introspection/string.hpp>

namespace details{

#define STATIC_TREE true

/**
 * @brief Element of the tree. it has a single parent and N >= 0 children.
 */
template <typename T> class TreeElement{

public:
#if !STATIC_TREE
    typedef boost::container::stable_vector<TreeElement> ChildrenVector;
#else
    typedef std::vector<TreeElement> ChildrenVector; // dangerous beacause of pointer invalidation (but faster)
#endif

    TreeElement(const TreeElement* parent, const T& value );

    const TreeElement* parent() const       { return _parent; }
    const T& value() const                  { return _value; }
    T& value()                              { return _value; }
    const ChildrenVector& children()const   { return _children; }
    ChildrenVector& children()              { return _children; }

    void addChild(const T& child );

    std::string toStr() const;

private:
    const TreeElement*   _parent;
    T              _value;
    ChildrenVector _children;
};


template <typename T> class Tree
{
public:
    Tree(): _root(nullptr,"root") {}

    /**
     * Add the elements to the tree and return the pointer to the leaf.
     * The leaf correspnds to the last element of concatenated_values in the Tree.
     */
#if !STATIC_TREE // this operation is illegal in a static tree
    template<typename Vect> void insert(const Vect& concatenated_values);
#endif
    /**
     * Find a set of elements in the tree and return the pointer to the leaf.
     * The first element of the concatenated_values should be a root of the Tree.
     * The leaf corresponds to the last element of concatenated_values in the Tree.
     */
    template<typename Vect> const TreeElement<T>* find( const Vect& concatenated_values, bool partial_allowed = false);

    const TreeElement<T>* croot() const { return &_root; }
    TreeElement<T>* root() { return &_root; }

private:

    friend std::ostream& operator<<(std::ostream& os, const Tree& _this)
    {
        _this.print_impl(os, (_this._root.children()), 0);
        return os;
    }


    void print_impl(std::ostream& os, const typename TreeElement<T>::ChildrenVector& children, int indent ) const;

    TreeElement<T> _root;
};

//-----------------------------------------

template <typename T> inline
std::ostream& operator<<(std::ostream &os, const std::pair<const TreeElement<T>*, const TreeElement<T>* >& tail_head )
{
    const TreeElement<T>* tail = tail_head.first;
    const TreeElement<T>* head = tail_head.second;

    if( !head ) return os;

    const TreeElement<T>* array[64];
    int index = 0;
    array[index++] = head;

    while( !head || head != tail)
    {
        head = head->parent();
        array[index++] = head;
    };
    array[index] = nullptr;
    index--;

    while ( index >=0)
    {
        if( array[index] ){
            os << array[index]->value();
        }
        if( index >0 )  os << ".";
        index--;
    }
    return os;
}

template <typename T> inline
void Tree<T>::print_impl(std::ostream &os, const typename TreeElement<T>::ChildrenVector& children, int indent) const
{
    for (const auto& child: children)
    {
        for (int i=0; i<indent; i++) os << " ";
        os << child.value();
        if( child.parent())
          std::cout << "("<< child.parent()->value() << ")" << std::endl;
        else
          std::cout << "(null)" << std::endl;
        print_impl(os, child.children(), indent+3);
    }
}

template <typename T> inline
TreeElement<T>::TreeElement(const TreeElement *parent, const T& value):
    _parent(parent), _value(value)
{
  //std::cout << "ctor " << value << " <- ";
 // if( !parent ) std::cout << "NULL" << std::endl;
 // else std::cout << parent->value() << std::endl;
}
/*
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
}*/

template <typename T> inline
void TreeElement<T>::addChild(const T& value)
{
    _children.push_back( TreeElement<T>(this, value));
}

#if !STATIC_TREE
template <typename T> template<typename Vect> inline
void Tree<T>::insert(const Vect &concatenated_values)
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
            node->addChild( value );
            node = &(node->children().back());
        }
    }
}
#endif

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



#endif // STRINGTREE_H
