/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright 2016 Davide Faconti
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

#ifndef STRINGTREE_H
#define STRINGTREE_H

#include <vector>
#include <deque>
#include <iostream>
#include <boost/container/stable_vector.hpp>
#include <boost/noncopyable.hpp>
#include "ros_type_introspection/string.hpp"

namespace RosIntrospection {

namespace details{

// If you set this to true, Tree can not be modified
// once it is created. The main reason is that std::vector will be used to store
// the children. this is faster but might invalidate the pointer to the node's parent.
#define STATIC_TREE true

/**
 * @brief Element of the tree. it has a single parent and N >= 0 children.
 */
template <typename T> class TreeElement
{

public:
#if !STATIC_TREE
    typedef boost::container::stable_vector<TreeElement> ChildrenVector;
#else
    typedef std::vector<TreeElement> ChildrenVector; // dangerous because of pointer invalidation (but faster)
#endif

    TreeElement(const TreeElement* parent, const T& value );

    const TreeElement* parent() const       { return _parent; }

    const T& value() const                  { return _value; }
    T& value()                              { return _value; }

    const ChildrenVector& children()const   { return _children; }
    ChildrenVector& children()              { return _children; }

    void addChild(const T& child );

    bool isLeaf() const { return _children.empty(); }

private:
    const TreeElement*   _parent;
    T              _value;
    ChildrenVector _children;
};


template <typename T> class Tree : boost::noncopyable
{
public:
    Tree(): _root(nullptr,"root") {}

#if !STATIC_TREE // this operation is illegal in a static tree
    /**
     * Add the elements to the tree and return the pointer to the leaf.
     * The leaf correspnds to the last element of concatenated_values in the Tree.
     */

    template<typename Vect> void insert(const Vect& concatenated_values);
#endif

    /**
     * Find a set of elements in the tree and return the pointer to the leaf.
     * The first element of the concatenated_values should be a root of the Tree.
     * The leaf corresponds to the last element of concatenated_values in the Tree.
     */
    template<typename Vect> const TreeElement<T>* find( const Vect& concatenated_values, bool partial_allowed = false);

    /// Constant pointer to the root of the tree.
    const TreeElement<T>* croot() const { return &_root; }

    /// Mutable pointer to the root of the tree.
    TreeElement<T>* root() { return &_root; }


    friend std::ostream& operator<<(std::ostream& os, const Tree& _this){
        _this.print_impl(os, (_this._root.children()), 0);
        return os;
    }

private:

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

}

template <typename T> inline
void TreeElement<T>::addChild(const T& value)
{
    //skip existing child
    for (int i=0; i< _children.size(); i++){
      if( value == _children[i].value() ){
        return;
      }
    }
#if STATIC_TREE
   assert(_children.capacity() > _children.size() );
#endif
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

}



#endif // STRINGTREE_H
