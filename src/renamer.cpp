#include <ros_type_introspection/renamer.hpp>

namespace RosIntrospection{


bool PatternMatch(const StringElement* head,
                  const StringElement* node_ptr );

const SString* FindSubstitutionName(const StringElement* pattern_head,
                                    const ROSTypeFlat& container,
                                    const StringTreeLeaf& leaf );

//--------------------------------------

const SString* FindSubstitutionName(const StringElement* pattern_head,
                                    const ROSTypeFlat& container,
                                    const StringTreeLeaf& leaf )
{
    for(auto& it: container.name_id)
    {
        const StringTreeLeaf& name_leaf = it.first;
        bool arrays_eq = true;
        if( name_leaf.array_size != leaf.array_size) arrays_eq = false;
        for (int i=0; i <name_leaf.array_size && arrays_eq; i++)
        {
            arrays_eq = ( name_leaf.index_array[i] == leaf.index_array[i] );
        }

        if( arrays_eq ) // array part is ok, check the pattern
        {
            if( PatternMatch( pattern_head, name_leaf.node_ptr) ) {
                //std::cout << " substritution: " << it.second;
                return &(it.second);
            }
        }
    }
    return nullptr;
}

int find_pattern_count = 0;

inline bool FindPattern( const std::vector<SString>& pattern,  size_t index,
                         const StringElement* tail,
                         const  StringElement** head )
{
    if(  tail->children().empty() && index < pattern.size() -1)
    {
        return false;
    }

    //  find_pattern_count++;

    //  std::cout<< tail->value() << " == " << pattern[index] << std::endl;
    if( tail->value() == pattern[index])
    {
        index++;
    }
    else{
        if( index > 0 ){
            FindPattern( pattern, 0, tail, head);
            return false;
        }
        index = 0;
    }

    if( index == pattern.size()){
        *head = ( tail );
        return true;
    }

    bool found = false;

    for (auto& child: tail->children() ) {

        found = FindPattern( pattern, index, &child, head);
        if( found) break;
    }
    return found;
}

StringElement* applyNameTransform( const std::vector< SubstitutionRule >&  rules,
                                   const StringElement* element,
                                   const StringTree& tree, StringTree* new_tree)
{


    return nullptr;
}

bool PatternMatch(const StringElement* pattern_head, const StringElement *node_ptr)
{
    while( node_ptr ) {
        if( node_ptr == pattern_head ) {
            return true;
        }
        node_ptr = node_ptr->parent();
    }
    return false;
}

void applyNameTransform(const std::vector<SubstitutionRule>& rules,
                        ROSTypeFlat* container )
{

    const bool debug = false ;

    if( debug) std::cout << container->tree << std::endl;

    container->renamed_value.resize( container->value.size() );

    std::vector<uint8_t> substituted( container->value.size() );
    for(auto& sub: substituted) { sub = false; }

    size_t renamed_index = 0;

    for(const auto& rule: rules)
    {
        const StringElement* pattern_head = nullptr;
        const StringElement* location_head = nullptr;

        FindPattern( rule.pattern, 0, container->tree.croot(), &pattern_head );
        FindPattern( rule.location,0,  container->tree.croot(), &location_head );

        if( !pattern_head || !location_head) break;

        for(size_t i=0; i< container->value.size(); i++)
        {
            if( substituted[i]) continue;

            const auto& value_leaf = container->value[i];

            if( debug) std::cout << value_leaf.first << " >> " << value_leaf.second << " ... ";

            auto& leaf = value_leaf.first;
            if( PatternMatch( pattern_head, leaf.node_ptr))
            {
                if( debug) std::cout << " match pattern... ";

                const SString* new_name = FindSubstitutionName(location_head, *container, leaf );
                if( new_name )
                {
                    if( debug) std::cout << " CONCATENATION\n";
                    int char_count = 0;
                    std::vector<const SString*> concatenated_name;
                    concatenated_name.reserve( 10 );

                    const StringElement* node_ptr = leaf.node_ptr;
                    while( node_ptr != pattern_head)
                    {
                        const SString* value = &node_ptr->value();
                        concatenated_name.push_back( value );
                        char_count += value->size();
                        if( debug) std::cout << "A: " << *value << std::endl;;
                        node_ptr = node_ptr->parent();
                    }

                    for (int i = rule.substitution.size()-1; i >= 0; i--)
                    {
                        const SString* value = &rule.substitution[i];

                        if( value->size()==1 && value->at(0) == '#')
                        {
                            value = new_name;
                        }

                        concatenated_name.push_back( value );
                        char_count += value->size();
                        if( debug) std::cout << "B: " << *value << std::endl;;
                    }

                    for (int i = 0; i < rule.pattern.size() && node_ptr; i++)
                    {
                        node_ptr = node_ptr->parent();
                    }

                    while( node_ptr )
                    {
                        const SString* value = &node_ptr->value();
                        concatenated_name.push_back( value );
                        char_count += value->size();
                        if( debug) std::cout << "C: " << *value << std::endl;;
                        node_ptr = node_ptr->parent();
                    }

                    //------------------------
                    SString new_identifier;
                    new_identifier.reserve( char_count + concatenated_name.size() + 1 );

                    for (int i = concatenated_name.size()-1; i >= 0; i--)
                    {
                        new_identifier.append( *concatenated_name[i] );
                        if( i>0 ) new_identifier.append(".");
                    }
                    if( debug) std::cout << "Result: " << new_identifier << std::endl;

                    container->renamed_value[ renamed_index ].first = std::move( new_identifier );
                    container->renamed_value[renamed_index].second  =  value_leaf.second ;
                    renamed_index++;
                    substituted[i] = true;
                }
                if( debug) std::cout << std::endl;
            } // end if( PatternMatch )
        } // end for values
    } // end for rules

    for(size_t i=0; i< container->value.size(); i++)
    {
        if( substituted[i] == false)
        {
            const auto& value_leaf = container->value[i];
            container->renamed_value[renamed_index].first  =  value_leaf.first.toStr();
            container->renamed_value[renamed_index].second =  value_leaf.second ;
            renamed_index++;
        }
    }
}




} //end namespace
