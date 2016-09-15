#include <ros_type_introspection/renamer.hpp>

namespace RosIntrospection{

void SubstitutionRule::split_boost_ref(boost::string_ref& s,
                                       std::vector<ShortString>& pre,
                                       std::vector<ShortString>& post)
{
    size_t pos;
    bool is_previous = true;
    while (true) {
        pos = s.find_first_of('.');

        boost::string_ref str_item = s.substr(0,pos);
        if(str_item.compare("#") == 0)
        {
            is_previous = false;
        }
        else{
            if( is_previous )
                pre.push_back( str_item.to_string() );
            else
                post.push_back( str_item.to_string() );
        }
        if (pos == boost::string_ref::npos) {
            break;
        }
        s.remove_prefix(pos + 1);
    }
}

SubstitutionRule::SubstitutionRule(boost::string_ref pattern, boost::string_ref location, boost::string_ref substitution)
{
    split_boost_ref( pattern,      pattern_pre,      pattern_suf );
    split_boost_ref( location,     location_pre,     location_suf );
    split_boost_ref( substitution, substitution_pre, substitution_suf );
}


const StringElement* FindPatternTail( const ShortString& value,
                                      const StringElement* tail)
{

    std::cout <<  tail->value() << std::endl;
    if( tail->value() == value)
    {
        return tail;
    }

    for (const auto& child: tail->children() )
    {
        auto ret = FindPatternTail(value, &child);
        if( ret )
        {
            return ret;
        }
    }
    return nullptr;
}


bool FindPattern( const std::vector<ShortString>& pattern,  size_t index,
                  const StringElement* tail,
                  std::vector<const  StringElement*>& heads )
{
    if( index == pattern.size()){
        heads.push_back( tail );
        std::cout << " HEAD " << tail->value() << " HEAD " << std::endl;
        index = 0;
    }

    for (auto& child: tail->children() )
    {
        if( child.value() == pattern[index])
        {
            std::cout << child.value() << " match " << std::endl;
            index++;
         }
        else{
            index = 0;
            std::cout << child.value() << " DONT " << std::endl;
        }
        FindPattern( pattern, index, &child, heads);
    }
    return false;
}

StringElement* applyNameTransform( const std::vector< SubstitutionRule >&  rules,
                                   const StringElement* element,
                                   const StringTree& tree, StringTree* new_tree)
{


    return nullptr;
}

} //end namespace
