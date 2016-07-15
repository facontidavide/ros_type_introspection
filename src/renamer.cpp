#include <ros_type_introspection/renamer.hpp>

namespace ROSIntrospection{


SubstitutionRule::SubstitutionRule( std::string pattern, std::string location, std::string substitution):
    pattern_pre( pattern),
    pattern_suf( pattern),

    location_pre( location),
    location_suf( location),

    substitution_pre(substitution),
    substitution_suf(substitution)
{
    int pos;
    pos = pattern_pre.find_first_of('#');
    pattern_pre.erase( pos, pattern_pre.length() - pos );
    pattern_suf.erase( 0, pos+1 );

    pos = location_pre.find_first_of('#');
    location_pre.erase(pos, location_pre.length() - pos );
    location_suf.erase(0, pos+1 );

    pos = substitution_pre.find_first_of('#');
    substitution_pre.erase(pos, substitution_pre.length() - pos );
    substitution_suf.erase(0, pos+1 );
}


void applyNameTransform( const std::vector< SubstitutionRule >&  rules,
                         ROSTypeFlat* container)
{

    for (int index=0; index < container->value.size(); index++)
    {
        auto& value_name = container->value[index].first;
        boost::string_ref name ( value_name.data(), value_name.size());

        bool substitution_done = false;

        for (int r=0; r < rules.size(); r++)
        {
            const auto& rule = rules[r];

            int posA = name.find(rule.pattern_pre );
            if( posA == name.npos) { continue; }

            int posB = posA + rule.pattern_pre.length();
            int posC = posB;

            while( isdigit(  name.at(posC) ) && posC < name.npos)
            {
                posC++;
            }
            if( posC == name.npos) continue;

            boost::string_ref name_prefix = name.substr( 0, posA );
            boost::string_ref index       = name.substr(posB, posC-posB );
            boost::string_ref name_suffix = name.substr( posC, name.length() - posC );

            int res = std::strncmp( name_suffix.data(), rule.pattern_suf.data(),  rule.pattern_suf.length() );
            if( res != 0)
            {
                continue;
            }
            name_suffix.remove_prefix( rule.pattern_suf.length() );

            char key[256];
            int buffer_index = 0;
            for (const char c: name_prefix       )  key[buffer_index++] = c;
            for (const char c: rule.location_pre )  key[buffer_index++] = c;
            for (const char c: index             )  key[buffer_index++] = c;
            for (const char c: rule.location_suf )  key[buffer_index++] = c;
            key[buffer_index] = '\0';

            auto substitutor = std::lower_bound( container->name_id.begin(),
                                                 container->name_id.end(),
                                                 LongString(key),
                                                 []( const std::pair<LongString,LongString> item, const LongString& key )  { return item.first < key; }
            );

            if( substitutor != container->name_id.end())
            {
                auto& index_replacement = substitutor->second;

                char new_name[256];
                int name_index = 0;
                for (const char c: name_prefix           )  new_name[name_index++] = c;
                for (const char c: rule.substitution_pre )  new_name[name_index++] = c;

                for (int i=0; i< index_replacement.size(); i++ )
                    new_name[name_index++] = index_replacement.at(i);

                for (const char c: rule.substitution_suf )  new_name[name_index++] = c;
                for (const char c: name_suffix           )  new_name[name_index++] = c;
                new_name[name_index] = '\0';

                value_name = LongString(new_name);

                /*std::cout << "---------------" << std::endl;
                std::cout << "index        " << index << std::endl;
                std::cout << "name_prefix  " << name_prefix << std::endl;
                std::cout << "key  " << key << std::endl;
                std::cout << "new_name   " << new_name << std::endl;
                std::cout << "---------------" << std::endl;
                */
                // DON'T apply more than one rule
                substitution_done = true;
                break;
            }
        }
    }

}

} //end namespace
