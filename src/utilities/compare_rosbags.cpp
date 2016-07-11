#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include <sstream>
#include <stdio.h>
#include <boost/foreach.hpp>

bool areBagsEqual(rosbag::View&  viewA,  rosbag::View&  viewB )
{
    std::vector<const rosbag::ConnectionInfo *> connection_A = viewA.getConnections();
    std::vector<const rosbag::ConnectionInfo *> connection_B = viewB.getConnections();

    // step 1: check if they have the same amount of topics

    if( connection_A.size() != connection_B.size() )
    {
        printf("The number of topics is different: %ld != %ld\n",
               connection_A.size(), connection_B.size() );
        return false;
    }

    for(int i=0;  i<  connection_A.size(); i++)
    {
        const std::string& tA = connection_A[i]->topic;
        const std::string& tB = connection_B[i]->topic;

        if( tA.compare( tB ) != 0)
        {
            printf("One of the topics name is different: %s != %s\n", tA.c_str(), tB.c_str() );
            return false;
        }
    }

    if( viewA.size() != viewB.size())
    {
        printf("The number of messages is different: %d != %d\n", viewA.size(), viewB.size() );
        return false;
    }
    //----------------------------------------------------
    // step 2: message by message comparison

    uint8_t bufferA[1024*5];
    uint8_t bufferB[1024*5];

    rosbag::View::iterator mA = viewA.begin();
    rosbag::View::iterator mB = viewB.begin();

    while(  mA != viewA.end() && mB != viewB.end() )
    {
        const std::string& tA = mA->getTopic();
        const std::string& tB = mB->getTopic();

        if( tA.compare( tB) != 0)
        {
            printf("One of the topics name is different: %s != %s\n", tA.c_str(), tB.c_str() );
            return false;
        }

        const size_t sA = mA->size();
        const size_t sB = mB->size();

        if( sA != sB )
        {
            printf("The size of a messages (%s) is different: %ld != %ld\n", tA.c_str(), sA, sB );
            return false;
        }

        ros::serialization::OStream streamA( bufferA, sizeof(bufferA) );
        ros::serialization::OStream streamB( bufferB, sizeof(bufferB) );
        mA->write( streamA );
        mB->write( streamB );

        for (size_t i=0; i< 8; i++)
        {
            if( bufferA[i] != bufferB[i])
            {
                printf("Content of a messages (%s) is not exactly the same", tA.c_str() );
                return false;
            }
        }

        mA++;
        mB++;
    }
    printf(" EQUAL!! \n");
    return true;
}


int main(int argc, char **argv)
{
    if( argc != 3 )
    {
        std::cout << "provide the name of two robags " << std::endl;
        return 1;
    }

    rosbag::Bag bag_A;
    bag_A.open( argv[1], rosbag::bagmode::Read );

    rosbag::Bag bag_B;
    bag_B.open( argv[2], rosbag::bagmode::Read );

    std::cout <<  argv[1] << " "<<  argv[2] << std::endl;

    rosbag::View view_A ( bag_A );
    rosbag::View view_B ( bag_B );

    areBagsEqual( view_A, view_B );

    printf(" DONE!! \n");

    return 0;
}
