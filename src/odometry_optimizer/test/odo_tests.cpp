#include <gtest/gtest.h>
#include "../ISAMOptimizer.h"

TEST(ItWorks, itWorksCase1)
{
    ros::Publisher pub; // Dummy publisher
    ISAMOptimizer isamOptimizer(&pub);
    isamOptimizer.recv
}