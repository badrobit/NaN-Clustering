/*
 * TestRegionGrowingNaN.cpp
 *
 *  Created on: Feb 27, 2014
 *      Author: badrobit
 */

#include <gtest/gtest.h>

TEST(SquareRootTest, PositiveNos)
{
    ASSERT_EQ(6, squareRoot(36.0));
    ASSERT_EQ(18.0, squareRoot(324.0));
    ASSERT_EQ(25.4, squareRoot(645.16));
    ASSERT_EQ(0, squareRoot(0.0));
}
