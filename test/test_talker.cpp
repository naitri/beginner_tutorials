/******************************************************************************
 * MIT License
 * 
 * Copyright (c) 2021 Naitri Rajyaguru
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 ******************************************************************************/


/**
 * @file talker.cpp
 * @author Naitri Rajyaguru
 * @brief publishing simple custom string using service
 * @version 0.1
 * @date 2021-11-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <beginner_tutorials/update.h>

#include <ros/service_client.h>

#include <sstream>
#include <gtest/gtest.h>


std::shared_ptr<ros::NodeHandle> n;

/**
 * @brief Test to check update string service
 * 
 */

TEST(ROSPublisherTest, testServiceClient) {
    ros::ServiceClient client = n->serviceClient<beginner_tutorials::update>("update");
    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    beginner_tutorials::update serv;
    serv.request.input = "testing_service";
    client.call(serv);
    EXPECT_EQ(serv.response.output, serv.request.input);


}


int main(int argc,
         char **argv)
{
  ros::init(argc, argv, "tests");
  n.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

