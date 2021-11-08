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
#include <std_msgs/String.h>
#include <sstream>

/**
 * Import header file of service
 */
 #include <beginner_tutorials/update.h>

std::string message = "Software Development for Robotics";
int default_rate = 10;

/**
 * This tutorial demonstrates simple sending custom messages over the ROS system.
 */

/**
 * @brief Function to call service
 * @param request is input string to service
 * @param response is updated string by service
 * @return bool
 */
bool msg_modifier(beginner_tutorials::update::Request &request, beginner_tutorials::update::Response &response) {
  ROS_INFO_STREAM("Message modification service is called..");
  if(request.input.empty()) {
    ROS_ERROR_STREAM("Invalid empty message..modificationis denied");
  }

  else {
    ROS_WARN_STREAM("Message will be changed by publisher..");
    message = request.input;
    response.output = message;

    ROS_INFO_STREAM("The message is modified to.. " << response.output);
    return true;

  }
  

}


int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */

  ros::init(argc, argv, "talker");


  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */

  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  auto serv = n.advertiseService("update",msg_modifier);

  int rate;
  if (argc > 0) {
    ROS_DEBUG_STREAM("The change in rate will be:" << rate);
    rate = atoi(argv[1]);
  }

  
  if (rate == 0 ) {
    ROS_ERROR_STREAM("Rate cannot be zero");
    ROS_WARN_STREAM("Default rate will be set..");
    rate = default_rate;
  }

  else if (rate < 0) {
    ROS_FATAL_STREAM ("Rate cannot be non-negative");
    ROS_WARN_STREAM("Default rate will be set..");

    rate = default_rate;
  }

  else {
    ROS_INFO_STREAM("Rate is now changed to " << rate);
    rate = rate;
  }


  ros::Rate loop_rate(rate);


  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */

  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;
    std::stringstream ss;
    ss << message << count;
    msg.data = ss.str();
    ROS_INFO_STREAM(msg.data.c_str());


    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */

    chatter_pub.publish(msg);


    ros::spinOnce();



    loop_rate.sleep();

    ++count;
  }


  return 0;
}

