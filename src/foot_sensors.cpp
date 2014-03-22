/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <sstream>
#include <Arduino.h>
#include <SPI.h>
#include <interrupt.h>

#define chipSelectPin 10


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
float read_adc(int adc_num) {
    int commandout = adc_num;
    commandout |= 0x18; // start bit _ single-ended
    digitalWrite(chipSelectPin, 0);
    SPI.transfer(commandout);
    int readval = SPI.transfer(0x00);
    int readval2 = SPI.transfer(0x00);
    digitalWrite(chipSelectPin, 1);

    int intval = (readval << 8)  | readval2;
    // real data starts with 1 null bit
    //cout << " sdc# " << adc_num;
    //cout << "   before shifting: " << std::hex << intval << "\n";
    while(intval & 0x8000) {
        intval <<= 1;
        intval &= 0xffff;
    }
    intval <<= 1;
    intval &= 0xffff;
    //cout << "   after shifting:off null bit: " << std::hex << intval << "\n";
    intval >>= 6;
    //cout << "   after shifting:back: " << std::hex << intval << "\n";

    float floatval = intval * (3300.0 / 1024.0) / 1000.0;
    return floatval;

}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "foot_seonsors");

/* Arduino main init code: */
  interrupt_init();

  // Call Arduino init
  init(argc, argv);


  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, 1);


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
  ros::Publisher foot_pub = n.advertise<std_msgs::Float32MultiArray>("foot", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;
    std_msgs::Float32MultiArray foot_array;

    foot_array.data.clear();
    std::stringstream ss;
    ss << "Feet:" << read_adc(0) << " " << read_adc(1) << " " << read_adc(2) << " " << read_adc(3) << " " << read_adc(4) << " " << read_adc(5) << " " << read_adc(6) << " " << read_adc(7) << " reading# " << count;
    for (int i = 0; i < 8; i++) {
        float f = read_adc(i);
        ss << " " << f;
        foot_array.data.push_back( f );
    }


    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);
    foot_pub.publish( foot_array );

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
