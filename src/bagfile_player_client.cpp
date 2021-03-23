/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2021, Yuki Endo.
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

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <bagfile_player/BagPlayAction.h>

typedef actionlib::SimpleActionClient<bagfile_player::BagPlayAction> Client;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bagfile_player_client");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string bag_file_path;
    private_nh.getParam("bag_file_path", bag_file_path);

    Client client("task", true);
    client.waitForServer();

    ROS_INFO("sending action to rosbag player %s", bag_file_path.c_str());
    if (client.isServerConnected())
    {
        bagfile_player::BagPlayGoal goal;
        goal.bag_file_path = bag_file_path;
        client.sendGoal(goal);
    }

    ros::spin();

    return 0;
}