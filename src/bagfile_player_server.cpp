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
#include <rosbag/player.h>
#include <boost/program_options.hpp>

#include <memory>

#include <actionlib/server/simple_action_server.h>
#include <bagfile_player/BagPlayAction.h>

typedef actionlib::SimpleActionServer<bagfile_player::BagPlayAction> Server;

namespace po = boost::program_options;

rosbag::PlayerOptions parseOptions(int argc, char** argv) {
    rosbag::PlayerOptions opts;

    po::options_description desc("Allowed options");

    desc.add_options()
      ("help,h", "produce help message")
      ("prefix,p", po::value<std::string>()->default_value(""), "prefixes all output topics in replay")
      ("quiet,q", "suppress console output")
      ("immediate,i", "play back all messages without waiting")
      ("pause", "start in paused mode")
      ("queue", po::value<int>()->default_value(100), "use an outgoing queue of size SIZE")
      ("clock", "publish the clock time")
      ("hz", po::value<float>()->default_value(100.0f), "use a frequency of HZ when publishing clock time")
      ("delay,d", po::value<float>()->default_value(0.2f), "sleep SEC seconds after every advertise call (to allow subscribers to connect)")
      ("rate,r", po::value<float>()->default_value(1.0f), "multiply the publish rate by FACTOR")
      ("start,s", po::value<float>()->default_value(0.0f), "start SEC seconds into the bag files")
      ("duration,u", po::value<float>(), "play only SEC seconds from the bag files")
      ("skip-empty", po::value<float>(), "skip regions in the bag with no messages for more than SEC seconds")
      ("loop,l", "loop playback")
      ("keep-alive,k", "keep alive past end of bag (useful for publishing latched topics)")
      ("try-future-version", "still try to open a bag file, even if the version is not known to the player")
      ("topics", po::value< std::vector<std::string> >()->multitoken(), "topics to play back")
      ("pause-topics", po::value< std::vector<std::string> >()->multitoken(), "topics to pause playback on");
    
    po::positional_options_description p;
    po::variables_map vm;
    
    try 
    {
      po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    } catch (boost::program_options::invalid_command_line_syntax& e)
    {
      throw ros::Exception(e.what());
    }  catch (boost::program_options::unknown_option& e)
    {
      throw ros::Exception(e.what());
    }

    if (vm.count("help")) {
      std::cout << desc << std::endl;
      exit(0);
    }

    if (vm.count("prefix"))
      opts.prefix = vm["prefix"].as<std::string>();
    if (vm.count("quiet"))
      opts.quiet = true;
    if (vm.count("immediate"))
      opts.at_once = true;
    if (vm.count("pause"))
      opts.start_paused = true;
    if (vm.count("queue"))
      opts.queue_size = vm["queue"].as<int>();
    if (vm.count("hz"))
      opts.bag_time_frequency = vm["hz"].as<float>();
    if (vm.count("clock"))
      opts.bag_time = true;
    if (vm.count("delay"))
      opts.advertise_sleep = ros::WallDuration(vm["delay"].as<float>());
    if (vm.count("rate"))
      opts.time_scale = vm["rate"].as<float>();
    if (vm.count("start"))
    {
      opts.time = vm["start"].as<float>();
      opts.has_time = true;
    }
    if (vm.count("duration"))
    {
      opts.duration = vm["duration"].as<float>();
      opts.has_duration = true;
    }
    if (vm.count("skip-empty"))
      opts.skip_empty = ros::Duration(vm["skip-empty"].as<float>());
    if (vm.count("loop"))
      opts.loop = true;
    if (vm.count("keep-alive"))
      opts.keep_alive = true;

    if (vm.count("topics"))
    {
      std::vector<std::string> topics = vm["topics"].as< std::vector<std::string> >();
      for (std::vector<std::string>::iterator i = topics.begin();
           i != topics.end();
           i++)
        opts.topics.push_back(*i);
    }

    if (vm.count("pause-topics"))
    {
      std::vector<std::string> pause_topics = vm["pause-topics"].as< std::vector<std::string> >();
      for (std::vector<std::string>::iterator i = pause_topics.begin();
           i != pause_topics.end();
           i++)
        opts.pause_topics.push_back(*i);
    }

    return opts;
}

rosbag::PlayerOptions opts_;
std::shared_ptr<rosbag::Player> player_ptr_;

void execute(const bagfile_player::BagPlayGoalConstPtr& goal, Server* as)
{
  opts_.bags[0] = goal->bag_file_path;
  ROS_INFO("input file: %s", opts_.bags[0].c_str());

  player_ptr_.reset(new rosbag::Player(opts_));

  // loop
  player_ptr_->publish();

  as->setSucceeded();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bagfile_player_server");
    ros::NodeHandle nh;

    // Parse the command-line options
    try {
        opts_ = parseOptions(argc, argv);
    }
    catch (ros::Exception const& ex) {
        ROS_ERROR("Error reading options: %s", ex.what());
        return 1;
    }

    opts_.bags.resize(1);
    Server server(nh, "task", boost::bind(&execute, _1, &server), false);
    server.start();

    ROS_INFO("service started");

    ros::spin();
  
    return 0;
}