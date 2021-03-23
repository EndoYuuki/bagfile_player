# Bag File Player
- A server to play a bag file
- You can play a bag file at your good time (e.g. after initialization)

# Requirement
- ROS
- the rosbag package

# Build
```catkin build bagfile_player```

# Sample
Check `bagfile_player_client.cpp` in the source directory.

```roslaunch sample.launch node_name:=NODE_NAME bag_file_path:=BAG_FILE_PATH```

# Author
- Yuki Endo, endou@kmj.iis.u-tokyo.ac.jp

# License
This repository is under[BSD License 2.0 (3-clause, New or Revised) License](https://opensource.org/licenses/BSD-3-Clause)