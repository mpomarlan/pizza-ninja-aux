/*
* Copyright (C) 2015, 2016 Jannik Buckelo <jannikbu@cs.uni-bremen.de>,
* Georg Bartels <georg.bartels@cs.uni-bremen.de>
*
*
* This file is part of giskard_examples.
*
* giskard_examples is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <yaml-cpp/yaml.h>
#include <giskard/giskard.hpp>

#include <pizzaninja_msgs/SetEnable.h>
#include <pizzaninja_msgs/SegGoal.h>

#include <giskard_examples/GiskardControllerNode.hpp>

void segGoalParser(const pizzaninja_msgs::SegGoal::ConstPtr& msg, std::vector<double> &values)
{
    ROS_INFO("New goal: [%f, %f, %f] -> [%f, %f, %f]", msg->sx, msg->sy, msg->sz, msg->ex, msg->ey, msg->ez);

    values.resize(6);
    values[0] = msg->sx;
    values[1] = msg->sy;
    values[2] = msg->sz;
    values[3] = msg->ex;
    values[4] = msg->ey;
    values[5] = msg->ez;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pr2_seggoalcontroller");
  gcn::GiskardControllerNode<6, typename pizzaninja_msgs::SegGoal> segGoalController(segGoalParser);
  if(!segGoalController.isInitialized())
  {
      ROS_ERROR("Couldn't initialize controller");
      return 0;
  }
  ros::spin();

  return 0;
}
