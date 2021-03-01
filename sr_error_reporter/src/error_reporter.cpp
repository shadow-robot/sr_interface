/*
* Copyright (C) 2021 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#include <ros/ros.h>
#include "UnderactuationErrorReporter.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "error_reporter");
  ros::NodeHandle node_handle;
  UnderactuationErrorReporter underactuation_error_reporter(node_handle);
  ros::spin();
  return 0;
}
