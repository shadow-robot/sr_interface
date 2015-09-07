// Copyright Guillaume Walck <Guillaume Walck>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <moveit_msgs/GetPositionIK.h>

#include <map>
#include <string>
#include <vector>


#include <std_msgs/Float64.h>
#include <pr2_mechanism_msgs/ListControllers.h>

#define WAITTIME 15000000

std::map <std::string, std::string> jointControllerMap;
std::map<std::string, unsigned int> jointPubIdxMap;
ros::Publisher pub[5];
ros::ServiceClient ik_client;
moveit_msgs::GetPositionIK::Request gpik_req;
moveit_msgs::GetPositionIK::Response gpik_res;

moveit_msgs::GetKinematicSolverInfo::Request request;
moveit_msgs::GetKinematicSolverInfo::Response response;

// linear interpolation
// q must be included in [0.0 , 1.0]
tf::Vector3 interPolLin(tf::Vector3 start, tf::Vector3 end, float q)
{
  tf::Vector3 pos = tf::Vector3(0, 0, 0);
  if (q >= 0.0 && q <= 1.0)
  {
    pos = (1 - q) * start + q * end;

  }
  return pos;
}

// circle generation
// center and radius define the circle size
// direction is 1=>x, 2=>y, 3=>z (TO DO make it a direction vector)
// ang must be included in [0.0 , 2pi]
tf::Vector3 interPolCir(tf::Vector3 center, unsigned int direction, float radius, float ang)
{
  tf::Vector3 pos = tf::Vector3(0, 0, 0);
  if (ang >= 0.0 && ang <= 2 * M_PI)
  {
    switch (direction)
    {
      case 1:  // around x axis (  // to J3 axis)
        pos = center + tf::Vector3(0, radius * cos(ang), radius * sin(ang));
        break;
      case 2:  // around y axis (  // to J4 axis)
        pos = center + tf::Vector3(radius * cos(ang), 0, radius * sin(ang));
        break;
      case 3:  // around z axis (  // along finger length axis when straight at J3=0)
        pos = center + tf::Vector3(radius * cos(ang), radius * sin(ang), 0);
        break;
    }
  }

  return pos;
}

// get IK for x,y,z and send joint pos to joint pos controllers
int moveIK(float x, float y, float z)
{
  gpik_req.ik_request.pose_stamped.pose.position.x = x;
  gpik_req.ik_request.pose_stamped.pose.position.y = y;
  gpik_req.ik_request.pose_stamped.pose.position.z = z;

  if (ik_client.call(gpik_req, gpik_res))
  {
    if (gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
    {
      std_msgs::Float64 message;
      for (unsigned int i = 0; i < gpik_res.solution.joint_state.name.size(); i++)
      {
        ROS_DEBUG("Joint: %s %f", gpik_res.solution.joint_state.name[i].c_str(),
                  gpik_res.solution.joint_state.position[i]);
        ROS_DEBUG("we publish to %s", pub[i].getTopic().c_str());

        message.data = static_cast<double>(gpik_res.solution.joint_state.position[i]);

        pub[jointPubIdxMap[gpik_res.solution.joint_state.name[i]]].publish(message);
      }
      ros::spinOnce();
      return 0;
    }
    else
    {
      ROS_ERROR("Inverse kinematics failed");
    }
    return -1;
  }
  else
  {
    ROS_ERROR("Inverse kinematics service call failed");
  }
  return -1;
}

// get IK for Vector3 in_vec and send joint pos to joint pos controllers
int moveIK(tf::Vector3 in_vec)
{
  return moveIK(in_vec.x(), in_vec.y(), in_vec.z());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "square_ik");
  ros::NodeHandle rh;

  // prepare services
  ros::service::waitForService("th_kinematics/get_ik_solver_info");
  ros::service::waitForService("th_kinematics/get_ik");

  ros::ServiceClient query_client = rh.serviceClient<moveit_msgs::GetKinematicSolverInfo>(
          "th_kinematics/get_ik_solver_info");
  ik_client = rh.serviceClient<moveit_msgs::GetPositionIK>("th_kinematics/get_ik");

  ros::service::waitForService("pr2_controller_manager/list_controllers");
  ros::ServiceClient controller_list_client = rh.serviceClient<pr2_mechanism_msgs::ListControllers>(
          "pr2_controller_manager/list_controllers");

  // init treated joint (to be modified to get more generic behaviour)
  std::string controlled_joint_name;
  std::vector <std::string> joint_name;
  joint_name.push_back("THJ1");
  joint_name.push_back("THJ2");
  joint_name.push_back("THJ3");
  joint_name.push_back("THJ4");
  joint_name.push_back("THJ5");

  // init jointControllerMapping
  pr2_mechanism_msgs::ListControllers controller_list;
  controller_list_client.call(controller_list);
  for (unsigned int i = 0; i < controller_list.response.controllers.size(); i++)
  {
    if (rh.getParam("/" + controller_list.response.controllers[i] + "/joint", controlled_joint_name))
    {
      ROS_DEBUG("controller %d:%s controlls joint %s\n", i, controller_list.response.controllers[i].c_str(),
                controlled_joint_name.c_str());
      jointControllerMap[controlled_joint_name] = controller_list.response.controllers[i];
    }
  }

  // get possible computable joints
  if (query_client.call(request, response))
  {
    for (unsigned int i = 0; i < response.kinematic_solver_info.joint_names.size(); i++)
    {
      ROS_DEBUG("Joint: %d %s", i, response.kinematic_solver_info.joint_names[i].c_str());
    }
  }
  else
  {
    ROS_ERROR("Could not call query service");
    ros::shutdown();
    exit(1);
  }

  // define the service messages
  gpik_req.ik_request.timeout = ros::Duration(5.0);
  gpik_req.ik_request.ik_link_name = "thtip";
  gpik_req.ik_request.pose_stamped.header.frame_id = "palm";
  gpik_req.ik_request.pose_stamped.pose.position.x = 0;
  gpik_req.ik_request.pose_stamped.pose.position.y = 0;
  gpik_req.ik_request.pose_stamped.pose.position.z = 0;

  // pos is not relevant with the used ik_solver but set it anyway to identity
  gpik_req.ik_request.pose_stamped.pose.orientation.x = 0.0;
  gpik_req.ik_request.pose_stamped.pose.orientation.y = 0.0;
  gpik_req.ik_request.pose_stamped.pose.orientation.z = 0.0;
  gpik_req.ik_request.pose_stamped.pose.orientation.w = 1.0;
  gpik_req.ik_request.robot_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
  gpik_req.ik_request.robot_state.joint_state.name = response.kinematic_solver_info.joint_names;


  // prepare the publishers
  for (unsigned int i = 0; i < joint_name.size(); i++)
  {
    pub[i] = rh.advertise<std_msgs::Float64>("/" + jointControllerMap[joint_name[i]] + "/command", 2);
    jointPubIdxMap[joint_name[i]] = i;
  }

  ros::spinOnce();
  sleep(1);  // this is required otherwise publishers are not ready for first messages to be sent


  // do a linear forware and backward movement in reachable domain of the FF finger
  tf::Vector3 PointD = tf::Vector3(0.05, -0.065, 0.10);
  tf::Vector3 PointE = tf::Vector3(0.02, -0.065, 0.10);
  tf::Vector3 PointF = tf::Vector3(0.02, -0.065, 0.07);
  tf::Vector3 PointG = tf::Vector3(0.05, -0.065, 0.07);

  tf::Vector3 curpos;
  moveIK(PointD);
  usleep(WAITTIME);
  unsigned int steps = 50;
  for (unsigned int i = 0; i < steps; i++)
  {
    if (!ros::ok())
    {
      break;
    }
    curpos = interPolLin(PointD, PointE, (static_cast<float>(i)) / steps);
    moveIK(curpos);
    usleep(WAITTIME / steps);
  }
  for (unsigned int i = 0; i < steps; i++)
  {
    if (!ros::ok())
    {
      break;
    }
    curpos = interPolLin(PointE, PointF, (static_cast<float>(i)) / steps);
    moveIK(curpos);
    usleep(WAITTIME / steps);
  }
  for (unsigned int i = 0; i < steps; i++)
  {
    if (!ros::ok())
    {
      break;
    }
    curpos = interPolLin(PointF, PointG, (static_cast<float>(i)) / steps);
    moveIK(curpos);
    usleep(WAITTIME / steps);
  }
  for (unsigned int i = 0; i < steps; i++)
  {
    if (!ros::ok())
    {
      break;
    }
    curpos = interPolLin(PointG, PointD, (static_cast<float>(i)) / steps);
    moveIK(curpos);
    usleep(WAITTIME / steps);
  }
  ros::shutdown();
}


