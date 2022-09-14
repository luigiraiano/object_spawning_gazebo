#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/LinkStates.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>

void linkStateCallback(const gazebo_msgs::LinkStates& msgs);

static tf::TransformBroadcaster _br;
static std::string _token;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "target_pose_publisher_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  if( !nh_priv.getParam("token", _token) )
  {
      _token = "target";
  }


  std::string sub_name{"/gazebo/link_states"}, pub_name{"/tf"};
  ros::Subscriber gazebo_link_state_sub = nh.subscribe(sub_name, 10, linkStateCallback);

  ros::spin();

  return 0;
}

void linkStateCallback(const gazebo_msgs::LinkStates& msgs)
{
  for(size_t i=0; i<msgs.name.size(); i++ )
  {
//    if( find(msgs.name.at(i), _token) != std::string::npos)
//    {
        // 1. get link pose
        // 2. send to tf
//    }
  }
}
