#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/LinkStates.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>

void linkStateCallback(const gazebo_msgs::LinkStates& msgs);

static std::shared_ptr<tf::TransformBroadcaster> _br;
static std::string _token;
static std::vector<size_t> _target_idx;
static std::string _tf_target_name{"target"};
static std::string _world_name{"world"};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "target_pose_publisher_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  if( !nh_priv.getParam("token", _token) )
  {
      _token = "target";
  }

  if( !nh_priv.getParam("tf_target_name", _tf_target_name) )
  {
      _tf_target_name = "target";
  }

  if( !nh_priv.getParam("world_name", _world_name) )
  {
      _world_name = "world";
  }

  std::string sub_name{"/gazebo/link_states"};
  ros::Subscriber gazebo_link_state_sub = nh.subscribe(sub_name, 10, linkStateCallback);
  _br.reset(new tf::TransformBroadcaster);

  ros::spin();

  return 0;
}

void linkStateCallback(const gazebo_msgs::LinkStates& msgs)
{
  // Find target using token
  _target_idx.clear();

  for(size_t i=0; i<msgs.name.size(); i++ )
  {
    if( msgs.name.at(i).find(_token) != std::string::npos )
    {
      _target_idx.push_back(i);
    }
  }
  tf::Transform tf_tranform;

  size_t idx{0};
  for(size_t i = 0; i<_target_idx.size(); i++)
  {
    idx = _target_idx.at(i);
    tf_tranform.setOrigin( tf::Vector3(msgs.pose.at(idx).position.x, msgs.pose.at(idx).position.y, msgs.pose.at(idx).position.z) );
    tf_tranform.setRotation( tf::Quaternion( msgs.pose.at(idx).orientation.x, msgs.pose.at(idx).orientation.y, msgs.pose.at(idx).orientation.z, msgs.pose.at(i).orientation.w ) );
    _br->sendTransform( tf::StampedTransform(tf_tranform, ros::Time::now(), _world_name, _tf_target_name + "_" + std::to_string(i) ) );
  }
}
