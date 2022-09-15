#include "object_spawning_gazebo/BallShooterModelPlugin.h"

using namespace gazebo;

BallShooterModelPlugin::BallShooterModelPlugin() : ModelPlugin()
{
  ROS_INFO("BallShooterModelPlugin initiaized");
}

void BallShooterModelPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  ROS_INFO("Ball Shooter Model Plugin Loaded");

  model_ = model;
  GZ_ASSERT(model_ != NULL, "Got NULL Model pointer!");
  sdf_ = sdf;
  GZ_ASSERT(sdf_ != NULL, "Got NULL SDF element pointer!");

  ROS_INFO("Model Name: %s", model_->GetName().c_str());


  // Get Params
  ROS_INFO_STREAM("Name of BallShooterPlugin SDF: " << sdf_->ReferenceSDF() );
  sdf_->PrintValues("BallShooterPlugin");

  if (sdf->HasElement("x_axis_force"))
    force_x_ = sdf->Get<double>("x_axis_force");
  if (sdf->HasElement("y_axis_force"))
    force_y_ = sdf->Get<double>("y_axis_force");
  if (sdf->HasElement("z_axis_force"))
    forc_z_ = sdf->Get<double>("z_axis_force");

  if (sdf->HasElement("x_origin"))
    x_origin_ = sdf->Get<double>("x_origin");
  if (sdf->HasElement("y_origin"))
    y_origin_ = sdf->Get<double>("y_origin");
  if (sdf->HasElement("z_origin"))
    z_origin_ = sdf->Get<double>("z_origin");

  if(sdf->HasElement("target_token"))
  {
    token_ = sdf->Get<std::string>("target_token");
  }
  ROS_INFO_STREAM("Token: " << token_);

  if(sdf->HasElement("link_name"))
  {
    link_name_ = sdf->Get<std::string>("link_name");
  }
  ROS_INFO_STREAM("Link Name: " << link_name_);

  model_->SetName(token_);

  update_connection_ = event::Events::ConnectWorldUpdateBegin( std::bind(&BallShooterModelPlugin::OnUpdate, this) );
  time_reset_connection_ = event::Events::ConnectTimeReset( std::bind(&BallShooterModelPlugin::OnTimeReset, this) );
  pause_connection_ = event::Events::ConnectPause( std::bind(&BallShooterModelPlugin::OnPause, this) );

//  ros::Duration(5).sleep();

//  ResetPose(model_);
//  SetForce(model_, link_name_);
}

void BallShooterModelPlugin::OnUpdate()
{
  int t = static_cast<int>(ros::Time::now().toSec());

  if(t % 10 == 0)
  {
    ResetPose(model_);
    SetForce(model_, link_name_);
  }
//  ROS_INFO( "Model Plugin Updating - %f", ros::Time::now().toSec() );
}

void BallShooterModelPlugin::OnTimeReset()
{
  ROS_INFO( "Gazebo Time Reset - Resetting Ball Pose" );
  ResetPose(model_);
  SetForce(model_, link_name_);
}

void BallShooterModelPlugin::OnPause()
{
  ROS_INFO( "Gazebo Paused" );
}

void BallShooterModelPlugin::ResetPose(boost::shared_ptr<gazebo::physics::Model> model)
{
  double roll_rand{0.0};
  double pitch_rand{0.0};
  double yaw_rand{0.0};

  double x_pose{0.0};
  double y_pose{0.0};
  double z_pose{0.0};

  x_pose = x_origin_;
  y_pose = y_origin_;
  z_pose = z_origin_;

  model->SetWorldPose( ignition::math::Pose3d(
                                              ignition::math::Vector3d(x_pose, y_pose, z_pose),
                                              ignition::math::Quaterniond(roll_rand, pitch_rand, yaw_rand)
                                              ) );
}

void BallShooterModelPlugin::SetForce(boost::shared_ptr<gazebo::physics::Model> model, const std::string& link_name)
{
  model->GetLink(link_name)->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0.0));

  model->GetLink(link_name)->SetForce(ignition::math::Vector3d(0.0, 0.0, 0.0));
  model->GetLink(link_name)->SetForce(ignition::math::Vector3d(force_x_, force_y_, forc_z_));
}

double BallShooterModelPlugin::RandomFloat(const double& a, const double& b)
{
  double random = static_cast<double>( rand() / RAND_MAX );
  double diff = b - a;
  double r = random * diff;
  return a + r;
}
