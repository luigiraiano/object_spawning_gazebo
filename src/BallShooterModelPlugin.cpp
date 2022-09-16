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

  ROS_INFO( "Ball Shooter Model Plugin Loaded - Model Name: %s", model->GetName().c_str() );

  model_ = model;
  GZ_ASSERT(model_ != NULL, "Got NULL Model pointer!");
  sdf_ = sdf;
  GZ_ASSERT(sdf_ != NULL, "Got NULL SDF element pointer!");

  // Get Params
  ROS_INFO_STREAM("Name of BallShooterPlugin SDF: " << sdf_->ReferenceSDF() );
  sdf_->PrintValues("BallShooterPlugin");

  // Get Force
  if (sdf->HasElement("x_axis_force"))
  {
    force_x_ = sdf->Get<double>("x_axis_force");
  }
  if (sdf->HasElement("y_axis_force"))
  {
    force_y_ = sdf->Get<double>("y_axis_force");
  }
  if (sdf->HasElement("z_axis_force"))
  {
    force_z_ = sdf->Get<double>("z_axis_force");
  }

  // Get Origin
  if (sdf->HasElement("x_origin"))
  {
    x_origin_ = sdf->Get<double>("x_origin");
  }
  if (sdf->HasElement("y_origin"))
  {
    y_origin_ = sdf->Get<double>("y_origin");
  }
  if (sdf->HasElement("z_origin"))
  {
    z_origin_ = sdf->Get<double>("z_origin");
  }

  // Get token name
  if(sdf->HasElement("target_token"))
  {
    token_ = sdf->Get<std::string>("target_token");
  }
  ROS_INFO_STREAM("Token: " << token_);

  // Get Link Name
  if(sdf->HasElement("link_name"))
  {
    link_name_ = sdf->Get<std::string>("link_name");
  }
  ROS_INFO_STREAM("Link Name: " << link_name_);
  model_->SetName(token_);
  ROS_INFO( "Ball Shooter Model Name changed from %s to %s", model->GetName().c_str(), model->GetName().c_str() );

  // Get Restting period
  if( sdf->HasElement("reset_period") )
  {
    resetting_period_ = sdf->Get<int>("reset_period");
  }

  // Set Event-Slots connections
  update_connection_ = event::Events::ConnectWorldUpdateBegin( std::bind(&BallShooterModelPlugin::OnUpdateSlot, this) );
  time_reset_connection_ = event::Events::ConnectTimeReset( std::bind(&BallShooterModelPlugin::OnTimeResetSlot, this) );
}

void BallShooterModelPlugin::OnUpdateSlot()
{
  int t = static_cast<int>(ros::Time::now().toSec());

  if(t % resetting_period_ == 0)
  {
    ResetPose(model_);
    SetForce(model_, link_name_);
  }
}

void BallShooterModelPlugin::OnTimeResetSlot()
{
  ROS_INFO( "Gazebo Time Reset - Resetting Ball Pose" );
  ResetPose(model_);
  SetForce(model_, link_name_);
}

void BallShooterModelPlugin::ResetPose(boost::shared_ptr<gazebo::physics::Model> model)
{
  double roll_rand{0.0};
  double pitch_rand{0.0};
  double yaw_rand{0.0};

  double x_pose{0.0};
  double y_pose{0.0};
  double z_pose{0.0};

  double random_range{0.2};

//  x_pose = x_origin_;
//  y_pose = y_origin_;
//  z_pose = z_origin_;
//  x_pose = RandN(x_origin_, random_range);
//  y_pose = RandN(y_origin_, random_range);
//  z_pose = z_origin_;
  x_pose = RandomizeRange(x_origin_, random_range);
  y_pose = RandomizeRange(y_origin_, random_range);
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
  model->GetLink(link_name)->SetForce(ignition::math::Vector3d(force_x_, force_y_, force_z_));
}

double BallShooterModelPlugin::Randomize(const double& a, const double& b)
{
  double random = static_cast<double>( rand() / RAND_MAX );
  double diff = b - a;
  double r = random * diff;
  return a + r;
}

double BallShooterModelPlugin::RandomizeRange(const double& val, const double& range)
{
  double rand_val{0.0};

  std::random_device rd; // obtain a random number from hardware
  std::mt19937 gen(rd()); // seed the generator
  std::uniform_real_distribution<> distr(val-range, val + range); // define the range

  rand_val = distr(gen);

  return rand_val;
}
