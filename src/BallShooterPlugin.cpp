#include "object_spawning_gazebo/BallShooterPlugin.h"

using namespace gazebo;

BallShooterPlugin::BallShooterPlugin() : WorldPlugin()
{
  ROS_INFO("Ball Shooter Plugin Instanciated");
}

void BallShooterPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  world_ = world;
  GZ_ASSERT(world_ != NULL, "Got NULL world pointer!");
  sdf_ = sdf;
  GZ_ASSERT(sdf_ != NULL, "Got NULL SDF element pointer!");

  ROS_INFO_STREAM("Name of BallShooterPlugin SDF: " << sdf_->ReferenceSDF() );
  sdf_->PrintValues("BallShooterPlugin");

  // Check if Config Elements exist, otherwise they will have default value
  if (sdf->HasElement("reset_frequency"))
    reset_frequency_ = sdf->Get<double>("reset_frequency");

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

  if (sdf->HasElement("random_range"))
    random_range_ = sdf->Get<double>("random_range");

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

  // We wait for all system to be ready an amount of seconds
  double seconds_to_wait = 5.0;
  WaitForseconds(seconds_to_wait);

  // Update Time Init
  old_secs_ = world_->SimTime().Double();

  // Connect to the update event. This event is broadcast every simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&BallShooterPlugin::OnUpdate, this));

  // Connect to Gazebo World Reset
  time_reset_connection_ = event::Events::ConnectTimeReset( std::bind(&BallShooterPlugin::OnTimeReset, this) );

  // Connect to Gazebo Pause
  pause_connection_ = event::Events::ConnectPause( std::bind(&BallShooterPlugin::OnPause, this) );

  // Connect To World Reset
//  world_reset_ = event::Events::ConnectWorldReset( std::bind(&BallShooterPlugin::OnReset, this) );

  GetTargetList(token_);
  PrintTargetList();

  // Delete the Ball and Spawn a new one
  for (auto model : target_model_map_)
  {
    ResetBallPose(model.second);
    SetForceParticle(model.second);
  }

  ROS_DEBUG("Particle Shooter Ready....");
}

void BallShooterPlugin::Reset()
{
  reseting_plugin_ = true;
  ROS_ERROR("Reseted the simulation world, we Restart the time clock");
  // Update Time Init
  old_secs_ = 0.0;
  double new_secs = 0.0;
  double delta = -1.0;

  while (delta < 0.0)
  {
    // We change Direction
    ROS_ERROR("Waiting until Clock is reseted and delta is not negative > Update delta=%f, new_secs=%f", delta,  new_secs);
    new_secs = world_->SimTime().Double();
    delta = new_secs - old_secs_;
    ROS_ERROR("Updated until Clock is reseted > Update delta=%f, new_secs=%f", delta,  new_secs);
  }

  reseting_plugin_ = false;
}

void BallShooterPlugin::OnUpdate()
{

  ROS_INFO("ROS Time: %f - SimTime: %i",ros::Time::now().toSec(), world_->SimTime().sec);

  if( world_->SimTime().sec >= resetting_period_ )
  {
    world_->SetPaused(true);
  }
  else
  {
    for(auto model : target_model_map_)
    {
    }
  }

}

void BallShooterPlugin::OnTimeReset()
{
  ROS_INFO("Time Reset Occurred");

  old_secs_ = world_->SimTime().Double();

  // Reset Ball
  for (auto model : target_model_map_)
  {
    ResetBallPose(model.second);
    SetForceParticle(model.second);
  }
}

void BallShooterPlugin::OnReset()
{
  ROS_INFO("World Reset Occurred");

  old_secs_ = world_->SimTime().Double();

  // Reset Ball
  for (auto model : target_model_map_)
  {
    ResetBallPose(model.second);
    SetForceParticle(model.second);
  }
}

void BallShooterPlugin::OnPause()
{
  ROS_INFO("Pause Occurred");

  world_->ResetTime();
}

void BallShooterPlugin::WaitForseconds(const double& seconds_to_wait)
{
  unsigned int microseconds;
  microseconds = static_cast<unsigned int>(seconds_to_wait * 1e6);
  ROS_WARN("Waiting for %f seconds",seconds_to_wait);
  usleep(microseconds);
  ROS_WARN("Done waiting...");

}

void BallShooterPlugin::GetTargetList(const std::string& target_token)
{
    target_model_map_.clear();
    // Initialize color map.
    model_map_size_ = 0;

    int i = 0;
    for (auto model : world_->Models())
    {
      std::string model_name = model->GetName();
      if (model_name.find(target_token) != std::string::npos)
      {
        target_model_map_[i] = model;
        i ++;
      }

    }

    model_map_size_ = static_cast<int>( target_model_map_.size() );
    ROS_INFO("Model Maps Size : %i", model_map_size_);
}

void BallShooterPlugin::PrintTargetList()
{
  ROS_INFO("List of Selected Target: ");

  for (auto x : target_model_map_)
  {
    ROS_INFO("ModelID=%i, Name=%s", x.first, x.second->GetName().c_str() );
  }

}

void BallShooterPlugin::ResetBallPose(boost::shared_ptr<gazebo::physics::Model> model)
{
  std::string model_name = model->GetName();
  double roll_rand = 0.0;
  double pitch_rand = 0.0;
  double yaw_rand = 0.0;

  double x_pos_rand = RandomFloat(x_origin_ - random_range_, x_origin_ + random_range_);
  double y_pos_rand = RandomFloat(y_origin_ - random_range_, y_origin_ + random_range_);
  double z_pos_rand = RandomFloat(z_origin_ - random_range_, z_origin_ + random_range_);

  ROS_DEBUG("POSE-RANDOM[X,Y,Z,Roll,Pitch,Yaw=[%f,%f,%f,%f,%f,%f], model=%s", x_pos_rand,y_pos_rand,z_pos_rand,roll_rand,pitch_rand,yaw_rand,model_name.c_str());
  //ignition::math::Pose3 initPose(ignition::math::Vector3<float>(x_pos_rand, y_pos_rand, z_pos_rand), ignition::math::Quaternion<float>(roll_rand, pitch_rand, yaw_rand));

  model->SetWorldPose( ignition::math::Pose3d(
                         ignition::math::Vector3d(x_origin_, x_origin_, x_origin_),
                         ignition::math::Quaterniond(roll_rand, pitch_rand, yaw_rand)) );

  ROS_DEBUG("Resetting model=%s....END",model_name.c_str());
}

void BallShooterPlugin::SetForceParticle(boost::shared_ptr<gazebo::physics::Model> model)
{
  model->GetLink(link_name_)->SetForce(ignition::math::Vector3d(0.0, 0.0, 0.0));
  ROS_WARN("FORCE APPLIED[X,Y,Z]=[%f,%f,%f]", force_x_, force_y_, forc_z_);
  model->GetLink(link_name_)->SetForce(ignition::math::Vector3d(force_x_, force_y_, forc_z_));
}

double BallShooterPlugin::RandomFloat(const double& a, const double& b)
{
  double random = static_cast<double>( rand() / RAND_MAX );
  double diff = b - a;
  double r = random * diff;
  return a + r;
}
