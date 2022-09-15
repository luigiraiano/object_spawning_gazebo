#ifndef BALLSHOOTERPLUGIN_H
#define BALLSHOOTERPLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/physics.hh>

#include <unistd.h>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>


#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

namespace gazebo
{

class BallShooterPlugin : public WorldPlugin
{
public:
  BallShooterPlugin();
  virtual ~BallShooterPlugin() override = default;

  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override;

  void Reset() override;

public:
  void OnUpdate();

  void OnTimeReset();

  void OnPause();

  void OnReset();

  void WaitForseconds(const double& seconds_to_wait);

  void GetTargetList(const std::string& target_token);

  void PrintTargetList();

  void ResetBallPose(boost::shared_ptr<gazebo::physics::Model> model);

  void SetForceParticle(boost::shared_ptr<gazebo::physics::Model> model);

  double RandomFloat(const double& a, const double& b);

protected:
  // World pointer
  gazebo::physics::WorldPtr world_;

  // SDF pointer
  sdf::ElementPtr sdf_;

private:
  // Pointer to the update event connection
  event::ConnectionPtr update_connection_;
  event::ConnectionPtr time_reset_connection_;
  event::ConnectionPtr pause_connection_;
  event::ConnectionPtr world_reset_;

  // map of targets
  std::map<int, physics::ModelPtr> target_model_map_;

  // Update Loop frequency, rate at which we restart the positions and apply force to particles
  double reset_frequency_ = 2.0;
  // Time Memory
  double old_secs_;
  // Force Direction
  double force_x_ = 0.0;
  double force_y_ = 0.0;
  double forc_z_ = 0.0;
  double x_origin_ = 0.0;
  double y_origin_ = 0.0;
  double z_origin_ = 1.0;

  double random_range_ = 0.1;

  // Reseting Flag
  bool reseting_plugin_ = false;

  int model_map_size_ = 0;
  int model_to_update_index_now_ = 0;

  std::string particle_base_name_ = "particle";
  std::string token_{""};
  std::string link_name_{""};

  int32_t resetting_period_{1000};
};

GZ_REGISTER_WORLD_PLUGIN(BallShooterPlugin);

}

#endif // BALLSHOOTERPLUGIN_H
