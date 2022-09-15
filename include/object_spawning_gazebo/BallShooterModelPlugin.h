#ifndef BALLSHOOTERMODELPLUGIN_H
#define BALLSHOOTERMODELPLUGIN_H

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

class BallShooterModelPlugin : public ModelPlugin
{
public:
  BallShooterModelPlugin();
  virtual ~BallShooterModelPlugin() override = default;

public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf) override;

private:
  void OnUpdate();
  void OnTimeReset();
  void OnPause();

private:

  void ResetPose(boost::shared_ptr<gazebo::physics::Model> model);

  void SetForce(boost::shared_ptr<gazebo::physics::Model> model, const std::string& link_name);

  double RandomFloat(const double& a, const double& b);

protected:
  // World pointer
  gazebo::physics::WorldPtr world_;

  // Model Pointer
  gazebo::physics::ModelPtr model_;

  // SDF pointer
  sdf::ElementPtr sdf_;

private:
  event::ConnectionPtr update_connection_;
  event::ConnectionPtr time_reset_connection_;
  event::ConnectionPtr pause_connection_;

  // Force Direction
  double force_x_ = 0.0;
  double force_y_ = 0.0;
  double forc_z_ = 0.0;

  // Target Origin
  double x_origin_ = 0.0;
  double y_origin_ = 0.0;
  double z_origin_ = 1.0;

  std::string token_{""};
  std::string link_name_{""};

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(BallShooterModelPlugin)

}


#endif // BALLSHOOTERMODELPLUGIN_H