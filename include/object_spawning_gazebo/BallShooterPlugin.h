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
#include <gazebo/physics/physics.hh>
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
  virtual ~BallShooterPlugin() = default;
};

}

#endif // BALLSHOOTERPLUGIN_H
