/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

// ROS includes
#include <ros/ros.h>

// Transformation helper code
#include <tf2_ros/transform_listener.h>

// RVIZ visualization
#include <visualization_msgs/Marker.h>

// Gazebo includes
#include <astrobee_gazebo/astrobee_gazebo.h>
#include <gazebo/physics/Model.hh>

// Freeflyer messages
#include <ff_hw_msgs/SetEnabled.h>

// STL includes
#include <string>
#include <thread>

namespace gazebo {

class GazeboModelPluginLaser : public FreeFlyerModelPlugin {
 public:
  GazeboModelPluginLaser()
      : FreeFlyerModelPlugin("laser", "laser", true),
        rate_(10.0),
        range_(50.0),
        width_(0.0025),
        height_(0.02),
        depth_(0.005),
        ready_(false),
        laser_enabled_(false) {}

  ~GazeboModelPluginLaser() {
    if (update_) {
      #if GAZEBO_MAJOR_VERSION > 7
      update_.reset();
      #else
      event::Events::DisconnectWorldUpdateBegin(update_);
      #endif
    }
  }

 protected:
  // Called when the plugin is loaded into the simulator
  void LoadCallback(ros::NodeHandle *nh, physics::ModelPtr model, sdf::ElementPtr sdf) {
    if (sdf->HasElement("rate"))
      rate_ = sdf->Get<double>("rate");
    if (sdf->HasElement("range"))
      range_ = sdf->Get<double>("range");
    if (sdf->HasElement("width"))
      width_ = sdf->Get<double>("width");

    // Use the message system to toggle visibility of visual elements
    gz_ = transport::NodePtr(new transport::Node());
    gz_->Init();
    pub_gazebo_ = gz_->Advertise<msgs::Visual>("~/visual");
    pub_factory_ = gz_->Advertise<msgs::Light>("~/factory/light");
    pub_light_ = gz_->Advertise<msgs::Light>("~/light/modify");

    // For the RVIZ marker array
    pub_rviz_ = nh->advertise<visualization_msgs::Marker>(
      TOPIC_HARDWARE_LASER_RVIZ, 0, true);
  }

  // Only send measurements when extrinsics are available
  void OnExtrinsicsReceived(ros::NodeHandle *nh) {
    // Advertise the presence of the laser
    srv_ = nh->advertiseService(SERVICE_HARDWARE_LASER_ENABLE,
      &GazeboModelPluginLaser::ToggleCallback, this);
    srv_check_ = nh->advertiseService("/hw/laser/check",
      &GazeboModelPluginLaser::CheckCallback, this);
  }

  // Manage the extrinsics based on the sensor type
  bool ExtrinsicsCallback(geometry_msgs::TransformStamped const* tf) {
    if (!tf) {
      ROS_WARN("Laser extrinsics are null");
      return false;
    }

    // Rotate from the cylinder frame Z fwd to X fwd
    pose_ = ignition::math::Pose3d(0.0, 0, 0, 0.70710678, 0, -0.70710678, 0);

    // Aggregate pose
    pose_ = pose_ + ignition::math::Pose3d(
      tf->transform.translation.x,
      tf->transform.translation.y,
      tf->transform.translation.z,
      tf->transform.rotation.w,
      tf->transform.rotation.x,
      tf->transform.rotation.y,
      tf->transform.rotation.z);

    // Create a visual
    visual_.set_name(GetFrame("laser_visual", "_"));
    visual_.set_parent_name(GetModel()->GetLink()->GetScopedName());
    msgs::Geometry *geometry = visual_.mutable_geometry();
    geometry->set_type(msgs::Geometry::BOX);
    msgs::Set(geometry->mutable_box()->mutable_size(),
      ignition::math::Vector3d(depth_, width_, height_));
    visual_.mutable_material()->mutable_script()->set_name("Astrobee/Laser");
    msgs::Set(visual_.mutable_pose(), pose_);
    visual_.set_is_static(false);
    visual_.set_visible(false);
    visual_.set_cast_shadows(false);
    visual_.set_transparency(0.8);
    pub_gazebo_->Publish(visual_);

    // Create the gazebo light
    laser_name_ = GetFrame("laser", "_");
    light_.set_name(laser_name_);
    light_.set_type(msgs::Light::SPOT);
    // light_.set_attenuation_constant(1.0);
    light_.set_attenuation_constant(0);
    light_.set_attenuation_linear(0.02);
    light_.set_attenuation_quadratic(0.0);
    light_.set_range(range_);
    light_.set_cast_shadows(false);
    light_.set_spot_inner_angle(0.0001);
    light_.set_spot_outer_angle(0.03);
    light_.set_spot_falloff(1.0);
    // msgs::Set(light_.mutable_diffuse(), ignition::math::Color(0.5, 0.5, 0.5, 1));
    msgs::Set(light_.mutable_diffuse(), ignition::math::Color(0., 0., 0., 1));
    msgs::Set(light_.mutable_specular(), ignition::math::Color(0.1, 0.1, 0.1, 1));
# if GAZEBO_MAJOR_VERSION >= 8
    msgs::Set(light_.mutable_pose(), pose_
        + GetModel()->GetLink()->WorldPose());
# else
    msgs::Set(light_.mutable_pose(), pose_
        + GetModel()->GetLink()->GetWorldPose().Ign());
# endif
    pub_factory_->Publish(light_);


    // Setup boilerplate marke code for rviz
    marker_.header.stamp = ros::Time::now();
    marker_.header.frame_id = GetFrame("laser");
    marker_.ns = GetFrame("laser_visual", "_");
    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::CYLINDER;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose.position.x = range_ / 2;
    marker_.pose.position.y = 0;
    marker_.pose.position.z = 0;
    marker_.pose.orientation.x = 0;
    marker_.pose.orientation.y = 0.7071067811;
    marker_.pose.orientation.z = 0;
    marker_.pose.orientation.w = 0.7071067811;
    marker_.scale.x = width_ * 2;
    marker_.scale.y = width_ * 2;
    marker_.scale.z = range_;
    marker_.color.a = 0.0;
    marker_.color.r = 0.0;
    marker_.color.g = 1.0;
    marker_.color.b = 0.0;
    pub_rviz_.publish(marker_);

    // Modify the new entity to be only visible in the GUI
    update_ = event::Events::ConnectWorldUpdateBegin(std::bind(
      &GazeboModelPluginLaser::WorldUpdateBegin, this));

    // Success
    return true;
  }

  // Called when check the laser validity
  bool CheckCallback(ff_hw_msgs::SetEnabled::Request &req,
                      ff_hw_msgs::SetEnabled::Response &res) {
     if (req.enabled) {
        // create a new laser
        laser_name_ = GetFrame("laser", "_") + std::to_string((cnt_));

        // add a light to the scene for the light stability
        rendering::ScenePtr scene = rendering::get_scene(this->GetWorld()->Name());
        rendering::LightPtr light_to_scene = scene->LightByName(GetFrame("laser", "_"))->Clone(laser_name_, scene);
        light_to_scene->SetName(laser_name_);
        // wide lighting
        light_to_scene->SetSpotInnerAngle(10);
        light_to_scene->SetSpotOuterAngle(10);
        ignition::math::Color color = ignition::math::Color(0.3, 0.3, 0.3, 1.0);
        light_to_scene->SetDiffuseColor(color);

        // publish a light to gazebo
        light_.set_name(laser_name_);
        // wide lighting
        light_.set_spot_inner_angle(10);
        light_.set_spot_outer_angle(10);
        msgs::Set(light_.mutable_diffuse(), ignition::math::Color(0.3, 1.0, 0.3, 1));

        pub_factory_->Publish(light_);
        cnt_++;

      } else {
        // turn the laser off
        msgs::Set(light_.mutable_diffuse(), ignition::math::Color(0., 0., 0., 1.));
        light_.set_name(laser_name_);
        light_.set_spot_inner_angle(0.0001);
        light_.set_spot_outer_angle(0.03);
        pub_light_->Publish(light_);
      }
      res.success = true;
      res.status_message = "Laser Check toggled successfully";
      return true;
  }

  // Called when the laser needs to be toggled
  bool ToggleCallback(ff_hw_msgs::SetEnabled::Request &req,
                      ff_hw_msgs::SetEnabled::Response &res) {
    laser_enabled_ = req.enabled;

    // Update the visual marker
    visual_.set_transparency(req.enabled ? 0.0 : 1.0);
    pub_gazebo_->Publish(visual_);

    // Update the gazebo light
    // light_.set_attenuation_constant(req.enabled ? 0.0 : 1.0);
    light_.set_name(laser_name_);
    light_.set_spot_inner_angle(0.0001);
    light_.set_spot_outer_angle(0.03);
    if (req.enabled) {
        msgs::Set(light_.mutable_diffuse(), ignition::math::Color(0.3, 1.0, 0.3, 1));
    } else {
        msgs::Set(light_.mutable_diffuse(), ignition::math::Color(0., 0., 0., 1));
    }
    pub_light_->Publish(light_);


    // Update RVIZ marker
    marker_.action = visualization_msgs::Marker::MODIFY;
    marker_.header.stamp = ros::Time::now();
    marker_.color.a = (req.enabled ? 0.5 : 0.0);
    pub_rviz_.publish(marker_);

    // Print success and return
    res.success = true;
    res.status_message = "Laser toggled successfully";
    return true;
  }

  // Called when a new entity is created
  void WorldUpdateBegin() {
    if (laser_enabled_) {
# if GAZEBO_MAJOR_VERSION >= 8
        msgs::Set(light_.mutable_pose(), pose_ +
             GetModel()->GetLink()->WorldPose());
# else
        msgs::Set(light_.mutable_pose(), pose_ +
             GetModel()->GetLink()->GetWorldPose().Ign());
# endif
        pub_light_->Publish(light_);

        marker_.action = visualization_msgs::Marker::MODIFY;
        marker_.header.stamp = ros::Time::now();
        pub_rviz_.publish(marker_);
    }
  }

 private:
  double rate_, range_, width_, height_, depth_;
  common::Time next_tick_;
  transport::NodePtr gz_;
  transport::PublisherPtr pub_gazebo_, pub_factory_, pub_light_;
  event::ConnectionPtr update_;
  ros::Timer timer_;
  ros::ServiceServer srv_, srv_check_;
  ros::Publisher pub_rviz_;
  visualization_msgs::Marker marker_;
  msgs::Visual visual_;
  msgs::Light light_;
  bool ready_;
  bool laser_enabled_;
  ignition::math::Pose3d pose_;
  std::string laser_name_ = "laser";
  int cnt_ = 0;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboModelPluginLaser)

}   // namespace gazebo

