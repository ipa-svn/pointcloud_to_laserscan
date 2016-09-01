
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Fraunhofer IPA.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Sofie Nilsson
 */
 
#include <pointcloud_to_laserscan/frame_publisher.h>

bool FramePublisher::initialize()
{
  priv_nh_ = ros::NodeHandle("~");

  priv_nh_.param<double>("update_rate", update_rate_, 0.01); // 100Hz

  priv_nh_.param<std::string>("base_frame", base_frame_, "base_link");
  priv_nh_.param<std::string>("rotation_frame", rotation_frame_, "torso_center_link");
  priv_nh_.param<std::string>("target_frame", target_frame_, "torso_rotated_base_link");

  priv_nh_.param<bool>("rot_z", rot_z_, false);
  priv_nh_.param<bool>("rot_x", rot_x_, false);
  priv_nh_.param<bool>("rot_y", rot_y_, false);

  frame_broadcast_timer_ = nh_.createTimer(ros::Duration(update_rate_), &FramePublisher::frameBroadcastCallback, this);

  ros::Duration(1.0).sleep(); //give tf_listener some time

  return true;
}

/// Broadcast a frame aligned with the base frame but rotated around specified axes as rotation_frame 
void FramePublisher::frameBroadcastCallback(const ros::TimerEvent& event)
{   
  tf::StampedTransform frame_transform;
  try
  {
    tf_listener_.waitForTransform(base_frame_, rotation_frame_, event.current_real, ros::Duration(0.1));
    tf_listener_.lookupTransform(base_frame_, rotation_frame_, ros::Time(0), frame_transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("FramePublisher::getTransform: \n%s", ex.what());
    return;
  }

  double rot_frame_roll, rot_frame_pitch, rot_frame_yaw;
  frame_transform.getBasis().getRPY(rot_frame_roll, rot_frame_pitch, rot_frame_yaw);
  tf::Transform target_transform(frame_transform.getRotation());

  // Use rotations according to settings
  double target_frame_roll = 0;
  double target_frame_pitch = 0;
  double target_frame_yaw = 0; 
  if (rot_z_){ target_frame_yaw = rot_frame_yaw;}
  if (rot_x_){ target_frame_roll = rot_frame_roll;}
  if (rot_y_){ target_frame_pitch = rot_frame_pitch;}

  target_transform.getBasis().setRPY(target_frame_roll, target_frame_pitch, target_frame_yaw);

  // Broadcast new frame
  tf_broadcaster_.sendTransform(tf::StampedTransform(target_transform, frame_transform.stamp_, base_frame_, target_frame_));
}
