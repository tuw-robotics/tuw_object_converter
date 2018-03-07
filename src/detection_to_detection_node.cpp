/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2017 by Florian Beck <florian.beck@tuwien.ac.at>        *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/

#include "detection_to_detection_node.h"
#include <geometry_msgs/PoseWithCovariance.h>
#include <cmath>
#include <tf/tf.h>

DetectionToDetectionNode::DetectionToDetectionNode(ros::NodeHandle& nh) : nh_(nh), nh_private_("~")
{  
  sub_object_array_ = nh_.subscribe("detection_in", 1, &DetectionToDetectionNode::detectionCallback, this);
  pub_detection_ = nh_.advertise<tuw_object_msgs::ObjectDetection>("detection_out", 5);
  pub_fov_visual_ = nh_.advertise<visualization_msgs::Marker>("fov_visual_marker", 5);
  
  reconfigureFnc_ = boost::bind(&DetectionToDetectionNode::callbackParameters, this, _1, _2);
  reconfigureServer_.setCallback(reconfigureFnc_);
  
  nh_private_.param("base_link_frame", base_link_frame_, std::string("base_link"));
  
  tf_listener_ = std::make_shared<tf::TransformListener>();
}

void DetectionToDetectionNode::detectionCallback(const tuw_object_msgs::ObjectDetection& msg)
{
  tuw_object_msgs::ObjectDetection detection = msg;
  
  if(fov_filter_)
    fov_filter(detection);
  
  pub_detection_.publish(detection);
}

void DetectionToDetectionNode::callbackParameters(tuw_object_converter::array_to_detectionConfig &config, uint32_t level)
{
  fov_filter_ = config.fov_filter;
  fov_h_ = config.fov_h * M_PI / 180;
  fov_min_dist_ = config.fov_min_dist;
  fov_max_dist_ = config.fov_max_dist;
}

void DetectionToDetectionNode::fov_filter(tuw_object_msgs::ObjectDetection& detection)
{
  double ya = sin(fov_h_ / 2);
  double yb = -ya;
  geometry_msgs::Point point;
  
  // draw fov with marker msgs
  visualization_msgs::Marker marker;
  marker.header.frame_id = base_link_frame_;
  marker.header.stamp = ros::Time::now();
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  point.x = 0;
  point.y = ya;
  marker.points.emplace_back(point);
  point.x = fov_max_dist_;
  point.y = ya;
  marker.points.emplace_back(point);
  point.x = fov_max_dist_;
  point.y = yb;
  marker.points.emplace_back(point);
  point.x = 0;
  point.y = yb;
  marker.points.emplace_back(point);
  pub_fov_visual_.publish(marker);
  
  // transform points into base_link
  tf::StampedTransform detection2base;
  try
  {
    tf_listener_->lookupTransform(detection.header.frame_id, base_link_frame_, ros::Time(0), detection2base);    
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("detectionCallback: %s", ex.what());
    return;
  }

  // transform pose with covariance
  //geometry_msgs::Pose p1;                 // target coordinate system
  //geometry_msgs::PoseWithCovariance p2;   // pose with cov to transform
  //geometry_msgs::PoseWithCovariance p;    // result
  
  geometry_msgs::PointStamped p;
  geometry_msgs::PointStamped transformed_p;
  
  for(auto it = detection.objects.begin(); it != detection.objects.end();)
  {

    try
    {
      p.header.frame_id = detection.header.frame_id;
      p.header.stamp = ros::Time();
      p.point.x = it->object.pose.position.x;
      p.point.y = it->object.pose.position.y;
      p.point.z = 0;
      tf_listener_->transformPoint(base_link_frame_, p, transformed_p);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("detectionCallback: %s", ex.what());
      return;
    }
    
    // check if detection is inside fov
    // rectangular region
    
    if((transformed_p.point.x > fov_max_dist_) ||
                   (transformed_p.point.x < 0) || 
                  (transformed_p.point.y < yb) || 
                  (transformed_p.point.y > ya))
    {
      detection.objects.erase(it);
      std::cout << "transformed_p = (" << transformed_p.point.x << ", " << transformed_p.point.y << ")" << std::endl;
    }
    else
    {
      it++;
    }
    
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detection_to_detection_node");
  ros::NodeHandle nh;

  DetectionToDetectionNode detection_to_detection_node(nh);  
  
  ros::spin();
  
  return 0;
}
