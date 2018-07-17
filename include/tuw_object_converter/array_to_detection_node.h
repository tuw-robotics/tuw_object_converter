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

#include <ros/ros.h>
#include <tuw_object_msgs/ObjectWithCovarianceArray.h>
#include <tuw_object_msgs/ObjectDetection.h>
#include <tuw_object_converter/array_to_detectionConfig.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <random>

class ArrayToDetectionNode
{
public:
  ArrayToDetectionNode(ros::NodeHandle& nh);
  
  void objectArrayCallback(const tuw_object_msgs::ObjectWithCovarianceArray& msg);
  void callbackParameters(tuw_object_converter::array_to_detectionConfig &config, uint32_t level);
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber sub_object_array_;
  ros::Publisher pub_detection_;
  ros::Publisher pub_fov_visual_;
  
  dynamic_reconfigure::Server<tuw_object_converter::array_to_detectionConfig> reconfigureServer_;
  dynamic_reconfigure::Server<tuw_object_converter::array_to_detectionConfig>::CallbackType reconfigureFnc_;
  
  std::shared_ptr<tf::TransformListener> tf_listener_;
  
  std::mt19937 generator_;                                /// random number generator
  std::normal_distribution<double> normal_distribution_;  /// normal distribution for generic use
  
  double sigma_x_;
  double sigma_y_;
  
  bool fov_filter_;
  double fov_h_;
  double fov_min_dist_;
  double fov_max_dist_;
  
  std::string base_link_frame_;
  
  void fov_filter(tuw_object_msgs::ObjectDetection& detection);
};
