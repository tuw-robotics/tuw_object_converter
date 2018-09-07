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

#include "tuw_object_converter/spencer_detected_person_to_detection.h"

SpencerDetectedPersonToDetectionNode::SpencerDetectedPersonToDetectionNode(ros::NodeHandle& nh) : nh_(nh), nh_private_("~")
{  
  sub_detection_ = nh_.subscribe("detected_persons_topic", 1, &SpencerDetectedPersonToDetectionNode::detectionCallback, this);
  pub_detection_ = nh_.advertise<tuw_object_msgs::ObjectDetection>("object_detection_topic", 100);
}

void SpencerDetectedPersonToDetectionNode::detectionCallback(const spencer_tracking_msgs::DetectedPersons& msg)
{
  tuw_object_msgs::ObjectDetection detection;
  detection.header = msg.header;
  detection.type = tuw_object_msgs::ObjectDetection::OBJECT_TYPE_PERSON;
  if(msg.detections.size() > 0)
    detection.sensor_type = msg.detections[0].modality;
  
  for(auto&& person_it = msg.detections.begin(); person_it != msg.detections.end(); person_it++)
  {
    tuw_object_msgs::ObjectWithCovariance obj;
    
    obj.object.ids.emplace_back(person_it->detection_id);
    obj.object.ids_confidence.emplace_back(person_it->confidence);
    obj.object.pose = person_it->pose.pose;
    
    if(obj.object.pose.orientation.x == 0.0 && obj.object.pose.orientation.y == 0.0 && obj.object.pose.orientation.z == 0.0 && obj.object.pose.orientation.w == 0.0)
    {
      obj.object.pose.orientation.w = 1.0;
    }
    
    // tuw measurements are limited to 3x3 position w/o velocity
    if(person_it->pose.covariance.size() > 9)
    {
      for(size_t i = 0; i < person_it->pose.covariance.size(); i++)
      {
        if(i % 6 < 3)
          obj.covariance_pose.emplace_back(person_it->pose.covariance[i]);
      }
    }
    else
    {
      for(size_t i = 0; i < person_it->pose.covariance.size(); i++)
      {
        obj.covariance_pose.emplace_back(person_it->pose.covariance[i]);
      }
    }
    
    detection.objects.emplace_back(obj);
  }
  
  pub_detection_.publish(detection);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spencer_detected_person_to_detection_node");
  ros::NodeHandle nh;

  SpencerDetectedPersonToDetectionNode spencer_detected_person_to_detection_node (nh);  
  
  ros::spin();
  
  return 0;
}
