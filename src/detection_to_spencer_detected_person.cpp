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

#include "tuw_object_converter/detection_to_spencer_detected_person.h"

DetectionToSpencerDetectedPersonNode::DetectionToSpencerDetectedPersonNode(ros::NodeHandle& nh) : nh_(nh), nh_private_("~")
{  
  sub_detection_ = nh_.subscribe("object_detection_topic", 1, &DetectionToSpencerDetectedPersonNode::detectionCallback, this);
  pub_tracks_ = nh_.advertise<spencer_tracking_msgs::DetectedPersons>("detected_persons_topic", 100);
}

void DetectionToSpencerDetectedPersonNode::detectionCallback(const tuw_object_msgs::ObjectDetection& msg)
{
  spencer_tracking_msgs::DetectedPersons detected_persons;
  
  detected_persons.header = msg.header;
  
  for(auto&& obj_it = msg.objects.begin(); obj_it != msg.objects.end(); obj_it++)
  {
    spencer_tracking_msgs::DetectedPerson detected_person;
    detected_person.detection_id = obj_it->object.ids[0];
    detected_person.confidence = obj_it->object.ids_confidence[0];
    detected_person.pose.pose = obj_it->object.pose;
    detected_person.modality = msg.sensor_type;
    
    for(size_t i = 0; i < obj_it->covariance_pose.size() && i < 36; i++)
    {
      detected_person.pose.covariance[i] = obj_it->covariance_pose[i];
    }

    detected_persons.detections.emplace_back(detected_person);
  }
  pub_tracks_.publish(detected_persons);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detection_to_spencer_detected_person_node");
  ros::NodeHandle nh;

  DetectionToSpencerDetectedPersonNode detection_to_spencer_detected_person_node(nh);  
  
  ros::spin();
  
  return 0;
}
