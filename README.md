# tuw_object_converter

Contains utilities to convert between ```tuw_object_msgs``` types as well as [SPENCER people messages](https://github.com/spencer-project/spencer_people_tracking)

## Available conversions:
* `tuw_object_msgs::ObjectWithCovarianceArray` to `tuw_object_msgs::ObjectDetection` adds covariance and object type, FOV filtering possible
* `tuw_object_msgs::ObjectDetection` to `tuw_object_msgs::ObjectDetection` provides FOV filtering
* `tuw_object_msgs::ObjectDetection` to `spencer_tracking_msgs::DetectedPersons`
* `tuw_object_msgs::ObjectDetection` to `spencer_tracking_msgs::TrackedPersons`

## Dependencies:

### ROS dependencies:
* [tuw_msgs](https://github.com/tuw-robotics/tuw_msgs)
* [spencer_tracking_msgs](https://github.com/spencer-project/spencer_people_tracking/tree/master/messages/spencer_tracking_msgs)