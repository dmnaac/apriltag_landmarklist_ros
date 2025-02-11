/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 *
 ** continuous_detector.h ******************************************************
 *
 * Wrapper class of TagDetector class which calls TagDetector::detectTags on
 * each newly arrived image published by a camera.
 *
 * $Revision: 1.0 $
 * $Date: 2017/12/17 13:25:52 $
 * $Author: dmalyuta $
 *
 * Originator:        Danylo Malyuta, JPL
 ******************************************************************************/

#ifndef APRILTAG_ROS_CONTINUOUS_DETECTOR_H
#define APRILTAG_ROS_CONTINUOUS_DETECTOR_H

#include "apriltag_ros/common_functions.h"
#include "apriltag_ros/WriteTags.h"

#include <memory>
#include <mutex>

#include <nodelet/nodelet.h>
#include <ros/service_server.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

namespace apriltag_ros
{

  class ContinuousDetector : public nodelet::Nodelet
  {
  public:
    ContinuousDetector() = default;
    ~ContinuousDetector() = default;

    void onInit();

    void imageCallback(const sensor_msgs::ImageConstPtr &image_rect,
                       const sensor_msgs::CameraInfoConstPtr &camera_info);

    void refreshTagParameters();

  private:
    struct TagPose2Camera
    {
      int id;
      geometry_msgs::PoseStamped pose;

      TagPose2Camera(int new_id, geometry_msgs::PoseStamped new_pose) : id(new_id), pose(new_pose) {}
    };

    struct TagPose2Map
    {
      int id;
      geometry_msgs::PoseStamped pose;

      TagPose2Map(int new_id, geometry_msgs::PoseStamped new_pose) : id(new_id), pose(new_pose) {}
    };

    std::mutex detection_mutex_;
    std::shared_ptr<TagDetector> tag_detector_;
    bool draw_tag_detections_image_;
    cv_bridge::CvImagePtr cv_image_;

    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::CameraSubscriber camera_image_subscriber_;
    image_transport::Publisher tag_detections_image_publisher_;
    ros::Publisher tag_detections_publisher_;
    ros::Publisher landmarks_publisher_;
    ros::Publisher april_pose_publisher_;
    ros::Subscriber amcl_pose_subscriber_;
    ros::Publisher amcl_initialpose_publisher_;

    ros::ServiceServer refresh_params_service_;
    bool refreshParamsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    ros::ServiceServer write_tags_service_;
    bool writeTagsServiceCallback(apriltag_ros::WriteTags::Request &request, apriltag_ros::WriteTags::Response &response);
    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

    geometry_msgs::Pose transformToPose(const tf::Transform &transform);
    AprilTagDetectionArray AprilTagDetectionFilter(AprilTagDetectionArray &tag_detection_array);

  public:
    bool publish_landmarks_;
    double translation_weight_;
    double rotation_weight_;

    std::string tracking_frame_;
    std::string map_frame_;
    std::vector<TagPose2Camera> tag_poses_to_camera_;
    std::vector<TagPose2Map> tag_poses_to_map_;
    std::string path_to_pose_txt_;
    bool publish_april_pose_;
    bool enable_write_tags_service_;

    AprilTagDetectionArray tag_detection_array_;
    tf::TransformListener tf_listener_;
    tf::StampedTransform transform_tagToMap_;
    tf::StampedTransform transform_cameraToRobot_;
    geometry_msgs::PoseStamped robot_pose_to_map_;

    double position_tolerance_;
    double orientation_tolerance_;
  };

} // namespace apriltag_ros

#endif // APRILTAG_ROS_CONTINUOUS_DETECTOR_H
