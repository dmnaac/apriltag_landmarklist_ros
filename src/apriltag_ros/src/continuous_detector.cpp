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
 */

#include "apriltag_ros/continuous_detector.h"

#include <pluginlib/class_list_macros.hpp>

#include "cartographer_ros_msgs/LandmarkList.h"

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>

#include <algorithm>

PLUGINLIB_EXPORT_CLASS(apriltag_ros::ContinuousDetector, nodelet::Nodelet);

namespace apriltag_ros
{
  void ContinuousDetector::onInit()
  {
    ros::NodeHandle &nh = getNodeHandle();
    ros::NodeHandle &pnh = getPrivateNodeHandle();

    tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(pnh));
    draw_tag_detections_image_ = getAprilTagOption<bool>(pnh, "publish_tag_detections_image", false);
    it_ = std::shared_ptr<image_transport::ImageTransport>(
        new image_transport::ImageTransport(nh));

    publish_landmarks_ = getAprilTagOption<bool>(pnh, "publish_landmarks", true);

    std::string transport_hint;
    pnh.param<std::string>("transport_hint", transport_hint, "raw");

    int queue_size;
    pnh.param<int>("queue_size", queue_size, 1);
    camera_image_subscriber_ =
        it_->subscribeCamera("image_rect", queue_size,
                             &ContinuousDetector::imageCallback, this,
                             image_transport::TransportHints(transport_hint));
    tag_detections_publisher_ =
        nh.advertise<AprilTagDetectionArray>("tag_detections", 1);

    enable_write_tags_service_ = getAprilTagOption<bool>(pnh, "enable_write_tags", false);
    if (enable_write_tags_service_)
    {
      // Check if tf is published
      if (!tag_detector_->get_publish_tf())
      {
        ROS_ERROR("publish_tf must be true to enable write_tags service.");
        return;
      }
      write_tags_service_ = pnh.advertiseService("write_tags", &ContinuousDetector::writeTagsServiceCallback, this);
      map_frame_ = getAprilTagOption<std::string>(pnh, "map_frame", "map");
    }

    if (publish_landmarks_)
    {
      landmarks_publisher_ = nh.advertise<cartographer_ros_msgs::LandmarkList>("landmark", 10);
      translation_weight_ = getAprilTagOption<double>(pnh, "translation_weight", 1.0);
      rotation_weight_ = getAprilTagOption<double>(pnh, "rotation_weight", 1.0);
      tracking_frame_ = getAprilTagOption<std::string>(pnh, "tracking_frame", "base_link");
    }

    if (draw_tag_detections_image_)
    {
      tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
    }

    refresh_params_service_ =
        pnh.advertiseService("refresh_tag_params",
                             &ContinuousDetector::refreshParamsCallback, this);
  }

  void ContinuousDetector::refreshTagParameters()
  {
    // Resetting the tag detector will cause a new param server lookup
    // So if the parameters have changed (by someone/something),
    // they will be updated dynamically
    std::scoped_lock<std::mutex> lock(detection_mutex_);
    ros::NodeHandle &pnh = getPrivateNodeHandle();
    tag_detector_.reset(new TagDetector(pnh));
  }

  bool ContinuousDetector::refreshParamsCallback(std_srvs::Empty::Request &req,
                                                 std_srvs::Empty::Response &res)
  {
    refreshTagParameters();
    return true;
  }

  void ContinuousDetector::imageCallback(
      const sensor_msgs::ImageConstPtr &image_rect,
      const sensor_msgs::CameraInfoConstPtr &camera_info)
  {
    std::scoped_lock<std::mutex> lock(detection_mutex_);
    // Lazy updates:
    // When there are no subscribers _and_ when tf is not published,
    // skip detection.
    if (tag_detections_publisher_.getNumSubscribers() == 0 &&
        landmarks_publisher_.getNumSubscribers() == 0 &&
        tag_detections_image_publisher_.getNumSubscribers() == 0 &&
        !tag_detector_->get_publish_tf())
    {
      ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
      return;
    }

    // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
    // AprilTag 2 on the iamge
    try
    {
      cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Publish detected tags in the image by AprilTag 2
    tag_detection_array_ = tag_detector_->detectTags(cv_image_, camera_info);

    tag_detections_publisher_.publish(tag_detection_array_);

    // If enable_write_tags_service_ is true, the pose of each detected tag relative to camera is stored.
    // If one tag is detected more than once, only the latest pose relative to camera is stored.
    if (enable_write_tags_service_)
    {
      for (int i = 0; i < tag_detection_array_.detections.size(); i++)
      {
        AprilTagDetection item = tag_detection_array_.detections[i];
        int item_id = item.id[0];
        auto it = std::find_if(tag_poses_to_camera_.begin(), tag_poses_to_camera_.end(), [item_id](const TagPose2Camera &s)
                               { return s.id == item_id; });
        if (it != tag_poses_to_camera_.end())
        {
          it->pose.header = item.pose.header;
          it->pose.pose = item.pose.pose.pose;
        }
        else
        {
          geometry_msgs::PoseStamped pose_stamp;
          pose_stamp.header = item.pose.header;
          pose_stamp.pose = item.pose.pose.pose;
          tag_poses_to_camera_.emplace_back(item.id[0], pose_stamp);
        }
      }
    }

    // Publish detected tags as LandmarkList for Cartographer.
    // The cartographer_ros_msgs/LandmarkList should be provided at a sample rate comparable to the other sensors.
    // The list can be empty but has to be provided.
    if (publish_landmarks_)
    {
      cartographer_ros_msgs::LandmarkList landMarkList;
      tf::TransformListener tf_listener;

      for (int i = 0; i < tag_detection_array_.detections.size(); i++)
      {
        AprilTagDetection item = tag_detection_array_.detections[i];
        if (!tag_detector_->checkIDexists(item.id[0]))
        {
          continue;
        }

        cartographer_ros_msgs::LandmarkEntry landMarkEntry;
        geometry_msgs::PoseStamped pose_relative_to_robot;
        geometry_msgs::PoseStamped pose_relative_to_camera;
        pose_relative_to_camera.header = item.pose.header;
        pose_relative_to_camera.pose = item.pose.pose.pose;
        tf_listener.waitForTransform(tracking_frame_, item.pose.header.frame_id, ros::Time::now(), ros::Duration(3.0));
        tf_listener.transformPose(tracking_frame_, pose_relative_to_camera, pose_relative_to_robot);
        landMarkEntry.id = std::to_string(item.id[0]);
        landMarkEntry.tracking_from_landmark_transform.position = pose_relative_to_robot.pose.position;
        landMarkEntry.tracking_from_landmark_transform.orientation = pose_relative_to_robot.pose.orientation;
        landMarkEntry.translation_weight = translation_weight_;
        landMarkEntry.rotation_weight = rotation_weight_;

        landMarkList.landmarks.push_back(landMarkEntry);
      }

      landMarkList.header = tag_detection_array_.header;
      landMarkList.header.frame_id = tracking_frame_;
      landmarks_publisher_.publish(landMarkList);
    }

    // Publish the camera image overlaid by outlines of the detected tags and
    // their payload values
    if (draw_tag_detections_image_)
    {
      tag_detector_->drawDetections(cv_image_);
      tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
    }
  }

  bool ContinuousDetector::writeTagsServiceCallback(apriltag_ros::WriteTags::Request &request, apriltag_ros::WriteTags::Response &response)
  {
    // Check if tf is published
    if (!tag_detector_->get_publish_tf())
    {
      ROS_ERROR("WriteTags service needs published tf.");
      response.write_state = 0;
      return false;
    }

    std::string filename = request.file_directory + "/" + request.file_basename + ".txt";
    std::ofstream file;
    file.open(filename, std::ios::app);
    if (!file.is_open())
    {
      ROS_ERROR("Unable to open file for writing: %s", filename.c_str());
      response.write_state = 0;
      return false;
    }

    tf::TransformListener tf_listener;

    for (const int &frame_id : tag_detector_->published_tf_id_)
    {
      auto it = std::find_if(tag_poses_to_camera_.begin(), tag_poses_to_camera_.end(),
                             [frame_id](const TagPose2Camera &s)
                             { return s.id == frame_id; });
      if (it != tag_poses_to_camera_.end())
      {
        geometry_msgs::PoseStamped tag_pose_to_map;
        try
        {
          tf_listener.waitForTransform(map_frame_, it->pose.header.frame_id, ros::Time::now(), ros::Duration(3.0));
          tf_listener.transformPose(map_frame_, it->pose, tag_pose_to_map);
        }
        catch (tf::TransformException &ex)
        {
          ROS_WARN("Transform between %s and %s not found: %s", map_frame_.c_str(), it->pose.header.frame_id.c_str(), ex.what());
          continue;
        }
        file << frame_id << " " << tag_pose_to_map.pose.position.x << " " << tag_pose_to_map.pose.position.y << " " << tag_pose_to_map.pose.position.z << " "
             << tag_pose_to_map.pose.orientation.x << " " << tag_pose_to_map.pose.orientation.y << " " << tag_pose_to_map.pose.orientation.z << " " << tag_pose_to_map.pose.orientation.w << "\n";
      }
      else
      {
        ROS_WARN("Cannot retrieve tag %d from detected tags", frame_id);
        continue;
      }
    }

    file.close();
  }

} // namespace apriltag_ros
