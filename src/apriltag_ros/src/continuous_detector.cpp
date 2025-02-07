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
    map_frame_ = getAprilTagOption<std::string>(pnh, "map_frame", "map");
    if (enable_write_tags_service_)
    {
      // Check if tf is published
      if (!tag_detector_->get_publish_tf())
      {
        ROS_ERROR("publish_tf must be true to enable write_tags service.");
        return;
      }
      write_tags_service_ = pnh.advertiseService("write_tags", &ContinuousDetector::writeTagsServiceCallback, this);
    }
    else
    {
      path_to_pose_txt_ = getAprilTagOption<std::string>(pnh, "path_to_pose_txt", "");
      tracking_frame_ = getAprilTagOption<std::string>(pnh, "tracking_frame", "base_link");
      publish_robot_pose_ = getAprilTagOption<bool>(pnh, "publish_robot_pose", false);
      if (publish_robot_pose_)
      {
        robot_pose_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("april_pose", 10);
      }
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

  // Convert tf::Transform to geometry_msgs::Pose
  geometry_msgs::Pose ContinuousDetector::transformToPose(const tf::Transform &transform)
  {
    geometry_msgs::Pose pose;
    tf::Vector3 translation = transform.getOrigin();
    tf::Quaternion rotation = transform.getRotation();
    pose.position.x = translation.x();
    pose.position.y = translation.y();
    pose.position.z = translation.z();
    pose.orientation.x = rotation.x();
    pose.orientation.y = rotation.y();
    pose.orientation.z = rotation.z();
    pose.orientation.w = rotation.w();
    return pose;
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
          tag_poses_to_camera_.emplace_back(item_id, pose_stamp);
        }
        // ROS_INFO("Detected ID: %d", item_id);
      }
    }
    else
    {
      if (tag_detection_array_.detections.size() > 0)
      {
        std::ifstream file(path_to_pose_txt_);
        if (!file.is_open())
        {
          ROS_ERROR("Failed to open the pose.txt file.");
          return;
        }

        for (int i = 0; i < tag_detection_array_.detections.size(); i++)
        {
          AprilTagDetection item = tag_detection_array_.detections[i];
          int item_id = item.id[0];
          if (!tag_detector_->checkIDexists(item_id))
          {
            continue;
          }

          std::string line;
          while (std::getline(file, line))
          {
            std::istringstream iss(line);

            int tag_id;
            double px, py, pz;
            double ox, oy, oz, ow;

            if (iss >> tag_id >> px >> py >> pz >> ox >> oy >> oz >> ow)
            {
              if (tag_id == item_id)
              {
                geometry_msgs::PoseStamped tag_pose_to_map;
                geometry_msgs::PoseStamped tag_pose_to_camera;
                geometry_msgs::PoseStamped camera_pose_to_map;

                tag_pose_to_map.header = item.pose.header;
                tag_pose_to_map.header.frame_id = map_frame_;
                tag_pose_to_map.pose.position.x = px;
                tag_pose_to_map.pose.position.y = py;
                tag_pose_to_map.pose.position.z = pz;
                tag_pose_to_map.pose.orientation.x = ox;
                tag_pose_to_map.pose.orientation.y = oy;
                tag_pose_to_map.pose.orientation.z = oz;
                tag_pose_to_map.pose.orientation.w = ow;

                tag_pose_to_camera.header = item.pose.header;
                tag_pose_to_camera.pose = item.pose.pose.pose;

                tf::Stamped<tf::Pose> tagPoseToMap, tagPoseToCamera;
                tf::poseStampedMsgToTF(tag_pose_to_map, tagPoseToMap);
                tf::poseStampedMsgToTF(tag_pose_to_camera, tagPoseToCamera);
                tf::Pose cameraPoseToTag = tagPoseToCamera.inverse();
                tf::Pose cameraPoseToMap = tagPoseToMap * cameraPoseToTag;

                tf::StampedTransform transform_cameraToRobot;
                try
                {
                  tf_listener_.waitForTransform(tracking_frame_, item.pose.header.frame_id, ros::Time(), ros::Duration(3.0));
                  tf_listener_.lookupTransform(tracking_frame_, item.pose.header.frame_id, ros::Time(), transform_cameraToRobot);
                }
                catch (tf::TransformException &ex)
                {
                  ROS_ERROR("Transform between %s and %s : %s", tracking_frame_.c_str(), item.pose.header.frame_id.c_str(), ex.what());
                  return;
                }
                tf::Transform robotPoseToMap = cameraPoseToMap * transform_cameraToRobot.inverse();

                geometry_msgs::PoseStamped robot_pose_to_map;
                robot_pose_to_map.header = item.pose.header;
                robot_pose_to_map.header.frame_id = map_frame_;
                robot_pose_to_map.pose = transformToPose(robotPoseToMap);

                if (publish_robot_pose_)
                {
                  robot_pose_publisher_.publish(robot_pose_to_map);
                }
              }
            }
          }
        }
      }
    }

    // Publish detected tags as LandmarkList for Cartographer.
    // The cartographer_ros_msgs/LandmarkList should be provided at a sample rate comparable to the other sensors.
    // The list can be empty but has to be provided.
    if (publish_landmarks_)
    {
      cartographer_ros_msgs::LandmarkList landMarkList;

      for (int i = 0; i < tag_detection_array_.detections.size(); i++)
      {
        AprilTagDetection item = tag_detection_array_.detections[i];
        int item_id = item.id[0];
        if (!tag_detector_->checkIDexists(item_id))
        {
          continue;
        }

        cartographer_ros_msgs::LandmarkEntry landMarkEntry;

        geometry_msgs::PoseStamped pose_relative_to_robot;
        geometry_msgs::PoseStamped pose_relative_to_camera;
        pose_relative_to_camera.header = item.pose.header;
        pose_relative_to_camera.pose = item.pose.pose.pose;

        tf::Stamped<tf::Pose> tagPoseToCamera;
        tf::Stamped<tf::Pose> tagPoseToRobot;
        tf::poseStampedMsgToTF(pose_relative_to_camera, tagPoseToCamera);
        try
        {
          tf_listener_.waitForTransform(tracking_frame_, item.pose.header.frame_id, ros::Time(), ros::Duration(3.0));
          tf_listener_.transformPose(tracking_frame_, tagPoseToCamera, tagPoseToRobot);
        }
        catch (tf::TransformException &ex)
        {
          ROS_ERROR("Transform between %s and %s : %s", tracking_frame_.c_str(), item.pose.header.frame_id.c_str(), ex.what());
        }

        tf::poseStampedTFToMsg(tagPoseToRobot, pose_relative_to_robot);

        landMarkEntry.id = std::to_string(item_id);
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

    if (!tag_detector_->published_tf_id_.size() > 0)
    {
      response.write_state = 0;
      ROS_ERROR("Not found transform between %s and any tag: saving failed.", map_frame_.c_str());
      return false;
    }

    for (const int &tag_id : tag_detector_->published_tf_id_)
    {
      std::string tag_frame = "tag_" + std::to_string(tag_id);
      tf::StampedTransform transform_tagToMap;

      try
      {
        tf_listener_.waitForTransform(map_frame_, tag_frame, ros::Time(), ros::Duration(3.0));
        tf_listener_.lookupTransform(map_frame_, tag_frame, ros::Time(), transform_tagToMap);
      }
      catch (const std::exception &e)
      {
        ROS_WARN("Transform between %s and %s : %s", map_frame_.c_str(), tag_frame.c_str(), ex.what());
        continue;
      }

      tf::Vector3 translation = transform_tagToMap.getOrigin();
      tf::Quaternion rotation = transform_tagToMap.getRotation();

      file << tag_id << " " << translation.x() << " " << translation.y() << " " << translation.z() << " "
           << rotation.x() << " " << rotation.y() << " " << rotation.z() << " " << rotation.w() << "\n";
    }

    file.close();
    ROS_INFO("Saving successfully.");
    response.write_state = 1;
    return true;
  }

} // namespace apriltag_ros
