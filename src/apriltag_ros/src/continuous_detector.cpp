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
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <algorithm>
#include <iomanip>

PLUGINLIB_EXPORT_CLASS(apriltag_ros::ContinuousDetector, nodelet::Nodelet);

namespace apriltag_ros
{
  /**
   * @brief Create visualization_msgs::Marker object
   *
   * @param type Type of object
   * @param tag_id Object id
   * @param pose Pose of the object
   * @return visualization_msgs::Marker
   */
  visualization_msgs::Marker ContinuousDetector::createTagMarker(const int type, const int tag_id, const geometry_msgs::Pose pose)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = ros::Time::now();
    std::string ns = "tag_" + std::to_string(type);
    marker.ns = ns;
    marker.id = tag_id;
    marker.type = type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    if (type == 9)
    {
      marker.text = "tag_" + std::to_string(tag_id);
      marker.scale.z = 0.5;
    }

    return marker;
  }

  visualization_msgs::MarkerArray ContinuousDetector::createTagPosesList(const std::vector<TagPose2Map> tagPose2Map)
  {
    visualization_msgs::MarkerArray markerarray;

    for (auto item : tagPose2Map)
    {
      geometry_msgs::Pose pose;
      pose = item.pose.pose;
      markerarray.markers.push_back(createTagMarker(visualization_msgs::Marker::ARROW, item.id, pose));
      pose.position.y = item.pose.pose.position.y + 0.2;
      pose.position.z = item.pose.pose.position.z + 0.2;
      markerarray.markers.push_back(createTagMarker(visualization_msgs::Marker::TEXT_VIEW_FACING, item.id, pose));
    }

    return markerarray;
  }

  void ContinuousDetector::appendTagPosesList(const std::vector<TagPose2Map> tagPose2Map)
  {
    for (auto item : tagPose2Map)
    {
      geometry_msgs::Pose pose;
      pose = item.pose.pose;
      tag_poses_list_.markers.push_back(createTagMarker(visualization_msgs::Marker::ARROW, item.id, pose));
      pose.position.y = item.pose.pose.position.y + 0.2;
      pose.position.z = item.pose.pose.position.z + 0.2;
      tag_poses_list_.markers.push_back(createTagMarker(visualization_msgs::Marker::TEXT_VIEW_FACING, item_id, pose));
    }
  }

  /**
   * @brief Create visualization_msgs::MarkerArray object.
   *
   * @param path Path to the pose.txt file.
   */
  visualization_msgs::MarkerArray ContinuousDetector::createTagPosesList(const std::string path)
  {
    visualization_msgs::MarkerArray markerarray;

    std::ifstream file(path);
    if (!file.is_open())
    {
      ROS_ERROR("Failed to open file %s.", path.c_str());
      return markerarray;
    }

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = map_frame_;
    std::string line;

    while (std::getline(file, line))
    {
      std::istringstream iss(line);
      int tag_id;
      double px, py, pz;
      double ox, oy, oz, ow;

      if (iss >> tag_id >> px >> py >> pz >> ox >> oy >> oz >> ow)
      {
        geometry_msgs::PoseStamped posestamped;
        posestamped.header = header;
        posestamped.pose.position.x = px;
        posestamped.pose.position.y = py;
        posestamped.pose.position.z = pz;
        posestamped.pose.orientation.x = ox;
        posestamped.pose.orientation.y = oy;
        posestamped.pose.orientation.z = oz;
        posestamped.pose.orientation.w = ow;

        tag_poses_to_map_.emplace_back(tag_id, posestamped);
      }
    }

    markerarray = createTagPosesList(tag_poses_to_map_);

    file.close();

    return markerarray;
  }

  /**
   * @brief Initialization
   *
   */
  void ContinuousDetector::onInit()
  {
    ros::NodeHandle &nh = getNodeHandle();
    ros::NodeHandle &pnh = getPrivateNodeHandle();

    tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(pnh));

    it_ = std::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh));

    draw_tag_detections_image_ = getAprilTagOption<bool>(pnh, "publish_tag_detections_image", false);
    publish_landmarks_ = getAprilTagOption<bool>(pnh, "publish_landmarks", true);
    publish_robot_pose_ = getAprilTagOption<bool>(pnh, "publish_robot_pose", true);
    enable_write_tags_service_ = getAprilTagOption<bool>(pnh, "enable_write_tags", false);
    map_frame_ = getAprilTagOption<std::string>(pnh, "map_frame", "map");
    path_to_pose_txt_ = getAprilTagOption<std::string>(pnh, "path_to_pose_txt", "");
    tracking_frame_ = getAprilTagOption<std::string>(pnh, "tracking_frame", "base_link");
    position_tolerance_ = getAprilTagOption<double>(pnh, "position_tolerance", 1.0);          // Unit: meters
    orientation_tolerance_ = getAprilTagOption<double>(pnh, "orientation_tolerance", 0.3491); // Unit: radians

    tag_detections_publisher_ = nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
    robot_pose_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("april_pose", 10);
    tag_poses_list_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("tag_poses_list", 10);
    amcl_pose_subscriber_ = nh.subscribe("amcl_pose", 1000, &ContinuousDetector::amclPoseCallback, this);
    amcl_initialpose_publisher_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

    set_file_service_ = pnh.advertiseService("set_file", &ContinuousDetector::setFileServiceCallback, this);
    write_tags_service_ = pnh.advertiseService("write_tags", &ContinuousDetector::writeTagsServiceCallback, this);

    std::string transport_hint;
    pnh.param<std::string>("transport_hint", transport_hint, "raw");

    int queue_size;
    pnh.param<int>("queue_size", queue_size, 1);
    camera_image_subscriber_ = it_->subscribeCamera("image_rect", queue_size,
                                                    &ContinuousDetector::imageCallback, this,
                                                    image_transport::TransportHints(transport_hint));

    // Check if tf is published
    if (!tag_detector_->get_publish_tf())
    {
      ROS_ERROR("publish_tf must be true to enable write_tags service.");
      return;
    }

    if (!path_to_pose_txt_.empty())
    {
      tag_poses_list_ = createTagPosesList(path_to_pose_txt_);
      tag_poses_list_publisher_.publish(tag_poses_list_);
      robot_pose_to_map_.header.seq = -1;
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

    refresh_params_service_ = pnh.advertiseService("refresh_tag_params", &ContinuousDetector::refreshParamsCallback, this);
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

  /**
   * @brief Convert tf::Transform to geometry_msgs::Pose
   *
   * @param transform tf::Transform
   * @return geometry_msgs::Pose
   */
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

  /**
   * @brief Filter out AprilTagDetection with larger distance.
   *
   * @param tag_detection_array
   * @return AprilTagDetectionArray
   */
  AprilTagDetectionArray ContinuousDetector::AprilTagDetectionFilter(AprilTagDetectionArray &tag_detection_array)
  {
    AprilTagDetectionArray array;
    for (int i = 0; i < tag_detection_array.detections.size(); i++)
    {
      AprilTagDetection item = tag_detection_array.detections[i];
      double dist = sqrt(item.pose.pose.pose.position.x * item.pose.pose.pose.position.x +
                         item.pose.pose.pose.position.y * item.pose.pose.pose.position.y +
                         item.pose.pose.pose.position.z * item.pose.pose.pose.position.z);
      if (dist < 2.0)
      {
        array.detections.push_back(item);
      }
    }
    array.header = tag_detection_array.header;
    return array;
  }

  /**
   * @brief Image callback function
   *
   * @param image_rect Rectified image
   * @param camera_info Camera extrinsics and intrinsics
   */
  void ContinuousDetector::imageCallback(const sensor_msgs::ImageConstPtr &image_rect, const sensor_msgs::CameraInfoConstPtr &camera_info)
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
    AprilTagDetectionArray raw_array;
    raw_array = tag_detector_->detectTags(cv_image_, camera_info);
    tag_detection_array_ = AprilTagDetectionFilter(raw_array);
    tag_detections_publisher_.publish(tag_detection_array_);

    // If enable_write_tags_service_ is true, the write_tags service can be called to save
    // the poses relative to map of all detected tags.
    // If false, the pose relative to map of each detected tag will be used to calculate
    // the pose of the robot relative to map. If the detected tag is not listed in the pose.txt file,
    // its pose will be saved in this file.
    if (path_to_pose_txt_.empty())
    {
      for (int i = 0; i < tag_detection_array_.detections.size(); i++)
      {
        AprilTagDetection item = tag_detection_array_.detections[i];
        int item_id = item.id[0];

        if (!tag_detector_->checkIDexists(item_id))
        {
          continue;
        }

        std::string tag_frame = "tag_" + std::to_string(item_id);

        auto it = std::find_if(tag_poses_to_map_.begin(), tag_poses_to_map_.end(), [item_id](const TagPose2Map &s)
                               { return s.id == item_id; });
        if (it != tag_poses_to_map_.end())
        {
          try
          {
            tf_listener_.waitForTransform(map_frame_, tag_frame, ros::Time(0), ros::Duration(3.0));
            tf_listener_.lookupTransform(map_frame_, tag_frame, ros::Time(0), transform_tagToMap_);
          }
          catch (const std::exception &ex)
          {
            ROS_WARN("Transform between %s and %s : %s", map_frame_.c_str(), tag_frame.c_str(), ex.what());
            continue;
          }

          it->pose.header = item.pose.header;
          it->pose.header.frame_id = map_frame_;
          it->pose.pose = transformToPose(transform_tagToMap_);
        }
        else
        {
          try
          {
            tf_listener_.waitForTransform(map_frame_, tag_frame, ros::Time(0), ros::Duration(3.0));
            tf_listener_.lookupTransform(map_frame_, tag_frame, ros::Time(0), transform_tagToMap_);
          }
          catch (const std::exception &ex)
          {
            ROS_WARN("Transform between %s and %s : %s", map_frame_.c_str(), tag_frame.c_str(), ex.what());
            continue;
          }

          geometry_msgs::PoseStamped pose_stamp;
          pose_stamp.header = item.pose.header;
          pose_stamp.header.frame_id = map_frame_;
          pose_stamp.pose = transformToPose(transform_tagToMap_);

          tag_poses_to_map_.emplace_back(item_id, pose_stamp);
        }
      }
      tag_poses_list_ = createTagPosesList(tag_poses_to_map_);
    }
    else
    {
      for (int i = 0; i < tag_detection_array_.detections.size(); i++)
      {
        AprilTagDetection item = tag_detection_array_.detections[i];
        int item_id = item.id[0];
        bool hasFound = false;

        if (!tag_detector_->checkIDexists(item_id))
        {
          continue;
        }

        for (auto elem : tag_poses_to_map_)
        {
          if (item_id == elem.id)
          {
            hasFound = true;

            geometry_msgs::PoseStamped tag_pose_to_camera;
            geometry_msgs::PoseStamped camera_pose_to_map;
            tag_pose_to_camera.header = item.pose.header;
            tag_pose_to_camera.pose = item.pose.pose.pose;
            elem.pose.header.stamp = tag_pose_to_camera.header.stamp;

            tf::Stamped<tf::Pose> tagPoseToMap, tagPoseToCamera;
            tf::poseStampedMsgToTF(elem.pose, tagPoseToMap);
            tf::poseStampedMsgToTF(tag_pose_to_camera, tagPoseToCamera);
            tf::Pose cameraPoseToTag = tagPoseToCamera.inverse();
            tf::Pose cameraPoseToMap = tagPoseToMap * cameraPoseToTag;

            try
            {
              tf_listener_.waitForTransform(tracking_frame_, item.pose.header.frame_id, ros::Time(0), ros::Duration(3.0));
              tf_listener_.lookupTransform(tracking_frame_, item.pose.header.frame_id, ros::Time(0), transform_cameraToRobot_);
            }
            catch (tf::TransformException &ex)
            {
              ROS_ERROR("Transform between %s and %s : %s", tracking_frame_.c_str(), item.pose.header.frame_id.c_str(), ex.what());
              break;
            }

            tf::Transform robotPoseToMap = cameraPoseToMap * transform_cameraToRobot_.inverse();

            robot_pose_to_map_.header = item.pose.header;
            robot_pose_to_map_.header.frame_id = map_frame_;
            robot_pose_to_map_.pose = transformToPose(robotPoseToMap);
            robot_pose_to_map_.pose.position.z = 0.0;
            robot_pose_to_map_.pose.orientation.x = 0.0;
            robot_pose_to_map_.pose.orientation.y = 0.0;

            if (publish_robot_pose_)
            {
              robot_pose_publisher_.publish(robot_pose_to_map_);
            }
            break;
          }
        }

        if (!hasFound)
        {
          std::string tag_frame = "tag_" + std::to_string(item_id);
          try
          {
            tf_listener_.waitForTransform(map_frame_, tag_frame, ros::Time(0), ros::Duration(3.0));
            tf_listener_.lookupTransform(map_frame_, tag_frame, ros::Time(0), transform_tagToMap_);
          }
          catch (const std::exception &ex)
          {
            ROS_WARN("Transform between %s and %s : %s", map_frame_.c_str(), tag_frame.c_str(), ex.what());
            continue;
          }

          auto it = std::find_if(new_tag_poses_to_map_.begin(), new_tag_poses_to_map_.end(),
                                 [item_id](const TagPose2Map &s)
                                 { return s.id == item_id; });

          if (it != new_tag_poses_to_map_.end())
          {
            it->pose.header.stamp = item.pose.header.stamp;
            it->pose.pose = transformToPose(transform_tagToMap_);
            ROS_INFO("New tag (id: %d) has been updated.", item_id);
          }
          else
          {
            geometry_msgs::PoseStamped posestamped;
            posestamped.header.stamp = item.pose.header.stamp;
            posestamped.header.frame_id = map_frame_;
            posestamped.pose = transformToPose(transform_tagToMap_);
            new_tag_poses_to_map_.emplace_back(item_id, posestamped);
            ROS_INFO("New tag (id: %d) has been added.", item_id);
          }

          appendTagPosesList(new_tag_poses_to_map_);
        }
      }
    }

    tag_poses_list_publisher_.publish(tag_poses_list_);

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

  /**
   * @brief write_tags service callback function. The service will save the poses of tags in a .txt file.
   *
   * @param request
   * @param response
   * @return true
   * @return false
   */
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

    if (!tag_poses_to_map_.size() > 0 && !new_tag_poses_to_map_.size() > 0)
    {
      response.write_state = 0;
      ROS_ERROR("Not found transform between %s and any tag: saving failed.", map_frame_.c_str());
      return false;
    }

    file << std::fixed << std::setprecision(13);

    if (filename == path_to_pose_txt_)
    {
      for (const TagPose2Map &item : new_tag_poses_to_map_)
      {
        file << item.id << " "
             << item.pose.pose.position.x << " "
             << item.pose.pose.position.y << " "
             << item.pose.pose.position.z << " "
             << item.pose.pose.orientation.x << " "
             << item.pose.pose.orientation.y << " "
             << item.pose.pose.orientation.z << " "
             << item.pose.pose.orientation.w << "\n";

        tag_poses_to_map_.emplace_back(item.id, item.pose);
      }
      new_tag_poses_to_map_.clear();
    }
    else
    {
      for (const TagPose2Map &item : tag_poses_to_map_)
      {
        file << item.id << " "
             << item.pose.pose.position.x << " "
             << item.pose.pose.position.y << " "
             << item.pose.pose.position.z << " "
             << item.pose.pose.orientation.x << " "
             << item.pose.pose.orientation.y << " "
             << item.pose.pose.orientation.z << " "
             << item.pose.pose.orientation.w << "\n";
      }
      for (const TagPose2Map &item : new_tag_poses_to_map_)
      {
        file << item.id << " "
             << item.pose.pose.position.x << " "
             << item.pose.pose.position.y << " "
             << item.pose.pose.position.z << " "
             << item.pose.pose.orientation.x << " "
             << item.pose.pose.orientation.y << " "
             << item.pose.pose.orientation.z << " "
             << item.pose.pose.orientation.w << "\n";
      }
    }

    file.close();
    ROS_INFO("Saving successfully.");
    response.write_state = 1;
    return true;
  }

  /**
   * @brief set_file service callback function. The service will change the path to the target .txt file.
   *
   * @param request
   * @param response
   * @return true
   * @return false
   */
  bool ContinuousDetector::setFileServiceCallback(apriltag_ros::SetFile::Request &request, apriltag_ros::SetFile::Response &response)
  {
    std::string filename = request.file_directory + "/" + request.file_basename + ".txt";
    std::ifstream file(filename);
    if (file.is_open())
    {
      path_to_pose_txt_ = filename;
      tag_poses_to_map_.clear();
      tag_poses_list_ = createTagPosesList(path_to_pose_txt_);
      tag_poses_list_publisher_.publish(tag_poses_list_);
      response.set_state = 1;
      file.close();
      return true;
    }
    else
    {
      ROS_ERROR("Cannot change to the target file.");
      response.set_state = 0;
      return false;
    }
  }

  /**
   * @brief amcl_pose message callback function.
   *
   * @param msg
   */
  void ContinuousDetector::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
  {
    ros::Time amcl_pose_stamp = msg->header.stamp;
    ros::Time april_pose_stamp = robot_pose_to_map_.header.stamp;
    const double sync_threshold = 0.1; // Unit: second
    if (robot_pose_to_map_.header.seq == -1)
    {
      // robot_pose_to_map_ is not initialized.
      ROS_INFO("No poses of robot relative to map is obtained.");
      return;
    }

    if ((amcl_pose_stamp - april_pose_stamp).toSec() < sync_threshold && (april_pose_stamp - amcl_pose_stamp).toSec() < sync_threshold)
    {
      double dx = msg->pose.pose.position.x - robot_pose_to_map_.pose.position.x;
      double dy = msg->pose.pose.position.y - robot_pose_to_map_.pose.position.y;
      double position_diff = std::sqrt(dx * dx + dy * dy);

      tf::Quaternion q1(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
      tf::Quaternion q2(robot_pose_to_map_.pose.orientation.x, robot_pose_to_map_.pose.orientation.y, robot_pose_to_map_.pose.orientation.z, robot_pose_to_map_.pose.orientation.w);
      tf::Quaternion orientation_diff = q1.inverse() * q2;
      double roll, pitch, yaw;
      tf::Matrix3x3 m(orientation_diff);
      m.getRPY(roll, pitch, yaw);

      if (position_diff > position_tolerance_ || yaw > orientation_tolerance_)
      {
        geometry_msgs::PoseWithCovarianceStamped pose_to_be_published;
        pose_to_be_published.header.frame_id = map_frame_;
        pose_to_be_published.header.stamp = ros::Time::now();
        pose_to_be_published.pose.pose = robot_pose_to_map_.pose;
        for (int i = 0; i < 36; i++)
        {
          pose_to_be_published.pose.covariance[i] = 0.0;
        }

        amcl_initialpose_publisher_.publish(pose_to_be_published);
        ROS_INFO("New robot pose is published.");
      }
    }
    else
    {
      return;
    }
  }

} // namespace apriltag_ros
