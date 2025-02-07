auto it = std::find_if(tag_poses_to_camera_.begin(), tag_poses_to_camera_.end(),
                       [tag_id](const TagPose2Camera &s)
                       { return s.id == tag_id; });
if (it != tag_poses_to_camera_.end())
{
    tf::Stamped<tf::Pose> tagPoseToCamera;
    tf::Transform tagPoseToMap;
    tf::StampedTransform transform_cameraToMap;
    tf::poseStampedMsgToTF(it->pose, tagPoseToCamera);
    try
    {
        tf_listener_.waitForTransform(map_frame_, it->pose.header.frame_id, ros::Time(0), ros::Duration(3.0));
        tf_listener_.lookupTransform(map_frame_, it->pose.header.frame_id, ros::Time(0), transform_cameraToMap);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("Transform between %s and %s : %s", map_frame_.c_str(), it->pose.header.frame_id.c_str(), ex.what());
        response.write_state = 0;
        return false;
    }
    tagPoseToMap = tagPoseToCamera * transform_cameraToMap;
    tf::Vector3 translation = tagPoseToMap.getOrigin();
    tf::Quaternion rotation = tagPoseToMap.getRotation();
    file << tag_id << " " << translation.x() << " " << translation.y() << " " << translation.z() << " "
         << rotation.x() << " " << rotation.y() << " " << rotation.z() << " " << rotation.w() << "\n";
}
else
{
    ROS_WARN("Cannot retrieve tag %d from detected tags", tag_id);
    continue;
}

for (const int &tag_id : tag_detector_->published_tf_id_)
{
    std::string tag_frame = "tag_" + std::to_string(tag_id);
    tf::StampedTransform transform_tagToMap;

    try
    {
        tf_listener_.waitForTransform(map_frame_, tag_frame, ros::Time(0), ros::Duration(3.0));
        tf_listener_.lookupTransform(map_frame_, tag_frame, ros::Time(0), transform_tagToMap);
    }
    catch (const std::exception &ex)
    {
        ROS_WARN("Transform between %s and %s : %s", map_frame_.c_str(), tag_frame.c_str(), ex.what());
        continue;
    }

    tf::Vector3 translation = transform_tagToMap.getOrigin();
    tf::Quaternion rotation = transform_tagToMap.getRotation();

    file << tag_id << " " << translation.x() << " " << translation.y() << " " << translation.z() << " "
         << rotation.x() << " " << rotation.y() << " " << rotation.z() << " " << rotation.w() << "\n";
}