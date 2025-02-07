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
        tf_listener_.waitForTransform(map_frame_, it->pose.header.frame_id, ros::Time(), ros::Duration(3.0));
        tf_listener_.lookupTransform(map_frame_, it->pose.header.frame_id, ros::Time(), transform_cameraToMap);
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