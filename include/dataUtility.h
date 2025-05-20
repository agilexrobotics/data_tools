#pragma once
#ifndef _DATA_UTILITY_H_
#define _DATA_UTILITY_H_

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <deque>
#ifdef _USENOETIC
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#else
#include <opencv/cv.h>
#endif

class DataUtility
{
public:

    ros::NodeHandle nh;

    std::vector<std::string> cameraColorNames;
    std::vector<std::string> cameraDepthNames;
    std::vector<std::string> cameraPointCloudNames;
    std::vector<std::string> armJointStateNames;
    std::vector<std::string> armEndPoseNames;
    std::vector<std::string> localizationPoseNames;
    std::vector<std::string> gripperEncoderNames;
    std::vector<std::string> imu9AxisNames;
    std::vector<std::string> lidarPointCloudNames;
    std::vector<std::string> robotBaseVelNames;
    std::vector<std::string> liftMotorNames;

    std::vector<std::string> cameraColorTopics;
    std::vector<std::string> cameraDepthTopics;
    std::vector<std::string> cameraPointCloudTopics;
    std::vector<std::string> armJointStateTopics;
    std::vector<std::string> armEndPoseTopics;
    std::vector<std::string> localizationPoseTopics;
    std::vector<std::string> gripperEncoderTopics;
    std::vector<std::string> imu9AxisTopics;
    std::vector<std::string> lidarPointCloudTopics;
    std::vector<std::string> robotBaseVelTopics;
    std::vector<std::string> liftMotorTopics;

    std::vector<std::string> cameraColorConfigTopics;
    std::vector<std::string> cameraDepthConfigTopics;
    std::vector<std::string> cameraPointCloudConfigTopics;
    std::vector<std::string> armJointStateConfigTopics;
    std::vector<std::string> armEndPoseConfigTopics;
    std::vector<std::string> localizationPoseConfigTopics;
    std::vector<std::string> gripperEncoderConfigTopics;
    std::vector<std::string> imu9AxisConfigTopics;
    std::vector<std::string> lidarPointCloudConfigTopics;
    std::vector<std::string> robotBaseVelConfigTopics;
    std::vector<std::string> liftMotorConfigTopics;

    std::vector<std::string> cameraColorPublishTopics;
    std::vector<std::string> cameraDepthPublishTopics;
    std::vector<std::string> cameraPointCloudPublishTopics;
    std::vector<std::string> armJointStatePublishTopics;
    std::vector<std::string> armEndPosePublishTopics;
    std::vector<std::string> localizationPosePublishTopics;
    std::vector<std::string> gripperEncoderPublishTopics;
    std::vector<std::string> imu9AxisPublishTopics;
    std::vector<std::string> lidarPointCloudPublishTopics;
    std::vector<std::string> robotBaseVelPublishTopics;
    std::vector<std::string> liftMotorPublishTopics;

    std::vector<std::string> cameraColorConfigPublishTopics;
    std::vector<std::string> cameraDepthConfigPublishTopics;
    std::vector<std::string> cameraPointCloudConfigPublishTopics;
    std::vector<std::string> armJointStateConfigPublishTopics;
    std::vector<std::string> armEndPoseConfigPublishTopics;
    std::vector<std::string> localizationPoseConfigPublishTopics;
    std::vector<std::string> gripperEncoderConfigPublishTopics;
    std::vector<std::string> imu9AxisConfigPublishTopics;
    std::vector<std::string> lidarPointCloudConfigPublishTopics;
    std::vector<std::string> robotBaseVelConfigPublishTopics;
    std::vector<std::string> liftMotorConfigPublishTopics;

    std::vector<std::string> cameraColorParentFrames;
    std::vector<std::string> cameraDepthParentFrames;
    std::vector<std::string> cameraPointCloudParentFrames;
    std::vector<std::string> armJointStateParentFrames;
    std::vector<std::string> armEndPoseParentFrames;
    std::vector<std::string> localizationPoseParentFrames;
    std::vector<std::string> gripperEncoderParentFrames;
    std::vector<std::string> imu9AxisParentFrames;
    std::vector<std::string> lidarPointCloudParentFrames;
    std::vector<std::string> robotBaseVelParentFrames;
    std::vector<std::string> liftMotorParentFrames;

    std::vector<std::string> tfTransformParentFrames;
    std::vector<std::string> tfTransformChildFrames;

    std::vector<std::string> cameraColorDirs;
    std::vector<std::string> cameraDepthDirs;
    std::vector<std::string> cameraPointCloudDirs;
    std::vector<std::string> armJointStateDirs;
    std::vector<std::string> armEndPoseDirs;
    std::vector<std::string> localizationPoseDirs;
    std::vector<std::string> gripperEncoderDirs;
    std::vector<std::string> imu9AxisDirs;
    std::vector<std::string> lidarPointCloudDirs;
    std::vector<std::string> robotBaseVelDirs;
    std::vector<std::string> liftMotorDirs;
    std::vector<std::string> tfTransformDirs;
    std::string instructionsDir;
    std::string statisticsDir;

    std::vector<bool> cameraColorToSyncs;
    std::vector<bool> cameraDepthToSyncs;
    std::vector<bool> cameraPointCloudToSyncs;
    std::vector<bool> armJointStateToSyncs;
    std::vector<bool> armEndPoseToSyncs;
    std::vector<bool> localizationPoseToSyncs;
    std::vector<bool> gripperEncoderToSyncs;
    std::vector<bool> imu9AxisToSyncs;
    std::vector<bool> lidarPointCloudToSyncs;
    std::vector<bool> robotBaseVelToSyncs;
    std::vector<bool> liftMotorToSyncs;

    std::vector<bool> cameraColorToPublishs;
    std::vector<bool> cameraDepthToPublishs;
    std::vector<bool> cameraPointCloudToPublishs;
    std::vector<bool> armJointStateToPublishs;
    std::vector<bool> armEndPoseToPublishs;
    std::vector<bool> localizationPoseToPublishs;
    std::vector<bool> gripperEncoderToPublishs;
    std::vector<bool> imu9AxisToPublishs;
    std::vector<bool> lidarPointCloudToPublishs;
    std::vector<bool> robotBaseVelToPublishs;
    std::vector<bool> liftMotorToPublishs;
    std::vector<bool> tfTransformToPublishs;

    std::vector<float> cameraPointCloudMaxDistances;
    std::vector<float> cameraPointCloudDownSizes;

    std::vector<float> lidarPointCloudXDistanceUppers;
    std::vector<float> lidarPointCloudXDistancelowers;
    std::vector<float> lidarPointCloudYDistanceUppers;
    std::vector<float> lidarPointCloudYDistancelowers;
    std::vector<float> lidarPointCloudZDistanceUppers;
    std::vector<float> lidarPointCloudZDistancelowers;
    std::vector<float> lidarPointCloudDownSizes;

    std::vector<bool> armEndPoseOrients;

    float publishRate;
    int captureFrameNum;

    std::string datasetDir;
    int episodeIndex;

    std::string episodeDir;

    std::string cameraDir;
    std::string armDir;
    std::string localizationDir;
    std::string gripperDir;
    std::string imuDir;
    std::string lidarDir;
    std::string robotBaseDir;
    std::string liftDir;
    std::string tfDir;

    std::string cameraColorDir;
    std::string cameraDepthDir;
    std::string cameraPointCloudDir;

    std::string armJointStateDir;
    std::string armEndPoseDir;

    std::string localizationPoseDir;

    std::string gripperEncoderDir;

    std::string imu9AxisDir;

    std::string lidarPointCloudDir;

    std::string robotBaseVelDir;

    std::string liftMotorDir;

    std::string tfTransformDir;

    DataUtility(std::string datasetDirParam, int episodeIndexParam)
    {
        nh.param<std::vector<std::string>>("dataInfo/camera/color/names", cameraColorNames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/depth/names", cameraDepthNames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/pointCloud/names", cameraPointCloudNames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/jointState/names", armJointStateNames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/endPose/names", armEndPoseNames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/localization/pose/names", localizationPoseNames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/gripper/encoder/names", gripperEncoderNames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/imu/9axis/names", imu9AxisNames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/lidar/pointCloud/names", lidarPointCloudNames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/robotBase/vel/names", robotBaseVelNames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/lift/motor/names", liftMotorNames, std::vector<std::string>());

        nh.param<std::vector<std::string>>("dataInfo/camera/color/parentFrames", cameraColorParentFrames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/depth/parentFrames", cameraDepthParentFrames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/pointCloud/parentFrames", cameraPointCloudParentFrames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/jointState/parentFrames", armJointStateParentFrames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/endPose/parentFrames", armEndPoseParentFrames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/localization/pose/parentFrames", localizationPoseParentFrames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/gripper/encoder/parentFrames", gripperEncoderParentFrames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/imu/9axis/parentFrames", imu9AxisParentFrames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/lidar/pointCloud/parentFrames", lidarPointCloudParentFrames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/robotBase/vel/parentFrames", robotBaseVelParentFrames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/lift/motor/parentFrames", liftMotorParentFrames, std::vector<std::string>());

        nh.param<std::vector<std::string>>("dataInfo/tf/transform/parentFrames", tfTransformParentFrames, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/tf/transform/childFrames", tfTransformChildFrames, std::vector<std::string>());

        nh.param<std::vector<std::string>>("dataInfo/camera/color/topics", cameraColorTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/depth/topics", cameraDepthTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/pointCloud/topics", cameraPointCloudTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/jointState/topics", armJointStateTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/endPose/topics", armEndPoseTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/localization/pose/topics", localizationPoseTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/gripper/encoder/topics", gripperEncoderTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/imu/9axis/topics", imu9AxisTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/lidar/pointCloud/topics", lidarPointCloudTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/robotBase/vel/topics", robotBaseVelTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/lift/motor/topics", liftMotorTopics, std::vector<std::string>());

        nh.param<std::vector<std::string>>("dataInfo/camera/color/pubTopics", cameraColorPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/depth/pubTopics", cameraDepthPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/pointCloud/pubTopics", cameraPointCloudPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/jointState/pubTopics", armJointStatePublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/endPose/pubTopics", armEndPosePublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/localization/pose/pubTopics", localizationPosePublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/gripper/encoder/pubTopics", gripperEncoderPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/imu/9axis/pubTopics", imu9AxisPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/lidar/pointCloud/pubTopics", lidarPointCloudPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/robotBase/vel/pubTopics", robotBaseVelPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/lift/motor/pubTopics", liftMotorPublishTopics, std::vector<std::string>());
        cameraColorPublishTopics = cameraColorPublishTopics.size() == 0 ? cameraColorTopics :cameraColorPublishTopics;
        cameraDepthPublishTopics = cameraDepthPublishTopics.size() == 0 ? cameraDepthTopics :cameraDepthPublishTopics;
        cameraPointCloudPublishTopics = cameraPointCloudPublishTopics.size() == 0 ? cameraPointCloudTopics :cameraPointCloudPublishTopics;
        armJointStatePublishTopics = armJointStatePublishTopics.size() == 0 ? armJointStateTopics :armJointStatePublishTopics;
        armEndPosePublishTopics = armEndPosePublishTopics.size() == 0 ? armEndPoseTopics :armEndPosePublishTopics;
        localizationPosePublishTopics = localizationPosePublishTopics.size() == 0 ? localizationPoseTopics :localizationPosePublishTopics;
        gripperEncoderPublishTopics = gripperEncoderPublishTopics.size() == 0 ? gripperEncoderTopics :gripperEncoderPublishTopics;
        imu9AxisPublishTopics = imu9AxisPublishTopics.size() == 0 ? imu9AxisTopics :imu9AxisPublishTopics;
        lidarPointCloudPublishTopics = lidarPointCloudPublishTopics.size() == 0 ? lidarPointCloudTopics :lidarPointCloudPublishTopics;
        robotBaseVelPublishTopics = robotBaseVelPublishTopics.size() == 0 ? robotBaseVelTopics :robotBaseVelPublishTopics;
        liftMotorPublishTopics = liftMotorPublishTopics.size() == 0 ? liftMotorTopics :liftMotorPublishTopics;

        nh.param<std::vector<std::string>>("dataInfo/camera/color/configTopics", cameraColorConfigTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/depth/configTopics", cameraDepthConfigTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/pointCloud/configTopics", cameraPointCloudConfigTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/jointState/configTopics", armJointStateConfigTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/endPose/configTopics", armEndPoseConfigTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/localization/pose/configTopics", localizationPoseConfigTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/gripper/encoder/configTopics", gripperEncoderConfigTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/imu/9axis/configTopics", imu9AxisConfigTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/lidar/pointCloud/configTopics", lidarPointCloudConfigTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/robotBase/vel/configTopics", robotBaseVelConfigTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/lift/motor/configTopics", liftMotorConfigTopics, std::vector<std::string>());

        nh.param<std::vector<std::string>>("dataInfo/camera/color/pubConfigTopics", cameraColorConfigPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/depth/pubConfigTopics", cameraDepthConfigPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/camera/pointCloud/pubConfigTopics", cameraPointCloudConfigPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/jointState/pubConfigTopics", armJointStateConfigPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/arm/endPose/pubConfigTopics", armEndPoseConfigPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/localization/pose/pubConfigTopics", localizationPoseConfigPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/gripper/encoder/pubConfigTopics", gripperEncoderConfigPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/imu/9axis/pubConfigTopics", imu9AxisConfigPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/lidar/pointCloud/pubConfigTopics", lidarPointCloudConfigPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/robotBase/vel/pubConfigTopics", robotBaseVelConfigPublishTopics, std::vector<std::string>());
        nh.param<std::vector<std::string>>("dataInfo/lift/motor/pubConfigTopics", liftMotorConfigPublishTopics, std::vector<std::string>());
        cameraColorConfigPublishTopics = cameraColorConfigPublishTopics.size() == 0 ? cameraColorConfigTopics :cameraColorConfigPublishTopics;
        cameraDepthConfigPublishTopics = cameraDepthConfigPublishTopics.size() == 0 ? cameraDepthConfigTopics :cameraDepthConfigPublishTopics;
        cameraPointCloudConfigPublishTopics = cameraPointCloudConfigPublishTopics.size() == 0 ? cameraPointCloudConfigTopics :cameraPointCloudConfigPublishTopics;
        armJointStateConfigPublishTopics = armJointStateConfigPublishTopics.size() == 0 ? armJointStateConfigTopics :armJointStateConfigPublishTopics;
        armEndPoseConfigPublishTopics = armEndPoseConfigPublishTopics.size() == 0 ? armEndPoseConfigTopics :armEndPoseConfigPublishTopics;
        localizationPoseConfigPublishTopics = localizationPoseConfigPublishTopics.size() == 0 ? localizationPoseConfigTopics :localizationPoseConfigPublishTopics;
        gripperEncoderConfigPublishTopics = gripperEncoderConfigPublishTopics.size() == 0 ? gripperEncoderConfigTopics :gripperEncoderConfigPublishTopics;
        imu9AxisConfigPublishTopics = imu9AxisConfigPublishTopics.size() == 0 ? imu9AxisConfigTopics :imu9AxisConfigPublishTopics;
        lidarPointCloudConfigPublishTopics = lidarPointCloudConfigPublishTopics.size() == 0 ? lidarPointCloudConfigTopics :lidarPointCloudConfigPublishTopics;
        robotBaseVelConfigPublishTopics = robotBaseVelConfigPublishTopics.size() == 0 ? robotBaseVelConfigTopics :robotBaseVelConfigPublishTopics;
        liftMotorConfigPublishTopics = liftMotorConfigPublishTopics.size() == 0 ? liftMotorConfigTopics :liftMotorConfigPublishTopics;

        nh.param<std::vector<bool>>("dataInfo/camera/color/toSyncs", cameraColorToSyncs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/camera/depth/toSyncs", cameraDepthToSyncs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/camera/pointCloud/toSyncs", cameraPointCloudToSyncs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/arm/jointState/toSyncs", armJointStateToSyncs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/arm/endPose/toSyncs", armEndPoseToSyncs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/localization/pose/toSyncs", localizationPoseToSyncs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/gripper/encoder/toSyncs", gripperEncoderToSyncs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/imu/9axis/toSyncs", imu9AxisToSyncs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/lidar/pointCloud/toSyncs", lidarPointCloudToSyncs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/robotBase/vel/toSyncs", robotBaseVelToSyncs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/lift/motor/toSyncs", liftMotorToSyncs, std::vector<bool>());
        cameraColorToSyncs = cameraColorToSyncs.size() == 0 ? std::vector<bool>(cameraColorTopics.size(), true) : cameraColorToSyncs;
        cameraDepthToSyncs = cameraDepthToSyncs.size() == 0 ? std::vector<bool>(cameraDepthTopics.size(), true) : cameraDepthToSyncs;
        cameraPointCloudToSyncs = cameraPointCloudToSyncs.size() == 0 ? std::vector<bool>(cameraPointCloudTopics.size(), true) : cameraPointCloudToSyncs;
        armJointStateToSyncs = armJointStateToSyncs.size() == 0 ? std::vector<bool>(armJointStateTopics.size(), true) : armJointStateToSyncs;
        armEndPoseToSyncs = armEndPoseToSyncs.size() == 0 ? std::vector<bool>(armEndPoseTopics.size(), true) : armEndPoseToSyncs;
        localizationPoseToSyncs = localizationPoseToSyncs.size() == 0 ? std::vector<bool>(localizationPoseTopics.size(), true) : localizationPoseToSyncs;
        gripperEncoderToSyncs = gripperEncoderToSyncs.size() == 0 ? std::vector<bool>(gripperEncoderTopics.size(), true) : gripperEncoderToSyncs;
        imu9AxisToSyncs = imu9AxisToSyncs.size() == 0 ? std::vector<bool>(imu9AxisTopics.size(), true) : imu9AxisToSyncs;
        lidarPointCloudToSyncs = lidarPointCloudToSyncs.size() == 0 ? std::vector<bool>(lidarPointCloudTopics.size(), true) : lidarPointCloudToSyncs;
        robotBaseVelToSyncs = robotBaseVelToSyncs.size() == 0 ? std::vector<bool>(robotBaseVelTopics.size(), true) : robotBaseVelToSyncs;
        liftMotorToSyncs = liftMotorToSyncs.size() == 0 ? std::vector<bool>(liftMotorTopics.size(), true) : liftMotorToSyncs;

        nh.param<std::vector<bool>>("dataInfo/camera/color/toPublishs", cameraColorToPublishs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/camera/depth/toPublishs", cameraDepthToPublishs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/camera/pointCloud/toPublishs", cameraPointCloudToPublishs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/arm/jointState/toPublishs", armJointStateToPublishs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/arm/endPose/toPublishs", armEndPoseToPublishs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/localization/pose/toPublishs", localizationPoseToPublishs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/gripper/encoder/toPublishs", gripperEncoderToPublishs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/imu/9axis/toPublishs", imu9AxisToPublishs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/lidar/pointCloud/toPublishs", lidarPointCloudToPublishs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/robotBase/vel/toPublishs", robotBaseVelToPublishs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/lift/motor/toPublishs", liftMotorToPublishs, std::vector<bool>());
        nh.param<std::vector<bool>>("dataInfo/tf/transform/toPublishs", tfTransformToPublishs, std::vector<bool>());
        cameraColorToPublishs = cameraColorToPublishs.size() == 0 ? std::vector<bool>(cameraColorTopics.size(), true) : cameraColorToPublishs;
        cameraDepthToPublishs = cameraDepthToPublishs.size() == 0 ? std::vector<bool>(cameraDepthTopics.size(), true) : cameraDepthToPublishs;
        cameraPointCloudToPublishs = cameraPointCloudToPublishs.size() == 0 ? std::vector<bool>(cameraPointCloudTopics.size(), true) : cameraPointCloudToPublishs;
        armJointStateToPublishs = armJointStateToPublishs.size() == 0 ? std::vector<bool>(armJointStateTopics.size(), true) : armJointStateToPublishs;
        armEndPoseToPublishs = armEndPoseToPublishs.size() == 0 ? std::vector<bool>(armEndPoseTopics.size(), true) : armEndPoseToPublishs;
        localizationPoseToPublishs = localizationPoseToPublishs.size() == 0 ? std::vector<bool>(localizationPoseTopics.size(), true) : localizationPoseToPublishs;
        gripperEncoderToPublishs = gripperEncoderToPublishs.size() == 0 ? std::vector<bool>(gripperEncoderTopics.size(), true) : gripperEncoderToPublishs;
        imu9AxisToPublishs = imu9AxisToPublishs.size() == 0 ? std::vector<bool>(imu9AxisTopics.size(), true) : imu9AxisToPublishs;
        lidarPointCloudToPublishs = lidarPointCloudToPublishs.size() == 0 ? std::vector<bool>(lidarPointCloudTopics.size(), true) : lidarPointCloudToPublishs;
        robotBaseVelToPublishs = robotBaseVelToPublishs.size() == 0 ? std::vector<bool>(robotBaseVelTopics.size(), true) : robotBaseVelToPublishs;
        liftMotorToPublishs = liftMotorToPublishs.size() == 0 ? std::vector<bool>(liftMotorTopics.size(), true) : liftMotorToPublishs;
        tfTransformToPublishs = tfTransformToPublishs.size() == 0 ? std::vector<bool>(tfTransformParentFrames.size(), true) : robotBaseVelToPublishs;

        nh.param<std::vector<float>>("dataInfo/camera/pointCloud/maxDistances", cameraPointCloudMaxDistances, std::vector<float>());
        nh.param<std::vector<float>>("dataInfo/camera/pointCloud/downSizes", cameraPointCloudDownSizes, std::vector<float>());

        nh.param<std::vector<float>>("dataInfo/lidar/pointCloud/xDistanceUppers", lidarPointCloudXDistanceUppers, std::vector<float>());
        nh.param<std::vector<float>>("dataInfo/lidar/pointCloud/xDistanceLowers", lidarPointCloudXDistancelowers, std::vector<float>());
        nh.param<std::vector<float>>("dataInfo/lidar/pointCloud/yDistanceUppers", lidarPointCloudYDistanceUppers, std::vector<float>());
        nh.param<std::vector<float>>("dataInfo/lidar/pointCloud/yDistanceLowers", lidarPointCloudYDistancelowers, std::vector<float>());
        nh.param<std::vector<float>>("dataInfo/lidar/pointCloud/zDistanceUppers", lidarPointCloudZDistanceUppers, std::vector<float>());
        nh.param<std::vector<float>>("dataInfo/lidar/pointCloud/zDistanceLowers", lidarPointCloudZDistancelowers, std::vector<float>());
        nh.param<std::vector<float>>("dataInfo/lidar/pointCloud/downSizes", lidarPointCloudDownSizes, std::vector<float>());
        
        nh.param<std::vector<bool>>("dataInfo/arm/endPose/orients", armEndPoseOrients, std::vector<bool>());
        armEndPoseOrients = armEndPoseOrients.size() == 0 ? std::vector<bool>(armEndPoseNames.size(), true) : armEndPoseOrients;

        datasetDir = datasetDirParam;
        episodeIndex = episodeIndexParam;

        episodeDir = datasetDir + "/episode" + std::to_string(episodeIndex);
        // episodeDir = episodeDir.replace("//", "/");

        cameraDir = episodeDir + "/camera";
        armDir = episodeDir + "/arm";
        localizationDir = episodeDir + "/localization";
        gripperDir = episodeDir + "/gripper";
        imuDir = episodeDir + "/imu";
        lidarDir = episodeDir + "/lidar";
        robotBaseDir = episodeDir + "/robotBase";
        liftDir = episodeDir + "/lift";
        tfDir = episodeDir + "/tf";

        cameraColorDir = cameraDir + "/color";
        cameraDepthDir = cameraDir + "/depth";
        cameraPointCloudDir = cameraDir + "/pointCloud";
        armJointStateDir = armDir + "/jointState";
        armEndPoseDir = armDir + "/endPose";
        localizationPoseDir = localizationDir + "/pose";
        gripperEncoderDir = gripperDir + "/encoder";
        imu9AxisDir = imuDir + "/9axis";
        lidarPointCloudDir = lidarDir + "/pointCloud";
        robotBaseVelDir = robotBaseDir + "/vel";
        liftMotorDir = liftDir + "/motor";
        tfTransformDir = tfDir + "/transform";

        instructionsDir = episodeDir + "/instructions.json";
        statisticsDir = episodeDir + "/statistic.txt";

        for(int i = 0; i < cameraColorNames.size(); i++){
            std::string dir = cameraColorDir + "/" + cameraColorNames.at(i);
            cameraColorDirs.push_back(dir);
        }
        for(int i = 0; i < cameraDepthNames.size(); i++){
            std::string dir = cameraDepthDir + "/" + cameraDepthNames.at(i);
            cameraDepthDirs.push_back(dir);
        }
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            std::string dir = cameraPointCloudDir + "/" + cameraPointCloudNames.at(i);
            cameraPointCloudDirs.push_back(dir);
        }
        for(int i = 0; i < armJointStateNames.size(); i++){
            std::string dir = armJointStateDir + "/" + armJointStateNames.at(i);
            armJointStateDirs.push_back(dir);
        }
        for(int i = 0; i < armEndPoseNames.size(); i++){
            std::string dir = armEndPoseDir + "/" + armEndPoseNames.at(i);
            armEndPoseDirs.push_back(dir);
        }
        for(int i = 0; i < localizationPoseNames.size(); i++){
            std::string dir = localizationPoseDir + "/" + localizationPoseNames.at(i);
            localizationPoseDirs.push_back(dir);
        }
        for(int i = 0; i < gripperEncoderNames.size(); i++){
            std::string dir = gripperEncoderDir + "/" + gripperEncoderNames.at(i);
            gripperEncoderDirs.push_back(dir);
        }
        for(int i = 0; i < imu9AxisNames.size(); i++){
            std::string dir = imu9AxisDir + "/" + imu9AxisNames.at(i);
            imu9AxisDirs.push_back(dir);
        }
        for(int i = 0; i < lidarPointCloudNames.size(); i++){
            std::string dir = lidarPointCloudDir + "/" + lidarPointCloudNames.at(i);
            lidarPointCloudDirs.push_back(dir);
        }
        for(int i = 0; i < robotBaseVelNames.size(); i++){
            std::string dir = robotBaseVelDir + "/" + robotBaseVelNames.at(i);
            robotBaseVelDirs.push_back(dir);
        }
        for(int i = 0; i < liftMotorNames.size(); i++){
            std::string dir = liftMotorDir + "/" + liftMotorNames.at(i);
            liftMotorDirs.push_back(dir);
        }
        for(int i = 0; i < tfTransformParentFrames.size(); i++){
            std::string dir = tfTransformDir + "/" + tfTransformParentFrames.at(i) + "-" + tfTransformChildFrames.at(i) + ".json";
            tfTransformDirs.push_back(dir);
        }
    }
};

#endif
