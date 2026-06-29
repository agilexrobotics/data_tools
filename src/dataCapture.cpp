#include "dataUtility.h"
#include "blockingDeque.h"
#include <mutex>
#include <math.h>
#include <condition_variable>
#include <thread>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <fstream>
#include <sensor_msgs/msg/imu.hpp>
#include <data_msgs/msg/gripper.hpp>
#include <data_msgs/msg/instruction.hpp>
#include <data_msgs/msg/capture_status.hpp>
#include "data_msgs/srv/capture_service.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include "jsoncpp/json/json.h"
#ifdef _USELIFT
#include <bt_task_msgs/msg/lift_motor_msg.hpp>
#endif

class DataCapture: public DataUtility{
public:
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subCameraColors;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subCameraDepths;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subCameraPointClouds;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> subArmJointStates;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> subArmEndPoses;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> subLocalizationPoses;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr> subForce6dims;
    std::vector<rclcpp::Subscription<data_msgs::msg::Gripper>::SharedPtr> subGripperEncoders;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> subImu9Axiss;
    std::vector<rclcpp::Subscription<data_msgs::msg::Array>::SharedPtr> subArrayFloat32s;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subLidarPointClouds;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subRobotBaseOdometrys;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr> subRobotBaseVelocitys;
    std::vector<rclcpp::Subscription<data_msgs::msg::Instruction>::SharedPtr> subInstructionTexts;
    #ifdef _USELIFT
    std::vector<rclcpp::Subscription<bt_task_msgs::msg::LiftMotorMsg>::SharedPtr> subLiftMotors;
    #endif
    std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> subCameraColorConfigs;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> subCameraDepthConfigs;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> subCameraPointCloudConfigs;
    // std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> subArmJointStateConfigs;
    // std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> subArmEndPoseConfigs;
    // std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> subLocalizationPoseConfigs;
    // std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> subGripperEncoderConfigs;
    // std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> subImu9AxisConfigs;
    // std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> subLidarPointCloudConfigs;
    // std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> subrobotBaseOdometryConfigs;
    // std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> subRobotBaseVelocityConfigs;
    rclcpp::Publisher<data_msgs::msg::CaptureStatus>::SharedPtr pubCaptureStatus;

    std::vector<BlockingDeque<sensor_msgs::msg::Image>> cameraColorMsgDeques;
    std::vector<BlockingDeque<sensor_msgs::msg::Image>> cameraDepthMsgDeques;
    std::vector<BlockingDeque<sensor_msgs::msg::PointCloud2>> cameraPointCloudMsgDeques;
    std::vector<BlockingDeque<sensor_msgs::msg::JointState>> armJointStateMsgDeques;
    std::vector<BlockingDeque<geometry_msgs::msg::PoseStamped>> armEndPoseMsgDeques;
    std::vector<BlockingDeque<geometry_msgs::msg::PoseStamped>> localizationPoseMsgDeques;
    std::vector<BlockingDeque<geometry_msgs::msg::WrenchStamped>> force6dimMsgDeques;
    std::vector<BlockingDeque<data_msgs::msg::Gripper>> gripperEncoderMsgDeques;
    std::vector<BlockingDeque<sensor_msgs::msg::Imu>> imu9AxisMsgDeques;
    std::vector<BlockingDeque<data_msgs::msg::Array>> arrayFloat32MsgDeques;
    std::vector<BlockingDeque<sensor_msgs::msg::PointCloud2>> lidarPointCloudMsgDeques;
    std::vector<BlockingDeque<nav_msgs::msg::Odometry>> robotBaseOdometryMsgDeques;
    std::vector<BlockingDeque<geometry_msgs::msg::TwistStamped>> robotBaseVelocityMsgDeques;
    #ifdef _USELIFT
    std::vector<BlockingDeque<bt_task_msgs::msg::LiftMotorMsg>> liftMotorMsgDeques;
    #endif
    std::vector<BlockingDeque<data_msgs::msg::Instruction>> instructionTextDeques;

    std::vector<std::thread*> cameraColorSavingThreads;
    std::vector<std::thread*> cameraDepthSavingThreads;
    std::vector<std::thread*> cameraPointCloudSavingThreads;
    std::vector<std::thread*> armJointStateSavingThreads;
    std::vector<std::thread*> armEndPoseSavingThreads;
    std::vector<std::thread*> localizationPoseSavingThreads;
    std::vector<std::thread*> force6dimSavingThreads;
    std::vector<std::thread*> gripperEncoderSavingThreads;
    std::vector<std::thread*> imu9AxisSavingThreads;
    std::vector<std::thread*> arrayFloat32SavingThreads;
    std::vector<std::thread*> lidarPointCloudSavingThreads;
    std::vector<std::thread*> robotBaseOdometrysavingThreads;
    std::vector<std::thread*> robotBaseVelocitySavingThreads;
    std::vector<std::thread*> liftMotorSavingThreads;
    std::vector<std::thread*> tfTransformSavingThreads;

    std::vector<int> cameraColorMsgCounts;
    std::vector<int> cameraDepthMsgCounts;
    std::vector<int> cameraPointCloudMsgCounts;
    std::vector<int> armJointStateMsgCounts;
    std::vector<int> armEndPoseMsgCounts;
    std::vector<int> localizationPoseMsgCounts;
    std::vector<int> force6dimMsgCounts;
    std::vector<int> gripperEncoderMsgCounts;
    std::vector<int> imu9AxisMsgCounts;
    std::vector<int> arrayFloat32MsgCounts;
    std::vector<int> lidarPointCloudMsgCounts;
    std::vector<int> robotBaseOdometryMsgCounts;
    std::vector<int> robotBaseVelocityMsgCounts;
    std::vector<int> liftMotorMsgCounts;

    std::vector<int> cameraColorConfigMsgCounts;
    std::vector<int> cameraDepthConfigMsgCounts;
    std::vector<int> cameraPointCloudConfigMsgCounts;
    std::vector<int> armJointStateConfigMsgCounts;
    std::vector<int> armEndPoseConfigMsgCounts;
    std::vector<int> localizationPoseConfigMsgCounts;
    std::vector<int> force6dimConfigMsgCounts;
    std::vector<int> gripperEncoderConfigMsgCounts;
    std::vector<int> imu9AxisConfigMsgCounts;
    std::vector<int> arrayFloat32ConfigMsgCounts;
    std::vector<int> lidarPointCloudConfigMsgCounts;
    std::vector<int> robotBaseOdometryConfigMsgCounts;
    std::vector<int> robotBaseVelocityConfigMsgCounts;
    std::vector<int> liftMotorConfigMsgCounts;
    std::vector<int> tfTransformMsgCounts;
    
    std::vector<double> cameraColorStartTimeStamps;
    std::vector<double> cameraDepthStartTimeStamps;
    std::vector<double> cameraPointCloudStartTimeStamps;
    std::vector<double> armJointStateStartTimeStamps;
    std::vector<double> armEndPoseStartTimeStamps;
    std::vector<double> localizationPoseStartTimeStamps;
    std::vector<double> force6dimStartTimeStamps;
    std::vector<double> gripperEncoderStartTimeStamps;
    std::vector<double> imu9AxisStartTimeStamps;
    std::vector<double> arrayFloat32StartTimeStamps;
    std::vector<double> lidarPointCloudStartTimeStamps;
    std::vector<double> robotBaseOdometrystartTimeStamps;
    std::vector<double> robotBaseVelocityStartTimeStamps;
    std::vector<double> liftMotorStartTimeStamps;

    std::vector<double> cameraColorLastTimeStamps;
    std::vector<double> cameraDepthLastTimeStamps;
    std::vector<double> cameraPointCloudLastTimeStamps;
    std::vector<double> armJointStateLastTimeStamps;
    std::vector<double> armEndPoseLastTimeStamps;
    std::vector<double> localizationPoseLastTimeStamps;
    std::vector<double> force6dimLastTimeStamps;
    std::vector<double> gripperEncoderLastTimeStamps;
    std::vector<double> imu9AxisLastTimeStamps;
    std::vector<double> arrayFloat32LastTimeStamps;
    std::vector<double> lidarPointCloudLastTimeStamps;
    std::vector<double> robotBaseOdometryLastTimeStamps;
    std::vector<double> robotBaseVelocityLastTimeStamps;
    std::vector<double> liftMotorLastTimeStamps;
    
    std::vector<std::mutex> cameraColorMsgCountMtxs;
    std::vector<std::mutex> cameraDepthMsgCountMtxs;
    std::vector<std::mutex> cameraPointCloudMsgCountMtxs;
    std::vector<std::mutex> armJointStateMsgCountMtxs;
    std::vector<std::mutex> armEndPoseMsgCountMtxs;
    std::vector<std::mutex> localizationPoseMsgCountMtxs;
    std::vector<std::mutex> force6dimMsgCountMtxs;
    std::vector<std::mutex> gripperEncoderMsgCountMtxs;
    std::vector<std::mutex> imu9AxisMsgCountMtxs;
    std::vector<std::mutex> arrayFloat32MsgCountMtxs;
    std::vector<std::mutex> lidarPointCloudMsgCountMtxs;
    std::vector<std::mutex> robotBaseOdometryMsgCountMtxs;
    std::vector<std::mutex> robotBaseVelocityMsgCountMtxs;
    std::vector<std::mutex> liftMotorMsgCountMtxs;
    std::vector<std::mutex> tfTransformMsgCountMtxs;

    std::vector<std::mutex> cameraColorConfigMsgCountMtxs;
    std::vector<std::mutex> cameraDepthConfigMsgCountMtxs;
    std::vector<std::mutex> cameraPointCloudConfigMsgCountMtxs;
    std::vector<std::mutex> armJointStateConfigMsgCountMtxs;
    std::vector<std::mutex> armEndPoseConfigMsgCountMtxs;
    std::vector<std::mutex> localizationPoseConfigMsgCountMtxs;
    std::vector<std::mutex> force6dimConfigMsgCountMtxs;
    std::vector<std::mutex> gripperEncoderConfigMsgCountMtxs;
    std::vector<std::mutex> imu9AxisConfigMsgCountMtxs;
    std::vector<std::mutex> arrayFloat32ConfigMsgCountMtxs;
    std::vector<std::mutex> lidarPointCloudConfigMsgCountMtxs;
    std::vector<std::mutex> robotBaseOdometryConfigMsgCountMtxs;
    std::vector<std::mutex> robotBaseVelocityConfigMsgCountMtxs;
    std::vector<std::mutex> liftMotorConfigMsgCountMtxs;

    std::vector<std::string> cameraColorFrameIds;
    std::vector<std::string> cameraDepthFrameIds;
    std::vector<std::string> cameraPointCloudFrameIds;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;

    std::thread* keyboardInterruptCheckingThread;
    std::thread* monitoringThread;
    bool keyboardInterrupt = false;
    bool captureStop = false;
    std::mutex captureStopMtx;

    bool useService;

    int hz;
    int timeout;
    double cropTime;

    std::vector<sensor_msgs::msg::JointState> armJointStateList;
    std::vector<bool> armJointStateChangeList;

    DataCapture(std::string name, const rclcpp::NodeOptions & options, std::string datasetDir, int episodeIndex, int hz, int timeout, double cropTime=-1, bool useService=false): DataUtility(name, options, datasetDir, episodeIndex) {
        this->useService = useService;
        this->hz = hz;
        this->timeout = timeout;
        this->cropTime = useService ? cropTime:-1;
        int unused = system((std::string("rm -rf ") + episodeDir).c_str());
        unused = system((std::string("mkdir -p ") + cameraColorDir).c_str());
        unused = system((std::string("mkdir -p ") + cameraDepthDir).c_str());
        unused = system((std::string("mkdir -p ") + cameraPointCloudDir).c_str());
        unused = system((std::string("mkdir -p ") + armJointStateDir).c_str());
        unused = system((std::string("mkdir -p ") + armEndPoseDir).c_str());
        unused = system((std::string("mkdir -p ") + localizationPoseDir).c_str());
        unused = system((std::string("mkdir -p ") + force6dimDir).c_str());
        unused = system((std::string("mkdir -p ") + gripperEncoderDir).c_str());
        unused = system((std::string("mkdir -p ") + imu9AxisDir).c_str());
        unused = system((std::string("mkdir -p ") + arrayFloat32Dir).c_str());
        unused = system((std::string("mkdir -p ") + lidarPointCloudDir).c_str());
        unused = system((std::string("mkdir -p ") + robotBaseOdometryDir).c_str());
        unused = system((std::string("mkdir -p ") + robotBaseVelocityDir).c_str());
        unused = system((std::string("mkdir -p ") + liftMotorDir).c_str());
        unused = system((std::string("mkdir -p ") + tfTransformDir).c_str());

        cameraColorMsgDeques = std::vector<BlockingDeque<sensor_msgs::msg::Image>>(cameraColorNames.size());
        cameraDepthMsgDeques = std::vector<BlockingDeque<sensor_msgs::msg::Image>>(cameraDepthNames.size());
        cameraPointCloudMsgDeques = std::vector<BlockingDeque<sensor_msgs::msg::PointCloud2>>(cameraPointCloudNames.size());
        armJointStateMsgDeques = std::vector<BlockingDeque<sensor_msgs::msg::JointState>>(armJointStateNames.size());
        armEndPoseMsgDeques = std::vector<BlockingDeque<geometry_msgs::msg::PoseStamped>>(armEndPoseNames.size());
        localizationPoseMsgDeques = std::vector<BlockingDeque<geometry_msgs::msg::PoseStamped>>(localizationPoseNames.size());
        force6dimMsgDeques = std::vector<BlockingDeque<geometry_msgs::msg::WrenchStamped>>(force6dimNames.size());
        gripperEncoderMsgDeques = std::vector<BlockingDeque<data_msgs::msg::Gripper>>(gripperEncoderNames.size());
        imu9AxisMsgDeques = std::vector<BlockingDeque<sensor_msgs::msg::Imu>>(imu9AxisNames.size());
        arrayFloat32MsgDeques = std::vector<BlockingDeque<data_msgs::msg::Array>>(arrayFloat32Names.size());
        lidarPointCloudMsgDeques = std::vector<BlockingDeque<sensor_msgs::msg::PointCloud2>>(lidarPointCloudNames.size());
        robotBaseOdometryMsgDeques = std::vector<BlockingDeque<nav_msgs::msg::Odometry>>(robotBaseOdometryNames.size());
        robotBaseVelocityMsgDeques = std::vector<BlockingDeque<geometry_msgs::msg::TwistStamped>>(robotBaseVelocityNames.size());
        #ifdef _USELIFT
        liftMotorMsgDeques = std::vector<BlockingDeque<bt_task_msgs::msg::LiftMotorMsg>>(liftMotorNames.size());
        #endif
        instructionTextDeques = std::vector<BlockingDeque<data_msgs::msg::Instruction>>(instructionTextTopics.size());

        subCameraColors = std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>(cameraColorNames.size());
        subCameraDepths = std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>(cameraDepthNames.size());
        subCameraPointClouds = std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>(cameraPointCloudNames.size());
        subArmJointStates = std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr>(armJointStateNames.size());
        subArmEndPoses = std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>(armEndPoseNames.size());
        subLocalizationPoses = std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>(localizationPoseNames.size());
        subForce6dims = std::vector<rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr>(force6dimNames.size());
        subGripperEncoders = std::vector<rclcpp::Subscription<data_msgs::msg::Gripper>::SharedPtr>(gripperEncoderNames.size());
        subImu9Axiss = std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr>(imu9AxisNames.size());
        subArrayFloat32s = std::vector<rclcpp::Subscription<data_msgs::msg::Array>::SharedPtr>(arrayFloat32Names.size());
        subLidarPointClouds = std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>(lidarPointCloudNames.size());
        subRobotBaseOdometrys = std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>(robotBaseOdometryNames.size());
        subRobotBaseVelocitys = std::vector<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr>(robotBaseVelocityNames.size());
        #ifdef _USELIFT
        subLiftMotors = std::vector<rclcpp::Subscription<bt_task_msgs::msg::LiftMotorMsg>::SharedPtr>(liftMotorNames.size());
        #endif
        subInstructionTexts = std::vector<rclcpp::Subscription<data_msgs::msg::Instruction>::SharedPtr>(instructionTextTopics.size());

        subCameraColorConfigs = std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr>(cameraColorNames.size());
        subCameraDepthConfigs = std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr>(cameraDepthNames.size());
        subCameraPointCloudConfigs = std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr>(cameraPointCloudNames.size());

        armJointStateList = std::vector<sensor_msgs::msg::JointState>(armJointStateNames.size());
        armJointStateChangeList = std::vector<bool>(armJointStateNames.size());

        pubCaptureStatus = create_publisher<data_msgs::msg::CaptureStatus>("/data_tools_dataCapture/status", 2000);

        cameraColorMsgCounts = std::vector<int>(cameraColorNames.size(), 0);
        cameraDepthMsgCounts = std::vector<int>(cameraDepthNames.size(), 0);
        cameraPointCloudMsgCounts = std::vector<int>(cameraPointCloudNames.size(), 0);
        armJointStateMsgCounts = std::vector<int>(armJointStateNames.size(), 0);
        armEndPoseMsgCounts = std::vector<int>(armEndPoseNames.size(), 0);
        localizationPoseMsgCounts = std::vector<int>(localizationPoseNames.size(), 0);
        force6dimMsgCounts = std::vector<int>(force6dimNames.size(), 0);
        gripperEncoderMsgCounts = std::vector<int>(gripperEncoderNames.size(), 0);
        imu9AxisMsgCounts = std::vector<int>(imu9AxisNames.size(), 0);
        arrayFloat32MsgCounts = std::vector<int>(arrayFloat32Names.size(), 0);
        lidarPointCloudMsgCounts = std::vector<int>(lidarPointCloudNames.size(), 0);
        robotBaseOdometryMsgCounts = std::vector<int>(robotBaseOdometryNames.size(), 0);
        robotBaseVelocityMsgCounts = std::vector<int>(robotBaseVelocityNames.size(), 0);
        liftMotorMsgCounts = std::vector<int>(liftMotorNames.size(), 0);
        tfTransformMsgCounts = std::vector<int>(tfTransformParentFrames.size(), 0);

        cameraColorConfigMsgCounts = std::vector<int>(cameraColorNames.size(), 0);
        cameraDepthConfigMsgCounts = std::vector<int>(cameraDepthNames.size(), 0);
        cameraPointCloudConfigMsgCounts = std::vector<int>(cameraPointCloudNames.size(), 0);
        armJointStateConfigMsgCounts = std::vector<int>(armJointStateNames.size(), 0);
        armEndPoseConfigMsgCounts = std::vector<int>(armEndPoseNames.size(), 0);
        localizationPoseConfigMsgCounts = std::vector<int>(localizationPoseNames.size(), 0);
        force6dimConfigMsgCounts = std::vector<int>(force6dimNames.size(), 0);
        gripperEncoderConfigMsgCounts = std::vector<int>(gripperEncoderNames.size(), 0);
        imu9AxisConfigMsgCounts = std::vector<int>(imu9AxisNames.size(), 0);
        arrayFloat32ConfigMsgCounts = std::vector<int>(arrayFloat32Names.size(), 0);
        lidarPointCloudConfigMsgCounts = std::vector<int>(lidarPointCloudNames.size(), 0);
        robotBaseOdometryConfigMsgCounts = std::vector<int>(robotBaseOdometryNames.size(), 0);
        robotBaseVelocityConfigMsgCounts = std::vector<int>(robotBaseVelocityNames.size(), 0);
        liftMotorConfigMsgCounts = std::vector<int>(liftMotorNames.size(), 0);

        cameraColorStartTimeStamps = std::vector<double>(cameraColorNames.size(), 0);
        cameraDepthStartTimeStamps = std::vector<double>(cameraDepthNames.size(), 0);
        cameraPointCloudStartTimeStamps = std::vector<double>(cameraPointCloudNames.size(), 0);
        armJointStateStartTimeStamps = std::vector<double>(armJointStateNames.size(), 0);
        armEndPoseStartTimeStamps = std::vector<double>(armEndPoseNames.size(), 0);
        localizationPoseStartTimeStamps = std::vector<double>(localizationPoseNames.size(), 0);
        force6dimStartTimeStamps = std::vector<double>(force6dimNames.size(), 0);
        gripperEncoderStartTimeStamps = std::vector<double>(gripperEncoderNames.size(), 0);
        imu9AxisStartTimeStamps = std::vector<double>(imu9AxisNames.size(), 0);
        arrayFloat32StartTimeStamps = std::vector<double>(arrayFloat32Names.size(), 0);
        lidarPointCloudStartTimeStamps = std::vector<double>(lidarPointCloudNames.size(), 0);
        robotBaseOdometrystartTimeStamps = std::vector<double>(robotBaseOdometryNames.size(), 0);
        robotBaseVelocityStartTimeStamps = std::vector<double>(robotBaseVelocityNames.size(), 0);
        liftMotorStartTimeStamps = std::vector<double>(liftMotorNames.size(), 0);

        cameraColorLastTimeStamps = std::vector<double>(cameraColorNames.size(), 0);
        cameraDepthLastTimeStamps = std::vector<double>(cameraDepthNames.size(), 0);
        cameraPointCloudLastTimeStamps = std::vector<double>(cameraPointCloudNames.size(), 0);
        armJointStateLastTimeStamps = std::vector<double>(armJointStateNames.size(), 0);
        armEndPoseLastTimeStamps = std::vector<double>(armEndPoseNames.size(), 0);
        localizationPoseLastTimeStamps = std::vector<double>(localizationPoseNames.size(), 0);
        force6dimLastTimeStamps = std::vector<double>(force6dimNames.size(), 0);
        gripperEncoderLastTimeStamps = std::vector<double>(gripperEncoderNames.size(), 0);
        imu9AxisLastTimeStamps = std::vector<double>(imu9AxisNames.size(), 0);
        arrayFloat32LastTimeStamps = std::vector<double>(arrayFloat32Names.size(), 0);
        lidarPointCloudLastTimeStamps = std::vector<double>(lidarPointCloudNames.size(), 0);
        robotBaseOdometryLastTimeStamps = std::vector<double>(robotBaseOdometryNames.size(), 0);
        robotBaseVelocityLastTimeStamps = std::vector<double>(robotBaseVelocityNames.size(), 0);
        liftMotorLastTimeStamps = std::vector<double>(liftMotorNames.size(), 0);

        cameraColorMsgCountMtxs = std::vector<std::mutex>(cameraColorNames.size());
        cameraDepthMsgCountMtxs = std::vector<std::mutex>(cameraDepthNames.size());
        cameraPointCloudMsgCountMtxs = std::vector<std::mutex>(cameraPointCloudNames.size());
        armJointStateMsgCountMtxs = std::vector<std::mutex>(armJointStateNames.size());
        armEndPoseMsgCountMtxs = std::vector<std::mutex>(armEndPoseNames.size());
        localizationPoseMsgCountMtxs = std::vector<std::mutex>(localizationPoseNames.size());
        force6dimMsgCountMtxs = std::vector<std::mutex>(force6dimNames.size());
        gripperEncoderMsgCountMtxs = std::vector<std::mutex>(gripperEncoderNames.size());
        imu9AxisMsgCountMtxs = std::vector<std::mutex>(imu9AxisNames.size());
        arrayFloat32MsgCountMtxs = std::vector<std::mutex>(arrayFloat32Names.size());
        lidarPointCloudMsgCountMtxs = std::vector<std::mutex>(lidarPointCloudNames.size());
        robotBaseOdometryMsgCountMtxs = std::vector<std::mutex>(robotBaseOdometryNames.size());
        robotBaseVelocityMsgCountMtxs = std::vector<std::mutex>(robotBaseVelocityNames.size());
        liftMotorMsgCountMtxs = std::vector<std::mutex>(liftMotorNames.size());
        tfTransformMsgCountMtxs = std::vector<std::mutex>(tfTransformParentFrames.size());

        cameraColorConfigMsgCountMtxs = std::vector<std::mutex>(cameraColorNames.size());
        cameraDepthConfigMsgCountMtxs = std::vector<std::mutex>(cameraDepthNames.size());
        cameraPointCloudConfigMsgCountMtxs = std::vector<std::mutex>(cameraPointCloudNames.size());
        armJointStateConfigMsgCountMtxs = std::vector<std::mutex>(armJointStateNames.size());
        armEndPoseConfigMsgCountMtxs = std::vector<std::mutex>(armEndPoseNames.size());
        localizationPoseConfigMsgCountMtxs = std::vector<std::mutex>(localizationPoseNames.size());
        force6dimConfigMsgCountMtxs = std::vector<std::mutex>(force6dimNames.size());
        gripperEncoderConfigMsgCountMtxs = std::vector<std::mutex>(gripperEncoderNames.size());
        imu9AxisConfigMsgCountMtxs = std::vector<std::mutex>(imu9AxisNames.size());
        arrayFloat32ConfigMsgCountMtxs = std::vector<std::mutex>(arrayFloat32Names.size());
        lidarPointCloudConfigMsgCountMtxs = std::vector<std::mutex>(lidarPointCloudNames.size());
        robotBaseOdometryConfigMsgCountMtxs = std::vector<std::mutex>(robotBaseOdometryNames.size());
        robotBaseVelocityConfigMsgCountMtxs = std::vector<std::mutex>(robotBaseVelocityNames.size());
        liftMotorConfigMsgCountMtxs = std::vector<std::mutex>(liftMotorNames.size());

        cameraColorFrameIds = std::vector<std::string>(cameraColorNames.size(), "");
        cameraDepthFrameIds = std::vector<std::string>(cameraDepthNames.size(), "");
        cameraPointCloudFrameIds = std::vector<std::string>(cameraPointCloudNames.size(), "");

        tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

        for(int i = 0; i < cameraColorNames.size(); i++){
            unused = system((std::string("mkdir -p ") + cameraColorDirs.at(i)).c_str());
            subCameraColors[i] = create_subscription<sensor_msgs::msg::Image>(cameraColorTopics[i], 2000, [this, i](const sensor_msgs::msg::Image::SharedPtr msg) { this->cameraColorHandler(msg, i);});
            subCameraColorConfigs[i] = create_subscription<sensor_msgs::msg::CameraInfo>(cameraColorConfigTopics[i], 2000, [this, i](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { this->cameraColorConfigHandler(msg, i);});
        }
        for(int i = 0; i < cameraDepthNames.size(); i++){
            unused = system((std::string("mkdir -p ") + cameraDepthDirs.at(i)).c_str());
            subCameraDepths[i] = create_subscription<sensor_msgs::msg::Image>(cameraDepthTopics[i], 2000, [this, i](const sensor_msgs::msg::Image::SharedPtr msg) { this->cameraDepthHandler(msg, i);});
            subCameraDepthConfigs[i] = create_subscription<sensor_msgs::msg::CameraInfo>(cameraDepthConfigTopics[i], 2000, [this, i](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { this->cameraDepthConfigHandler(msg, i);});
        }
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            unused = system((std::string("mkdir -p ") + cameraPointCloudDirs.at(i)).c_str());
            subCameraPointClouds[i] = create_subscription<sensor_msgs::msg::PointCloud2>(cameraPointCloudTopics[i], 2000, [this, i](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { this->cameraPointCloudHandler(msg, i);});
            subCameraPointCloudConfigs[i] = create_subscription<sensor_msgs::msg::CameraInfo>(cameraPointCloudConfigTopics[i], 2000, [this, i](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { this->cameraPointCloudConfigHandler(msg, i);});
        }
        for(int i = 0; i < armJointStateNames.size(); i++){
            armJointStateChangeList[i] = false;
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = rclcpp::Time(0);
            armJointStateList[i] = msg;
            unused = system((std::string("mkdir -p ") + armJointStateDirs.at(i)).c_str());
            subArmJointStates[i] = create_subscription<sensor_msgs::msg::JointState>(armJointStateTopics[i], 2000, [this, i](const sensor_msgs::msg::JointState::SharedPtr msg) { this->armJointStateHandler(msg, i);});
        }
        for(int i = 0; i < armEndPoseNames.size(); i++){
            unused = system((std::string("mkdir -p ") + armEndPoseDirs.at(i)).c_str());
            subArmEndPoses[i] = create_subscription<geometry_msgs::msg::PoseStamped>(armEndPoseTopics[i], 2000, [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->armEndPoseHandler(msg, i);});
        }
        for(int i = 0; i < localizationPoseNames.size(); i++){
            unused = system((std::string("mkdir -p ") + localizationPoseDirs.at(i)).c_str());
            subLocalizationPoses[i] = create_subscription<geometry_msgs::msg::PoseStamped>(localizationPoseTopics[i], 2000, [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->localizationPoseHandler(msg, i);});
        }
        for(int i = 0; i < force6dimNames.size(); i++){
            unused = system((std::string("mkdir -p ") + force6dimDirs.at(i)).c_str());
            subForce6dims[i] = create_subscription<geometry_msgs::msg::WrenchStamped>(force6dimTopics[i], 2000, [this, i](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) { this->force6dimHandler(msg, i);});
        }
        for(int i = 0; i < gripperEncoderNames.size(); i++){
            unused = system((std::string("mkdir -p ") + gripperEncoderDirs.at(i)).c_str());
            subGripperEncoders[i] = create_subscription<data_msgs::msg::Gripper>(gripperEncoderTopics[i], 2000, [this, i](const data_msgs::msg::Gripper::SharedPtr msg) { this->gripperEncoderHandler(msg, i);});
        }
        for(int i = 0; i < imu9AxisNames.size(); i++){
            unused = system((std::string("mkdir -p ") + imu9AxisDirs.at(i)).c_str());
            subImu9Axiss[i] = create_subscription<sensor_msgs::msg::Imu>(imu9AxisTopics[i], 2000, [this, i](const sensor_msgs::msg::Imu::SharedPtr msg) { this->imu9AxisHandler(msg, i);});
        }
        for(int i = 0; i < arrayFloat32Names.size(); i++){
            unused = system((std::string("mkdir -p ") + arrayFloat32Dirs.at(i)).c_str());
            subArrayFloat32s[i] = create_subscription<data_msgs::msg::Array>(arrayFloat32Topics[i], 2000, [this, i](const data_msgs::msg::Array::SharedPtr msg) { this->arrayFloat32Handler(msg, i);});
        }
        for(int i = 0; i < lidarPointCloudNames.size(); i++){
            unused = system((std::string("mkdir -p ") + lidarPointCloudDirs.at(i)).c_str());
            subLidarPointClouds[i] = create_subscription<sensor_msgs::msg::PointCloud2>(lidarPointCloudTopics[i], 2000, [this, i](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { this->lidarPointCloudHandler(msg, i);});
        }
        for(int i = 0; i < robotBaseOdometryNames.size(); i++){
            unused = system((std::string("mkdir -p ") + robotBaseOdometryDirs.at(i)).c_str());
            subRobotBaseOdometrys[i] = create_subscription<nav_msgs::msg::Odometry>(robotBaseOdometryTopics[i], 2000, [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) { this->robotBaseOdometryHandler(msg, i);});
        }
        for(int i = 0; i < robotBaseVelocityNames.size(); i++){
            unused = system((std::string("mkdir -p ") + robotBaseVelocityDirs.at(i)).c_str());
            subRobotBaseVelocitys[i] = create_subscription<geometry_msgs::msg::TwistStamped>(robotBaseVelocityTopics[i], 2000, [this, i](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { this->robotBaseVelocityHandler(msg, i);});
        }
        #ifdef _USELIFT
        for(int i = 0; i < liftMotorNames.size(); i++){
            unused = system((std::string("mkdir -p ") + liftMotorDirs.at(i)).c_str());
            subLiftMotors[i] = create_subscription<bt_task_msgs::msg::LiftMotorMsg>(liftMotorTopics[i], 2000, [this, i](const bt_task_msgs::msg::LiftMotorMsg::SharedPtr msg) { this->liftMotorHandler(msg, i);});
        }
        #endif
        for(int i = 0; i < instructionTextTopics.size(); i++){
            subInstructionTexts[i] = create_subscription<data_msgs::msg::Instruction>(instructionTextTopics[i], 2000, [this, i](const data_msgs::msg::Instruction::SharedPtr msg) { this->instructionTextHandler(msg, i);});
        }
    }

    void run(){
        for(int i = 0; i < cameraColorNames.size(); i++){
            cameraColorSavingThreads.push_back(new std::thread(&DataCapture::cameraColorSaving, this, i));
        }
        for(int i = 0; i < cameraDepthNames.size(); i++){
            cameraDepthSavingThreads.push_back(new std::thread(&DataCapture::cameraDepthSaving, this, i));
        }
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            cameraPointCloudSavingThreads.push_back(new std::thread(&DataCapture::cameraPointCloudSaving, this, i));
        }
        for(int i = 0; i < armJointStateNames.size(); i++){
            armJointStateSavingThreads.push_back(new std::thread(&DataCapture::armJointStateSaving, this, i));
        }
        for(int i = 0; i < armEndPoseNames.size(); i++){
            armEndPoseSavingThreads.push_back(new std::thread(&DataCapture::armEndPoseSaving, this, i));
        }
        for(int i = 0; i < localizationPoseNames.size(); i++){
            localizationPoseSavingThreads.push_back(new std::thread(&DataCapture::localizationPoseSaving, this, i));
        }
        for(int i = 0; i < force6dimNames.size(); i++){
            force6dimSavingThreads.push_back(new std::thread(&DataCapture::force6dimSaving, this, i));
        }
        for(int i = 0; i < gripperEncoderNames.size(); i++){
            gripperEncoderSavingThreads.push_back(new std::thread(&DataCapture::gripperEncoderSaving, this, i));
        }
        for(int i = 0; i < imu9AxisNames.size(); i++){
            imu9AxisSavingThreads.push_back(new std::thread(&DataCapture::imu9AxisSaving, this, i));
        }
        for(int i = 0; i < arrayFloat32Names.size(); i++){
            arrayFloat32SavingThreads.push_back(new std::thread(&DataCapture::arrayFloat32Saving, this, i));
        }
        for(int i = 0; i < lidarPointCloudNames.size(); i++){
            lidarPointCloudSavingThreads.push_back(new std::thread(&DataCapture::lidarPointCloudSaving, this, i));
        }
        for(int i = 0; i < robotBaseOdometryNames.size(); i++){
            robotBaseOdometrysavingThreads.push_back(new std::thread(&DataCapture::robotBaseOdometrysaving, this, i));
        }
        for(int i = 0; i < robotBaseVelocityNames.size(); i++){
            robotBaseVelocitySavingThreads.push_back(new std::thread(&DataCapture::robotBaseVelocitySaving, this, i));
        }
        #ifdef _USELIFT
        for(int i = 0; i < liftMotorNames.size(); i++){
            liftMotorSavingThreads.push_back(new std::thread(&DataCapture::liftMotorSaving, this, i));
        }
        #endif
        for(int i = 0; i < tfTransformParentFrames.size(); i++){
            tfTransformSavingThreads.push_back(new std::thread(&DataCapture::tfTransformSaving, this, i));
        }
        if(!this->useService){
            keyboardInterruptCheckingThread = new std::thread(&DataCapture::keyboardInterruptChecking, this);
        }
        monitoringThread = new std::thread(&DataCapture::monitoring, this);
    }

    void join(){
        for(int i = 0; i < cameraColorNames.size(); i++){
            cameraColorSavingThreads.at(i)->join();
            delete cameraColorSavingThreads.at(i);
            cameraColorSavingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < cameraDepthNames.size(); i++){
            cameraDepthSavingThreads.at(i)->join();
            delete cameraDepthSavingThreads.at(i);
            cameraDepthSavingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            cameraPointCloudSavingThreads.at(i)->join();
            delete cameraPointCloudSavingThreads.at(i);
            cameraPointCloudSavingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < armJointStateNames.size(); i++){
            armJointStateSavingThreads.at(i)->join();
            delete armJointStateSavingThreads.at(i);
            armJointStateSavingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < armEndPoseNames.size(); i++){
            armEndPoseSavingThreads.at(i)->join();
            delete armEndPoseSavingThreads.at(i);
            armEndPoseSavingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < localizationPoseNames.size(); i++){
            localizationPoseSavingThreads.at(i)->join();
            delete localizationPoseSavingThreads.at(i);
            localizationPoseSavingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < force6dimNames.size(); i++){
            force6dimSavingThreads.at(i)->join();
            delete force6dimSavingThreads.at(i);
            force6dimSavingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < gripperEncoderNames.size(); i++){
            gripperEncoderSavingThreads.at(i)->join();
            delete gripperEncoderSavingThreads.at(i);
            gripperEncoderSavingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < imu9AxisNames.size(); i++){
            imu9AxisSavingThreads.at(i)->join();
            delete imu9AxisSavingThreads.at(i);
            imu9AxisSavingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < arrayFloat32Names.size(); i++){
            arrayFloat32SavingThreads.at(i)->join();
            delete arrayFloat32SavingThreads.at(i);
            arrayFloat32SavingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < lidarPointCloudNames.size(); i++){
            lidarPointCloudSavingThreads.at(i)->join();
            delete lidarPointCloudSavingThreads.at(i);
            lidarPointCloudSavingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < robotBaseOdometryNames.size(); i++){
            robotBaseOdometrysavingThreads.at(i)->join();
            delete robotBaseOdometrysavingThreads.at(i);
            robotBaseOdometrysavingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < robotBaseVelocityNames.size(); i++){
            robotBaseVelocitySavingThreads.at(i)->join();
            delete robotBaseVelocitySavingThreads.at(i);
            robotBaseVelocitySavingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < liftMotorNames.size(); i++){
            liftMotorSavingThreads.at(i)->join();
            delete liftMotorSavingThreads.at(i);
            liftMotorSavingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < tfTransformParentFrames.size(); i++){
            tfTransformSavingThreads.at(i)->join();
            delete tfTransformSavingThreads.at(i);
            tfTransformSavingThreads.at(i) = nullptr;
        }
        if(!this->useService){
            keyboardInterruptCheckingThread->join();
            delete keyboardInterruptCheckingThread;
            keyboardInterruptCheckingThread = nullptr;
        }
        monitoringThread->join();
        delete monitoringThread;
        monitoringThread = nullptr;
        tfListener.reset();
        tfBuffer.reset();
        data_msgs::msg::CaptureStatus captureStatus;
        captureStatus.quit = true;
        pubCaptureStatus->publish(captureStatus);
    }

    void cameraColorHandler(const sensor_msgs::msg::Image::SharedPtr& msg, const int& index){
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cameraColorMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(cameraColorMsgCountMtxs.at(index));
        if(cameraColorMsgCounts.at(index) == 0){
            cameraColorStartTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }else{
            cameraColorLastTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }
        cameraColorMsgCounts.at(index) += 1;
        cameraColorFrameIds.at(index) = msg->header.frame_id;
    }

    void cameraDepthHandler(const sensor_msgs::msg::Image::SharedPtr& msg, const int& index){
        cameraDepthMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(cameraDepthMsgCountMtxs.at(index));
        if(cameraDepthMsgCounts.at(index) == 0){
            cameraDepthStartTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }else{
            cameraDepthLastTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }
        cameraDepthMsgCounts.at(index) += 1;
        cameraDepthFrameIds.at(index) = msg->header.frame_id;
    }

    void cameraPointCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, const int& index){
        cameraPointCloudMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(cameraPointCloudMsgCountMtxs.at(index));
        if(cameraPointCloudMsgCounts.at(index) == 0){
            cameraPointCloudStartTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }else{
            cameraPointCloudLastTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }
        cameraPointCloudMsgCounts.at(index) += 1;
        cameraPointCloudFrameIds.at(index) = msg->header.frame_id;
    }

    void cameraColorConfigHandler(const sensor_msgs::msg::CameraInfo::SharedPtr& msg, const int& index){
        if(cameraColorFrameIds.at(index) == "")
            return;
        geometry_msgs::msg::TransformStamped transform;
		while(cameraColorFrameIds.at(index) != ""){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop)
                    break;
            }
            try {
                transform = tfBuffer->lookupTransform(cameraColorParentFrames.at(index), cameraColorFrameIds.at(index), tf2::TimePointZero);
                break;
            } catch (const tf2::TransformException & ex) {
                return;
            }
		}
		double x = transform.transform.translation.x;
        double y = transform.transform.translation.y;
        double z = transform.transform.translation.z;
        double roll, pitch, yaw;
        tf2::Quaternion q(transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        Json::Value root;
        root["height"] = msg->height;
        root["width"] = msg->width;
        root["distortion_model"] = msg->distortion_model;
        Json::Value D(Json::arrayValue);
        Json::Value K(Json::arrayValue);
        Json::Value R(Json::arrayValue);
        Json::Value P(Json::arrayValue);
        for(int i = 0; i < msg->d.size(); i++){
            D.append(msg->d.at(i));
        }
        for(int i = 0; i < msg->k.size(); i++){
            K.append(msg->k.at(i));
        }
        for(int i = 0; i < msg->r.size(); i++){
            R.append(msg->r.at(i));
        }
        for(int i = 0; i < msg->p.size(); i++){
            P.append(msg->p.at(i));
        }
        root["D"] = D;
        root["K"] = K;
        root["R"] = R;
        root["P"] = P;
        root["binning_x"] = msg->binning_x;
        root["binning_y"] = msg->binning_y;
        root["roi"]["x_offset"] = msg->roi.x_offset;
        root["roi"]["y_offset"] = msg->roi.y_offset;
        root["roi"]["height"] = msg->roi.height;
        root["roi"]["width"] = msg->roi.width;
        root["roi"]["do_rectify"] = msg->roi.do_rectify;
        root["parent_frame"]["x"] = x;
        root["parent_frame"]["y"] = y;
        root["parent_frame"]["z"] = z;
        root["parent_frame"]["roll"] = roll;
        root["parent_frame"]["pitch"] = pitch;
        root["parent_frame"]["yaw"] = yaw;
        Json::StyledStreamWriter streamWriter;
        std::ofstream file(cameraColorDirs.at(index) + "/config.json");
        streamWriter.write(file, root);
        file.close();

        std::lock_guard<std::mutex> lock(cameraColorConfigMsgCountMtxs.at(index));
        cameraColorConfigMsgCounts.at(index) += 1;
        subCameraColorConfigs.at(index).reset();
    }

    void cameraDepthConfigHandler(const sensor_msgs::msg::CameraInfo::SharedPtr& msg, const int& index){
        if(cameraDepthFrameIds.at(index) == "")
            return;
        geometry_msgs::msg::TransformStamped transform;
		while(cameraDepthFrameIds.at(index) != ""){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop)
                    break;
            }
            try {
                transform = tfBuffer->lookupTransform(cameraDepthParentFrames.at(index), cameraDepthFrameIds.at(index), tf2::TimePointZero);
                break;
            } catch (const tf2::TransformException & ex) {
                return;
            }
		}
		double x = transform.transform.translation.x;
        double y = transform.transform.translation.y;
        double z = transform.transform.translation.z;
        double roll, pitch, yaw;
        tf2::Quaternion q(transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        Json::Value root;
        root["height"] = msg->height;
        root["width"] = msg->width;
        root["distortion_model"] = msg->distortion_model;
        Json::Value D(Json::arrayValue);
        Json::Value K(Json::arrayValue);
        Json::Value R(Json::arrayValue);
        Json::Value P(Json::arrayValue);
        for(int i = 0; i < msg->d.size(); i++){
            D.append(msg->d.at(i));
        }
        for(int i = 0; i < msg->k.size(); i++){
            K.append(msg->k.at(i));
        }
        for(int i = 0; i < msg->r.size(); i++){
            R.append(msg->r.at(i));
        }
        for(int i = 0; i < msg->p.size(); i++){
            P.append(msg->p.at(i));
        }
        root["D"] = D;
        root["K"] = K;
        root["R"] = R;
        root["P"] = P;
        root["binning_x"] = msg->binning_x;
        root["binning_y"] = msg->binning_y;
        root["roi"]["x_offset"] = msg->roi.x_offset;
        root["roi"]["y_offset"] = msg->roi.y_offset;
        root["roi"]["height"] = msg->roi.height;
        root["roi"]["width"] = msg->roi.width;
        root["roi"]["do_rectify"] = msg->roi.do_rectify;
        root["parent_frame"]["x"] = x;
        root["parent_frame"]["y"] = y;
        root["parent_frame"]["z"] = z;
        root["parent_frame"]["roll"] = roll;
        root["parent_frame"]["pitch"] = pitch;
        root["parent_frame"]["yaw"] = yaw;
        Json::StyledStreamWriter streamWriter;
        std::ofstream file(cameraDepthDirs.at(index) + "/config.json");
        streamWriter.write(file, root);
        file.close();

        std::lock_guard<std::mutex> lock(cameraDepthConfigMsgCountMtxs.at(index));
        cameraDepthConfigMsgCounts.at(index) += 1;
        subCameraDepthConfigs.at(index).reset();
    }

    void cameraPointCloudConfigHandler(const sensor_msgs::msg::CameraInfo::SharedPtr& msg, const int& index){
        if(cameraPointCloudFrameIds.at(index) == "")
            return;
        geometry_msgs::msg::TransformStamped transform;
		while(cameraPointCloudFrameIds.at(index) != ""){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop)
                    break;
            }
            try {
                transform = tfBuffer->lookupTransform(cameraPointCloudParentFrames.at(index), cameraPointCloudFrameIds.at(index), tf2::TimePointZero);
                break;
            } catch (const tf2::TransformException & ex) {
                return;
            }
		}
		double x = transform.transform.translation.x;
        double y = transform.transform.translation.y;
        double z = transform.transform.translation.z;
        double roll, pitch, yaw;
        tf2::Quaternion q(transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        Json::Value root;
        root["height"] = msg->height;
        root["width"] = msg->width;
        root["distortion_model"] = msg->distortion_model;
        Json::Value D(Json::arrayValue);
        Json::Value K(Json::arrayValue);
        Json::Value R(Json::arrayValue);
        Json::Value P(Json::arrayValue);
        for(int i = 0; i < msg->d.size(); i++){
            D.append(msg->d.at(i));
        }
        for(int i = 0; i < msg->k.size(); i++){
            K.append(msg->k.at(i));
        }
        for(int i = 0; i < msg->r.size(); i++){
            R.append(msg->r.at(i));
        }
        for(int i = 0; i < msg->p.size(); i++){
            P.append(msg->p.at(i));
        }
        root["D"] = D;
        root["K"] = K;
        root["R"] = R;
        root["P"] = P;
        root["binning_x"] = msg->binning_x;
        root["binning_y"] = msg->binning_y;
        root["roi"]["x_offset"] = msg->roi.x_offset;
        root["roi"]["y_offset"] = msg->roi.y_offset;
        root["roi"]["height"] = msg->roi.height;
        root["roi"]["width"] = msg->roi.width;
        root["roi"]["do_rectify"] = msg->roi.do_rectify;
        root["parent_frame"]["x"] = x;
        root["parent_frame"]["y"] = y;
        root["parent_frame"]["z"] = z;
        root["parent_frame"]["roll"] = roll;
        root["parent_frame"]["pitch"] = pitch;
        root["parent_frame"]["yaw"] = yaw;
        Json::StyledStreamWriter streamWriter;
        std::ofstream file(cameraPointCloudDirs.at(index) + "/config.json");
        streamWriter.write(file, root);
        file.close();

        std::lock_guard<std::mutex> lock(cameraPointCloudConfigMsgCountMtxs.at(index));
        cameraPointCloudConfigMsgCounts.at(index) += 1;
        subCameraPointCloudConfigs.at(index).reset();
    }

    void armJointStateHandler(const sensor_msgs::msg::JointState::SharedPtr& msg, const int& index){
        if(rclcpp::Time(armJointStateList.at(index).header.stamp).seconds() != 0 && !armJointStateChangeList.at(index)){
            for(int i=0; i<msg->position.size(); i++){
                if(msg->position.at(i) - armJointStateList.at(index).position.at(i) != 0){
                    armJointStateChangeList.at(index) = true;
                }
            }
        }
        armJointStateList.at(index) = *msg;
        armJointStateMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(armJointStateMsgCountMtxs.at(index));
        if(armJointStateMsgCounts.at(index) == 0){
            armJointStateStartTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }else{
            armJointStateLastTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }
        armJointStateMsgCounts.at(index) += 1;
    }

    void armEndPoseHandler(const geometry_msgs::msg::PoseStamped::SharedPtr& msg, const int& index){
        armEndPoseMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(armEndPoseMsgCountMtxs.at(index));
        if(armEndPoseMsgCounts.at(index) == 0){
            armEndPoseStartTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }else{
            armEndPoseLastTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }
        armEndPoseMsgCounts.at(index) += 1;
    }

    void localizationPoseHandler(const geometry_msgs::msg::PoseStamped::SharedPtr& msg, const int& index){
        localizationPoseMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(localizationPoseMsgCountMtxs.at(index));
        if(localizationPoseMsgCounts.at(index) == 0){
            localizationPoseStartTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }else{
            localizationPoseLastTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }
        localizationPoseMsgCounts.at(index) += 1;
    }

    void force6dimHandler(const geometry_msgs::msg::WrenchStamped::SharedPtr& msg, const int& index){
        force6dimMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(force6dimMsgCountMtxs.at(index));
        if(force6dimMsgCounts.at(index) == 0){
            force6dimStartTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }else{
            force6dimLastTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }
        force6dimMsgCounts.at(index) += 1;
    }

    void gripperEncoderHandler(const data_msgs::msg::Gripper::SharedPtr& msg, const int& index){
        gripperEncoderMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(gripperEncoderMsgCountMtxs.at(index));
        if(gripperEncoderMsgCounts.at(index) == 0){
            gripperEncoderStartTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }else{
            gripperEncoderLastTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }
        gripperEncoderMsgCounts.at(index) += 1;
    }

    void imu9AxisHandler(const sensor_msgs::msg::Imu::SharedPtr& msg, const int& index){
        imu9AxisMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(imu9AxisMsgCountMtxs.at(index));
        if(imu9AxisMsgCounts.at(index) == 0){
            imu9AxisStartTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }else{
            imu9AxisLastTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }
        imu9AxisMsgCounts.at(index) += 1;
    }

    void arrayFloat32Handler(const data_msgs::msg::Array::SharedPtr& msg, const int& index){
        arrayFloat32MsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(arrayFloat32MsgCountMtxs.at(index));
        if(arrayFloat32MsgCounts.at(index) == 0){
            arrayFloat32StartTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }else{
            arrayFloat32LastTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }
        arrayFloat32MsgCounts.at(index) += 1;
    }

    void lidarPointCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, const int& index){
        lidarPointCloudMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(lidarPointCloudMsgCountMtxs.at(index));
        if(lidarPointCloudMsgCounts.at(index) == 0){
            lidarPointCloudStartTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }else{
            lidarPointCloudLastTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }
        lidarPointCloudMsgCounts.at(index) += 1;
    }

    void robotBaseOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr& msg, const int& index){
        robotBaseOdometryMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(robotBaseOdometryMsgCountMtxs.at(index));
        if(robotBaseOdometryMsgCounts.at(index) == 0){
            robotBaseOdometrystartTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }else{
            robotBaseOdometryLastTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }
        robotBaseOdometryMsgCounts.at(index) += 1;
    }

    void robotBaseVelocityHandler(const geometry_msgs::msg::TwistStamped::SharedPtr& msg, const int& index){
        robotBaseVelocityMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(robotBaseVelocityMsgCountMtxs.at(index));
        if(robotBaseVelocityMsgCounts.at(index) == 0){
            robotBaseVelocityStartTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }else{
            robotBaseVelocityLastTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }
        robotBaseVelocityMsgCounts.at(index) += 1;
    }

    #ifdef _USELIFT
    void liftMotorHandler(const bt_task_msgs::msg::LiftMotorMsg::SharedPtr& msg, const int& index){
        liftMotorMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(liftMotorMsgCountMtxs.at(index));
        if(liftMotorMsgCounts.at(index) == 0){
            liftMotorStartTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }else{
            liftMotorLastTimeStamps.at(index) = rclcpp::Time(msg->header.stamp).seconds();
        }
        liftMotorMsgCounts.at(index) += 1;
    }
    #endif

    void instructionTextHandler(const data_msgs::msg::Instruction::SharedPtr& msg, const int& index){
        instructionTextDeques.at(index).push_back(*msg);
        instructionSaving();
    }

    void cameraColorSaving(const int index){
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && cameraColorMsgDeques.at(index).size() == 0)
                    break;
            }
            sensor_msgs::msg::Image msg = cameraColorMsgDeques.at(index).pop_front();
            if(rclcpp::Time(msg.header.stamp).seconds() == 0)
                break;
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::imwrite(cameraColorDirs.at(index) + "/" + std::to_string(rclcpp::Time(msg.header.stamp).seconds()) + ".jpg", cv_ptr->image);
        }
    }

    void cameraDepthSaving(const int index){
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && cameraDepthMsgDeques.at(index).size() == 0)
                    break;
            }
            sensor_msgs::msg::Image msg = cameraDepthMsgDeques.at(index).pop_front();
            if(rclcpp::Time(msg.header.stamp).seconds() == 0)
                break;
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            cv::imwrite(cameraDepthDirs.at(index) + "/" + std::to_string(rclcpp::Time(msg.header.stamp).seconds()) + ".png", cv_ptr->image);
        }
    }

    void cameraPointCloudSaving(const int index){
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && cameraPointCloudMsgDeques.at(index).size() == 0)
                    break;
            }
            sensor_msgs::msg::PointCloud2 msg = cameraPointCloudMsgDeques.at(index).pop_front();
            if(rclcpp::Time(msg.header.stamp).seconds() == 0)
                break;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::fromROSMsg(msg, *pointCloud);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudNorm(new pcl::PointCloud<pcl::PointXYZRGB>());
            if(cameraPointCloudMaxDistances.size() != 0 && cameraPointCloudMaxDistances[index] != 0){
                pcl::PassThrough<pcl::PointXYZRGB> pass;
                pass.setInputCloud(pointCloud);
                pass.setFilterFieldName("z");
                pass.setFilterLimits(0, cameraPointCloudMaxDistances[index]);
                pass.setFilterLimitsNegative(false);
                pass.filter(*pointCloudNorm);
            }else{
                *pointCloudNorm = *pointCloud;
            }
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudDownSize(new pcl::PointCloud<pcl::PointXYZRGB>());
            if(cameraPointCloudDownSizes.size() != 0 && cameraPointCloudDownSizes[index] != 0){
                pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilter;
                downSizeFilter.setLeafSize(cameraPointCloudDownSizes[index], cameraPointCloudDownSizes[index], cameraPointCloudDownSizes[index]);
                downSizeFilter.setInputCloud(pointCloudNorm);
                downSizeFilter.filter(*pointCloudDownSize);
            }else{
                *pointCloudDownSize = *pointCloudNorm;
            }
            pcl::io::savePCDFileBinary(cameraPointCloudDirs.at(index) + "/" + std::to_string(rclcpp::Time(msg.header.stamp).seconds()) + ".pcd", *pointCloudDownSize);
        }
    }

    void armJointStateSaving(const int index){
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && armJointStateMsgDeques.at(index).size() == 0)
                    break;
            }
            sensor_msgs::msg::JointState msg = armJointStateMsgDeques.at(index).pop_front();
            if(rclcpp::Time(msg.header.stamp).seconds() == 0)
                break;
            Json::Value root;
            Json::Value effort(Json::arrayValue);
            Json::Value position(Json::arrayValue);
            Json::Value velocity(Json::arrayValue);
            for(int i = 0; i < msg.effort.size(); i++){
                effort.append(msg.effort.at(i));
            }
            for(int i = 0; i < msg.position.size(); i++){
                position.append(msg.position.at(i));
            }
            for(int i = 0; i < msg.velocity.size(); i++){
                velocity.append(msg.velocity.at(i));
            }
            root["effort"] = effort;
            root["position"] = position;
            root["velocity"] = velocity;
            Json::StyledStreamWriter streamWriter;
            std::ofstream file(armJointStateDirs.at(index) + "/" + std::to_string(rclcpp::Time(msg.header.stamp).seconds()) + ".json");
            streamWriter.write(file, root);
            file.close();
        }
    }

    void armEndPoseSaving(const int index){
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && armEndPoseMsgDeques.at(index).size() == 0)
                    break;
            }
            geometry_msgs::msg::PoseStamped msg = armEndPoseMsgDeques.at(index).pop_front();
            if(rclcpp::Time(msg.header.stamp).seconds() == 0)
                break;
            Json::Value root;
            if(armEndPoseOrients.at(index)){
                root["x"] = msg.pose.position.x;
                root["y"] = msg.pose.position.y;
                root["z"] = msg.pose.position.z;
                tf2::Quaternion quat;
                tf2::fromMsg(msg.pose.orientation, quat);
                tf2::Matrix3x3 matrix(quat);
                double roll, pitch, yaw;
                matrix.getRPY(roll, pitch, yaw);
                root["roll"] = roll;
                root["pitch"] = pitch;
                root["yaw"] = yaw; 
            }else{
                root["x"] = msg.pose.position.x;
                root["y"] = msg.pose.position.y;
                root["z"] = msg.pose.position.z;
                root["roll"] = msg.pose.orientation.x;
                root["pitch"] = msg.pose.orientation.y;
                root["yaw"] = msg.pose.orientation.z;
                root["grasper"] = msg.pose.orientation.w;
            }
            Json::StyledStreamWriter streamWriter;
            std::ofstream file(armEndPoseDirs.at(index) + "/" + std::to_string(rclcpp::Time(msg.header.stamp).seconds()) + ".json");
            streamWriter.write(file, root);
            file.close();
        }
    }

    void localizationPoseSaving(const int index){
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && localizationPoseMsgDeques.at(index).size() == 0)
                    break;
            }
            geometry_msgs::msg::PoseStamped msg = localizationPoseMsgDeques.at(index).pop_front();
            if(rclcpp::Time(msg.header.stamp).seconds() == 0)
                break;
            Json::Value root;
            root["x"] = msg.pose.position.x;
            root["y"] = msg.pose.position.y;
            root["z"] = msg.pose.position.z;
            tf2::Quaternion quat;
            tf2::fromMsg(msg.pose.orientation, quat);
            tf2::Matrix3x3 matrix(quat);
            double roll, pitch, yaw;
            matrix.getRPY(roll, pitch, yaw);
            root["roll"] = roll;
            root["pitch"] = pitch;
            root["yaw"] = yaw;            
            Json::StyledStreamWriter streamWriter;
            std::ofstream file(localizationPoseDirs.at(index) + "/" + std::to_string(rclcpp::Time(msg.header.stamp).seconds()) + ".json");
            streamWriter.write(file, root);
            file.close();
        }
    }

    void force6dimSaving(const int index){
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && force6dimMsgDeques.at(index).size() == 0)
                    break;
            }
            geometry_msgs::msg::WrenchStamped msg = force6dimMsgDeques.at(index).pop_front();
            if(rclcpp::Time(msg.header.stamp).seconds() == 0)
                break;
            Json::Value root;
            root["force"]["x"] = msg.wrench.force.x;
            root["force"]["y"] = msg.wrench.force.y;
            root["force"]["z"] = msg.wrench.force.z;
            root["torque"]["x"] = msg.wrench.torque.x;
            root["torque"]["y"] = msg.wrench.torque.y;
            root["torque"]["z"] = msg.wrench.torque.z;
            Json::StyledStreamWriter streamWriter;
            std::ofstream file(force6dimDirs.at(index) + "/" + std::to_string(rclcpp::Time(msg.header.stamp).seconds()) + ".json");
            streamWriter.write(file, root);
            file.close();
        }
    }
    
    void gripperEncoderSaving(const int index){
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && gripperEncoderMsgDeques.at(index).size() == 0)
                    break;
            }
            data_msgs::msg::Gripper msg = gripperEncoderMsgDeques.at(index).pop_front();
            if(rclcpp::Time(msg.header.stamp).seconds() == 0)
                break;
            Json::Value root;
            root["angle"] = msg.angle;
            root["distance"] = msg.distance;
            Json::StyledStreamWriter streamWriter;
            std::ofstream file(gripperEncoderDirs.at(index) + "/" + std::to_string(rclcpp::Time(msg.header.stamp).seconds()) + ".json");
            streamWriter.write(file, root);
            file.close();
        }
    }

    void imu9AxisSaving(const int index){
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && imu9AxisMsgDeques.at(index).size() == 0)
                    break;
            }
            sensor_msgs::msg::Imu msg = imu9AxisMsgDeques.at(index).pop_front();
            if(rclcpp::Time(msg.header.stamp).seconds() == 0)
                break;
            Json::Value root;
            root["orientation"]["x"] = msg.orientation.x;
            root["orientation"]["y"] = msg.orientation.y;
            root["orientation"]["z"] = msg.orientation.z;
            root["orientation"]["w"] = msg.orientation.w;
            root["angular_velocity"]["x"] = msg.angular_velocity.x;
            root["angular_velocity"]["y"] = msg.angular_velocity.y;
            root["angular_velocity"]["z"] = msg.angular_velocity.z;
            root["linear_acceleration"]["x"] = msg.linear_acceleration.x;
            root["linear_acceleration"]["y"] = msg.linear_acceleration.y;
            root["linear_acceleration"]["z"] = msg.linear_acceleration.z;
            Json::StyledStreamWriter streamWriter;
            std::ofstream file(imu9AxisDirs.at(index) + "/" + std::to_string(rclcpp::Time(msg.header.stamp).seconds()) + ".json");
            streamWriter.write(file, root);
            file.close();
        }
    }

    void arrayFloat32Saving(const int index){
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && arrayFloat32MsgDeques.at(index).size() == 0)
                    break;
            }
            data_msgs::msg::Array msg = arrayFloat32MsgDeques.at(index).pop_front();
            if(rclcpp::Time(msg.header.stamp).seconds() == 0)
                break;
            
            std::string filePath = arrayFloat32Dirs.at(index) + "/" + std::to_string(rclcpp::Time(msg.header.stamp).seconds()) + ".npy";
            std::ofstream file(filePath, std::ios::binary);
            
            // Use NPY structured array with separate named fields
            // Fields: 'shape', 'dim_description', 'data'
            // Each field is stored independently for clearer structure
            
            // Prepare shape data (uint32 array)
            std::vector<uint32_t> shapeData = msg.shape;
            
            // Prepare dim_description data (encode as uint8 array: [count, len0, str0..., len1, str1...])
            std::vector<uint8_t> dimDescData;
            dimDescData.push_back(static_cast<uint8_t>(msg.dim_description.size()));
            for(const auto& desc : msg.dim_description) {
                dimDescData.push_back(static_cast<uint8_t>(desc.length()));
                for(char c : desc) {
                    dimDescData.push_back(static_cast<uint8_t>(c));
                }
            }
            
            // Write NPY 1.0 with structured dtype for compatibility with np.load()
            const char magic[] = "\x93NUMPY";
            file.write(magic, 6);
            uint8_t versionMajor = 1, versionMinor = 0;
            file.write(reinterpret_cast<const char*>(&versionMajor), 1);
            file.write(reinterpret_cast<const char*>(&versionMinor), 1);
            
            // Build structured dtype with three separate fields (use double quotes for Python 3 compatibility)
            // [("shape", "<u4", (shapeLen,)), ("dim_description", "|u1", (dimDescLen,)), ("data", "<f4", (dataLen,))]
            std::string dtypeStr = "[(\"shape\", \"<u4\", (" + std::to_string(shapeData.size()) + ",)), ";
            dtypeStr += "(\"dim_description\", \"|u1\", (" + std::to_string(dimDescData.size()) + ",)), ";
            dtypeStr += "(\"data\", \"<f4\", (" + std::to_string(msg.data.size()) + ",))]";
            std::string headerDict = "{\"descr\": " + dtypeStr + ", \"fortran_order\": False, \"shape\": (), }";
            
            // Calculate padding: header must end at 64-byte boundary (NPY 1.0 requirement)
            // Total header = magic(6) + version(2) + headerLen(2) + headerDict + '\n' + padding
            size_t prefixLen = 6 + 2 + 2;  // magic(6) + version(2) + headerLen(2)
            std::string headerWithNewline = headerDict + "\n";
            size_t totalBeforePadding = prefixLen + headerWithNewline.length();
            
            // Find padding needed to align to 64 bytes (changed from 16 to 64)
            size_t padding = 0;
            if (totalBeforePadding % 64 != 0) {
                padding = 64 - (totalBeforePadding % 64);
            }
            
            // Append padding spaces to header
            for (size_t i = 0; i < padding; ++i) {
                headerWithNewline += " ";
            }
            
            // NPY 1.0 uses uint16 for header length (2 bytes)
            uint16_t headerLenLe = static_cast<uint16_t>(headerWithNewline.length());
            file.write(reinterpret_cast<const char*>(&headerLenLe), 2);
            file.write(headerWithNewline.c_str(), headerWithNewline.length());
            
            // Write structured data: shape, dim_description, then data (in order of dtype)
            file.write(reinterpret_cast<const char*>(shapeData.data()), shapeData.size() * sizeof(uint32_t));
            file.write(reinterpret_cast<const char*>(dimDescData.data()), dimDescData.size() * sizeof(uint8_t));
            file.write(reinterpret_cast<const char*>(msg.data.data()), msg.data.size() * sizeof(float));
            file.close();
        }
    }

    void lidarPointCloudSaving(const int index){
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && lidarPointCloudMsgDeques.at(index).size() == 0)
                    break;
            }
            sensor_msgs::msg::PointCloud2 msg = lidarPointCloudMsgDeques.at(index).pop_front();
            if(rclcpp::Time(msg.header.stamp).seconds() == 0)
                break;
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(msg, *pointCloud);

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudNorm(new pcl::PointCloud<pcl::PointXYZI>());
            if(lidarPointCloudXDistancelowers.size() != 0 && lidarPointCloudXDistanceUppers.size() != 0 && lidarPointCloudXDistancelowers[index] != lidarPointCloudXDistanceUppers[index] && 
               lidarPointCloudYDistancelowers.size() != 0 && lidarPointCloudYDistanceUppers.size() != 0 && lidarPointCloudYDistancelowers[index] != lidarPointCloudYDistanceUppers[index] && 
               lidarPointCloudZDistancelowers.size() != 0 && lidarPointCloudZDistanceUppers.size() != 0 && lidarPointCloudZDistancelowers[index] != lidarPointCloudZDistanceUppers[index]){
                pcl::CropBox<pcl::PointXYZI> cropFilter;
                cropFilter.setMin(Eigen::Vector4f(lidarPointCloudXDistancelowers[index], lidarPointCloudYDistancelowers[index], lidarPointCloudZDistancelowers[index], 1.0));
                cropFilter.setMax(Eigen::Vector4f(lidarPointCloudXDistanceUppers[index], lidarPointCloudYDistanceUppers[index], lidarPointCloudZDistanceUppers[index], 1.0));
                cropFilter.setInputCloud(pointCloud);
                cropFilter.filter(*pointCloudNorm);
            }else{
                *pointCloudNorm = *pointCloud;
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudDownSize(new pcl::PointCloud<pcl::PointXYZI>());
            if(lidarPointCloudDownSizes.size() != 0 && lidarPointCloudDownSizes[index] != 0){
                pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
                downSizeFilter.setLeafSize(lidarPointCloudDownSizes[index], lidarPointCloudDownSizes[index], lidarPointCloudDownSizes[index]);
                downSizeFilter.setInputCloud(pointCloudNorm);
                downSizeFilter.filter(*pointCloudDownSize);
            }else{
                *pointCloudDownSize = *pointCloudNorm;
            }

            pcl::io::savePCDFileBinary(lidarPointCloudDirs.at(index) + "/" + std::to_string(rclcpp::Time(msg.header.stamp).seconds()) + ".pcd", *pointCloudDownSize);
        }
    }

    void robotBaseOdometrysaving(const int index){
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && robotBaseOdometryMsgDeques.at(index).size() == 0)
                    break;
            }
            nav_msgs::msg::Odometry msg = robotBaseOdometryMsgDeques.at(index).pop_front();
            if(rclcpp::Time(msg.header.stamp).seconds() == 0)
                break;
            Json::Value root;
            root["linear"]["x"] = msg.twist.twist.linear.x;
            root["linear"]["y"] = msg.twist.twist.linear.y;
            root["linear"]["z"] = msg.twist.twist.linear.z;
            root["angular"]["x"] = msg.twist.twist.angular.x;
            root["angular"]["y"] = msg.twist.twist.angular.y;
            root["angular"]["z"] = msg.twist.twist.angular.z;
            root["position"]["x"] = msg.pose.pose.position.x;
            root["position"]["y"] = msg.pose.pose.position.y;
            root["position"]["z"] = msg.pose.pose.position.z;
            root["orientation"]["x"] = msg.pose.pose.orientation.x;
            root["orientation"]["y"] = msg.pose.pose.orientation.y;
            root["orientation"]["z"] = msg.pose.pose.orientation.z;
            root["orientation"]["w"] = msg.pose.pose.orientation.w;
            Json::StyledStreamWriter streamWriter;
            std::ofstream file(robotBaseOdometryDirs.at(index) + "/" + std::to_string(rclcpp::Time(msg.header.stamp).seconds()) + ".json");
            streamWriter.write(file, root);
            file.close();
        }
    }

    void robotBaseVelocitySaving(const int index){
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && robotBaseVelocityMsgDeques.at(index).size() == 0)
                    break;
            }
            geometry_msgs::msg::TwistStamped msg = robotBaseVelocityMsgDeques.at(index).pop_front();
            if(rclcpp::Time(msg.header.stamp).seconds() == 0)
                break;
            Json::Value root;
            root["linear"]["x"] = msg.twist.linear.x;
            root["linear"]["y"] = msg.twist.linear.y;
            root["linear"]["z"] = msg.twist.linear.z;
            root["angular"]["x"] = msg.twist.angular.x;
            root["angular"]["y"] = msg.twist.angular.y;
            root["angular"]["z"] = msg.twist.angular.z;
            Json::StyledStreamWriter streamWriter;
            std::ofstream file(robotBaseVelocityDirs.at(index) + "/" + std::to_string(rclcpp::Time(msg.header.stamp).seconds()) + ".json");
            streamWriter.write(file, root);
            file.close();
        }
    }

    #ifdef _USELIFT
    void liftMotorSaving(const int index){
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && liftMotorMsgDeques.at(index).size() == 0)
                    break;
            }
            bt_task_msgs::msg::LiftMotorMsg msg = liftMotorMsgDeques.at(index).pop_front();
            if(rclcpp::Time(msg.header.stamp).seconds() == 0)
                break;
            Json::Value root;
            root["backHeight"] = msg.backHeight;
            Json::StyledStreamWriter streamWriter;
            std::ofstream file(liftMotorDirs.at(index) + "/" + std::to_string(rclcpp::Time(msg.header.stamp).seconds()) + ".json");
            streamWriter.write(file, root);
            file.close();
        }
    }
    #endif


    void tfTransformSaving(const int index){
        geometry_msgs::msg::TransformStamped transform;
        bool getFrame = false;
		while(rclcpp::ok()){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop)
                    break;
            }
            try {
                transform = tfBuffer->lookupTransform(tfTransformParentFrames.at(index), tfTransformChildFrames.at(index), tf2::TimePointZero);
                getFrame = true;
                break;
            } catch (const tf2::TransformException & ex) {
                continue;
            }
		}
        if(getFrame){
            double x = transform.transform.translation.x;
            double y = transform.transform.translation.y;
            double z = transform.transform.translation.z;
            double roll, pitch, yaw;
            tf2::Quaternion q(transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);

            Json::Value root;
            root["parent_frame"] = tfTransformParentFrames.at(index);
            root["child_frame"] = tfTransformChildFrames.at(index);
            root["x"] = x;
            root["y"] = y;
            root["z"] = z;
            root["roll"] = roll;
            root["pitch"] = pitch;
            root["yaw"] = yaw;
            Json::StyledStreamWriter streamWriter;
            std::ofstream file(tfTransformDirs.at(index));
            streamWriter.write(file, root);
            file.close();
            std::lock_guard<std::mutex> lock(tfTransformMsgCountMtxs.at(index));
            tfTransformMsgCounts.at(index) += 1;
        }
    }

    void instructionSaving(std::string instructions){
        if (instructions.front() != '\\' || instructions.back() != ']') {
             std::cout << "Error parsing JSON: " << instructions << std::endl;
            return;
        }
        std::string content = instructions.substr(2, instructions.size() - 4);
        std::istringstream iss(content);
        std::string token;
        std::string jsonArray = "[";
        while (std::getline(iss, token, ',')) {
            if (!jsonArray.empty() && jsonArray.back() != '[') {
                jsonArray += ", ";
            }
            jsonArray += "\"" + token + "\"";
        }
        jsonArray += "]";

        Json::Value root, result;
        Json::CharReaderBuilder builder;
        Json::CharReader* reader = builder.newCharReader();

        std::string errors;
        bool parsingSuccessful = reader->parse(jsonArray.c_str(), jsonArray.c_str() + jsonArray.size(), &result, &errors);
        delete reader;

        if (!parsingSuccessful) {
            std::cout << "Error parsing JSON: " << jsonArray << std::endl;
            return;
        }

        root["full-instructions"] = result;
        root["segment-instructions"] = Json::Value(Json::arrayValue);
        Json::StyledStreamWriter streamWriter;
        std::ofstream file(instructionsDir);
        streamWriter.write(file, root);
        file.close();
    }
        
    void instructionSaving(){
        Json::Reader jsonReader;
        Json::Value root;
        std::ifstream file(instructionsDir, std::iostream::binary);
        jsonReader.parse(file, root);
        file.close();
        for (int i = 0; i < instructionTextDeques.size(); i++) {
            while (instructionTextDeques.at(i).size()) {
                Json::Value segment_instruction;
                data_msgs::msg::Instruction instruction = instructionTextDeques.at(i).pop_front();
                segment_instruction["start_time"] = rclcpp::Time(instruction.start_stamp).seconds();
                segment_instruction["end_time"] = rclcpp::Time(instruction.end_stamp).seconds();
                
                segment_instruction["instructions"] = Json::Value(Json::arrayValue);
                segment_instruction["instructions"].append(instruction.text);

                root["segment-instructions"].append(segment_instruction);
            }
        }
        Json::StyledStreamWriter streamWriter;
        std::ofstream fileSave(instructionsDir);
        streamWriter.write(fileSave, root);
        fileSave.close();
    }

    // void keyboardInterruptChecking(){
    //     std::string line;
    //     rclcpp::Rate rate(10);
    //     while (rclcpp::ok()) {
    //         rate.sleep();
    //     }
    //     shutdown();
    // }

    void keyboardInterruptChecking(){
        std::string line;
        if (std::getline(std::cin, line)) {
            shutdown();
        }
    }

    void shutdown(){
        captureStopMtx.lock();
        captureStop = true;
        captureStopMtx.unlock();
        for(int i = 0; i < cameraColorNames.size(); i++){
            subCameraColors.at(i).reset();
            sensor_msgs::msg::Image msg;
            msg.header.stamp = rclcpp::Time(0);
            cameraColorMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < cameraDepthNames.size(); i++){
            subCameraDepths.at(i).reset();
            sensor_msgs::msg::Image msg;
            msg.header.stamp = rclcpp::Time(0);
            cameraDepthMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            subCameraPointClouds.at(i).reset();
            sensor_msgs::msg::PointCloud2 msg;
            msg.header.stamp = rclcpp::Time(0);
            cameraPointCloudMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < armJointStateNames.size(); i++){
            subArmJointStates.at(i).reset();
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = rclcpp::Time(0);
            armJointStateMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < armEndPoseNames.size(); i++){
            subArmEndPoses.at(i).reset();
            geometry_msgs::msg::PoseStamped msg;
            msg.header.stamp = rclcpp::Time(0);
            armEndPoseMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < localizationPoseNames.size(); i++){
            subLocalizationPoses.at(i).reset();
            geometry_msgs::msg::PoseStamped msg;
            msg.header.stamp = rclcpp::Time(0);
            localizationPoseMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < force6dimNames.size(); i++){
            subForce6dims.at(i).reset();
            geometry_msgs::msg::WrenchStamped msg;
            msg.header.stamp = rclcpp::Time(0);
            force6dimMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < gripperEncoderNames.size(); i++){
            subGripperEncoders.at(i).reset();
            data_msgs::msg::Gripper msg;
            msg.header.stamp = rclcpp::Time(0);
            gripperEncoderMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < imu9AxisNames.size(); i++){
            subImu9Axiss.at(i).reset();
            sensor_msgs::msg::Imu msg;
            msg.header.stamp = rclcpp::Time(0);
            imu9AxisMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < arrayFloat32Names.size(); i++){
            subArrayFloat32s.at(i).reset();
            data_msgs::msg::Array msg;
            msg.header.stamp = rclcpp::Time(0);
            arrayFloat32MsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < lidarPointCloudNames.size(); i++){
            subLidarPointClouds.at(i).reset();
            sensor_msgs::msg::PointCloud2 msg;
            msg.header.stamp = rclcpp::Time(0);
            lidarPointCloudMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < robotBaseOdometryNames.size(); i++){
            subRobotBaseOdometrys.at(i).reset();
            nav_msgs::msg::Odometry msg;
            msg.header.stamp = rclcpp::Time(0);
            robotBaseOdometryMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < robotBaseVelocityNames.size(); i++){
            subRobotBaseVelocitys.at(i).reset();
            geometry_msgs::msg::TwistStamped msg;
            msg.header.stamp = rclcpp::Time(0);
            robotBaseVelocityMsgDeques.at(i).push_back(msg);
        }
        #ifdef _USELIFT
        for(int i = 0; i < liftMotorNames.size(); i++){
            subLiftMotors.at(i).reset();
            bt_task_msgs::msg::LiftMotorMsg msg;
            msg.header.stamp = rclcpp::Time(0);
            liftMotorMsgDeques.at(i).push_back(msg);
        }
        #endif
        if(!this->useService)
            rclcpp::shutdown();
    }

    void monitoring(){
        rclcpp::Rate rate(1);
        std::vector<int> cameraColorMsgLastCounts = std::vector<int>(cameraColorNames.size(), 0);
        std::vector<int> cameraDepthMsgLastCounts = std::vector<int>(cameraDepthNames.size(), 0);
        std::vector<int> cameraPointCloudMsgLastCounts = std::vector<int>(cameraPointCloudNames.size(), 0);
        std::vector<int> armJointStateMsgLastCounts = std::vector<int>(armJointStateNames.size(), 0);
        std::vector<int> armEndPoseMsgLastCounts = std::vector<int>(armEndPoseNames.size(), 0);
        std::vector<int> localizationPoseMsgLastCounts = std::vector<int>(localizationPoseNames.size(), 0);
        std::vector<int> force6dimMsgLastCounts = std::vector<int>(force6dimNames.size(), 0);
        std::vector<int> gripperEncoderMsgLastCounts = std::vector<int>(gripperEncoderNames.size(), 0);
        std::vector<int> imu9AxisMsgLastCounts = std::vector<int>(imu9AxisNames.size(), 0);
        std::vector<int> arrayFloat32MsgLastCounts = std::vector<int>(arrayFloat32Names.size(), 0);
        std::vector<int> lidarPointCloudMsgLastCounts = std::vector<int>(lidarPointCloudNames.size(), 0);
        std::vector<int> robotBaseOdometryMsgLastCounts = std::vector<int>(robotBaseOdometryNames.size(), 0);
        std::vector<int> robotBaseVelocityMsgLastCounts = std::vector<int>(robotBaseVelocityNames.size(), 0);
        std::vector<int> liftMotorMsgLastCounts = std::vector<int>(liftMotorNames.size(), 0);
        std::vector<double> cameraColorMsgLastUpHzTimes = std::vector<double>(cameraColorNames.size(), 0);
        std::vector<double> cameraDepthMsgLastUpHzTimes = std::vector<double>(cameraDepthNames.size(), 0);
        std::vector<double> cameraPointCloudMsgLastUpHzTimes = std::vector<double>(cameraPointCloudNames.size(), 0);
        std::vector<double> armJointStateMsgLastUpHzTimes = std::vector<double>(armJointStateNames.size(), 0);
        std::vector<double> armEndPoseMsgLastUpHzTimes = std::vector<double>(armEndPoseNames.size(), 0);
        std::vector<double> localizationPoseMsgLastUpHzTimes = std::vector<double>(localizationPoseNames.size(), 0);
        std::vector<double> force6dimMsgLastUpHzTimes = std::vector<double>(force6dimNames.size(), 0);
        std::vector<double> gripperEncoderMsgLastUpHzTimes = std::vector<double>(gripperEncoderNames.size(), 0);
        std::vector<double> imu9AxisMsgLastUpHzTimes = std::vector<double>(imu9AxisNames.size(), 0);
        std::vector<double> arrayFloat32MsgLastUpHzTimes = std::vector<double>(arrayFloat32Names.size(), 0);
        std::vector<double> lidarPointCloudMsgLastUpHzTimes = std::vector<double>(lidarPointCloudNames.size(), 0);
        std::vector<double> robotBaseOdometryMsgLastUpHzTimes = std::vector<double>(robotBaseOdometryNames.size(), 0);
        std::vector<double> robotBaseVelocityMsgLastUpHzTimes = std::vector<double>(robotBaseVelocityNames.size(), 0);
        std::vector<double> liftMotorMsgLastUpHzTimes = std::vector<double>(liftMotorNames.size(), 0);
        rclcpp::Time beginTime = rclcpp::Clock().now();
        data_msgs::msg::CaptureStatus captureStatus;
        while(true){
            captureStatus.topics.clear();
            captureStatus.count_in_seconds.clear();
            captureStatus.frequencies.clear();
            captureStatus.fail = false;
            std::ofstream file(statisticsDir);
            system("clear");
            std::cout<<"path: "<<episodeDir<<std::endl;
            std::cout<<"total time: "<<(rclcpp::Clock().now() - beginTime).seconds()<<std::endl;
            int allCount = 0;
            std::cout<<"topic: frame in 1 second / total frame"<<std::endl;
            file<<(rclcpp::Clock().now() - beginTime).seconds()<<std::endl;
            file<<"topic:"<<std::endl;
            for(int i = 0; i < cameraColorNames.size(); i++){
                cameraColorMsgCountMtxs.at(i).lock();
                int count = cameraColorMsgCounts.at(i);
                double startTime = cameraColorStartTimeStamps.at(i);
                double lastTime = cameraColorLastTimeStamps.at(i);
                cameraColorMsgCountMtxs.at(i).unlock();
                int countInSecond = count - cameraColorMsgLastCounts.at(i);
                double frequency = ((double)count / (lastTime - startTime));
                std::cout<<cameraColorTopics.at(i)<<": "<<countInSecond<<" / "<<count<<"("<<frequency<<"hz)"<<std::endl;
                if(cameraColorMsgLastUpHzTimes.at(i) == 0){
                    cameraColorMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(rclcpp::Clock().now().seconds() - cameraColorMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<cameraColorTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    cameraColorMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                cameraColorMsgLastCounts.at(i) = count;
                allCount += count;
                file<<"camera/color/"<<cameraColorNames.at(i)<<" "<<count<<" "<<frequency<<std::endl;
                captureStatus.topics.push_back(cameraColorTopics.at(i));
                captureStatus.count_in_seconds.push_back(countInSecond);
                captureStatus.frequencies.push_back(frequency);
            }
            for(int i = 0; i < cameraDepthNames.size(); i++){
                cameraDepthMsgCountMtxs.at(i).lock();
                int count = cameraDepthMsgCounts.at(i);
                double startTime = cameraDepthStartTimeStamps.at(i);
                double lastTime = cameraDepthLastTimeStamps.at(i);
                cameraDepthMsgCountMtxs.at(i).unlock();
                int countInSecond = count - cameraDepthMsgLastCounts.at(i);
                double frequency = ((double)count / (lastTime - startTime));
                std::cout<<cameraDepthTopics.at(i)<<": "<<countInSecond<<" / "<<count<<"("<<frequency<<"hz)"<<std::endl;
                if(cameraDepthMsgLastUpHzTimes.at(i) == 0){
                    cameraDepthMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(rclcpp::Clock().now().seconds() - cameraDepthMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<cameraDepthTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    cameraDepthMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                cameraDepthMsgLastCounts.at(i) = count;
                allCount += count;
                file<<"camera/depth/"<<cameraDepthNames.at(i)<<" "<<count<<" "<<frequency<<std::endl;
                captureStatus.topics.push_back(cameraDepthTopics.at(i));
                captureStatus.count_in_seconds.push_back(countInSecond);
                captureStatus.frequencies.push_back(frequency);
            }
            for(int i = 0; i < cameraPointCloudNames.size(); i++){
                cameraPointCloudMsgCountMtxs.at(i).lock();
                int count = cameraPointCloudMsgCounts.at(i);
                double startTime = cameraPointCloudStartTimeStamps.at(i);
                double lastTime = cameraPointCloudLastTimeStamps.at(i);
                cameraPointCloudMsgCountMtxs.at(i).unlock();
                int countInSecond = count - cameraPointCloudMsgLastCounts.at(i);
                double frequency = ((double)count / (lastTime - startTime));
                std::cout<<cameraPointCloudTopics.at(i)<<": "<<countInSecond<<" / "<<count<<"("<<frequency<<"hz)"<<std::endl;
                if(cameraPointCloudMsgLastUpHzTimes.at(i) == 0){
                    cameraPointCloudMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(rclcpp::Clock().now().seconds() - cameraPointCloudMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<cameraPointCloudTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    cameraPointCloudMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                cameraPointCloudMsgLastCounts.at(i) = count;
                allCount += count;
                file<<"camera/pointCloud/"<<cameraPointCloudNames.at(i)<<" "<<count<<" "<<frequency<<std::endl;
                captureStatus.topics.push_back(cameraPointCloudTopics.at(i));
                captureStatus.count_in_seconds.push_back(countInSecond);
                captureStatus.frequencies.push_back(frequency);
            }
            for(int i = 0; i < armJointStateNames.size(); i++){
                armJointStateMsgCountMtxs.at(i).lock();
                int count = armJointStateMsgCounts.at(i);
                double startTime = armJointStateStartTimeStamps.at(i);
                double lastTime = armJointStateLastTimeStamps.at(i);
                armJointStateMsgCountMtxs.at(i).unlock();
                int countInSecond = count - armJointStateMsgLastCounts.at(i);
                double frequency = ((double)count / (lastTime - startTime));
                std::cout<<armJointStateTopics.at(i)<<": "<<countInSecond<<" / "<<count<<"("<<frequency<<"hz)"<<std::endl;
                if(armJointStateMsgLastUpHzTimes.at(i) == 0){
                    armJointStateMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(rclcpp::Clock().now().seconds() - armJointStateMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<armJointStateTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    armJointStateMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                if(!armJointStateChangeList.at(i)){
                    std::cout<<"armJointState "<<armJointStateTopics.at(i)<<" not changing!!!"<<std::endl;
                }
                armJointStateMsgLastCounts.at(i) = count;
                allCount += count;
                file<<"arm/jointState/"<<armJointStateNames.at(i)<<" "<<count<<" "<<frequency<<std::endl;
                captureStatus.topics.push_back(armJointStateTopics.at(i));
                captureStatus.count_in_seconds.push_back(countInSecond);
                captureStatus.frequencies.push_back(frequency);
            }
            for(int i = 0; i < armEndPoseNames.size(); i++){
                armEndPoseMsgCountMtxs.at(i).lock();
                int count = armEndPoseMsgCounts.at(i);
                double startTime = armEndPoseStartTimeStamps.at(i);
                double lastTime = armEndPoseLastTimeStamps.at(i);
                armEndPoseMsgCountMtxs.at(i).unlock();
                int countInSecond = count - armEndPoseMsgLastCounts.at(i);
                double frequency = ((double)count / (lastTime - startTime));
                std::cout<<armEndPoseTopics.at(i)<<": "<<countInSecond<<" / "<<count<<"("<<frequency<<"hz)"<<std::endl;
                if(armEndPoseMsgLastUpHzTimes.at(i) == 0){
                    armEndPoseMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(rclcpp::Clock().now().seconds() - armEndPoseMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<armEndPoseTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    armEndPoseMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                armEndPoseMsgLastCounts.at(i) = count;
                allCount += count;
                file<<"arm/endPose/"<<armEndPoseNames.at(i)<<" "<<count<<" "<<frequency<<std::endl;
                captureStatus.topics.push_back(armEndPoseTopics.at(i));
                captureStatus.count_in_seconds.push_back(countInSecond);
                captureStatus.frequencies.push_back(frequency);
            }
            for(int i = 0; i < localizationPoseNames.size(); i++){
                localizationPoseMsgCountMtxs.at(i).lock();
                int count = localizationPoseMsgCounts.at(i);
                double startTime = localizationPoseStartTimeStamps.at(i);
                double lastTime = localizationPoseLastTimeStamps.at(i);
                localizationPoseMsgCountMtxs.at(i).unlock();
                int countInSecond = count - localizationPoseMsgLastCounts.at(i);
                double frequency = ((double)count / (lastTime - startTime));
                std::cout<<localizationPoseTopics.at(i)<<": "<<countInSecond<<" / "<<count<<"("<<frequency<<"hz)"<<std::endl;
                if(localizationPoseMsgLastUpHzTimes.at(i) == 0){
                    localizationPoseMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(rclcpp::Clock().now().seconds() - localizationPoseMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<localizationPoseTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    localizationPoseMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                localizationPoseMsgLastCounts.at(i) = count;
                allCount += count;
                file<<"localization/pose/"<<localizationPoseNames.at(i)<<" "<<count<<" "<<frequency<<std::endl;
                captureStatus.topics.push_back(localizationPoseTopics.at(i));
                captureStatus.count_in_seconds.push_back(countInSecond);
                captureStatus.frequencies.push_back(frequency);
            }
            for(int i = 0; i < force6dimNames.size(); i++){
                force6dimMsgCountMtxs.at(i).lock();
                int count = force6dimMsgCounts.at(i);
                double startTime = force6dimStartTimeStamps.at(i);
                double lastTime = force6dimLastTimeStamps.at(i);
                force6dimMsgCountMtxs.at(i).unlock();
                int countInSecond = count - force6dimMsgLastCounts.at(i);
                double frequency = ((double)count / (lastTime - startTime));
                std::cout<<force6dimTopics.at(i)<<": "<<countInSecond<<" / "<<count<<"("<<frequency<<"hz)"<<std::endl;
                if(force6dimMsgLastUpHzTimes.at(i) == 0){
                    force6dimMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(rclcpp::Clock().now().seconds() - force6dimMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<force6dimTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    force6dimMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                force6dimMsgLastCounts.at(i) = count;
                allCount += count;
                file<<"force/6dim/"<<force6dimNames.at(i)<<" "<<count<<" "<<frequency<<std::endl;
                captureStatus.topics.push_back(force6dimTopics.at(i));
                captureStatus.count_in_seconds.push_back(countInSecond);
                captureStatus.frequencies.push_back(frequency);
            }
            for(int i = 0; i < gripperEncoderNames.size(); i++){
                gripperEncoderMsgCountMtxs.at(i).lock();
                int count = gripperEncoderMsgCounts.at(i);
                double startTime = gripperEncoderStartTimeStamps.at(i);
                double lastTime = gripperEncoderLastTimeStamps.at(i);
                gripperEncoderMsgCountMtxs.at(i).unlock();
                int countInSecond = count - gripperEncoderMsgLastCounts.at(i);
                double frequency = ((double)count / (lastTime - startTime));
                std::cout<<gripperEncoderTopics.at(i)<<": "<<countInSecond<<" / "<<count<<"("<<frequency<<"hz)"<<std::endl;
                if(gripperEncoderMsgLastUpHzTimes.at(i) == 0){
                    gripperEncoderMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(rclcpp::Clock().now().seconds() - gripperEncoderMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<gripperEncoderTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    gripperEncoderMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                gripperEncoderMsgLastCounts.at(i) = count;
                allCount += count;
                file<<"gripper/encoder/"<<gripperEncoderNames.at(i)<<" "<<count<<" "<<frequency<<std::endl;
                captureStatus.topics.push_back(gripperEncoderTopics.at(i));
                captureStatus.count_in_seconds.push_back(countInSecond);
                captureStatus.frequencies.push_back(frequency);
            }
            for(int i = 0; i < imu9AxisNames.size(); i++){
                imu9AxisMsgCountMtxs.at(i).lock();
                int count = imu9AxisMsgCounts.at(i);
                double startTime = imu9AxisStartTimeStamps.at(i);
                double lastTime = imu9AxisLastTimeStamps.at(i);
                imu9AxisMsgCountMtxs.at(i).unlock();
                int countInSecond = count - imu9AxisMsgLastCounts.at(i);
                double frequency = ((double)count / (lastTime - startTime));
                std::cout<<imu9AxisTopics.at(i)<<": "<<countInSecond<<" / "<<count<<"("<<frequency<<"hz)"<<std::endl;                
                if(imu9AxisMsgLastUpHzTimes.at(i) == 0){
                    imu9AxisMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(rclcpp::Clock().now().seconds() - imu9AxisMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<imu9AxisTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    imu9AxisMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                imu9AxisMsgLastCounts.at(i) = count;
                allCount += count;
                file<<"imu/9axis/"<<imu9AxisNames.at(i)<<" "<<count<<" "<<frequency<<std::endl;
                captureStatus.topics.push_back(imu9AxisTopics.at(i));
                captureStatus.count_in_seconds.push_back(countInSecond);
                captureStatus.frequencies.push_back(frequency);
            }
            for(int i = 0; i < arrayFloat32Names.size(); i++){
                arrayFloat32MsgCountMtxs.at(i).lock();
                int count = arrayFloat32MsgCounts.at(i);
                double startTime = arrayFloat32StartTimeStamps.at(i);
                double lastTime = arrayFloat32LastTimeStamps.at(i);
                arrayFloat32MsgCountMtxs.at(i).unlock();
                int countInSecond = count - arrayFloat32MsgLastCounts.at(i);
                double frequency = ((double)count / (lastTime - startTime));
                std::cout<<arrayFloat32Topics.at(i)<<": "<<countInSecond<<" / "<<count<<"("<<frequency<<"hz)"<<std::endl;
                if(arrayFloat32MsgLastUpHzTimes.at(i) == 0){
                    arrayFloat32MsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(rclcpp::Clock().now().seconds() - arrayFloat32MsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<arrayFloat32Topics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    arrayFloat32MsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                arrayFloat32MsgLastCounts.at(i) = count;
                allCount += count;
                file<<"array/float32/"<<arrayFloat32Names.at(i)<<" "<<count<<" "<<frequency<<std::endl;
                captureStatus.topics.push_back(arrayFloat32Topics.at(i));
                captureStatus.count_in_seconds.push_back(countInSecond);
                captureStatus.frequencies.push_back(frequency);
            }
            for(int i = 0; i < lidarPointCloudNames.size(); i++){
                lidarPointCloudMsgCountMtxs.at(i).lock();
                int count = lidarPointCloudMsgCounts.at(i);
                double startTime = lidarPointCloudStartTimeStamps.at(i);
                double lastTime = lidarPointCloudLastTimeStamps.at(i);
                lidarPointCloudMsgCountMtxs.at(i).unlock();
                int countInSecond = count - lidarPointCloudMsgLastCounts.at(i);
                double frequency = ((double)count / (lastTime - startTime));
                std::cout<<lidarPointCloudTopics.at(i)<<": "<<countInSecond<<" / "<<count<<"("<<frequency<<"hz)"<<std::endl;
                if(lidarPointCloudMsgLastUpHzTimes.at(i) == 0){
                    lidarPointCloudMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(rclcpp::Clock().now().seconds() - lidarPointCloudMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<lidarPointCloudTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    lidarPointCloudMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                lidarPointCloudMsgLastCounts.at(i) = count;
                allCount += count;
                file<<"lidar/pointCloud/"<<lidarPointCloudNames.at(i)<<" "<<count<<" "<<frequency<<std::endl;
                captureStatus.topics.push_back(lidarPointCloudTopics.at(i));
                captureStatus.count_in_seconds.push_back(countInSecond);
                captureStatus.frequencies.push_back(frequency);
            }
            for(int i = 0; i < robotBaseOdometryNames.size(); i++){
                robotBaseOdometryMsgCountMtxs.at(i).lock();
                int count = robotBaseOdometryMsgCounts.at(i);
                double startTime = robotBaseOdometrystartTimeStamps.at(i);
                double lastTime = robotBaseOdometryLastTimeStamps.at(i);
                robotBaseOdometryMsgCountMtxs.at(i).unlock();
                int countInSecond = count - robotBaseOdometryMsgLastCounts.at(i);
                double frequency = ((double)count / (lastTime - startTime));
                std::cout<<robotBaseOdometryTopics.at(i)<<": "<<countInSecond<<" / "<<count<<"("<<frequency<<"hz)"<<std::endl;
                if(robotBaseOdometryMsgLastUpHzTimes.at(i) == 0){
                    robotBaseOdometryMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(rclcpp::Clock().now().seconds() - robotBaseOdometryMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<robotBaseOdometryTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    robotBaseOdometryMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                robotBaseOdometryMsgLastCounts.at(i) = count;
                allCount += count;
                file<<"robotBase/odometry/"<<robotBaseOdometryNames.at(i)<<" "<<count<<" "<<frequency<<std::endl;
                captureStatus.topics.push_back(robotBaseOdometryTopics.at(i));
                captureStatus.count_in_seconds.push_back(countInSecond);
                captureStatus.frequencies.push_back(frequency);
            }
            for(int i = 0; i < robotBaseVelocityNames.size(); i++){
                robotBaseVelocityMsgCountMtxs.at(i).lock();
                int count = robotBaseVelocityMsgCounts.at(i);
                double startTime = robotBaseVelocityStartTimeStamps.at(i);
                double lastTime = robotBaseVelocityLastTimeStamps.at(i);
                robotBaseVelocityMsgCountMtxs.at(i).unlock();
                int countInSecond = count - robotBaseVelocityMsgLastCounts.at(i);
                double frequency = ((double)count / (lastTime - startTime));
                std::cout<<robotBaseVelocityTopics.at(i)<<": "<<countInSecond<<" / "<<count<<"("<<frequency<<"hz)"<<std::endl;
                if(robotBaseVelocityMsgLastUpHzTimes.at(i) == 0){
                    robotBaseVelocityMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(rclcpp::Clock().now().seconds() - robotBaseVelocityMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<robotBaseVelocityTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    robotBaseVelocityMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                robotBaseVelocityMsgLastCounts.at(i) = count;
                allCount += count;
                file<<"robotBase/velocity/"<<robotBaseVelocityNames.at(i)<<" "<<count<<" "<<frequency<<std::endl;
                captureStatus.topics.push_back(robotBaseVelocityTopics.at(i));
                captureStatus.count_in_seconds.push_back(countInSecond);
                captureStatus.frequencies.push_back(frequency);
            }
            for(int i = 0; i < liftMotorNames.size(); i++){
                liftMotorMsgCountMtxs.at(i).lock();
                int count = liftMotorMsgCounts.at(i);
                double startTime = liftMotorStartTimeStamps.at(i);
                double lastTime = liftMotorLastTimeStamps.at(i);
                liftMotorMsgCountMtxs.at(i).unlock();
                int countInSecond = count - liftMotorMsgLastCounts.at(i);
                double frequency = ((double)count / (lastTime - startTime));
                std::cout<<liftMotorTopics.at(i)<<": "<<countInSecond<<" / "<<count<<"("<<frequency<<"hz)"<<std::endl;
                if(liftMotorMsgLastUpHzTimes.at(i) == 0){
                    liftMotorMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(rclcpp::Clock().now().seconds() - liftMotorMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<liftMotorTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    liftMotorMsgLastUpHzTimes.at(i) = rclcpp::Clock().now().seconds();
                }
                liftMotorMsgLastCounts.at(i) = count;
                allCount += count;
                file<<"lift/motor/"<<liftMotorNames.at(i)<<" "<<count<<" "<<frequency<<std::endl;
                captureStatus.topics.push_back(liftMotorTopics.at(i));
                captureStatus.count_in_seconds.push_back(countInSecond);
                captureStatus.frequencies.push_back(frequency);
            }
            captureStatus.quit = false;
            pubCaptureStatus->publish(captureStatus);
            std::cout<<"sum total frame: "<<allCount<<std::endl;
            std::cout<<std::endl;

            std::cout<<"config topic: total frame"<<std::endl;
            file<<"config:"<<std::endl;
            for(int i = 0; i < cameraColorNames.size() && !cameraColorConfigTopics.empty(); i++){
                cameraColorConfigMsgCountMtxs.at(i).lock();
                int count = cameraColorConfigMsgCounts.at(i);
                cameraColorConfigMsgCountMtxs.at(i).unlock();
                std::cout<<cameraColorConfigTopics.at(i)<<": "<<count<<std::endl;
                file<<"camera/color/"<<cameraColorNames.at(i)<<" "<<count<<std::endl;
            }
            for(int i = 0; i < cameraDepthNames.size() && !cameraDepthConfigTopics.empty(); i++){
                cameraDepthConfigMsgCountMtxs.at(i).lock();
                int count = cameraDepthConfigMsgCounts.at(i);
                cameraDepthConfigMsgCountMtxs.at(i).unlock();
                std::cout<<cameraDepthConfigTopics.at(i)<<": "<<count<<std::endl;
                file<<"camera/depth/"<<cameraDepthNames.at(i)<<" "<<count<<std::endl;
            }
            for(int i = 0; i < cameraPointCloudNames.size() && !cameraPointCloudConfigTopics.empty(); i++){
                cameraPointCloudConfigMsgCountMtxs.at(i).lock();
                int count = cameraPointCloudConfigMsgCounts.at(i);
                cameraPointCloudConfigMsgCountMtxs.at(i).unlock();
                std::cout<<cameraPointCloudConfigTopics.at(i)<<": "<<count<<std::endl;
                file<<"camera/pointCloud/"<<cameraPointCloudNames.at(i)<<" "<<count<<std::endl;
            }
            std::cout<<std::endl;

            if(tfTransformParentFrames.size() != 0){
                std::cout<<"tf:"<<std::endl;
                file<<"tf:"<<std::endl;
            }
            for(int i = 0; i < tfTransformParentFrames.size(); i++){
                tfTransformMsgCountMtxs.at(i).lock();
                int count = tfTransformMsgCounts.at(i);
                tfTransformMsgCountMtxs.at(i).unlock();
                std::cout<<tfTransformParentFrames.at(i)<<"-"<<tfTransformChildFrames.at(i)<<": "<<count<<std::endl;
                file<<tfTransformParentFrames.at(i)<<" "<<tfTransformChildFrames.at(i)<<" "<<count<<std::endl;
            }
            std::cout<<std::endl;
            if(!this->useService)
                std::cout<<"press ENTER to stop capture:"<<std::endl;
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop){
                    captureStatus.quit = true;
                    pubCaptureStatus->publish(captureStatus);
                    break;
                }
            }
            if(captureStatus.fail)
                break;
            rate.sleep();
        }
        if(captureStatus.fail)
            if(this->useService){
                std::cout<<"The device frequency does not match, stop, waiting for the end signal."<<std::endl;
                shutdown();
            }
            else
                exit(0);
    }
};


class DataCaptureService: public rclcpp::Node{
    public:
    rclcpp::Service<data_msgs::srv::CaptureService>::SharedPtr srvDataCapture;
    rclcpp::executors::MultiThreadedExecutor *exec;
    std::shared_ptr<DataCapture> dataCapture;
    bool useService;
    std::string datasetDir;
    int episodeIndex;
    std::string instructions;
    rclcpp::NodeOptions options;
    std::string name;

    std::thread *spinThread;

    int hz;
    int timeout;
    double cropTime;

    void spining(){
        exec->spin();
    }

    DataCaptureService(std::string name, const rclcpp::NodeOptions & options): rclcpp::Node(name, options) {
        this->datasetDir = datasetDir;
        this->episodeIndex = episodeIndex;
        exec = nullptr;
        this->options = options;
        this->name = name;
        declare_parameter("useService", true);get_parameter("useService", useService);
        declare_parameter("datasetDir", "/home/agilex/data");get_parameter("datasetDir", datasetDir);
        declare_parameter("episodeIndex", 0);get_parameter("episodeIndex", episodeIndex);
        declare_parameter("instructions", "[null]");get_parameter("instructions", instructions);
        declare_parameter("hz", -1);get_parameter("hz", hz);
        declare_parameter("timeout", 2);get_parameter("timeout", timeout);
        declare_parameter("cropTime", -1.0);get_parameter("cropTime", cropTime);
        if(useService){
            auto captureService = [this, name, options](const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<data_msgs::srv::CaptureService::Request> req, std::shared_ptr<data_msgs::srv::CaptureService::Response> res) -> void 
            {
                (void)request_header;
                if(req->start && req->end){
                    if(exec != nullptr){
                        // res->success = false;
                        ((DataCapture *)dataCapture.get())->shutdown();
                        ((DataCapture *)dataCapture.get())->join();
                        exec->remove_node(dataCapture);
                        exec->cancel();
                        spinThread->join();
                        delete spinThread;
                        delete exec;
                        spinThread = nullptr;
                        exec = nullptr;
                        dataCapture = nullptr;
                    }else{
                        std::string datasetDir = this->datasetDir;
                        int episodeIndex = this->episodeIndex;
                        if(req->dataset_dir != ""){
                            datasetDir = req->dataset_dir;
                        }
                        if(req->episode_index != -1){
                            episodeIndex = req->episode_index;
                        }
                        rclcpp::sleep_for(std::chrono::seconds(1));
                        exec = new rclcpp::executors::MultiThreadedExecutor;
                        std::string workerName = name + "_worker_" + std::to_string(rclcpp::Clock().now().nanoseconds());
                        dataCapture = std::make_shared<DataCapture>(workerName, options, datasetDir, episodeIndex, hz, timeout, cropTime, true);
                        if(req->episode_index == -1){
                            this->episodeIndex++;
                        }
                        exec->add_node(dataCapture);
                        ((DataCapture *)dataCapture.get())->instructionSaving(instructions);
                        ((DataCapture *)dataCapture.get())->run();
                        spinThread = new std::thread(&DataCaptureService::spining, this);
                        res->success = true;
                    }
                }else{
                    if(req->start){
                        if(exec != nullptr){
                            // res->success = false;
                            ((DataCapture *)dataCapture.get())->shutdown();
                            ((DataCapture *)dataCapture.get())->join();
                            exec->remove_node(dataCapture);
                            exec->cancel();
                            spinThread->join();
                            delete spinThread;
                            delete exec;
                            spinThread = nullptr;
                            exec = nullptr;
                        }
                        // else{
                            std::string datasetDir = this->datasetDir;
                            int episodeIndex = this->episodeIndex;
                            if(req->dataset_dir != ""){
                                datasetDir = req->dataset_dir;
                            }
                            if(req->episode_index != -1){
                                episodeIndex = req->episode_index;
                            }
                            rclcpp::sleep_for(std::chrono::seconds(1));
                            exec = new rclcpp::executors::MultiThreadedExecutor;
                            std::string workerName = name + "_worker_" + std::to_string(rclcpp::Clock().now().nanoseconds());
                            dataCapture = std::make_shared<DataCapture>(workerName, options, datasetDir, episodeIndex, hz, timeout, cropTime, true);
                            if(req->episode_index == -1){
                                this->episodeIndex++;
                            }
                            exec->add_node(dataCapture);
                            ((DataCapture *)dataCapture.get())->instructionSaving(instructions);
                            ((DataCapture *)dataCapture.get())->run();
                            spinThread = new std::thread(&DataCaptureService::spining, this);
                            res->success = true;
                        // }
                    }else if(req->end){
                        if(dataCapture != nullptr){
                            ((DataCapture *)dataCapture.get())->shutdown();
                            ((DataCapture *)dataCapture.get())->join();
                            exec->remove_node(dataCapture);
                            exec->cancel();
                            spinThread->join();
                            delete spinThread;
                            delete exec;
                            spinThread = nullptr;
                            exec = nullptr;
                            res->success = true;
                            std::cout<<"wait for start signal"<<std::endl;
                        }else{
                            res->success = false;
                        }
                    }else{
                        if(req->dataset_dir != ""){
                            datasetDir = req->dataset_dir;
                        }
                        if(req->episode_index != -1){
                            episodeIndex = req->episode_index;
                        }else{
                            episodeIndex = episodeIndex - 1;
                        }
                        int unused = system((std::string("rm -rf ") + datasetDir + "/episode" + std::to_string(episodeIndex)).c_str());
                    }
                }
            };
            srvDataCapture = create_service<data_msgs::srv::CaptureService>("/data_tools_dataCapture/capture_service", captureService);
        }
        else{
            exec = new rclcpp::executors::MultiThreadedExecutor;
            std::string workerName = name + "_worker_" + std::to_string(rclcpp::Clock().now().nanoseconds());
            dataCapture = std::make_shared<DataCapture>(workerName, options, datasetDir, episodeIndex, hz, timeout);
            exec->add_node(dataCapture);
            ((DataCapture *)dataCapture.get())->instructionSaving(instructions);
            ((DataCapture *)dataCapture.get())->run();
            exec->spin();
            ((DataCapture *)dataCapture.get())->join();
            rclcpp::shutdown();
            std::cout<<"Done"<<std::endl;
        }
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> data capture Started.\033[0m");
    // rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::executors::MultiThreadedExecutor exec;
    auto dataCaptureService = std::make_shared<DataCaptureService>("data_capture", options);
    exec.add_node(dataCaptureService);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
