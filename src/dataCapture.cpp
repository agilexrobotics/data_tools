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
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <sensor_msgs/Imu.h>
#include <data_msgs/Gripper.h>
#include <data_msgs/CaptureService.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include "jsoncpp/json/json.h"
#include <data_msgs/CaptureStatus.h>
#ifdef _USELIFT
#include <bt_task_msgs/LiftMotorMsg.h>
#endif

class DataCapture: public DataUtility{
public:
    std::vector<ros::Subscriber> subCameraColors;
    std::vector<ros::Subscriber> subCameraDepths;
    std::vector<ros::Subscriber> subCameraPointClouds;
    std::vector<ros::Subscriber> subArmJointStates;
    std::vector<ros::Subscriber> subArmEndPoses;
    std::vector<ros::Subscriber> subLocalizationPoses;
    std::vector<ros::Subscriber> subGripperEncoders;
    std::vector<ros::Subscriber> subImu9Axiss;
    std::vector<ros::Subscriber> subLidarPointClouds;
    std::vector<ros::Subscriber> subRobotBaseVels;
    std::vector<ros::Subscriber> subLiftMotors;

    ros::Publisher pubCaptureStatus;

    std::vector<ros::Subscriber> subCameraColorConfigs;
    std::vector<ros::Subscriber> subCameraDepthConfigs;
    std::vector<ros::Subscriber> subCameraPointCloudConfigs;
    std::vector<ros::Subscriber> subArmJointStateConfigs;
    std::vector<ros::Subscriber> subArmEndPoseConfigs;
    std::vector<ros::Subscriber> subLocalizationPoseConfigs;
    std::vector<ros::Subscriber> subGripperEncoderConfigs;
    std::vector<ros::Subscriber> subImu9AxisConfigs;
    std::vector<ros::Subscriber> subLidarPointCloudConfigs;
    std::vector<ros::Subscriber> subRobotBaseVelConfigs;
    std::vector<ros::Subscriber> subLiftMotorConfigs;

    std::vector<BlockingDeque<sensor_msgs::Image>> cameraColorMsgDeques;
    std::vector<BlockingDeque<sensor_msgs::Image>> cameraDepthMsgDeques;
    std::vector<BlockingDeque<sensor_msgs::PointCloud2>> cameraPointCloudMsgDeques;
    std::vector<BlockingDeque<sensor_msgs::JointState>> armJointStateMsgDeques;
    std::vector<BlockingDeque<geometry_msgs::PoseStamped>> armEndPoseMsgDeques;
    std::vector<BlockingDeque<geometry_msgs::PoseStamped>> localizationPoseMsgDeques;
    std::vector<BlockingDeque<data_msgs::Gripper>> gripperEncoderMsgDeques;
    std::vector<BlockingDeque<sensor_msgs::Imu>> imu9AxisMsgDeques;
    std::vector<BlockingDeque<sensor_msgs::PointCloud2>> lidarPointCloudMsgDeques;
    std::vector<BlockingDeque<nav_msgs::Odometry>> robotBaseVelMsgDeques;
    #ifdef _USELIFT
    std::vector<BlockingDeque<bt_task_msgs::LiftMotorMsg>> liftMotorMsgDeques;
    #endif

    std::vector<std::thread*> cameraColorSavingThreads;
    std::vector<std::thread*> cameraDepthSavingThreads;
    std::vector<std::thread*> cameraPointCloudSavingThreads;
    std::vector<std::thread*> armJointStateSavingThreads;
    std::vector<std::thread*> armEndPoseSavingThreads;
    std::vector<std::thread*> localizationPoseSavingThreads;
    std::vector<std::thread*> gripperEncoderSavingThreads;
    std::vector<std::thread*> imu9AxisSavingThreads;
    std::vector<std::thread*> lidarPointCloudSavingThreads;
    std::vector<std::thread*> robotBaseVelSavingThreads;
    std::vector<std::thread*> liftMotorSavingThreads;
    std::vector<std::thread*> tfTransformSavingThreads;

    std::vector<int> cameraColorMsgCounts;
    std::vector<int> cameraDepthMsgCounts;
    std::vector<int> cameraPointCloudMsgCounts;
    std::vector<int> armJointStateMsgCounts;
    std::vector<int> armEndPoseMsgCounts;
    std::vector<int> localizationPoseMsgCounts;
    std::vector<int> gripperEncoderMsgCounts;
    std::vector<int> imu9AxisMsgCounts;
    std::vector<int> lidarPointCloudMsgCounts;
    std::vector<int> robotBaseVelMsgCounts;
    std::vector<int> liftMotorMsgCounts;

    std::vector<int> cameraColorConfigMsgCounts;
    std::vector<int> cameraDepthConfigMsgCounts;
    std::vector<int> cameraPointCloudConfigMsgCounts;
    std::vector<int> armJointStateConfigMsgCounts;
    std::vector<int> armEndPoseConfigMsgCounts;
    std::vector<int> localizationPoseConfigMsgCounts;
    std::vector<int> gripperEncoderConfigMsgCounts;
    std::vector<int> imu9AxisConfigMsgCounts;
    std::vector<int> lidarPointCloudConfigMsgCounts;
    std::vector<int> robotBaseVelConfigMsgCounts;
    std::vector<int> liftMotorConfigMsgCounts;
    std::vector<int> tfTransformMsgCounts;

    std::vector<double> cameraColorStartTimeStamps;
    std::vector<double> cameraDepthStartTimeStamps;
    std::vector<double> cameraPointCloudStartTimeStamps;
    std::vector<double> armJointStateStartTimeStamps;
    std::vector<double> armEndPoseStartTimeStamps;
    std::vector<double> localizationPoseStartTimeStamps;
    std::vector<double> gripperEncoderStartTimeStamps;
    std::vector<double> imu9AxisStartTimeStamps;
    std::vector<double> lidarPointCloudStartTimeStamps;
    std::vector<double> robotBaseVelStartTimeStamps;
    std::vector<double> liftMotorStartTimeStamps;

    std::vector<double> cameraColorLastTimeStamps;
    std::vector<double> cameraDepthLastTimeStamps;
    std::vector<double> cameraPointCloudLastTimeStamps;
    std::vector<double> armJointStateLastTimeStamps;
    std::vector<double> armEndPoseLastTimeStamps;
    std::vector<double> localizationPoseLastTimeStamps;
    std::vector<double> gripperEncoderLastTimeStamps;
    std::vector<double> imu9AxisLastTimeStamps;
    std::vector<double> lidarPointCloudLastTimeStamps;
    std::vector<double> robotBaseVelLastTimeStamps;
    std::vector<double> liftMotorLastTimeStamps;
    
    std::vector<std::mutex> cameraColorMsgCountMtxs;
    std::vector<std::mutex> cameraDepthMsgCountMtxs;
    std::vector<std::mutex> cameraPointCloudMsgCountMtxs;
    std::vector<std::mutex> armJointStateMsgCountMtxs;
    std::vector<std::mutex> armEndPoseMsgCountMtxs;
    std::vector<std::mutex> localizationPoseMsgCountMtxs;
    std::vector<std::mutex> gripperEncoderMsgCountMtxs;
    std::vector<std::mutex> imu9AxisMsgCountMtxs;
    std::vector<std::mutex> lidarPointCloudMsgCountMtxs;
    std::vector<std::mutex> robotBaseVelMsgCountMtxs;
    std::vector<std::mutex> liftMotorMsgCountMtxs;
    std::vector<std::mutex> tfTransformMsgCountMtxs;

    std::vector<std::mutex> cameraColorConfigMsgCountMtxs;
    std::vector<std::mutex> cameraDepthConfigMsgCountMtxs;
    std::vector<std::mutex> cameraPointCloudConfigMsgCountMtxs;
    std::vector<std::mutex> armJointStateConfigMsgCountMtxs;
    std::vector<std::mutex> armEndPoseConfigMsgCountMtxs;
    std::vector<std::mutex> localizationPoseConfigMsgCountMtxs;
    std::vector<std::mutex> gripperEncoderConfigMsgCountMtxs;
    std::vector<std::mutex> imu9AxisConfigMsgCountMtxs;
    std::vector<std::mutex> lidarPointCloudConfigMsgCountMtxs;
    std::vector<std::mutex> robotBaseVelConfigMsgCountMtxs;
    std::vector<std::mutex> liftMotorConfigMsgCountMtxs;

    std::vector<std::string> cameraColorFrameIds;
    std::vector<std::string> cameraDepthFrameIds;
    std::vector<std::string> cameraPointCloudFrameIds;

    std::thread* keyboardInterruptCheckingThread;
    std::thread* monitoringThread;
    bool keyboardInterrupt = false;
    bool captureStop = false;
    std::mutex captureStopMtx;

    bool useService;

    int hz;
    int timeout;
    double cropTime;

    DataCapture(std::string datasetDir, int episodeIndex, int hz, int timeout, double cropTime=-1, bool useService=false): DataUtility(datasetDir, episodeIndex) {
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
        unused = system((std::string("mkdir -p ") + gripperEncoderDir).c_str());
        unused = system((std::string("mkdir -p ") + imu9AxisDir).c_str());
        unused = system((std::string("mkdir -p ") + lidarPointCloudDir).c_str());
        unused = system((std::string("mkdir -p ") + robotBaseVelDir).c_str());
        unused = system((std::string("mkdir -p ") + liftMotorDir).c_str());
        unused = system((std::string("mkdir -p ") + tfTransformDir).c_str());

        cameraColorMsgDeques = std::vector<BlockingDeque<sensor_msgs::Image>>(cameraColorNames.size());
        cameraDepthMsgDeques = std::vector<BlockingDeque<sensor_msgs::Image>>(cameraDepthNames.size());
        cameraPointCloudMsgDeques = std::vector<BlockingDeque<sensor_msgs::PointCloud2>>(cameraPointCloudNames.size());
        armJointStateMsgDeques = std::vector<BlockingDeque<sensor_msgs::JointState>>(armJointStateNames.size());
        armEndPoseMsgDeques = std::vector<BlockingDeque<geometry_msgs::PoseStamped>>(armEndPoseNames.size());
        localizationPoseMsgDeques = std::vector<BlockingDeque<geometry_msgs::PoseStamped>>(localizationPoseNames.size());
        gripperEncoderMsgDeques = std::vector<BlockingDeque<data_msgs::Gripper>>(gripperEncoderNames.size());
        imu9AxisMsgDeques = std::vector<BlockingDeque<sensor_msgs::Imu>>(imu9AxisNames.size());
        lidarPointCloudMsgDeques = std::vector<BlockingDeque<sensor_msgs::PointCloud2>>(lidarPointCloudNames.size());
        robotBaseVelMsgDeques = std::vector<BlockingDeque<nav_msgs::Odometry>>(robotBaseVelNames.size());
        #ifdef _USELIFT
        liftMotorMsgDeques = std::vector<BlockingDeque<bt_task_msgs::LiftMotorMsg>>(liftMotorNames.size());
        #endif

        cameraColorMsgCounts = std::vector<int>(cameraColorNames.size(), 0);
        cameraDepthMsgCounts = std::vector<int>(cameraDepthNames.size(), 0);
        cameraPointCloudMsgCounts = std::vector<int>(cameraPointCloudNames.size(), 0);
        armJointStateMsgCounts = std::vector<int>(armJointStateNames.size(), 0);
        armEndPoseMsgCounts = std::vector<int>(armEndPoseNames.size(), 0);
        localizationPoseMsgCounts = std::vector<int>(localizationPoseNames.size(), 0);
        gripperEncoderMsgCounts = std::vector<int>(gripperEncoderNames.size(), 0);
        imu9AxisMsgCounts = std::vector<int>(imu9AxisNames.size(), 0);
        lidarPointCloudMsgCounts = std::vector<int>(lidarPointCloudNames.size(), 0);
        robotBaseVelMsgCounts = std::vector<int>(robotBaseVelNames.size(), 0);
        liftMotorMsgCounts = std::vector<int>(liftMotorNames.size(), 0);

        cameraColorStartTimeStamps = std::vector<double>(cameraColorNames.size(), 0);
        cameraDepthStartTimeStamps = std::vector<double>(cameraDepthNames.size(), 0);
        cameraPointCloudStartTimeStamps = std::vector<double>(cameraPointCloudNames.size(), 0);
        armJointStateStartTimeStamps = std::vector<double>(armJointStateNames.size(), 0);
        armEndPoseStartTimeStamps = std::vector<double>(armEndPoseNames.size(), 0);
        localizationPoseStartTimeStamps = std::vector<double>(localizationPoseNames.size(), 0);
        gripperEncoderStartTimeStamps = std::vector<double>(gripperEncoderNames.size(), 0);
        imu9AxisStartTimeStamps = std::vector<double>(imu9AxisNames.size(), 0);
        lidarPointCloudStartTimeStamps = std::vector<double>(lidarPointCloudNames.size(), 0);
        robotBaseVelStartTimeStamps = std::vector<double>(robotBaseVelNames.size(), 0);
        liftMotorStartTimeStamps = std::vector<double>(liftMotorNames.size(), 0);

        cameraColorLastTimeStamps = std::vector<double>(cameraColorNames.size(), 0);
        cameraDepthLastTimeStamps = std::vector<double>(cameraDepthNames.size(), 0);
        cameraPointCloudLastTimeStamps = std::vector<double>(cameraPointCloudNames.size(), 0);
        armJointStateLastTimeStamps = std::vector<double>(armJointStateNames.size(), 0);
        armEndPoseLastTimeStamps = std::vector<double>(armEndPoseNames.size(), 0);
        localizationPoseLastTimeStamps = std::vector<double>(localizationPoseNames.size(), 0);
        gripperEncoderLastTimeStamps = std::vector<double>(gripperEncoderNames.size(), 0);
        imu9AxisLastTimeStamps = std::vector<double>(imu9AxisNames.size(), 0);
        lidarPointCloudLastTimeStamps = std::vector<double>(lidarPointCloudNames.size(), 0);
        robotBaseVelLastTimeStamps = std::vector<double>(robotBaseVelNames.size(), 0);
        liftMotorLastTimeStamps = std::vector<double>(liftMotorNames.size(), 0);

        cameraColorConfigMsgCounts = std::vector<int>(cameraColorNames.size(), 0);
        cameraDepthConfigMsgCounts = std::vector<int>(cameraDepthNames.size(), 0);
        cameraPointCloudConfigMsgCounts = std::vector<int>(cameraPointCloudNames.size(), 0);
        armJointStateConfigMsgCounts = std::vector<int>(armJointStateNames.size(), 0);
        armEndPoseConfigMsgCounts = std::vector<int>(armEndPoseNames.size(), 0);
        localizationPoseConfigMsgCounts = std::vector<int>(localizationPoseNames.size(), 0);
        gripperEncoderConfigMsgCounts = std::vector<int>(gripperEncoderNames.size(), 0);
        imu9AxisConfigMsgCounts = std::vector<int>(imu9AxisNames.size(), 0);
        lidarPointCloudConfigMsgCounts = std::vector<int>(lidarPointCloudNames.size(), 0);
        robotBaseVelConfigMsgCounts = std::vector<int>(robotBaseVelNames.size(), 0);
        liftMotorConfigMsgCounts = std::vector<int>(liftMotorNames.size(), 0);
        tfTransformMsgCounts = std::vector<int>(tfTransformParentFrames.size(), 0);

        cameraColorMsgCountMtxs = std::vector<std::mutex>(cameraColorNames.size());
        cameraDepthMsgCountMtxs = std::vector<std::mutex>(cameraDepthNames.size());
        cameraPointCloudMsgCountMtxs = std::vector<std::mutex>(cameraPointCloudNames.size());
        armJointStateMsgCountMtxs = std::vector<std::mutex>(armJointStateNames.size());
        armEndPoseMsgCountMtxs = std::vector<std::mutex>(armEndPoseNames.size());
        localizationPoseMsgCountMtxs = std::vector<std::mutex>(localizationPoseNames.size());
        gripperEncoderMsgCountMtxs = std::vector<std::mutex>(gripperEncoderNames.size());
        imu9AxisMsgCountMtxs = std::vector<std::mutex>(imu9AxisNames.size());
        lidarPointCloudMsgCountMtxs = std::vector<std::mutex>(lidarPointCloudNames.size());
        robotBaseVelMsgCountMtxs = std::vector<std::mutex>(robotBaseVelNames.size());
        liftMotorMsgCountMtxs = std::vector<std::mutex>(liftMotorNames.size());
        tfTransformMsgCountMtxs = std::vector<std::mutex>(tfTransformParentFrames.size());

        cameraColorConfigMsgCountMtxs = std::vector<std::mutex>(cameraColorNames.size());
        cameraDepthConfigMsgCountMtxs = std::vector<std::mutex>(cameraDepthNames.size());
        cameraPointCloudConfigMsgCountMtxs = std::vector<std::mutex>(cameraPointCloudNames.size());
        armJointStateConfigMsgCountMtxs = std::vector<std::mutex>(armJointStateNames.size());
        armEndPoseConfigMsgCountMtxs = std::vector<std::mutex>(armEndPoseNames.size());
        localizationPoseConfigMsgCountMtxs = std::vector<std::mutex>(localizationPoseNames.size());
        gripperEncoderConfigMsgCountMtxs = std::vector<std::mutex>(gripperEncoderNames.size());
        imu9AxisConfigMsgCountMtxs = std::vector<std::mutex>(imu9AxisNames.size());
        lidarPointCloudConfigMsgCountMtxs = std::vector<std::mutex>(lidarPointCloudNames.size());
        robotBaseVelConfigMsgCountMtxs = std::vector<std::mutex>(robotBaseVelNames.size());
        liftMotorConfigMsgCountMtxs = std::vector<std::mutex>(liftMotorNames.size());

        cameraColorFrameIds = std::vector<std::string>(cameraColorNames.size(), "");
        cameraDepthFrameIds = std::vector<std::string>(cameraDepthNames.size(), "");
        cameraPointCloudFrameIds = std::vector<std::string>(cameraPointCloudNames.size(), "");

        for(int i = 0; i < cameraColorNames.size(); i++){
            unused = system((std::string("mkdir -p ") + cameraColorDirs.at(i)).c_str());
            subCameraColors.push_back(nh.subscribe<sensor_msgs::Image>(cameraColorTopics[i], 2000, boost::bind(&DataCapture::cameraColorHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
            subCameraColorConfigs.push_back(nh.subscribe<sensor_msgs::CameraInfo>(cameraColorConfigTopics[i], 2000, boost::bind(&DataCapture::cameraColorConfigHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
        }
        for(int i = 0; i < cameraDepthNames.size(); i++){
            unused = system((std::string("mkdir -p ") + cameraDepthDirs.at(i)).c_str());
            subCameraDepths.push_back(nh.subscribe<sensor_msgs::Image>(cameraDepthTopics[i], 2000, boost::bind(&DataCapture::cameraDepthHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
            subCameraDepthConfigs.push_back(nh.subscribe<sensor_msgs::CameraInfo>(cameraDepthConfigTopics[i], 2000, boost::bind(&DataCapture::cameraDepthConfigHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
        }
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            unused = system((std::string("mkdir -p ") + cameraPointCloudDirs.at(i)).c_str());
            subCameraPointClouds.push_back(nh.subscribe<sensor_msgs::PointCloud2>(cameraPointCloudTopics[i], 2000, boost::bind(&DataCapture::cameraPointCloudHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
            subCameraPointCloudConfigs.push_back(nh.subscribe<sensor_msgs::CameraInfo>(cameraPointCloudConfigTopics[i], 2000, boost::bind(&DataCapture::cameraPointCloudConfigHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
        }
        for(int i = 0; i < armJointStateNames.size(); i++){
            unused = system((std::string("mkdir -p ") + armJointStateDirs.at(i)).c_str());
            subArmJointStates.push_back(nh.subscribe<sensor_msgs::JointState>(armJointStateTopics[i], 2000, boost::bind(&DataCapture::armJointStateHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
        }
        for(int i = 0; i < armEndPoseNames.size(); i++){
            unused = system((std::string("mkdir -p ") + armEndPoseDirs.at(i)).c_str());
            subArmEndPoses.push_back(nh.subscribe<geometry_msgs::PoseStamped>(armEndPoseTopics[i], 2000, boost::bind(&DataCapture::armEndPoseHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
        }
        for(int i = 0; i < localizationPoseNames.size(); i++){
            unused = system((std::string("mkdir -p ") + localizationPoseDirs.at(i)).c_str());
            subLocalizationPoses.push_back(nh.subscribe<geometry_msgs::PoseStamped>(localizationPoseTopics[i], 2000, boost::bind(&DataCapture::localizationPoseHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
        }
        for(int i = 0; i < gripperEncoderNames.size(); i++){
            unused = system((std::string("mkdir -p ") + gripperEncoderDirs.at(i)).c_str());
            subGripperEncoders.push_back(nh.subscribe<data_msgs::Gripper>(gripperEncoderTopics[i], 2000, boost::bind(&DataCapture::gripperEncoderHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
        }
        for(int i = 0; i < imu9AxisNames.size(); i++){
            unused = system((std::string("mkdir -p ") + imu9AxisDirs.at(i)).c_str());
            subImu9Axiss.push_back(nh.subscribe<sensor_msgs::Imu>(imu9AxisTopics[i], 2000, boost::bind(&DataCapture::imu9AxisHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
        }
        for(int i = 0; i < lidarPointCloudNames.size(); i++){
            unused = system((std::string("mkdir -p ") + lidarPointCloudDirs.at(i)).c_str());
            subLidarPointClouds.push_back(nh.subscribe<sensor_msgs::PointCloud2>(lidarPointCloudTopics[i], 2000, boost::bind(&DataCapture::lidarPointCloudHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
        }
        for(int i = 0; i < robotBaseVelNames.size(); i++){
            unused = system((std::string("mkdir -p ") + robotBaseVelDirs.at(i)).c_str());
            subRobotBaseVels.push_back(nh.subscribe<nav_msgs::Odometry>(robotBaseVelTopics[i], 2000, boost::bind(&DataCapture::robotBaseVelHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
        }
        #ifdef _USELIFT
        for(int i = 0; i < liftMotorNames.size(); i++){
            unused = system((std::string("mkdir -p ") + liftMotorDirs.at(i)).c_str());
            subLiftMotors.push_back(nh.subscribe<bt_task_msgs::LiftMotorMsg>(liftMotorTopics[i], 2000, boost::bind(&DataCapture::liftMotorHandler, this, _1, i), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
        }
        #endif

        pubCaptureStatus = nh.advertise<data_msgs::CaptureStatus>("/data_tools_dataCapture/status", 2000);
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
        for(int i = 0; i < gripperEncoderNames.size(); i++){
            gripperEncoderSavingThreads.push_back(new std::thread(&DataCapture::gripperEncoderSaving, this, i));
        }
        for(int i = 0; i < imu9AxisNames.size(); i++){
            imu9AxisSavingThreads.push_back(new std::thread(&DataCapture::imu9AxisSaving, this, i));
        }
        for(int i = 0; i < lidarPointCloudNames.size(); i++){
            lidarPointCloudSavingThreads.push_back(new std::thread(&DataCapture::lidarPointCloudSaving, this, i));
        }
        for(int i = 0; i < robotBaseVelNames.size(); i++){
            robotBaseVelSavingThreads.push_back(new std::thread(&DataCapture::robotBaseVelSaving, this, i));
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
        for(int i = 0; i < lidarPointCloudNames.size(); i++){
            lidarPointCloudSavingThreads.at(i)->join();
            delete lidarPointCloudSavingThreads.at(i);
            lidarPointCloudSavingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < robotBaseVelNames.size(); i++){
            robotBaseVelSavingThreads.at(i)->join();
            delete robotBaseVelSavingThreads.at(i);
            robotBaseVelSavingThreads.at(i) = nullptr;
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
        data_msgs::CaptureStatus captureStatus;
        captureStatus.quit = true;
        pubCaptureStatus.publish(captureStatus);
    }

    void cameraColorHandler(const sensor_msgs::Image::ConstPtr& msg, const int& index){
        cameraColorMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(cameraColorMsgCountMtxs.at(index));
        if(cameraColorMsgCounts.at(index) == 0){
            cameraColorStartTimeStamps.at(index) = msg->header.stamp.toSec();
        }else{
            cameraColorLastTimeStamps.at(index) = msg->header.stamp.toSec();
        }
        cameraColorMsgCounts.at(index) += 1;
        cameraColorFrameIds.at(index) = msg->header.frame_id;
    }

    void cameraDepthHandler(const sensor_msgs::Image::ConstPtr& msg, const int& index){
        cameraDepthMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(cameraDepthMsgCountMtxs.at(index));
        if(cameraDepthMsgCounts.at(index) == 0){
            cameraDepthStartTimeStamps.at(index) = msg->header.stamp.toSec();
        }else{
            cameraDepthLastTimeStamps.at(index) = msg->header.stamp.toSec();
        }
        cameraDepthMsgCounts.at(index) += 1;
        cameraDepthFrameIds.at(index) = msg->header.frame_id;
    }

    void cameraPointCloudHandler(const sensor_msgs::PointCloud2::ConstPtr& msg, const int& index){
        cameraPointCloudMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(cameraPointCloudMsgCountMtxs.at(index));
        if(cameraPointCloudMsgCounts.at(index) == 0){
            cameraPointCloudStartTimeStamps.at(index) = msg->header.stamp.toSec();
        }else{
            cameraPointCloudLastTimeStamps.at(index) = msg->header.stamp.toSec();
        }
        cameraPointCloudMsgCounts.at(index) += 1;
        cameraPointCloudFrameIds.at(index) = msg->header.frame_id;
    }

    void cameraColorConfigHandler(const sensor_msgs::CameraInfo::ConstPtr& msg, const int& index){
        if(cameraColorFrameIds.at(index) == "")
            return;
        tf::TransformListener listener;
		tf::StampedTransform transform;
		while(cameraColorFrameIds.at(index) != ""){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop)
                    break;
            }
            try {
                listener.waitForTransform(cameraColorParentFrames.at(index), cameraColorFrameIds.at(index), ros::Time(0), ros::Duration(3.0));
                listener.lookupTransform(cameraColorParentFrames.at(index), cameraColorFrameIds.at(index), ros::Time(0), transform);
                break;
            } catch(tf::TransformException &ex) {
                ros::Duration(1.0).sleep();
                continue;
            }
		}
		double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();
        double z = transform.getOrigin().z();
        double roll, pitch, yaw;
        tf::Quaternion q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        Json::Value root;
        root["height"] = msg->height;
        root["width"] = msg->width;
        root["distortion_model"] = msg->distortion_model;
        Json::Value D(Json::arrayValue);
        Json::Value K(Json::arrayValue);
        Json::Value R(Json::arrayValue);
        Json::Value P(Json::arrayValue);
        for(int i = 0; i < msg->D.size(); i++){
            D.append(msg->D.at(i));
        }
        for(int i = 0; i < msg->K.size(); i++){
            K.append(msg->K.at(i));
        }
        for(int i = 0; i < msg->R.size(); i++){
            R.append(msg->R.at(i));
        }
        for(int i = 0; i < msg->P.size(); i++){
            P.append(msg->P.at(i));
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
        subCameraColorConfigs.at(index).shutdown();
    }

    void cameraDepthConfigHandler(const sensor_msgs::CameraInfo::ConstPtr& msg, const int& index){
        if(cameraDepthFrameIds.at(index) == "")
            return;
        tf::TransformListener listener;
		tf::StampedTransform transform;
		while(cameraDepthFrameIds.at(index) != ""){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop)
                    break;
            }
            try {
                listener.waitForTransform(cameraDepthParentFrames.at(index), cameraDepthFrameIds.at(index), ros::Time(0), ros::Duration(3.0));
                listener.lookupTransform(cameraDepthParentFrames.at(index), cameraDepthFrameIds.at(index), ros::Time(0), transform);
                break;
            } catch (tf::TransformException &ex) {
                ros::Duration(1.0).sleep();
                continue;
            }
		}
		double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();
        double z = transform.getOrigin().z();
        double roll, pitch, yaw;
        tf::Quaternion q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        Json::Value root;
        root["height"] = msg->height;
        root["width"] = msg->width;
        root["distortion_model"] = msg->distortion_model;
        Json::Value D(Json::arrayValue);
        Json::Value K(Json::arrayValue);
        Json::Value R(Json::arrayValue);
        Json::Value P(Json::arrayValue);
        for(int i = 0; i < msg->D.size(); i++){
            D.append(msg->D.at(i));
        }
        for(int i = 0; i < msg->K.size(); i++){
            K.append(msg->K.at(i));
        }
        for(int i = 0; i < msg->R.size(); i++){
            R.append(msg->R.at(i));
        }
        for(int i = 0; i < msg->P.size(); i++){
            P.append(msg->P.at(i));
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
        subCameraDepthConfigs.at(index).shutdown();
    }

    void cameraPointCloudConfigHandler(const sensor_msgs::CameraInfo::ConstPtr& msg, const int& index){
        if(cameraPointCloudFrameIds.at(index) == "")
            return;
        tf::TransformListener listener;
		tf::StampedTransform transform;
		while(cameraPointCloudFrameIds.at(index) != ""){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop)
                    break;
            }
            try {
                listener.waitForTransform(cameraPointCloudParentFrames.at(index), cameraPointCloudFrameIds.at(index), ros::Time(0), ros::Duration(3.0));
                listener.lookupTransform(cameraPointCloudParentFrames.at(index), cameraPointCloudFrameIds.at(index), ros::Time(0), transform);
                break;
            } catch (tf::TransformException &ex) {
                ros::Duration(1.0).sleep();
                continue;
            }
		}
		double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();
        double z = transform.getOrigin().z();
        double roll, pitch, yaw;
        tf::Quaternion q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        Json::Value root;
        root["height"] = msg->height;
        root["width"] = msg->width;
        root["distortion_model"] = msg->distortion_model;
        Json::Value D(Json::arrayValue);
        Json::Value K(Json::arrayValue);
        Json::Value R(Json::arrayValue);
        Json::Value P(Json::arrayValue);
        for(int i = 0; i < msg->D.size(); i++){
            D.append(msg->D.at(i));
        }
        for(int i = 0; i < msg->K.size(); i++){
            K.append(msg->K.at(i));
        }
        for(int i = 0; i < msg->R.size(); i++){
            R.append(msg->R.at(i));
        }
        for(int i = 0; i < msg->P.size(); i++){
            P.append(msg->P.at(i));
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
        subCameraPointCloudConfigs.at(index).shutdown();
    }

    void armJointStateHandler(const sensor_msgs::JointState::ConstPtr& msg, const int& index){
        armJointStateMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(armJointStateMsgCountMtxs.at(index));
        if(armJointStateMsgCounts.at(index) == 0){
            armJointStateStartTimeStamps.at(index) = msg->header.stamp.toSec();
        }else{
            armJointStateLastTimeStamps.at(index) = msg->header.stamp.toSec();
        }
        armJointStateMsgCounts.at(index) += 1;
    }

    void armEndPoseHandler(const geometry_msgs::PoseStamped::ConstPtr& msg, const int& index){
        armEndPoseMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(armEndPoseMsgCountMtxs.at(index));
        if(armEndPoseMsgCounts.at(index) == 0){
            armEndPoseStartTimeStamps.at(index) = msg->header.stamp.toSec();
        }else{
            armEndPoseLastTimeStamps.at(index) = msg->header.stamp.toSec();
        }
        armEndPoseMsgCounts.at(index) += 1;
    }

    void localizationPoseHandler(const geometry_msgs::PoseStamped::ConstPtr& msg, const int& index){
        localizationPoseMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(localizationPoseMsgCountMtxs.at(index));
        if(localizationPoseMsgCounts.at(index) == 0){
            localizationPoseStartTimeStamps.at(index) = msg->header.stamp.toSec();
        }else{
            localizationPoseLastTimeStamps.at(index) = msg->header.stamp.toSec();
        }
        localizationPoseMsgCounts.at(index) += 1;
    }

    void gripperEncoderHandler(const data_msgs::Gripper::ConstPtr& msg, const int& index){
        gripperEncoderMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(gripperEncoderMsgCountMtxs.at(index));
        if(gripperEncoderMsgCounts.at(index) == 0){
            gripperEncoderStartTimeStamps.at(index) = msg->header.stamp.toSec();
        }else{
            gripperEncoderLastTimeStamps.at(index) = msg->header.stamp.toSec();
        }
        gripperEncoderMsgCounts.at(index) += 1;
    }

    void imu9AxisHandler(const sensor_msgs::Imu::ConstPtr& msg, const int& index){
        imu9AxisMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(imu9AxisMsgCountMtxs.at(index));
        if(imu9AxisMsgCounts.at(index) == 0){
            imu9AxisStartTimeStamps.at(index) = msg->header.stamp.toSec();
        }else{
            imu9AxisLastTimeStamps.at(index) = msg->header.stamp.toSec();
        }
        imu9AxisMsgCounts.at(index) += 1;
    }

    void lidarPointCloudHandler(const sensor_msgs::PointCloud2::ConstPtr& msg, const int& index){
        lidarPointCloudMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(lidarPointCloudMsgCountMtxs.at(index));
        if(lidarPointCloudMsgCounts.at(index) == 0){
            lidarPointCloudStartTimeStamps.at(index) = msg->header.stamp.toSec();
        }else{
            lidarPointCloudLastTimeStamps.at(index) = msg->header.stamp.toSec();
        }
        lidarPointCloudMsgCounts.at(index) += 1;
    }

    void robotBaseVelHandler(const nav_msgs::Odometry::ConstPtr& msg, const int& index){
        robotBaseVelMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(robotBaseVelMsgCountMtxs.at(index));
        if(robotBaseVelMsgCounts.at(index) == 0){
            robotBaseVelStartTimeStamps.at(index) = msg->header.stamp.toSec();
        }else{
            robotBaseVelLastTimeStamps.at(index) = msg->header.stamp.toSec();
        }
        robotBaseVelMsgCounts.at(index) += 1;
    }

    #ifdef _USELIFT
    void liftMotorHandler(const bt_task_msgs::LiftMotorMsg::ConstPtr& msg, const int& index){
        liftMotorMsgDeques.at(index).push_back(*msg);
        std::lock_guard<std::mutex> lock(liftMotorMsgCountMtxs.at(index));
        if(liftMotorMsgCounts.at(index) == 0){
            liftMotorStartTimeStamps.at(index) = msg->header.stamp.toSec();
        }else{
            liftMotorLastTimeStamps.at(index) = msg->header.stamp.toSec();
        }
        liftMotorMsgCounts.at(index) += 1;
    }
    #endif

    void cameraColorSaving(const int index){
        ros::Rate rate = ros::Rate(100);
        bool quit = false;
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && cameraColorMsgDeques.at(index).size() == 0)
                    break;
            }
            if(cameraColorMsgDeques.at(index).back().header.stamp.toSec() == 0){
                quit = true;
                cameraColorMsgDeques.at(index).pop_back();
            }
            if(quit && (cameraColorMsgDeques.at(index).size() == 0 || cameraColorMsgDeques.at(index).back().header.stamp.toSec() - cameraColorMsgDeques.at(index).front().header.stamp.toSec() <= cropTime))
                break;
            if(quit || cameraColorMsgDeques.at(index).back().header.stamp.toSec() - cameraColorMsgDeques.at(index).front().header.stamp.toSec() > cropTime){
                sensor_msgs::Image msg = cameraColorMsgDeques.at(index).pop_front();
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                cv::imwrite(cameraColorDirs.at(index) + "/" + std::to_string(msg.header.stamp.toSec()) + ".jpg", cv_ptr->image);
            }else{
                rate.sleep();
            }
        }
    }

    void cameraDepthSaving(const int index){
        ros::Rate rate = ros::Rate(100);
        bool quit = false;
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && cameraDepthMsgDeques.at(index).size() == 0)
                    break;
            }
            if(cameraDepthMsgDeques.at(index).back().header.stamp.toSec() == 0){
                quit = true;
                cameraDepthMsgDeques.at(index).pop_back();
            }
            if(quit && (cameraDepthMsgDeques.at(index).size() == 0 || cameraDepthMsgDeques.at(index).back().header.stamp.toSec() - cameraDepthMsgDeques.at(index).front().header.stamp.toSec() <= cropTime))
                break;
            if(quit || cameraDepthMsgDeques.at(index).back().header.stamp.toSec() - cameraDepthMsgDeques.at(index).front().header.stamp.toSec() > cropTime){
                sensor_msgs::Image msg = cameraDepthMsgDeques.at(index).pop_front();
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
                cv::imwrite(cameraDepthDirs.at(index) + "/" + std::to_string(msg.header.stamp.toSec()) + ".png", cv_ptr->image);
            }else{
                rate.sleep();
            }
        }
    }

    void cameraPointCloudSaving(const int index){
        ros::Rate rate = ros::Rate(100);
        bool quit = false;
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && cameraPointCloudMsgDeques.at(index).size() == 0)
                    break;
            }
            if(cameraPointCloudMsgDeques.at(index).back().header.stamp.toSec() == 0){
                quit = true;
                cameraPointCloudMsgDeques.at(index).pop_back();
            }
            if(quit && (cameraPointCloudMsgDeques.at(index).size() == 0 || cameraPointCloudMsgDeques.at(index).back().header.stamp.toSec() - cameraPointCloudMsgDeques.at(index).front().header.stamp.toSec() <= cropTime))
                    break;
            if(quit || cameraPointCloudMsgDeques.at(index).back().header.stamp.toSec() - cameraPointCloudMsgDeques.at(index).front().header.stamp.toSec() > cropTime){
                sensor_msgs::PointCloud2 msg = cameraPointCloudMsgDeques.at(index).pop_front();
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

                // auto start = std::chrono::high_resolution_clock::now();

                // std::stringstream compressedData;
                // pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* compression = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(
                //     pcl::io::MANUAL_CONFIGURATION,			// 定义压缩配置文件
                //     false,									// 是否打印压缩统计信息
                //     0.001,									// 点坐标的精度
                //     0.001,									// 最低八叉树级别的八叉树分辨率，即八叉树最低层级的体素块的边长
                //     false,									// 点云压缩前，是否进行体素网格过滤
                //     100,									// 点云压缩模式对点云进行差分编码压缩，用这种方法对新引入的点云和之前编码的点云之间的差分进行编码，以便获得最大压缩性能，iFrameRate_arg指定了一个速率，如果数据流中的帧速率低于这个速率则不进行差分编码压缩
                //     true,									// 是否对颜色编码进行压缩
                //     8										// 每一个颜色编码所占的位数
                // );
                // compression->encodePointCloud(pointCloudDownSize, compressedData);
                // std::string dir = cameraPointCloudDirs.at(index) + "/" + std::to_string(msg.header.stamp.toSec()) + ".compressed.bin";
                // compressedData.write(dir.c_str(), sizeof(compressedData));								// 将压缩点云的文件结果写到bin文件中
                // std::ofstream file(cameraPointCloudDirs.at(index) + "/" + std::to_string(msg.header.stamp.toSec()) + ".bin");
                // if (file.is_open()) {
                //     // 将字符串写入文件
                //     file << compressedData.str();
                //     file.close(); // 关闭文件流
                // } else {
                //     std::cerr << "Unable to open file";
                // }

                // auto end = std::chrono::high_resolution_clock::now();

                // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

                // std::cout << "函数运行时间: " << duration << " 微秒" << std::endl;

                pcl::io::savePCDFileBinary(cameraPointCloudDirs.at(index) + "/" + std::to_string(msg.header.stamp.toSec()) + ".pcd", *pointCloudDownSize);
            }else{
                rate.sleep();
            }
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
            sensor_msgs::JointState msg = armJointStateMsgDeques.at(index).pop_front();
            if(msg.header.stamp.toSec() == 0)
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
            std::ofstream file(armJointStateDirs.at(index) + "/" + std::to_string(msg.header.stamp.toSec()) + ".json");
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
            geometry_msgs::PoseStamped msg = armEndPoseMsgDeques.at(index).pop_front();
            if(msg.header.stamp.toSec() == 0)
                break;
            Json::Value root;
            if(armEndPoseOrients.at(index)){
                root["x"] = msg.pose.position.x;
                root["y"] = msg.pose.position.y;
                root["z"] = msg.pose.position.z;
                tf::Quaternion quat;
                tf::quaternionMsgToTF(msg.pose.orientation, quat);
                double roll, pitch, yaw;
                tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
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
            std::ofstream file(armEndPoseDirs.at(index) + "/" + std::to_string(msg.header.stamp.toSec()) + ".json");
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
            geometry_msgs::PoseStamped msg = localizationPoseMsgDeques.at(index).pop_front();
            if(msg.header.stamp.toSec() == 0)
                break;
            Json::Value root;
            root["x"] = msg.pose.position.x;
            root["y"] = msg.pose.position.y;
            root["z"] = msg.pose.position.z;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(msg.pose.orientation, quat);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            root["roll"] = roll;
            root["pitch"] = pitch;
            root["yaw"] = yaw;
            Json::StyledStreamWriter streamWriter;
            std::ofstream file(localizationPoseDirs.at(index) + "/" + std::to_string(msg.header.stamp.toSec()) + ".json");
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
            data_msgs::Gripper msg = gripperEncoderMsgDeques.at(index).pop_front();
            if(msg.header.stamp.toSec() == 0)
                break;
            Json::Value root;
            root["angle"] = msg.angle;
            root["distance"] = msg.distance;
            Json::StyledStreamWriter streamWriter;
            std::ofstream file(gripperEncoderDirs.at(index) + "/" + std::to_string(msg.header.stamp.toSec()) + ".json");
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
            sensor_msgs::Imu msg = imu9AxisMsgDeques.at(index).pop_front();
            if(msg.header.stamp.toSec() == 0)
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
            std::ofstream file(imu9AxisDirs.at(index) + "/" + std::to_string(msg.header.stamp.toSec()) + ".json");
            streamWriter.write(file, root);
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
            sensor_msgs::PointCloud2 msg = lidarPointCloudMsgDeques.at(index).pop_front();
            if(msg.header.stamp.toSec() == 0)
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

            pcl::io::savePCDFileBinary(lidarPointCloudDirs.at(index) + "/" + std::to_string(msg.header.stamp.toSec()) + ".pcd", *pointCloudDownSize);
        }
    }

    void robotBaseVelSaving(const int index){
        while(true){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop && robotBaseVelMsgDeques.at(index).size() == 0)
                    break;
            }
            nav_msgs::Odometry msg = robotBaseVelMsgDeques.at(index).pop_front();
            if(msg.header.stamp.toSec() == 0)
                break;
            Json::Value root;
            root["linear"]["x"] = msg.twist.twist.linear.x;
            root["linear"]["y"] = msg.twist.twist.linear.y;
            root["angular"]["z"] = msg.twist.twist.angular.z;
            Json::StyledStreamWriter streamWriter;
            std::ofstream file(robotBaseVelDirs.at(index) + "/" + std::to_string(msg.header.stamp.toSec()) + ".json");
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
            bt_task_msgs::LiftMotorMsg msg = liftMotorMsgDeques.at(index).pop_front();
            if(msg.header.stamp.toSec() == 0)
                break;
            Json::Value root;
            root["backHeight"] = msg.backHeight;
            Json::StyledStreamWriter streamWriter;
            std::ofstream file(liftMotorDirs.at(index) + "/" + std::to_string(msg.header.stamp.toSec()) + ".json");
            streamWriter.write(file, root);
            file.close();
        }
    }
    #endif

    void tfTransformSaving(const int index){
        tf::TransformListener listener;
		tf::StampedTransform transform;
        bool getFrame = false;
		while(ros::ok()){
            if(captureStopMtx.try_lock()){
                bool stop = captureStop;
                captureStopMtx.unlock();
                if(stop)
                    break;
            }
            try {
                listener.waitForTransform(tfTransformParentFrames.at(index), tfTransformChildFrames.at(index), ros::Time(0), ros::Duration(3.0));
                listener.lookupTransform(tfTransformParentFrames.at(index), tfTransformChildFrames.at(index), ros::Time(0), transform);
                getFrame = true;
                break;
            } catch (tf::TransformException &ex) {
                ros::Duration(1.0).sleep();
                continue;
            }
		}
        if(getFrame){
            double x = transform.getOrigin().x();
            double y = transform.getOrigin().y();
            double z = transform.getOrigin().z();
            double roll, pitch, yaw;
            tf::Quaternion q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
            tf::Matrix3x3 m(q);
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
        if (instructions.front() != '[' || instructions.back() != ']') {
             std::cout << "Error parsing JSON: " << instructions << std::endl;
            return;
        }
        std::string content = instructions.substr(1, instructions.size() - 2);
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

        root["instructions"] = result;
        Json::StyledStreamWriter streamWriter;
        std::ofstream file(instructionsDir);
        streamWriter.write(file, root);
        file.close();
    }

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
            subCameraColors.at(i).shutdown();
            sensor_msgs::Image msg;
            msg.header.stamp = ros::Time().fromSec(0);
            cameraColorMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < cameraDepthNames.size(); i++){
            subCameraDepths.at(i).shutdown();
            sensor_msgs::Image msg;
            msg.header.stamp = ros::Time().fromSec(0);
            cameraDepthMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            subCameraPointClouds.at(i).shutdown();
            sensor_msgs::PointCloud2 msg;
            msg.header.stamp = ros::Time().fromSec(0);
            cameraPointCloudMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < armJointStateNames.size(); i++){
            subArmJointStates.at(i).shutdown();
            sensor_msgs::JointState msg;
            msg.header.stamp = ros::Time().fromSec(0);
            armJointStateMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < armEndPoseNames.size(); i++){
            subArmEndPoses.at(i).shutdown();
            geometry_msgs::PoseStamped msg;
            msg.header.stamp = ros::Time().fromSec(0);
            armEndPoseMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < localizationPoseNames.size(); i++){
            subLocalizationPoses.at(i).shutdown();
            geometry_msgs::PoseStamped msg;
            msg.header.stamp = ros::Time().fromSec(0);
            localizationPoseMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < gripperEncoderNames.size(); i++){
            subGripperEncoders.at(i).shutdown();
            data_msgs::Gripper msg;
            msg.header.stamp = ros::Time().fromSec(0);
            gripperEncoderMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < imu9AxisNames.size(); i++){
            subImu9Axiss.at(i).shutdown();
            sensor_msgs::Imu msg;
            msg.header.stamp = ros::Time().fromSec(0);
            imu9AxisMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < lidarPointCloudNames.size(); i++){
            subLidarPointClouds.at(i).shutdown();
            sensor_msgs::PointCloud2 msg;
            msg.header.stamp = ros::Time().fromSec(0);
            lidarPointCloudMsgDeques.at(i).push_back(msg);
        }
        for(int i = 0; i < robotBaseVelNames.size(); i++){
            subRobotBaseVels.at(i).shutdown();
            nav_msgs::Odometry msg;
            msg.header.stamp = ros::Time().fromSec(0);
            robotBaseVelMsgDeques.at(i).push_back(msg);
        }
        #ifdef _USELIFT
        for(int i = 0; i < liftMotorNames.size(); i++){
            subLiftMotors.at(i).shutdown();
            bt_task_msgs::LiftMotorMsg msg;
            msg.header.stamp = ros::Time().fromSec(0);
            liftMotorMsgDeques.at(i).push_back(msg);
        }
        #endif
        if(!this->useService)
            ros::shutdown();
    }

    void monitoring(){
        ros::Rate rate = ros::Rate(1);
        std::vector<int> cameraColorMsgLastCounts = std::vector<int>(cameraColorNames.size(), 0);
        std::vector<int> cameraDepthMsgLastCounts = std::vector<int>(cameraDepthNames.size(), 0);
        std::vector<int> cameraPointCloudMsgLastCounts = std::vector<int>(cameraPointCloudNames.size(), 0);
        std::vector<int> armJointStateMsgLastCounts = std::vector<int>(armJointStateNames.size(), 0);
        std::vector<int> armEndPoseMsgLastCounts = std::vector<int>(armEndPoseNames.size(), 0);
        std::vector<int> localizationPoseMsgLastCounts = std::vector<int>(localizationPoseNames.size(), 0);
        std::vector<int> gripperEncoderMsgLastCounts = std::vector<int>(gripperEncoderNames.size(), 0);
        std::vector<int> imu9AxisMsgLastCounts = std::vector<int>(imu9AxisNames.size(), 0);
        std::vector<int> lidarPointCloudMsgLastCounts = std::vector<int>(lidarPointCloudNames.size(), 0);
        std::vector<int> robotBaseVelMsgLastCounts = std::vector<int>(robotBaseVelNames.size(), 0);
        std::vector<int> liftMotorMsgLastCounts = std::vector<int>(liftMotorNames.size(), 0);
        std::vector<double> cameraColorMsgLastUpHzTimes = std::vector<double>(cameraColorNames.size(), 0);
        std::vector<double> cameraDepthMsgLastUpHzTimes = std::vector<double>(cameraDepthNames.size(), 0);
        std::vector<double> cameraPointCloudMsgLastUpHzTimes = std::vector<double>(cameraPointCloudNames.size(), 0);
        std::vector<double> armJointStateMsgLastUpHzTimes = std::vector<double>(armJointStateNames.size(), 0);
        std::vector<double> armEndPoseMsgLastUpHzTimes = std::vector<double>(armEndPoseNames.size(), 0);
        std::vector<double> localizationPoseMsgLastUpHzTimes = std::vector<double>(localizationPoseNames.size(), 0);
        std::vector<double> gripperEncoderMsgLastUpHzTimes = std::vector<double>(gripperEncoderNames.size(), 0);
        std::vector<double> imu9AxisMsgLastUpHzTimes = std::vector<double>(imu9AxisNames.size(), 0);
        std::vector<double> lidarPointCloudMsgLastUpHzTimes = std::vector<double>(lidarPointCloudNames.size(), 0);
        std::vector<double> robotBaseVelMsgLastUpHzTimes = std::vector<double>(robotBaseVelNames.size(), 0);
        std::vector<double> liftMotorMsgLastUpHzTimes = std::vector<double>(liftMotorNames.size(), 0);
        ros::Time beginTime = ros::Time::now();
        data_msgs::CaptureStatus captureStatus;
        while(true){
            captureStatus.topics.clear();
            captureStatus.count_in_seconds.clear();
            captureStatus.frequencies.clear();
            captureStatus.fail = false;
            std::ofstream file(statisticsDir);
            system("clear");
            std::cout<<"path: "<<episodeDir<<std::endl;
            std::cout<<"total time: "<<(ros::Time::now() - beginTime).toSec()<<std::endl;
            int allCount = 0;
            std::cout<<"topic: frame in 1 second / total frame"<<std::endl;
            file<<(ros::Time::now() - beginTime).toSec()<<std::endl;
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
                    cameraColorMsgLastUpHzTimes.at(i) = ros::Time::now().toSec();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(ros::Time::now().toSec() - cameraColorMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<cameraColorTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    cameraColorMsgLastUpHzTimes.at(i) = ros::Time::now().toSec();
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
                    cameraDepthMsgLastUpHzTimes.at(i) = ros::Time::now().toSec();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(ros::Time::now().toSec() - cameraDepthMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<cameraDepthTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    cameraDepthMsgLastUpHzTimes.at(i) = ros::Time::now().toSec();
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
                    cameraPointCloudMsgLastUpHzTimes.at(i) = ros::Time::now().toSec();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(ros::Time::now().toSec() - cameraPointCloudMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<cameraPointCloudTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    cameraPointCloudMsgLastUpHzTimes.at(i) = ros::Time::now().toSec();
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
                    armJointStateMsgLastUpHzTimes.at(i) = ros::Time::now().toSec();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(ros::Time::now().toSec() - armJointStateMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<armJointStateTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    armJointStateMsgLastUpHzTimes.at(i) = ros::Time::now().toSec();
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
                    armEndPoseMsgLastUpHzTimes.at(i) = ros::Time::now().toSec();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(ros::Time::now().toSec() - armEndPoseMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<armEndPoseTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    armEndPoseMsgLastUpHzTimes.at(i) = ros::Time::now().toSec();
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
                    localizationPoseMsgLastUpHzTimes.at(i) = ros::Time::now().toSec();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(ros::Time::now().toSec() - localizationPoseMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<localizationPoseTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    localizationPoseMsgLastUpHzTimes.at(i) = ros::Time::now().toSec();
                }
                localizationPoseMsgLastCounts.at(i) = count;
                allCount += count;
                file<<"localization/pose/"<<localizationPoseNames.at(i)<<" "<<count<<" "<<frequency<<std::endl;
                captureStatus.topics.push_back(localizationPoseTopics.at(i));
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
                    gripperEncoderMsgLastUpHzTimes.at(i) = ros::Time::now().toSec();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(ros::Time::now().toSec() - gripperEncoderMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<gripperEncoderTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    gripperEncoderMsgLastUpHzTimes.at(i) = ros::Time::now().toSec();
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
                    imu9AxisMsgLastUpHzTimes.at(i) = ros::Time::now().toSec();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(ros::Time::now().toSec() - imu9AxisMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<imu9AxisTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    imu9AxisMsgLastUpHzTimes.at(i) = ros::Time::now().toSec();
                }
                imu9AxisMsgLastCounts.at(i) = count;
                allCount += count;
                file<<"imu/9axis/"<<imu9AxisNames.at(i)<<" "<<count<<" "<<frequency<<std::endl;
                captureStatus.topics.push_back(imu9AxisTopics.at(i));
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
                    lidarPointCloudMsgLastUpHzTimes.at(i) = ros::Time::now().toSec();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(ros::Time::now().toSec() - lidarPointCloudMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<lidarPointCloudTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    lidarPointCloudMsgLastUpHzTimes.at(i) = ros::Time::now().toSec();
                }
                lidarPointCloudMsgLastCounts.at(i) = count;
                allCount += count;
                file<<"lidar/pointCloud/"<<lidarPointCloudNames.at(i)<<" "<<count<<" "<<frequency<<std::endl;
                captureStatus.topics.push_back(lidarPointCloudTopics.at(i));
                captureStatus.count_in_seconds.push_back(countInSecond);
                captureStatus.frequencies.push_back(frequency);
            }
            for(int i = 0; i < robotBaseVelNames.size(); i++){
                robotBaseVelMsgCountMtxs.at(i).lock();
                int count = robotBaseVelMsgCounts.at(i);
                double startTime = robotBaseVelStartTimeStamps.at(i);
                double lastTime = robotBaseVelLastTimeStamps.at(i);
                robotBaseVelMsgCountMtxs.at(i).unlock();
                int countInSecond = count - robotBaseVelMsgLastCounts.at(i);
                double frequency = ((double)count / (lastTime - startTime));
                std::cout<<robotBaseVelTopics.at(i)<<": "<<countInSecond<<" / "<<count<<"("<<frequency<<"hz)"<<std::endl;
                if(robotBaseVelMsgLastUpHzTimes.at(i) == 0){
                    robotBaseVelMsgLastUpHzTimes.at(i) = ros::Time::now().toSec();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(ros::Time::now().toSec() - robotBaseVelMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<robotBaseVelTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    robotBaseVelMsgLastUpHzTimes.at(i) = ros::Time::now().toSec();
                }
                robotBaseVelMsgLastCounts.at(i) = count;
                allCount += count;
                file<<"robotBase/vel/"<<robotBaseVelNames.at(i)<<" "<<count<<" "<<frequency<<std::endl;
                captureStatus.topics.push_back(robotBaseVelTopics.at(i));
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
                    liftMotorMsgLastUpHzTimes.at(i) = ros::Time::now().toSec();
                }
                if(this->hz != -1 && countInSecond <= this->hz){
                    if(ros::Time::now().toSec() - liftMotorMsgLastUpHzTimes.at(i) > this->timeout){
                        std::cout<<"Check the frequency of "<<liftMotorTopics.at(i)<<std::endl;
                        captureStatus.fail = true;
                    }
                }else{
                    liftMotorMsgLastUpHzTimes.at(i) = ros::Time::now().toSec();
                }
                liftMotorMsgLastCounts.at(i) = count;
                allCount += count;
                file<<"lift/motor/"<<liftMotorNames.at(i)<<" "<<count<<" "<<frequency<<std::endl;
                captureStatus.topics.push_back(liftMotorTopics.at(i));
                captureStatus.count_in_seconds.push_back(countInSecond);
                captureStatus.frequencies.push_back(frequency);
            }
            captureStatus.quit = false;
            pubCaptureStatus.publish(captureStatus);
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
                    pubCaptureStatus.publish(captureStatus);
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


class DataCaptureService{
    public:
    ros::NodeHandle nh;
    ros::ServiceServer srvDataCapture;
    DataCapture *dataCapture;
    std::string datasetDir;
    int episodeIndex;
    int hz;
    int timeout;
    double cropTime;

    DataCaptureService(std::string datasetDir, int episodeIndex, int hz, int timeout, double cropTime) {
        this->datasetDir = datasetDir;
        this->episodeIndex = episodeIndex;
        this->hz = hz;
        this->timeout = timeout;
        this->cropTime = cropTime;
        dataCapture = nullptr;
        srvDataCapture  = nh.advertiseService("/data_tools_dataCapture/capture_service", &DataCaptureService::captureService, this);
    }

    bool captureService(data_msgs::CaptureServiceRequest& req, data_msgs::CaptureServiceResponse& res)
    {
        if(req.start && req.end){
            if(dataCapture != nullptr){
                // res.success = false;
                dataCapture->shutdown();
                dataCapture->join();
                delete dataCapture;
                dataCapture = nullptr;
                res.success = true;
            }else{
                std::string datasetDir = this->datasetDir;
                int episodeIndex = this->episodeIndex;
                if(req.dataset_dir != ""){
                    datasetDir = req.dataset_dir;
                }
                if(req.episode_index != -1){
                    episodeIndex = req.episode_index;
                }
                dataCapture = new DataCapture(datasetDir, episodeIndex, hz, timeout, cropTime, true);
                if(req.episode_index == -1){
                    this->episodeIndex++;
                }
                ros::Duration(this->cropTime).sleep();
                dataCapture->instructionSaving(req.instructions);
                dataCapture->run();
                res.success = true;
            }
        }else{
            if(req.start){
                if(dataCapture != nullptr){
                    // res.success = false;
                    dataCapture->shutdown();
                    dataCapture->join();
                    delete dataCapture;
                    dataCapture = nullptr;
                }
                // else{
                    std::string datasetDir = this->datasetDir;
                    int episodeIndex = this->episodeIndex;
                    if(req.dataset_dir != ""){
                        datasetDir = req.dataset_dir;
                    }
                    if(req.episode_index != -1){
                        episodeIndex = req.episode_index;
                    }
                    dataCapture = new DataCapture(datasetDir, episodeIndex, hz, timeout, cropTime, true);
                    if(req.episode_index == -1){
                        this->episodeIndex++;
                    }
                    ros::Duration(this->cropTime).sleep();
                    dataCapture->instructionSaving(req.instructions);
                    dataCapture->run();
                    res.success = true;
                // }
            }else if(req.end){
                if(dataCapture != nullptr){
                    dataCapture->shutdown();
                    dataCapture->join();
                    delete dataCapture;
                    dataCapture = nullptr;
                    res.success = true;
                    std::cout<<"wait for start signal"<<std::endl;
                }else{
                    res.success = false;
                }
            }
        }
        return true;
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_capture");
    ros::NodeHandle nh;
    bool useService;
    std::string datasetDir;
    int episodeIndex;
    std::string instructions;
    int hz;
    int timeout;
    double cropTime;
    nh.param<bool>("useService", useService, false);
    nh.param<std::string>("datasetDir", datasetDir, "/home/agilex/data");
    nh.param<int>("episodeIndex", episodeIndex, 0);
    nh.param<std::string>("instructions", instructions, "");
    nh.param<int>("hz", hz, -1);
    nh.param<int>("timeout", timeout, 2);
    nh.param<double>("cropTime", cropTime, -1);
    ROS_INFO("\033[1;32m----> data capture Started.\033[0m");
    if(useService){
        DataCaptureService dataCaptureService(datasetDir, episodeIndex, hz, timeout, cropTime);
        ros::MultiThreadedSpinner spinner(16);
        spinner.spin();
        std::cout<<"Done"<<std::endl;
        return 0;
    }else{
        DataCapture dataCapture(datasetDir, episodeIndex, hz, timeout);
        dataCapture.instructionSaving(instructions);
        dataCapture.run();
        ros::MultiThreadedSpinner spinner(16);
        spinner.spin();
        dataCapture.join();
        std::cout<<"Done"<<std::endl;
        return 0;
    }
}
