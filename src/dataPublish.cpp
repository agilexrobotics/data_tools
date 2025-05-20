#include "dataUtility.h"
#include <mutex>
#include <math.h>
#include <condition_variable>
#include <thread>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <boost/filesystem.hpp>
#include <semaphore.h>
#include <iostream>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <data_msgs/Gripper.h>
#include "jsoncpp/json/json.h"
#ifdef _USELIFT
#include <bt_task_msgs/LiftMotorMsg.h>
#include <bt_task_msgs/LiftMotorSrv.h>
#endif

class DataPublish: public DataUtility{
public:
    std::vector<ros::Publisher> pubCameraColors;
    std::vector<ros::Publisher> pubCameraDepths;
    std::vector<ros::Publisher> pubCameraPointClouds;
    std::vector<ros::Publisher> pubArmJointStates;
    std::vector<ros::Publisher> pubArmEndPoses;
    std::vector<ros::Publisher> pubLocalizationPoses;
    std::vector<ros::Publisher> pubGripperEncoders;
    std::vector<ros::Publisher> pubImu9Axiss;
    std::vector<ros::Publisher> pubLidarPointClouds;
    std::vector<ros::Publisher> pubRobotBaseVels;
    std::vector<ros::Publisher> pubLiftMotors;

    std::vector<ros::Publisher> pubCameraColorConfigs;
    std::vector<ros::Publisher> pubCameraDepthConfigs;
    std::vector<ros::Publisher> pubCameraPointCloudConfigs;
    std::vector<ros::Publisher> pubArmJointStateConfigs;
    std::vector<ros::Publisher> pubArmEndPoseConfigs;
    std::vector<ros::Publisher> pubLocalizationPoseConfigs;
    std::vector<ros::Publisher> pubGripperEncoderConfigs;
    std::vector<ros::Publisher> pubImu9AxisConfigs;
    std::vector<ros::Publisher> pubLidarPointCloudConfigs;
    std::vector<ros::Publisher> pubRobotBaseVelConfigs;
    std::vector<ros::Publisher> pubLiftMotorConfigs;

    std::vector<sem_t> cameraColorSems;
    std::vector<sem_t> cameraDepthSems;
    std::vector<sem_t> cameraPointCloudSems;
    std::vector<sem_t> armJointStateSems;
    std::vector<sem_t> armEndPoseSems;
    std::vector<sem_t> localizationPoseSems;
    std::vector<sem_t> gripperEncoderSems;
    std::vector<sem_t> imu9AxisSems;
    std::vector<sem_t> lidarPointCloudSems;
    std::vector<sem_t> robotBaseVelSems;
    std::vector<sem_t> liftMotorSems;
    std::vector<sem_t> tfTransformSems;

    std::vector<std::ifstream> syncFileCameraColors;
    std::vector<std::ifstream> syncFileCameraDepths;
    std::vector<std::ifstream> syncFileCameraPointClouds;
    std::vector<std::ifstream> syncFileArmJointStates;
    std::vector<std::ifstream> syncFileArmEndPoses;
    std::vector<std::ifstream> syncFileLocalizationPoses;
    std::vector<std::ifstream> syncFileGripperEncoders;
    std::vector<std::ifstream> syncFileImu9Axiss;
    std::vector<std::ifstream> syncFileLidarPointClouds;
    std::vector<std::ifstream> syncFileRobotBaseVels;
    std::vector<std::ifstream> syncFileLiftMotors;

    std::vector<std::thread*> cameraColorPublishingThreads;
    std::vector<std::thread*> cameraDepthPublishingThreads;
    std::vector<std::thread*> cameraPointCloudPublishingThreads;
    std::vector<std::thread*> armJointStatePublishingThreads;
    std::vector<std::thread*> armEndPosePublishingThreads;
    std::vector<std::thread*> localizationPosePublishingThreads;
    std::vector<std::thread*> gripperEncoderPublishingThreads;
    std::vector<std::thread*> imu9AxisPublishingThreads;
    std::vector<std::thread*> lidarPointCloudPublishingThreads;
    std::vector<std::thread*> robotBaseVelPublishingThreads;
    std::vector<std::thread*> liftMotorPublishingThreads;
    std::vector<std::thread*> tfTransformPublishingThreads;

    std::thread* activatingThread;

    float publishRate;
    int publishIndex;

    DataPublish(std::string datasetDir, int episodeIndex, int publishIndexParam, float publishRateParam): DataUtility(datasetDir, episodeIndex) {
        publishRate = publishRateParam;
        publishIndex = publishIndexParam;

        syncFileCameraColors = std::vector<std::ifstream>(cameraColorNames.size());
        syncFileCameraDepths = std::vector<std::ifstream>(cameraDepthNames.size());
        syncFileCameraPointClouds = std::vector<std::ifstream>(cameraPointCloudNames.size());
        syncFileArmJointStates = std::vector<std::ifstream>(armJointStateNames.size());
        syncFileArmEndPoses = std::vector<std::ifstream>(armEndPoseNames.size());
        syncFileLocalizationPoses = std::vector<std::ifstream>(localizationPoseNames.size());
        syncFileGripperEncoders = std::vector<std::ifstream>(gripperEncoderNames.size());
        syncFileImu9Axiss = std::vector<std::ifstream>(imu9AxisNames.size());
        syncFileLidarPointClouds = std::vector<std::ifstream>(lidarPointCloudNames.size());
        syncFileRobotBaseVels = std::vector<std::ifstream>(robotBaseVelNames.size());
        syncFileLiftMotors = std::vector<std::ifstream>(liftMotorNames.size());

        cameraColorSems = std::vector<sem_t>(cameraColorNames.size());
        cameraDepthSems = std::vector<sem_t>(cameraDepthNames.size());
        cameraPointCloudSems = std::vector<sem_t>(cameraPointCloudNames.size());
        armJointStateSems = std::vector<sem_t>(armJointStateNames.size());
        armEndPoseSems = std::vector<sem_t>(armEndPoseNames.size());
        localizationPoseSems = std::vector<sem_t>(localizationPoseNames.size());
        gripperEncoderSems = std::vector<sem_t>(gripperEncoderNames.size());
        imu9AxisSems = std::vector<sem_t>(imu9AxisNames.size());
        lidarPointCloudSems = std::vector<sem_t>(lidarPointCloudNames.size());
        robotBaseVelSems = std::vector<sem_t>(robotBaseVelNames.size());
        liftMotorSems = std::vector<sem_t>(liftMotorNames.size());
        tfTransformSems = std::vector<sem_t>(tfTransformParentFrames.size());

        for(int i = 0; i < cameraColorNames.size(); i++){
            pubCameraColors.push_back(nh.advertise<sensor_msgs::Image>(cameraColorPublishTopics[i], 2000));
            if(!cameraColorConfigPublishTopics.empty())
                pubCameraColorConfigs.push_back(nh.advertise<sensor_msgs::CameraInfo>(cameraColorConfigPublishTopics[i], 2000));
            if(cameraColorToPublishs.at(i)){
                syncFileCameraColors.at(i).open(cameraColorDirs.at(i)+"/sync.txt");
                sem_init(&cameraColorSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < cameraDepthNames.size(); i++){
            pubCameraDepths.push_back(nh.advertise<sensor_msgs::Image>(cameraDepthPublishTopics[i], 2000));
            if(!cameraDepthConfigPublishTopics.empty())
                pubCameraDepthConfigs.push_back(nh.advertise<sensor_msgs::CameraInfo>(cameraDepthConfigPublishTopics[i], 2000));
            if(cameraDepthToPublishs.at(i)){
                syncFileCameraDepths.at(i).open(cameraDepthDirs.at(i)+"/sync.txt");
                sem_init(&cameraDepthSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            pubCameraPointClouds.push_back(nh.advertise<sensor_msgs::PointCloud2>(cameraPointCloudPublishTopics[i], 2000));
            if(!cameraPointCloudConfigPublishTopics.empty())
                pubCameraPointCloudConfigs.push_back(nh.advertise<sensor_msgs::CameraInfo>(cameraPointCloudConfigPublishTopics[i], 2000));
            if(cameraPointCloudToPublishs.at(i)){
                syncFileCameraPointClouds.at(i).open(cameraPointCloudDirs.at(i)+"/sync.txt");
                sem_init(&cameraPointCloudSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < armJointStateNames.size(); i++){
            pubArmJointStates.push_back(nh.advertise<sensor_msgs::JointState>(armJointStatePublishTopics[i], 2000));
            if(armJointStateToPublishs.at(i)){
                syncFileArmJointStates.at(i).open(armJointStateDirs.at(i)+"/sync.txt");
                sem_init(&armJointStateSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < armEndPoseNames.size(); i++){
            pubArmEndPoses.push_back(nh.advertise<geometry_msgs::PoseStamped>(armEndPosePublishTopics[i], 2000));
            if(armEndPoseToPublishs.at(i)){
                syncFileArmEndPoses.at(i).open(armEndPoseDirs.at(i)+"/sync.txt");
                sem_init(&armEndPoseSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < localizationPoseNames.size(); i++){
            pubLocalizationPoses.push_back(nh.advertise<geometry_msgs::PoseStamped>(localizationPosePublishTopics[i], 2000));
            if(localizationPoseToPublishs.at(i)){
                syncFileLocalizationPoses.at(i).open(localizationPoseDirs.at(i)+"/sync.txt");
                sem_init(&localizationPoseSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < gripperEncoderNames.size(); i++){
            pubGripperEncoders.push_back(nh.advertise<data_msgs::Gripper>(gripperEncoderPublishTopics[i], 2000));
            if(gripperEncoderToPublishs.at(i)){
                syncFileGripperEncoders.at(i).open(gripperEncoderDirs.at(i)+"/sync.txt");
                sem_init(&gripperEncoderSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < imu9AxisNames.size(); i++){
            pubImu9Axiss.push_back(nh.advertise<sensor_msgs::Imu>(imu9AxisPublishTopics[i], 2000));
            if(imu9AxisToPublishs.at(i)){
                syncFileImu9Axiss.at(i).open(imu9AxisDirs.at(i)+"/sync.txt");
                sem_init(&imu9AxisSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < lidarPointCloudNames.size(); i++){
            pubLidarPointClouds.push_back(nh.advertise<sensor_msgs::PointCloud2>(lidarPointCloudPublishTopics[i], 2000));
            if(lidarPointCloudToPublishs.at(i)){
                syncFileLidarPointClouds.at(i).open(lidarPointCloudDirs.at(i)+"/sync.txt");
                sem_init(&lidarPointCloudSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < robotBaseVelNames.size(); i++){
            pubRobotBaseVels.push_back(nh.advertise<nav_msgs::Odometry>(robotBaseVelPublishTopics[i], 2000));
            if(robotBaseVelToPublishs.at(i)){
                syncFileRobotBaseVels.at(i).open(robotBaseVelDirs.at(i)+"/sync.txt");
                sem_init(&robotBaseVelSems.at(i), 0, 0);
            }
        }
        #ifdef _USELIFT
        for(int i = 0; i < liftMotorNames.size(); i++){
            pubLiftMotors.push_back(nh.advertise<bt_task_msgs::LiftMotorMsg>(liftMotorPublishTopics[i], 2000));
            if(liftMotorToPublishs.at(i)){
                syncFileLiftMotors.at(i).open(liftMotorDirs.at(i)+"/sync.txt");
                sem_init(&liftMotorSems.at(i), 0, 0);
            }
        }
        #endif
    }

    void cameraColorPublishing(const int index){
        Json::Reader jsonReader;
        Json::Value root;
        std::ifstream file(cameraColorDirs.at(index)+"/config.json", std::iostream::binary);
        jsonReader.parse(file, root);
        sensor_msgs::CameraInfo cameraInfo;
        cameraInfo.header.stamp = ros::Time().now();
        cameraInfo.header.frame_id = cameraColorParentFrames.at(index) + "_color";
        cameraInfo.height = root["height"].asInt();
        cameraInfo.width = root["width"].asInt();
        cameraInfo.distortion_model = root["distortion_model"].asString();
        Json::Value D = root["D"];
        for (int i = 0; i < D.size(); i++)
            cameraInfo.D.push_back(D[i].asDouble());
        Json::Value K = root["K"];
        for (int i = 0; i < K.size(); i++)
            cameraInfo.K[i] = K[i].asDouble();
        Json::Value R = root["R"];
        for (int i = 0; i < R.size(); i++)
            cameraInfo.R[i] = R[i].asDouble();
        Json::Value P = root["P"];
        for (int i = 0; i < P.size(); i++)
            cameraInfo.P[i] = P[i].asDouble();
        cameraInfo.binning_x = root["binning_x"].asInt();
        cameraInfo.binning_y = root["binning_y"].asInt();
        cameraInfo.roi.x_offset = root["roi"]["x_offset"].asInt();
        cameraInfo.roi.y_offset = root["roi"]["y_offset"].asInt();
        cameraInfo.roi.height = root["roi"]["height"].asInt();
        cameraInfo.roi.width = root["roi"]["width"].asInt();
        cameraInfo.roi.do_rectify = root["roi"]["do_rectify"].asBool();

        static tf::TransformBroadcaster tfBroadcaster;
        tf::Transform trans;
        geometry_msgs::Pose pose;
        pose.position.x = root["parent_frame"]["x"].asDouble();
        pose.position.y = root["parent_frame"]["y"].asDouble();
        pose.position.z = root["parent_frame"]["z"].asDouble();
        pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(root["parent_frame"]["roll"].asDouble(), root["parent_frame"]["pitch"].asDouble(), root["parent_frame"]["yaw"].asDouble());
        tf::poseMsgToTF(pose, trans);

        std::string time0 = "";
        int count = 0;
        while(ros::ok()){
            std::string time;
            if(publishIndex != -1){
                if(time0 == ""){
                    getline(syncFileCameraColors.at(index), time0);
                    if(count != publishIndex){
                        count++;
                        time0 = "";
                        continue;
                    }
                }
                time = time0;
            }else{
                if(!getline(syncFileCameraColors.at(index), time))
                    break;
            }
            sem_wait(&cameraColorSems.at(index));
            cv::Mat image = cv::imread(cameraColorDirs.at(index) + "/" + time,cv::IMREAD_COLOR);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, image).toImageMsg();
            msg->header.stamp = ros::Time().now();
            msg->header.frame_id = cameraColorParentFrames.at(index) + "_color";
            pubCameraColors.at(index).publish(*msg);

            cameraInfo.header.stamp = ros::Time().now();
            pubCameraColorConfigs.at(index).publish(cameraInfo);
            tf::StampedTransform tfTrans = tf::StampedTransform(trans, ros::Time().now(), cameraColorParentFrames.at(index), cameraColorParentFrames.at(index) + "_color");
            tfBroadcaster.sendTransform(tfTrans);
        }
    }

    void cameraDepthPublishing(const int index){
        Json::Reader jsonReader;
        Json::Value root;
        std::ifstream file(cameraDepthDirs.at(index)+"/config.json", std::iostream::binary);
        jsonReader.parse(file, root);
        sensor_msgs::CameraInfo cameraInfo;
        cameraInfo.header.stamp = ros::Time().now();
        cameraInfo.header.frame_id = cameraDepthParentFrames.at(index) + "_depth";
        cameraInfo.height = root["height"].asInt();
        cameraInfo.width = root["width"].asInt();
        cameraInfo.distortion_model = root["distortion_model"].asString();
        Json::Value D = root["D"];
        for (int i = 0; i < D.size(); i++)
            cameraInfo.D.push_back(D[i].asDouble());
        Json::Value K = root["K"];
        for (int i = 0; i < K.size(); i++)
            cameraInfo.K[i] = K[i].asDouble();
        Json::Value R = root["R"];
        for (int i = 0; i < R.size(); i++)
            cameraInfo.R[i] = R[i].asDouble();
        Json::Value P = root["P"];
        for (int i = 0; i < P.size(); i++)
            cameraInfo.P[i] = P[i].asDouble();
        cameraInfo.binning_x = root["binning_x"].asInt();
        cameraInfo.binning_y = root["binning_y"].asInt();
        cameraInfo.roi.x_offset = root["roi"]["x_offset"].asInt();
        cameraInfo.roi.y_offset = root["roi"]["y_offset"].asInt();
        cameraInfo.roi.height = root["roi"]["height"].asInt();
        cameraInfo.roi.width = root["roi"]["width"].asInt();
        cameraInfo.roi.do_rectify = root["roi"]["do_rectify"].asBool();

        static tf::TransformBroadcaster tfBroadcaster;
        tf::Transform trans;
        geometry_msgs::Pose pose;
        pose.position.x = root["parent_frame"]["x"].asDouble();
        pose.position.y = root["parent_frame"]["y"].asDouble();
        pose.position.z = root["parent_frame"]["z"].asDouble();
        pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(root["parent_frame"]["roll"].asDouble(), root["parent_frame"]["pitch"].asDouble(), root["parent_frame"]["yaw"].asDouble());
        tf::poseMsgToTF(pose, trans);

        std::string time0 = "";
        int count = 0;
        while(ros::ok()){
            std::string time;
            if(publishIndex != -1){
                if(time0 == ""){
                    getline(syncFileCameraDepths.at(index), time0);
                    if(count != publishIndex){
                        count++;
                        time0 = "";
                        continue;
                    }
                }
                time = time0;
            }else{
                if(!getline(syncFileCameraDepths.at(index), time))
                    break;
            }
            sem_wait(&cameraDepthSems.at(index));
            cv::Mat image = cv::imread(cameraDepthDirs.at(index) + "/" + time,cv::IMREAD_ANYDEPTH);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, image).toImageMsg();
            msg->header.stamp = ros::Time().now();
            msg->header.frame_id = cameraDepthParentFrames.at(index) + "_depth";
            pubCameraDepths.at(index).publish(msg);

            cameraInfo.header.stamp = ros::Time().now();
            pubCameraDepthConfigs.at(index).publish(cameraInfo);
            tf::StampedTransform tfTrans = tf::StampedTransform(trans, ros::Time().now(), cameraDepthParentFrames.at(index), cameraDepthParentFrames.at(index) + "_depth");
            tfBroadcaster.sendTransform(tfTrans);
        }
    }

    void cameraPointCloudPublishing(const int index){
        Json::Reader jsonReader;
        Json::Value root;
        std::ifstream file(cameraPointCloudDirs.at(index)+"/config.json", std::iostream::binary);
        jsonReader.parse(file, root);
        sensor_msgs::CameraInfo cameraInfo;
        cameraInfo.header.stamp = ros::Time().now();
        cameraInfo.header.frame_id = cameraPointCloudParentFrames.at(index) + "_pointcloud";
        cameraInfo.height = root["height"].asInt();
        cameraInfo.width = root["width"].asInt();
        cameraInfo.distortion_model = root["distortion_model"].asString();
        Json::Value D = root["D"];
        for (int i = 0; i < D.size(); i++)
            cameraInfo.D.push_back(D[i].asDouble());
        Json::Value K = root["K"];
        for (int i = 0; i < K.size(); i++)
            cameraInfo.K[i] = K[i].asDouble();
        Json::Value R = root["R"];
        for (int i = 0; i < R.size(); i++)
            cameraInfo.R[i] = R[i].asDouble();
        Json::Value P = root["P"];
        for (int i = 0; i < P.size(); i++)
            cameraInfo.P[i] = P[i].asDouble();
        cameraInfo.binning_x = root["binning_x"].asInt();
        cameraInfo.binning_y = root["binning_y"].asInt();
        cameraInfo.roi.x_offset = root["roi"]["x_offset"].asInt();
        cameraInfo.roi.y_offset = root["roi"]["y_offset"].asInt();
        cameraInfo.roi.height = root["roi"]["height"].asInt();
        cameraInfo.roi.width = root["roi"]["width"].asInt();
        cameraInfo.roi.do_rectify = root["roi"]["do_rectify"].asBool();

        static tf::TransformBroadcaster tfBroadcaster;
        tf::Transform trans;
        geometry_msgs::Pose pose;
        pose.position.x = root["parent_frame"]["x"].asDouble();
        pose.position.y = root["parent_frame"]["y"].asDouble();
        pose.position.z = root["parent_frame"]["z"].asDouble();
        pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(root["parent_frame"]["roll"].asDouble(), root["parent_frame"]["pitch"].asDouble(), root["parent_frame"]["yaw"].asDouble());
        tf::poseMsgToTF(pose, trans);

        std::string time0 = "";
        int count = 0;
        while(ros::ok()){
            std::string time;
            if(publishIndex != -1){
                if(time0 == ""){
                    getline(syncFileCameraPointClouds.at(index), time0);
                    if(count != publishIndex){
                        count++;
                        time0 = "";
                        continue;
                    }
                }
                time = time0;
            }else{
                if(!getline(syncFileCameraPointClouds.at(index), time))
                    break;
            }
            sem_wait(&cameraPointCloudSems.at(index));
            // pcl::PointCloud<pcl::PointXYZ> pointcloud;
            // pcl::io::loadPCDFile<pcl::PointXYZ>(cameraPointCloudDirs.at(index) + "-normalization" + "/" + time, pointcloud);
            pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
            pcl::io::loadPCDFile<pcl::PointXYZRGB>(cameraPointCloudDirs.at(index) + "/" + time, pointcloud);
            sensor_msgs::PointCloud2 cloudMsg;
            pcl::toROSMsg(pointcloud, cloudMsg);
            cloudMsg.header.stamp = ros::Time().now();
            cloudMsg.header.frame_id = cameraPointCloudParentFrames.at(index) + "_pointcloud";
            pubCameraPointClouds.at(index).publish(cloudMsg);

            cameraInfo.header.stamp = ros::Time().now();
            pubCameraPointCloudConfigs.at(index).publish(cameraInfo);
            tf::StampedTransform tfTrans = tf::StampedTransform(trans, ros::Time().now(), cameraPointCloudParentFrames.at(index), cameraPointCloudParentFrames.at(index) + "_pointcloud");
            tfBroadcaster.sendTransform(tfTrans);
        }
    }

    void armJointStatePublishing(const int index){
        std::string time0 = "";
        int count = 0;
        while(ros::ok()){
            std::string time;
            if(publishIndex != -1){
                if(time0 == ""){
                    getline(syncFileArmJointStates.at(index), time0);
                    if(count != publishIndex){
                        count++;
                        time0 = "";
                        continue;
                    }
                }
                time = time0;
            }else{
                if(!getline(syncFileArmJointStates.at(index), time))
                    break;
            }
            sem_wait(&armJointStateSems.at(index));
            Json::Reader jsonReader;
            Json::Value root;
            std::ifstream file(armJointStateDirs.at(index) + "/" + time, std::iostream::binary);
            jsonReader.parse(file, root);
            Json::Value effort = root["effort"];
            std::vector<double> effortData;
            for (int i = 0; i < effort.size(); i++)
                effortData.push_back(effort[i].asDouble());
            Json::Value position = root["position"];
            std::vector<double> positionData;
            for (int i = 0; i < position.size(); i++)
                positionData.push_back(position[i].asDouble());
            Json::Value velocity = root["velocity"];
            std::vector<double> velocityData;
            for (int i = 0; i < velocity.size(); i++)
                velocityData.push_back(velocity[i].asDouble());
            sensor_msgs::JointState msg;
            msg.header.stamp = ros::Time().now();
            msg.position = positionData;
            pubArmJointStates.at(index).publish(msg);
        }
    }

    void armEndPosePublishing(const int index){
        std::string time0 = "";
        int count = 0;
        while(ros::ok()){
            std::string time;
            if(publishIndex != -1){
                if(time0 == ""){
                    getline(syncFileArmEndPoses.at(index), time0);
                    if(count != publishIndex){
                        count++;
                        time0 = "";
                        continue;
                    }
                }
                time = time0;
            }else{
                if(!getline(syncFileArmEndPoses.at(index), time))
                    break;
            }
            sem_wait(&armEndPoseSems.at(index));
            geometry_msgs::PoseStamped msg;
            msg.header.stamp = ros::Time().now();
            msg.header.frame_id = "map";
            Json::Reader jsonReader;
            Json::Value root;
            std::ifstream file(armEndPoseDirs.at(index) + "/" + time, std::iostream::binary);
            jsonReader.parse(file, root);
            if(armEndPoseOrients.at(index)){
                Eigen::Affine3f transBack = pcl::getTransformation(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 
                                                                root["roll"].asDouble(), root["pitch"].asDouble(), root["yaw"].asDouble());
                Eigen::Affine3f transIncre = pcl::getTransformation(0, 0, 0,
                                                                    0, 0, 0);
                Eigen::Affine3f transFinal = transBack * transIncre;
                float x, y, z, roll, pitch, yaw;
                pcl::getTranslationAndEulerAngles(transFinal, x, y, z, roll, pitch, yaw);
                tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
                msg.pose.position.x = x;
                msg.pose.position.y = y;
                msg.pose.position.z = z;
                msg.pose.orientation.x = q.x();
                msg.pose.orientation.y = q.y();
                msg.pose.orientation.z = q.z();
                msg.pose.orientation.w = q.w();
            }else{
                msg.pose.position.x = root["x"].asDouble();
                msg.pose.position.y = root["y"].asDouble();
                msg.pose.position.z = root["z"].asDouble();
                msg.pose.orientation.x = root["roll"].asDouble();
                msg.pose.orientation.y = root["pitch"].asDouble();
                msg.pose.orientation.z = root["yaw"].asDouble();
                msg.pose.orientation.w = root["grasper"].asDouble();
            }
            pubArmEndPoses.at(index).publish(msg);
        }
    }

    void localizationPosePublishing(const int index){
        std::string time0 = "";
        int count = 0;
        while(ros::ok()){
            std::string time;
            if(publishIndex != -1){
                if(time0 == ""){
                    getline(syncFileLocalizationPoses.at(index), time0);
                    if(count != publishIndex){
                        count++;
                        time0 = "";
                        continue;
                    }
                }
                time = time0;
            }else{
                if(!getline(syncFileLocalizationPoses.at(index), time))
                    break;
            }
            sem_wait(&localizationPoseSems.at(index));
            geometry_msgs::PoseStamped msg;
            msg.header.stamp = ros::Time().now();
            msg.header.frame_id = "map";
            Json::Reader jsonReader;
            Json::Value root;
            std::ifstream file(localizationPoseDirs.at(index) + "/" + time, std::iostream::binary);
            jsonReader.parse(file, root);
            msg.pose.position.x = root["x"].asDouble();
            msg.pose.position.y = root["y"].asDouble();
            msg.pose.position.z = root["z"].asDouble();
            Eigen::Affine3f transBack = pcl::getTransformation(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 
                                                               root["roll"].asDouble(), root["pitch"].asDouble(), root["yaw"].asDouble());
            Eigen::Affine3f transIncre = pcl::getTransformation(0, 0, 0,
                                                                0, 0, 0);
            Eigen::Affine3f transFinal = transBack * transIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(transFinal, x, y, z, roll, pitch, yaw);
            tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
            msg.pose.position.x = x;
            msg.pose.position.y = y;
            msg.pose.position.z = z;
            msg.pose.orientation.x = q.x();
            msg.pose.orientation.y = q.y();
            msg.pose.orientation.z = q.z();
            msg.pose.orientation.w = q.w();
            pubLocalizationPoses.at(index).publish(msg);
        }
    }

    void gripperEncoderPublishing(const int index){
        std::string time0 = "";
        int count = 0;
        while(ros::ok()){
            std::string time;
            if(publishIndex != -1){
                if(time0 == ""){
                    getline(syncFileGripperEncoders.at(index), time0);
                    if(count != publishIndex){
                        count++;
                        time0 = "";
                        continue;
                    }
                }
                time = time0;
            }else{
                if(!getline(syncFileGripperEncoders.at(index), time))
                    break;
            }
            sem_wait(&gripperEncoderSems.at(index));
            data_msgs::Gripper msg;
            msg.header.stamp = ros::Time().now();
            Json::Reader jsonReader;
            Json::Value root;
            std::ifstream file(gripperEncoderDirs.at(index) + "/" + time, std::iostream::binary);
            jsonReader.parse(file, root);
            msg.angle = root["angle"].asDouble();
            msg.distance = root["distance"].asDouble();
            pubGripperEncoders.at(index).publish(msg);
        }
    }

    void imu9AxisPublishing(const int index){
        std::string time0 = "";
        int count = 0;
        while(ros::ok()){
            std::string time;
            if(publishIndex != -1){
                if(time0 == ""){
                    getline(syncFileImu9Axiss.at(index), time0);
                    if(count != publishIndex){
                        count++;
                        time0 = "";
                        continue;
                    }
                }
                time = time0;
            }else{
                if(!getline(syncFileImu9Axiss.at(index), time))
                    break;
            }
            sem_wait(&imu9AxisSems.at(index));
            sensor_msgs::Imu msg;
            msg.header.stamp = ros::Time().now();
            Json::Reader jsonReader;
            Json::Value root;
            std::ifstream file(imu9AxisDirs.at(index) + "/" + time, std::iostream::binary);
            jsonReader.parse(file, root);
            msg.orientation.x = root["orientation"]["x"].asDouble();
            msg.orientation.y = root["orientation"]["y"].asDouble();
            msg.orientation.z = root["orientation"]["z"].asDouble();
            msg.orientation.w = root["orientation"]["w"].asDouble();
            msg.angular_velocity.x = root["angular_velocity"]["x"].asDouble();
            msg.angular_velocity.y = root["angular_velocity"]["y"].asDouble();
            msg.angular_velocity.z = root["angular_velocity"]["z"].asDouble();
            msg.linear_acceleration.x = root["linear_acceleration"]["x"].asDouble();
            msg.linear_acceleration.y = root["linear_acceleration"]["y"].asDouble();
            msg.linear_acceleration.z = root["linear_acceleration"]["z"].asDouble();
            pubImu9Axiss.at(index).publish(msg);
        }
    }

    void lidarPointCloudPublishing(const int index){
        std::string time0 = "";
        int count = 0;
        while(ros::ok()){
            std::string time;
            if(publishIndex != -1){
                if(time0 == ""){
                    getline(syncFileLidarPointClouds.at(index), time0);
                    if(count != publishIndex){
                        count++;
                        time0 = "";
                        continue;
                    }
                }
                time = time0;
            }else{
                if(!getline(syncFileLidarPointClouds.at(index), time))
                    break;
            }
            sem_wait(&lidarPointCloudSems.at(index));
            // pcl::PointCloud<pcl::PointXYZ> pointcloud;
            // pcl::io::loadPCDFile<pcl::PointXYZ>(lidarPointCloudDirs.at(index) + "-normalization" + "/" + time, pointcloud);
            pcl::PointCloud<pcl::PointXYZI> pointcloud;
            pcl::io::loadPCDFile<pcl::PointXYZI>(lidarPointCloudDirs.at(index) + "/" + time, pointcloud);
            sensor_msgs::PointCloud2 cloudMsg;
            pcl::toROSMsg(pointcloud, cloudMsg);
            cloudMsg.header.frame_id = "lidar";
            cloudMsg.header.stamp = ros::Time().now();
            pubLidarPointClouds.at(index).publish(cloudMsg);
        }
    }

    void robotBaseVelPublishing(const int index){
        std::string time0 = "";
        int count = 0;
        while(ros::ok()){
            std::string time;
            if(publishIndex != -1){
                if(time0 == ""){
                    getline(syncFileRobotBaseVels.at(index), time0);
                    if(count != publishIndex){
                        count++;
                        time0 = "";
                        continue;
                    }
                }
                time = time0;
            }else{
                if(!getline(syncFileRobotBaseVels.at(index), time))
                    break;
            }
            sem_wait(&robotBaseVelSems.at(index));
            nav_msgs::Odometry msg;
            msg.header.stamp = ros::Time().now();
            Json::Reader jsonReader;
            Json::Value root;
            std::ifstream file(robotBaseVelDirs.at(index) + "/" + time, std::iostream::binary);
            jsonReader.parse(file, root);
            msg.twist.twist.linear.x = root["linear"]["x"].asDouble();
            msg.twist.twist.linear.y = root["linear"]["y"].asDouble();
            msg.twist.twist.angular.z = root["angular"]["z"].asDouble();
            pubRobotBaseVels.at(index).publish(msg);
        }
    }

    void liftMotorPublishing(const int index){
        #ifdef _USELIFT
        std::string time0 = "";
        int count = 0;
        ros::ServiceClient client = nh.serviceClient<bt_task_msgs::LiftMotorSrv>(liftMotorPublishTopics.at(index));
        while(ros::ok()){
            std::string time;
            if(publishIndex != -1){
                if(time0 == ""){
                    getline(syncFileLiftMotors.at(index), time0);
                    if(count != publishIndex){
                        count++;
                        time0 = "";
                        continue;
                    }
                }
                time = time0;
            }else{
                if(!getline(syncFileLiftMotors.at(index), time))
                    break;
            }
            sem_wait(&liftMotorSems.at(index));
            Json::Reader jsonReader;
            Json::Value root;
            std::ifstream file(liftMotorDirs.at(index) + "/" + time, std::iostream::binary);
            jsonReader.parse(file, root);
            bt_task_msgs::LiftMotorSrv srv;
            srv.request.val = root["backHeight"].asDouble();
            srv.request.mode = 0;
            if (client.call(srv)) {

            } else {

            }
        }
        #endif
    }

    void tfTransformPublishing(const int index){
        Json::Reader jsonReader;
        Json::Value root;
        std::ifstream file(tfTransformDirs.at(index), std::iostream::binary);
        if(!file.good()){
            return;
        }
        jsonReader.parse(file, root);
        static tf::TransformBroadcaster tfBroadcaster;
        tf::Transform trans;
        geometry_msgs::Pose pose;
        pose.position.x = root["x"].asDouble();
        pose.position.y = root["y"].asDouble();
        pose.position.z = root["z"].asDouble();
        pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(root["roll"].asDouble(), root["pitch"].asDouble(), root["yaw"].asDouble());
        tf::poseMsgToTF(pose, trans);
        while(ros::ok()){
            sem_wait(&tfTransformSems.at(index));
            tf::StampedTransform tfTrans = tf::StampedTransform(trans, ros::Time().now(), tfTransformParentFrames.at(index), tfTransformChildFrames.at(index));
            tfBroadcaster.sendTransform(tfTrans);
        }
    }

    void activating(){
        ros::Rate rate(publishRate);
        while(ros::ok()){
            for(int i = 0; i < cameraColorNames.size(); i++){
                if(cameraColorToPublishs.at(i))
                    sem_post(&cameraColorSems.at(i));
            }
            for(int i = 0; i < cameraDepthNames.size(); i++){
                if(cameraDepthToPublishs.at(i))
                    sem_post(&cameraDepthSems.at(i));
            }
            for(int i = 0; i < cameraPointCloudNames.size(); i++){
                if(cameraPointCloudToPublishs.at(i))
                    sem_post(&cameraPointCloudSems.at(i));
            }
            for(int i = 0; i < armJointStateNames.size(); i++){
                if(armJointStateToPublishs.at(i))
                    sem_post(&armJointStateSems.at(i));
            }
            for(int i = 0; i < armEndPoseNames.size(); i++){
                if(armEndPoseToPublishs.at(i))
                    sem_post(&armEndPoseSems.at(i));
            }
            for(int i = 0; i < localizationPoseNames.size(); i++){
                if(localizationPoseToPublishs.at(i))
                    sem_post(&localizationPoseSems.at(i));
            }
            for(int i = 0; i < gripperEncoderNames.size(); i++){
                if(gripperEncoderToPublishs.at(i))
                    sem_post(&gripperEncoderSems.at(i));
            }
            for(int i = 0; i < imu9AxisNames.size(); i++){
                if(imu9AxisToPublishs.at(i))
                    sem_post(&imu9AxisSems.at(i));
            }
            for(int i = 0; i < lidarPointCloudNames.size(); i++){
                if(lidarPointCloudToPublishs.at(i))
                    sem_post(&lidarPointCloudSems.at(i));
            }
            for(int i = 0; i < robotBaseVelNames.size(); i++){
                if(robotBaseVelToPublishs.at(i))
                    sem_post(&robotBaseVelSems.at(i));
            }
            for(int i = 0; i < liftMotorNames.size(); i++){
                if(liftMotorToPublishs.at(i))
                    sem_post(&liftMotorSems.at(i));
            }
            for(int i = 0; i < tfTransformParentFrames.size(); i++){
                if(tfTransformToPublishs.at(i))
                    sem_post(&tfTransformSems.at(i));
            }
            rate.sleep();
        }
    }

    void join(){
        for(int i = 0; i < cameraColorPublishingThreads.size(); i++){
            cameraColorPublishingThreads.at(i)->join();
            delete cameraColorPublishingThreads.at(i);
            cameraColorPublishingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < cameraDepthPublishingThreads.size(); i++){
            cameraDepthPublishingThreads.at(i)->join();
            delete cameraDepthPublishingThreads.at(i);
            cameraDepthPublishingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < cameraPointCloudPublishingThreads.size(); i++){
            cameraPointCloudPublishingThreads.at(i)->join();
            delete cameraPointCloudPublishingThreads.at(i);
            cameraPointCloudPublishingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < armJointStatePublishingThreads.size(); i++){
            armJointStatePublishingThreads.at(i)->join();
            delete armJointStatePublishingThreads.at(i);
            armJointStatePublishingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < armEndPosePublishingThreads.size(); i++){
            armEndPosePublishingThreads.at(i)->join();
            delete armEndPosePublishingThreads.at(i);
            armEndPosePublishingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < localizationPosePublishingThreads.size(); i++){
            localizationPosePublishingThreads.at(i)->join();
            delete localizationPosePublishingThreads.at(i);
            localizationPosePublishingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < gripperEncoderPublishingThreads.size(); i++){
            gripperEncoderPublishingThreads.at(i)->join();
            delete gripperEncoderPublishingThreads.at(i);
            gripperEncoderPublishingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < imu9AxisPublishingThreads.size(); i++){
            imu9AxisPublishingThreads.at(i)->join();
            delete imu9AxisPublishingThreads.at(i);
            imu9AxisPublishingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < lidarPointCloudPublishingThreads.size(); i++){
            lidarPointCloudPublishingThreads.at(i)->join();
            delete lidarPointCloudPublishingThreads.at(i);
            lidarPointCloudPublishingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < robotBaseVelPublishingThreads.size(); i++){
            robotBaseVelPublishingThreads.at(i)->join();
            delete robotBaseVelPublishingThreads.at(i);
            robotBaseVelPublishingThreads.at(i) = nullptr;
        }
        for(int i = 0; i < liftMotorPublishingThreads.size(); i++){
            liftMotorPublishingThreads.at(i)->join();
            delete liftMotorPublishingThreads.at(i);
            liftMotorPublishingThreads.at(i) = nullptr;
        }
        ros::shutdown();
        for(int i = 0; i < tfTransformPublishingThreads.size(); i++){
            sem_post(&tfTransformSems.at(i));
            tfTransformPublishingThreads.at(i)->join();
            delete tfTransformPublishingThreads.at(i);
            tfTransformPublishingThreads.at(i) = nullptr;
        }
    }

    void run(){
        for(int i = 0; i < cameraColorNames.size(); i++){
            if(cameraColorToPublishs.at(i))
                cameraColorPublishingThreads.push_back(new std::thread(&DataPublish::cameraColorPublishing, this, i));
        }
        for(int i = 0; i < cameraDepthNames.size(); i++){
            if(cameraDepthToPublishs.at(i))
                cameraDepthPublishingThreads.push_back(new std::thread(&DataPublish::cameraDepthPublishing, this, i));
        }
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            if(cameraPointCloudToPublishs.at(i))
                cameraPointCloudPublishingThreads.push_back(new std::thread(&DataPublish::cameraPointCloudPublishing, this, i));
        }
        for(int i = 0; i < armJointStateNames.size(); i++){
            if(armJointStateToPublishs.at(i))
                armJointStatePublishingThreads.push_back(new std::thread(&DataPublish::armJointStatePublishing, this, i));
        }
        for(int i = 0; i < armEndPoseNames.size(); i++){
            if(armEndPoseToPublishs.at(i))
                armEndPosePublishingThreads.push_back(new std::thread(&DataPublish::armEndPosePublishing, this, i));
        }
        for(int i = 0; i < localizationPoseNames.size(); i++){
            if(localizationPoseToPublishs.at(i))
                localizationPosePublishingThreads.push_back(new std::thread(&DataPublish::localizationPosePublishing, this, i));
        }
        for(int i = 0; i < gripperEncoderNames.size(); i++){
            if(gripperEncoderToPublishs.at(i))
                gripperEncoderPublishingThreads.push_back(new std::thread(&DataPublish::gripperEncoderPublishing, this, i));
        }
        for(int i = 0; i < imu9AxisNames.size(); i++){
            if(imu9AxisToPublishs.at(i))
                imu9AxisPublishingThreads.push_back(new std::thread(&DataPublish::imu9AxisPublishing, this, i));
        }
        for(int i = 0; i < lidarPointCloudNames.size(); i++){
            if(lidarPointCloudToPublishs.at(i))
                lidarPointCloudPublishingThreads.push_back(new std::thread(&DataPublish::lidarPointCloudPublishing, this, i));
        }
        for(int i = 0; i < robotBaseVelNames.size(); i++){
            if(robotBaseVelToPublishs.at(i))
                robotBaseVelPublishingThreads.push_back(new std::thread(&DataPublish::robotBaseVelPublishing, this, i));
        }
        for(int i = 0; i < liftMotorNames.size(); i++){
            if(liftMotorToPublishs.at(i))
                liftMotorPublishingThreads.push_back(new std::thread(&DataPublish::liftMotorPublishing, this, i));
        }
        for(int i = 0; i < tfTransformParentFrames.size(); i++){
            if(tfTransformToPublishs.at(i))
                tfTransformPublishingThreads.push_back(new std::thread(&DataPublish::tfTransformPublishing, this, i));
        }
        activatingThread = new std::thread(&DataPublish::activating, this);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_sync");
    ros::NodeHandle nh;
    std::string datasetDir;
    int episodeIndex;
    float publishRate;
    int publishIndex;
    nh.param<std::string>("datasetDir", datasetDir, "/home/agilex/data");
    nh.param<int>("episodeIndex", episodeIndex, 0);
    nh.param<float>("publishRate", publishRate, 30);
    nh.param<int>("publishIndex", publishIndex, -1);
    ROS_INFO("\033[1;32m----> data publish Started.\033[0m");
    DataPublish dataPublish(datasetDir, episodeIndex, publishIndex, publishRate);
    dataPublish.run();
    dataPublish.join();
    std::cout<<"Done"<<std::endl;
    return 0;
}
