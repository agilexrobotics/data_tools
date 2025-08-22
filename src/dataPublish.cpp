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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/imu.hpp>
#include <data_msgs/msg/gripper.hpp>
#include "jsoncpp/json/json.h"
#ifdef _USELIFT
#include <bt_task_msgs/msg/lift_motor_msg.h>
#include <bt_task_msgs/srv/lift_motor_srv.h>
#endif

class DataPublish: public DataUtility{
public:
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> pubCameraColors;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> pubCameraDepths;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pubCameraPointClouds;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr> pubArmJointStates;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pubArmEndPoses;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pubLocalizationPoses;
    std::vector<rclcpp::Publisher<data_msgs::msg::Gripper>::SharedPtr> pubGripperEncoders;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> pubImu9Axiss;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pubLidarPointClouds;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> pubRobotBaseVels;
    #ifdef _USELIFT
    std::vector<rclcpp::Publisher<bt_task_msgs::msg::LiftMotorMsg>::SharedPtr> pubLiftMotors;
    #endif

    std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> pubCameraColorConfigs;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> pubCameraDepthConfigs;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> pubCameraPointCloudConfigs;
    // std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> pubArmJointStateConfigs;
    // std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> pubArmEndPoseConfigs;
    // std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> pubLocalizationPoseConfigs;
    // std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> pubGripperEncoderConfigs;
    // std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> pubImu9AxisConfigs;
    // std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> pubLidarPointCloudConfigs;
    // std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> pubRobotBaseVelConfigs;

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

    std::unique_ptr<tf2_ros::TransformBroadcaster> br;

    std::thread* activatingThread;

    float publishRate;
    int publishIndex;

    DataPublish(std::string name, const rclcpp::NodeOptions & options, std::string datasetDir, int episodeIndex, int publishIndexParam, float publishRateParam): DataUtility(name, options, datasetDir, episodeIndex) {
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

        br = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        for(int i = 0; i < cameraColorNames.size(); i++){
            pubCameraColors.push_back(create_publisher<sensor_msgs::msg::Image>(cameraColorPublishTopics[i], 2000));
            if(!cameraColorConfigPublishTopics.empty())
                pubCameraColorConfigs.push_back(create_publisher<sensor_msgs::msg::CameraInfo>(cameraColorConfigPublishTopics[i], 2000));
            if(cameraColorToPublishs.at(i)){
                syncFileCameraColors.at(i).open(cameraColorDirs.at(i)+"/sync.txt");
                sem_init(&cameraColorSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < cameraDepthNames.size(); i++){
            pubCameraDepths.push_back(create_publisher<sensor_msgs::msg::Image>(cameraDepthPublishTopics[i], 2000));
            if(!cameraDepthConfigPublishTopics.empty())
                pubCameraDepthConfigs.push_back(create_publisher<sensor_msgs::msg::CameraInfo>(cameraDepthConfigPublishTopics[i], 2000));
            if(cameraDepthToPublishs.at(i)){
                syncFileCameraDepths.at(i).open(cameraDepthDirs.at(i)+"/sync.txt");
                sem_init(&cameraDepthSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            pubCameraPointClouds.push_back(create_publisher<sensor_msgs::msg::PointCloud2>(cameraPointCloudPublishTopics[i], 2000));
            if(!cameraPointCloudConfigPublishTopics.empty())
                pubCameraPointCloudConfigs.push_back(create_publisher<sensor_msgs::msg::CameraInfo>(cameraPointCloudConfigPublishTopics[i], 2000));
            if(cameraPointCloudToPublishs.at(i)){
                syncFileCameraPointClouds.at(i).open(cameraPointCloudDirs.at(i)+"/sync.txt");
                sem_init(&cameraPointCloudSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < armJointStateNames.size(); i++){
            pubArmJointStates.push_back(create_publisher<sensor_msgs::msg::JointState>(armJointStatePublishTopics[i], 2000));
            if(armJointStateToPublishs.at(i)){
                syncFileArmJointStates.at(i).open(armJointStateDirs.at(i)+"/sync.txt");
                sem_init(&armJointStateSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < armEndPoseNames.size(); i++){
            pubArmEndPoses.push_back(create_publisher<geometry_msgs::msg::PoseStamped>(armEndPosePublishTopics[i], 2000));
            if(armEndPoseToPublishs.at(i)){
                syncFileArmEndPoses.at(i).open(armEndPoseDirs.at(i)+"/sync.txt");
                sem_init(&armEndPoseSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < localizationPoseNames.size(); i++){
            pubLocalizationPoses.push_back(create_publisher<geometry_msgs::msg::PoseStamped>(localizationPosePublishTopics[i], 2000));
            if(localizationPoseToPublishs.at(i)){
                syncFileLocalizationPoses.at(i).open(localizationPoseDirs.at(i)+"/sync.txt");
                sem_init(&localizationPoseSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < gripperEncoderNames.size(); i++){
            pubGripperEncoders.push_back(create_publisher<data_msgs::msg::Gripper>(gripperEncoderPublishTopics[i], 2000));
            if(gripperEncoderToPublishs.at(i)){
                syncFileGripperEncoders.at(i).open(gripperEncoderDirs.at(i)+"/sync.txt");
                sem_init(&gripperEncoderSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < imu9AxisNames.size(); i++){
            pubImu9Axiss.push_back(create_publisher<sensor_msgs::msg::Imu>(imu9AxisPublishTopics[i], 2000));
            if(imu9AxisToPublishs.at(i)){
                syncFileImu9Axiss.at(i).open(imu9AxisDirs.at(i)+"/sync.txt");
                sem_init(&imu9AxisSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < lidarPointCloudNames.size(); i++){
            pubLidarPointClouds.push_back(create_publisher<sensor_msgs::msg::PointCloud2>(lidarPointCloudPublishTopics[i], 2000));
            if(lidarPointCloudToPublishs.at(i)){
                syncFileLidarPointClouds.at(i).open(lidarPointCloudDirs.at(i)+"/sync.txt");
                sem_init(&lidarPointCloudSems.at(i), 0, 0);
            }
        }
        for(int i = 0; i < robotBaseVelNames.size(); i++){
            pubRobotBaseVels.push_back(create_publisher<nav_msgs::msg::Odometry>(robotBaseVelPublishTopics[i], 2000));
            if(robotBaseVelToPublishs.at(i)){
                syncFileRobotBaseVels.at(i).open(robotBaseVelDirs.at(i)+"/sync.txt");
                sem_init(&robotBaseVelSems.at(i), 0, 0);
            }
        }
        #ifdef _USELIFT
        for(int i = 0; i < liftMotorNames.size(); i++){
            pubLiftMotors.push_back(create_publisher<bt_task_msgs::msg::LiftMotorMsg>(liftMotorPublishTopics[i], 2000));
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
        sensor_msgs::msg::CameraInfo cameraInfo;
        cameraInfo.header.stamp = rclcpp::Clock().now();
        cameraInfo.header.frame_id = cameraColorParentFrames.at(index) + "_color";
        cameraInfo.height = root["height"].asInt();
        cameraInfo.width = root["width"].asInt();
        cameraInfo.distortion_model = root["distortion_model"].asString();
        Json::Value D = root["D"];
        for (int i = 0; i < D.size(); i++)
            cameraInfo.d.push_back(D[i].asDouble());
        Json::Value K = root["K"];
        for (int i = 0; i < K.size(); i++)
            cameraInfo.k[i] = K[i].asDouble();
        Json::Value R = root["R"];
        for (int i = 0; i < R.size(); i++)
            cameraInfo.r[i] = R[i].asDouble();
        Json::Value P = root["P"];
        for (int i = 0; i < P.size(); i++)
            cameraInfo.p[i] = P[i].asDouble();
        cameraInfo.binning_x = root["binning_x"].asInt();
        cameraInfo.binning_y = root["binning_y"].asInt();
        cameraInfo.roi.x_offset = root["roi"]["x_offset"].asInt();
        cameraInfo.roi.y_offset = root["roi"]["y_offset"].asInt();
        cameraInfo.roi.height = root["roi"]["height"].asInt();
        cameraInfo.roi.width = root["roi"]["width"].asInt();
        cameraInfo.roi.do_rectify = root["roi"]["do_rectify"].asBool();

        tf2::Quaternion q;
        q.setRPY(root["parent_frame"]["roll"].asDouble(), root["parent_frame"]["pitch"].asDouble(), root["parent_frame"]["yaw"].asDouble());
        geometry_msgs::msg::TransformStamped trans;
        trans.header.stamp = rclcpp::Clock().now();
        trans.header.frame_id = cameraColorParentFrames.at(index);
        trans.child_frame_id = cameraColorParentFrames.at(index) + "_color";
        trans.transform.translation.x = root["parent_frame"]["x"].asDouble();
        trans.transform.translation.y = root["parent_frame"]["y"].asDouble();
        trans.transform.translation.z = root["parent_frame"]["z"].asDouble();
        trans.transform.rotation.x = q.x();
        trans.transform.rotation.y = q.y();
        trans.transform.rotation.z = q.z();
        trans.transform.rotation.w = q.w();

        std::string time0 = "";
        int count = 0;
        while(rclcpp::ok()){
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
            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, image).toImageMsg();
            msg->header.stamp = rclcpp::Clock().now();
            msg->header.frame_id = cameraColorParentFrames.at(index) + "_color";
            pubCameraColors.at(index)->publish(*msg);

            cameraInfo.header.stamp = rclcpp::Clock().now();
            trans.header.stamp = rclcpp::Clock().now();
            pubCameraColorConfigs.at(index)->publish(cameraInfo);
            br->sendTransform(trans);
        }
    }

    void cameraDepthPublishing(const int index){
        Json::Reader jsonReader;
        Json::Value root;
        std::ifstream file(cameraDepthDirs.at(index)+"/config.json", std::iostream::binary);
        jsonReader.parse(file, root);
        sensor_msgs::msg::CameraInfo cameraInfo;
        cameraInfo.header.stamp = rclcpp::Clock().now();
        cameraInfo.header.frame_id = cameraDepthParentFrames.at(index) + "_depth";
        cameraInfo.height = root["height"].asInt();
        cameraInfo.width = root["width"].asInt();
        cameraInfo.distortion_model = root["distortion_model"].asString();
        Json::Value D = root["D"];
        for (int i = 0; i < D.size(); i++)
            cameraInfo.d.push_back(D[i].asDouble());
        Json::Value K = root["K"];
        for (int i = 0; i < K.size(); i++)
            cameraInfo.k[i] = K[i].asDouble();
        Json::Value R = root["R"];
        for (int i = 0; i < R.size(); i++)
            cameraInfo.r[i] = R[i].asDouble();
        Json::Value P = root["P"];
        for (int i = 0; i < P.size(); i++)
            cameraInfo.p[i] = P[i].asDouble();
        cameraInfo.binning_x = root["binning_x"].asInt();
        cameraInfo.binning_y = root["binning_y"].asInt();
        cameraInfo.roi.x_offset = root["roi"]["x_offset"].asInt();
        cameraInfo.roi.y_offset = root["roi"]["y_offset"].asInt();
        cameraInfo.roi.height = root["roi"]["height"].asInt();
        cameraInfo.roi.width = root["roi"]["width"].asInt();
        cameraInfo.roi.do_rectify = root["roi"]["do_rectify"].asBool();

        tf2::Quaternion q;
        q.setRPY(root["parent_frame"]["roll"].asDouble(), root["parent_frame"]["pitch"].asDouble(), root["parent_frame"]["yaw"].asDouble());
        geometry_msgs::msg::TransformStamped trans;
        trans.header.stamp = rclcpp::Clock().now();
        trans.header.frame_id = cameraDepthParentFrames.at(index);
        trans.child_frame_id = cameraDepthParentFrames.at(index) + "_depth";
        trans.transform.translation.x = root["parent_frame"]["x"].asDouble();
        trans.transform.translation.y = root["parent_frame"]["y"].asDouble();
        trans.transform.translation.z = root["parent_frame"]["z"].asDouble();
        trans.transform.rotation.x = q.x();
        trans.transform.rotation.y = q.y();
        trans.transform.rotation.z = q.z();
        trans.transform.rotation.w = q.w();

        std::string time0 = "";
        int count = 0;
        while(rclcpp::ok()){
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
            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, image).toImageMsg();
            msg->header.stamp = rclcpp::Clock().now();
            msg->header.frame_id = cameraDepthParentFrames.at(index) + "_depth";
            pubCameraDepths.at(index)->publish(*msg);

            cameraInfo.header.stamp = rclcpp::Clock().now();
            trans.header.stamp = rclcpp::Clock().now();
            pubCameraDepthConfigs.at(index)->publish(cameraInfo);
            br->sendTransform(trans);
        }
    }

    void cameraPointCloudPublishing(const int index){
        Json::Reader jsonReader;
        Json::Value root;
        std::ifstream file(cameraPointCloudDirs.at(index)+"/config.json", std::iostream::binary);
        jsonReader.parse(file, root);
        sensor_msgs::msg::CameraInfo cameraInfo;
        cameraInfo.header.stamp = rclcpp::Clock().now();
        cameraInfo.header.frame_id = cameraPointCloudParentFrames.at(index) + "_pointcloud";
        cameraInfo.height = root["height"].asInt();
        cameraInfo.width = root["width"].asInt();
        cameraInfo.distortion_model = root["distortion_model"].asString();
        Json::Value D = root["D"];
        for (int i = 0; i < D.size(); i++)
            cameraInfo.d.push_back(D[i].asDouble());
        Json::Value K = root["K"];
        for (int i = 0; i < K.size(); i++)
            cameraInfo.k[i] = K[i].asDouble();
        Json::Value R = root["R"];
        for (int i = 0; i < R.size(); i++)
            cameraInfo.r[i] = R[i].asDouble();
        Json::Value P = root["P"];
        for (int i = 0; i < P.size(); i++)
            cameraInfo.p[i] = P[i].asDouble();
        cameraInfo.binning_x = root["binning_x"].asInt();
        cameraInfo.binning_y = root["binning_y"].asInt();
        cameraInfo.roi.x_offset = root["roi"]["x_offset"].asInt();
        cameraInfo.roi.y_offset = root["roi"]["y_offset"].asInt();
        cameraInfo.roi.height = root["roi"]["height"].asInt();
        cameraInfo.roi.width = root["roi"]["width"].asInt();
        cameraInfo.roi.do_rectify = root["roi"]["do_rectify"].asBool();

        tf2::Quaternion q;
        q.setRPY(root["parent_frame"]["roll"].asDouble(), root["parent_frame"]["pitch"].asDouble(), root["parent_frame"]["yaw"].asDouble());
        geometry_msgs::msg::TransformStamped trans;
        trans.header.stamp = rclcpp::Clock().now();
        trans.header.frame_id = cameraPointCloudParentFrames.at(index);
        trans.child_frame_id = cameraPointCloudParentFrames.at(index) + "_pointcloud";
        trans.transform.translation.x = root["parent_frame"]["x"].asDouble();
        trans.transform.translation.y = root["parent_frame"]["y"].asDouble();
        trans.transform.translation.z = root["parent_frame"]["z"].asDouble();
        trans.transform.rotation.x = q.x();
        trans.transform.rotation.y = q.y();
        trans.transform.rotation.z = q.z();
        trans.transform.rotation.w = q.w();

        std::string time0 = "";
        int count = 0;
        while(rclcpp::ok()){
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
            sensor_msgs::msg::PointCloud2 cloudMsg;
            pcl::toROSMsg(pointcloud, cloudMsg);
            cloudMsg.header.stamp = rclcpp::Clock().now();
            cloudMsg.header.frame_id = cameraPointCloudParentFrames.at(index) + "_pointcloud";
            pubCameraPointClouds.at(index)->publish(cloudMsg);

            cameraInfo.header.stamp = rclcpp::Clock().now();
            trans.header.stamp = rclcpp::Clock().now();
            pubCameraPointCloudConfigs.at(index)->publish(cameraInfo);
            br->sendTransform(trans);
        }
    }

    void armJointStatePublishing(const int index){
        std::string time0 = "";
        int count = 0;
        while(rclcpp::ok()){
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
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = rclcpp::Clock().now();
            msg.position = positionData;
            pubArmJointStates.at(index)->publish(msg);
        }
    }

    void armEndPosePublishing(const int index){
        std::string time0 = "";
        int count = 0;
        while(rclcpp::ok()){
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
            geometry_msgs::msg::PoseStamped msg;
            msg.header.stamp = rclcpp::Clock().now();
            msg.header.frame_id = "map";
            Json::Reader jsonReader;
            Json::Value root;
            std::ifstream file(armEndPoseDirs.at(index) + "/" + time, std::iostream::binary);
            jsonReader.parse(file, root);
            if(armEndPoseOrients.at(index)){
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
                tf2::Quaternion quat_tf;
                quat_tf.setRPY(roll, pitch, yaw);
                geometry_msgs::msg::Quaternion quat_msg;
                tf2::convert(quat_tf, quat_msg);
                msg.pose.position.x = x;
                msg.pose.position.y = y;
                msg.pose.position.z = z;
                msg.pose.orientation = quat_msg;
            }else{
                msg.pose.position.x = root["x"].asDouble();
                msg.pose.position.y = root["y"].asDouble();
                msg.pose.position.z = root["z"].asDouble();
                msg.pose.orientation.x = root["roll"].asDouble();
                msg.pose.orientation.y = root["pitch"].asDouble();
                msg.pose.orientation.z = root["yaw"].asDouble();
                msg.pose.orientation.w = root["grasper"].asDouble();
            }
            pubArmEndPoses.at(index)->publish(msg);
        }
    }

    void localizationPosePublishing(const int index){
        std::string time0 = "";
        int count = 0;
        while(rclcpp::ok()){
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
            geometry_msgs::msg::PoseStamped msg;
            msg.header.stamp = rclcpp::Clock().now();
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
            tf2::Quaternion quat_tf;
            quat_tf.setRPY(roll, pitch, yaw);
            geometry_msgs::msg::Quaternion quat_msg;
            tf2::convert(quat_tf, quat_msg);
            msg.pose.position.x = x;
            msg.pose.position.y = y;
            msg.pose.position.z = z;
            msg.pose.orientation = quat_msg;
            pubLocalizationPoses.at(index)->publish(msg);
        }
    }

    void gripperEncoderPublishing(const int index){
        std::string time0 = "";
        int count = 0;
        while(rclcpp::ok()){
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
            data_msgs::msg::Gripper msg;
            msg.header.stamp = rclcpp::Clock().now();
            Json::Reader jsonReader;
            Json::Value root;
            std::ifstream file(gripperEncoderDirs.at(index) + "/" + time, std::iostream::binary);
            jsonReader.parse(file, root);
            msg.angle = root["angle"].asDouble();
            msg.distance = root["distance"].asDouble();
            pubGripperEncoders.at(index)->publish(msg);
        }
    }

    void imu9AxisPublishing(const int index){
        std::string time0 = "";
        int count = 0;
        while(rclcpp::ok()){
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
            sensor_msgs::msg::Imu msg;
            msg.header.stamp = rclcpp::Clock().now();
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
            pubImu9Axiss.at(index)->publish(msg);
        }
    }

    void lidarPointCloudPublishing(const int index){
        std::string time0 = "";
        int count = 0;
        while(rclcpp::ok()){
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
            sensor_msgs::msg::PointCloud2 cloudMsg;
            pcl::toROSMsg(pointcloud, cloudMsg);
            cloudMsg.header.frame_id = "lidar";
            cloudMsg.header.stamp = rclcpp::Clock().now();
            pubLidarPointClouds.at(index)->publish(cloudMsg);
        }
    }

    void robotBaseVelPublishing(const int index){
        std::string time0 = "";
        int count = 0;
        while(rclcpp::ok()){
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
            nav_msgs::msg::Odometry msg;
            msg.header.stamp = rclcpp::Clock().now();
            Json::Reader jsonReader;
            Json::Value root;
            std::ifstream file(robotBaseVelDirs.at(index) + "/" + time, std::iostream::binary);
            jsonReader.parse(file, root);
            msg.twist.twist.linear.x = root["linear"]["x"].asDouble();
            msg.twist.twist.linear.y = root["linear"]["y"].asDouble();
            msg.twist.twist.angular.z = root["angular"]["z"].asDouble();
            pubRobotBaseVels.at(index)->publish(msg);
        }
    }

    void liftMotorPublishing(const int index){
        #ifdef _USELIFT
        std::string time0 = "";
        int count = 0;
        rclcpp::Client<bt_task_msgs::srv::LiftMotorSrv>::SharedPtr client = this->create_client<bt_task_msgs::srv::LiftMotorSrv>(liftMotorPublishTopics.at(index));
        while(rclcpp::ok()){
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
            auto request = std::make_shared<bt_task_msgs::srv::LiftMotorSrv::Request>(); 
            request->val = root["backHeight"].asDouble();
            request->mode = 0;
            auto result = client->async_send_request(request);
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
        tf2::Quaternion q;
        q.setRPY(root["roll"].asDouble(), root["pitch"].asDouble(), root["yaw"].asDouble());
        geometry_msgs::msg::TransformStamped trans;
        trans.header.stamp = rclcpp::Clock().now();
        trans.header.frame_id = tfTransformParentFrames.at(index);
        trans.child_frame_id = tfTransformChildFrames.at(index);
        trans.transform.translation.x = root["x"].asDouble();
        trans.transform.translation.y = root["y"].asDouble();
        trans.transform.translation.z = root["z"].asDouble();
        trans.transform.rotation.x = q.x();
        trans.transform.rotation.y = q.y();
        trans.transform.rotation.z = q.z();
        trans.transform.rotation.w = q.w();
        while(rclcpp::ok()){
            sem_wait(&tfTransformSems.at(index));
            trans.header.stamp = rclcpp::Clock().now();
            br->sendTransform(trans);
        }
    }

    void activating(){
        rclcpp::Rate rate(publishRate);
        while(rclcpp::ok()){
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
        rclcpp::shutdown();
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


class DataPublishService: public rclcpp::Node{
    public:
    rclcpp::executors::SingleThreadedExecutor *exec;
    std::shared_ptr<DataPublish> dataPublish;
    std::string datasetDir;
    int episodeIndex;
    int publishIndex;
    int publishRate;
    rclcpp::NodeOptions options;
    std::string name;
    DataPublishService(std::string name, const rclcpp::NodeOptions & options): rclcpp::Node(name, options) {
        exec = nullptr;
        this->options = options;
        this->name = name;
        declare_parameter("datasetDir", "/home/agilex/data");get_parameter("datasetDir", datasetDir);
        declare_parameter("episodeIndex", 0);get_parameter("episodeIndex", episodeIndex);
        declare_parameter("publishIndex", -1);get_parameter("publishIndex", publishIndex);
        declare_parameter("publishRate", 30);get_parameter("publishRate", publishRate);
        exec = new rclcpp::executors::SingleThreadedExecutor;
        std::string workerName = name + "_worker_" + std::to_string(rclcpp::Clock().now().nanoseconds());
        dataPublish = std::make_shared<DataPublish>(workerName, options, datasetDir, episodeIndex, publishIndex, publishRate);
        exec->add_node(dataPublish);
        ((DataPublish *)dataPublish.get())->run();
        exec->spin();
        ((DataPublish *)dataPublish.get())->join();
        rclcpp::shutdown();
        std::cout<<"Done"<<std::endl;
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> data publish Started.\033[0m");
    rclcpp::executors::SingleThreadedExecutor exec;
    auto dataPublishService = std::make_shared<DataPublishService>("data_publish", options);
    exec.add_node(dataPublishService);
    exec.spin();
    rclcpp::shutdown();
    std::cout<<"Done"<<std::endl;
    return 0;
}
