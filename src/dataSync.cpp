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
// #include <filesystem>
#include <boost/filesystem.hpp>

class TimeSeries{
public:
    double time;
    std::vector<TimeSeries>* dataList;
    std::vector<TimeSeries>* syncList;

    TimeSeries(double time, std::vector<TimeSeries>* dataList, std::vector<TimeSeries>* syncList){
        this->dataList = dataList;
        this->syncList = syncList;
        this->time = time;
    }

    ~TimeSeries(){
        this->dataList = nullptr;
        this->syncList = nullptr;
    }

    void toDataList(){
        this->dataList->push_back(*this);
    }

    void toSyncList(){
        this->syncList->push_back(*this);
    }
};

class DataSync: public DataUtility{
public:
    std::vector<TimeSeries> allTimeSeries;

    std::vector<std::vector<TimeSeries>> cameraColorDataTimeSeries;
    std::vector<std::vector<TimeSeries>> cameraDepthDataTimeSeries;
    std::vector<std::vector<TimeSeries>> cameraPointCloudDataTimeSeries;
    std::vector<std::vector<TimeSeries>> armJointStateDataTimeSeries;
    std::vector<std::vector<TimeSeries>> armEndPoseDataTimeSeries;
    std::vector<std::vector<TimeSeries>> localizationPoseDataTimeSeries;
    std::vector<std::vector<TimeSeries>> gripperEncoderDataTimeSeries;
    std::vector<std::vector<TimeSeries>> imu9AxisDataTimeSeries;
    std::vector<std::vector<TimeSeries>> lidarPointCloudDataTimeSeries;
    std::vector<std::vector<TimeSeries>> robotBaseVelDataTimeSeries;
    std::vector<std::vector<TimeSeries>> liftMotorDataTimeSeries;

    std::vector<std::vector<TimeSeries>> cameraColorSyncTimeSeries;
    std::vector<std::vector<TimeSeries>> cameraDepthSyncTimeSeries;
    std::vector<std::vector<TimeSeries>> cameraPointCloudSyncTimeSeries;
    std::vector<std::vector<TimeSeries>> armJointStateSyncTimeSeries;
    std::vector<std::vector<TimeSeries>> armEndPoseSyncTimeSeries;
    std::vector<std::vector<TimeSeries>> localizationPoseSyncTimeSeries;
    std::vector<std::vector<TimeSeries>> gripperEncoderSyncTimeSeries;
    std::vector<std::vector<TimeSeries>> imu9AxisSyncTimeSeries;
    std::vector<std::vector<TimeSeries>> lidarPointCloudSyncTimeSeries;
    std::vector<std::vector<TimeSeries>> robotBaseVelSyncTimeSeries;
    std::vector<std::vector<TimeSeries>> liftMotorSyncTimeSeries;

    std::vector<std::string> cameraColorExts;
    std::vector<std::string> cameraDepthExts;
    std::vector<std::string> cameraPointCloudExts;
    std::vector<std::string> armJointStateExts;
    std::vector<std::string> armEndPoseExts;
    std::vector<std::string> localizationPoseExts;
    std::vector<std::string> gripperEncoderExts;
    std::vector<std::string> imu9AxisExts;
    std::vector<std::string> lidarPointCloudExts;
    std::vector<std::string> robotBaseVelExts;
    std::vector<std::string> liftMotorExts;

    double timeDiffLimit;

    DataSync(std::string datasetDir, int episodeIndex, double timeDiffLimit): DataUtility(datasetDir, episodeIndex) {
        this->timeDiffLimit = timeDiffLimit;
        cameraColorDataTimeSeries = std::vector<std::vector<TimeSeries>>(cameraColorNames.size());
        cameraDepthDataTimeSeries = std::vector<std::vector<TimeSeries>>(cameraDepthNames.size());
        cameraPointCloudDataTimeSeries = std::vector<std::vector<TimeSeries>>(cameraPointCloudNames.size());
        armJointStateDataTimeSeries = std::vector<std::vector<TimeSeries>>(armJointStateNames.size());
        armEndPoseDataTimeSeries = std::vector<std::vector<TimeSeries>>(armEndPoseNames.size());
        localizationPoseDataTimeSeries = std::vector<std::vector<TimeSeries>>(localizationPoseNames.size());
        gripperEncoderDataTimeSeries = std::vector<std::vector<TimeSeries>>(gripperEncoderNames.size());
        imu9AxisDataTimeSeries = std::vector<std::vector<TimeSeries>>(imu9AxisNames.size());
        lidarPointCloudDataTimeSeries = std::vector<std::vector<TimeSeries>>(lidarPointCloudNames.size());
        robotBaseVelDataTimeSeries = std::vector<std::vector<TimeSeries>>(robotBaseVelNames.size());
        liftMotorDataTimeSeries = std::vector<std::vector<TimeSeries>>(liftMotorNames.size());

        cameraColorSyncTimeSeries = std::vector<std::vector<TimeSeries>>(cameraColorNames.size());
        cameraDepthSyncTimeSeries = std::vector<std::vector<TimeSeries>>(cameraDepthNames.size());
        cameraPointCloudSyncTimeSeries = std::vector<std::vector<TimeSeries>>(cameraPointCloudNames.size());
        armJointStateSyncTimeSeries = std::vector<std::vector<TimeSeries>>(armJointStateNames.size());
        armEndPoseSyncTimeSeries = std::vector<std::vector<TimeSeries>>(armEndPoseNames.size());
        localizationPoseSyncTimeSeries = std::vector<std::vector<TimeSeries>>(localizationPoseNames.size());
        gripperEncoderSyncTimeSeries = std::vector<std::vector<TimeSeries>>(gripperEncoderNames.size());
        imu9AxisSyncTimeSeries = std::vector<std::vector<TimeSeries>>(imu9AxisNames.size());
        lidarPointCloudSyncTimeSeries = std::vector<std::vector<TimeSeries>>(lidarPointCloudNames.size());
        robotBaseVelSyncTimeSeries = std::vector<std::vector<TimeSeries>>(robotBaseVelNames.size());
        liftMotorSyncTimeSeries = std::vector<std::vector<TimeSeries>>(liftMotorNames.size());

        cameraColorExts = std::vector<std::string>(cameraColorNames.size());
        cameraDepthExts = std::vector<std::string>(cameraDepthNames.size());
        cameraPointCloudExts = std::vector<std::string>(cameraPointCloudNames.size());
        armJointStateExts = std::vector<std::string>(armJointStateNames.size());
        armEndPoseExts = std::vector<std::string>(armEndPoseNames.size());
        localizationPoseExts = std::vector<std::string>(localizationPoseNames.size());
        gripperEncoderExts = std::vector<std::string>(gripperEncoderNames.size());
        imu9AxisExts = std::vector<std::string>(imu9AxisNames.size());
        lidarPointCloudExts = std::vector<std::string>(lidarPointCloudNames.size());
        robotBaseVelExts = std::vector<std::string>(robotBaseVelNames.size());
        liftMotorExts = std::vector<std::string>(liftMotorNames.size());

        for(int i = 0; i < cameraColorNames.size(); i++){
            int count = getFileInPath(cameraColorDirs.at(i), ".jpg", &cameraColorDataTimeSeries.at(i), &cameraColorSyncTimeSeries.at(i));
            if(count == 0){
                getFileInPath(cameraColorDirs.at(i), ".png", &cameraColorDataTimeSeries.at(i), &cameraColorSyncTimeSeries.at(i));
                cameraColorExts.at(i) = ".png";
            }else{
                cameraColorExts.at(i) = ".jpg";
            }
        }
        for(int i = 0; i < cameraDepthNames.size(); i++){
            getFileInPath(cameraDepthDirs.at(i), ".png", &cameraDepthDataTimeSeries.at(i), &cameraDepthSyncTimeSeries.at(i));
            cameraDepthExts.at(i) = ".png";
        }
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            getFileInPath(cameraPointCloudDirs.at(i), ".pcd", &cameraPointCloudDataTimeSeries.at(i), &cameraPointCloudSyncTimeSeries.at(i));
            cameraPointCloudExts.at(i) = ".pcd";
        }
        for(int i = 0; i < armJointStateNames.size(); i++){
            getFileInPath(armJointStateDirs.at(i), ".json", &armJointStateDataTimeSeries.at(i), &armJointStateSyncTimeSeries.at(i));
            armJointStateExts.at(i) = ".json";
        }
        for(int i = 0; i < armEndPoseNames.size(); i++){
            getFileInPath(armEndPoseDirs.at(i), ".json", &armEndPoseDataTimeSeries.at(i), &armEndPoseSyncTimeSeries.at(i));
            armEndPoseExts.at(i) = ".json";
        }
        for(int i = 0; i < localizationPoseNames.size(); i++){
            getFileInPath(localizationPoseDirs.at(i), ".json", &localizationPoseDataTimeSeries.at(i), &localizationPoseSyncTimeSeries.at(i));
            localizationPoseExts.at(i) = ".json";
        }
        for(int i = 0; i < gripperEncoderNames.size(); i++){
            getFileInPath(gripperEncoderDirs.at(i), ".json", &gripperEncoderDataTimeSeries.at(i), &gripperEncoderSyncTimeSeries.at(i));
            gripperEncoderExts.at(i) = ".json";
        }
        for(int i = 0; i < imu9AxisNames.size(); i++){
            getFileInPath(imu9AxisDirs.at(i), ".json", &imu9AxisDataTimeSeries.at(i), &imu9AxisSyncTimeSeries.at(i));
            imu9AxisExts.at(i) = ".json";
        }
        for(int i = 0; i < lidarPointCloudNames.size(); i++){
            getFileInPath(lidarPointCloudDirs.at(i), ".json", &lidarPointCloudDataTimeSeries.at(i), &lidarPointCloudSyncTimeSeries.at(i));
            lidarPointCloudExts.at(i) = ".json";
        }
        for(int i = 0; i < robotBaseVelNames.size(); i++){
            getFileInPath(robotBaseVelDirs.at(i), ".json", &robotBaseVelDataTimeSeries.at(i), &robotBaseVelSyncTimeSeries.at(i));
            robotBaseVelExts.at(i) = ".json";
        }
        for(int i = 0; i < liftMotorNames.size(); i++){
            getFileInPath(liftMotorDirs.at(i), ".json", &liftMotorDataTimeSeries.at(i), &liftMotorSyncTimeSeries.at(i));
            liftMotorExts.at(i) = ".json";
        }
        std::sort(allTimeSeries.begin(), allTimeSeries.end(), [](const TimeSeries& a, const TimeSeries& b){
            return a.time < b.time;
        });
    }

    void sync(){
        int frameCount = 0;
        std::cout<<"allTimeSeries:"<<allTimeSeries.size()<<std::endl;
        for(int i = 0; i < allTimeSeries.size(); i++){
            allTimeSeries.at(i).toDataList();
            double frameTime = checkDataAdequacy();
            if(frameTime != INFINITY){
                std::vector<double> cameraColorCloserIndexs = std::vector<double>(cameraColorNames.size(), 0);
                std::vector<double> cameraDepthCloserIndexs = std::vector<double>(cameraDepthNames.size(), 0);
                std::vector<double> cameraPointCloudCloserIndexs = std::vector<double>(cameraPointCloudNames.size(), 0);
                std::vector<double> armJointStateCloserIndexs = std::vector<double>(armJointStateNames.size(), 0);
                std::vector<double> armEndPoseCloserIndexs = std::vector<double>(armEndPoseNames.size(), 0);
                std::vector<double> localizationPoseCloserIndexs = std::vector<double>(localizationPoseNames.size(), 0);
                std::vector<double> gripperEncoderCloserIndexs = std::vector<double>(gripperEncoderNames.size(), 0);
                std::vector<double> imu9AxisCloserIndexs = std::vector<double>(imu9AxisNames.size(), 0);
                std::vector<double> lidarPointCloudCloserIndexs = std::vector<double>(lidarPointCloudNames.size(), 0);
                std::vector<double> robotBaseVelCloserIndexs = std::vector<double>(robotBaseVelNames.size(), 0);
                std::vector<double> liftMotorCloserIndexs = std::vector<double>(liftMotorNames.size(), 0);
                // frameTime = allTimeSeries.at(i).time;
                bool timeDiffPass = true;
                for(int i = 0; i < cameraColorNames.size() && timeDiffPass; i++){
                    if(!cameraColorToSyncs.at(i))
                        continue;
                    int closerIndex = 0;
                    double closerTimeDiff = INFINITY;
                    for(int j = 0; j < cameraColorDataTimeSeries.at(i).size(); j++){
                        double timeDiff = fabs(cameraColorDataTimeSeries.at(i).at(j).time - frameTime);
                        if(timeDiff < closerTimeDiff){
                            closerTimeDiff = timeDiff;
                            closerIndex = j;
                        }
                    }
                    if(closerTimeDiff > timeDiffLimit)
                        timeDiffPass = false;
                    cameraColorCloserIndexs.at(i) = closerIndex;
                }
                for(int i = 0; i < cameraDepthNames.size() && timeDiffPass; i++){
                    if(!cameraDepthToSyncs.at(i))
                        continue;
                    int closerIndex = 0;
                    double closerTimeDiff = INFINITY;
                    for(int j = 0; j < cameraDepthDataTimeSeries.at(i).size(); j++){
                        double timeDiff = fabs(cameraDepthDataTimeSeries.at(i).at(j).time - frameTime);
                        if(timeDiff < closerTimeDiff){
                            closerTimeDiff = timeDiff;
                            closerIndex = j;
                        }
                    }
                    if(closerTimeDiff > timeDiffLimit)
                        timeDiffPass = false;
                    cameraDepthCloserIndexs.at(i) = closerIndex;
                }
                for(int i = 0; i < cameraPointCloudNames.size() && timeDiffPass; i++){
                    if(!cameraPointCloudToSyncs.at(i))
                        continue;
                    int closerIndex = 0;
                    double closerTimeDiff = INFINITY;
                    for(int j = 0; j < cameraPointCloudDataTimeSeries.at(i).size(); j++){
                        double timeDiff = fabs(cameraPointCloudDataTimeSeries.at(i).at(j).time - frameTime);
                        if(timeDiff < closerTimeDiff){
                            closerTimeDiff = timeDiff;
                            closerIndex = j;
                        }
                    }
                    if(closerTimeDiff > timeDiffLimit)
                        timeDiffPass = false;
                    cameraPointCloudCloserIndexs.at(i) = closerIndex;
                }
                for(int i = 0; i < armJointStateNames.size() && timeDiffPass; i++){
                    if(!armJointStateToSyncs.at(i))
                        continue;
                    int closerIndex = 0;
                    double closerTimeDiff = INFINITY;
                    for(int j = 0; j < armJointStateDataTimeSeries.at(i).size(); j++){
                        double timeDiff = fabs(armJointStateDataTimeSeries.at(i).at(j).time - frameTime);
                        if(timeDiff < closerTimeDiff){
                            closerTimeDiff = timeDiff;
                            closerIndex = j;
                        }
                    }
                    if(closerTimeDiff > timeDiffLimit)
                        timeDiffPass = false;
                    armJointStateCloserIndexs.at(i) = closerIndex;
                }
                for(int i = 0; i < armEndPoseNames.size() && timeDiffPass; i++){
                    if(!armEndPoseToSyncs.at(i))
                        continue;
                    int closerIndex = 0;
                    double closerTimeDiff = INFINITY;
                    for(int j = 0; j < armEndPoseDataTimeSeries.at(i).size(); j++){
                        double timeDiff = fabs(armEndPoseDataTimeSeries.at(i).at(j).time - frameTime);
                        if(timeDiff < closerTimeDiff){
                            closerTimeDiff = timeDiff;
                            closerIndex = j;
                        }
                    }
                    if(closerTimeDiff > timeDiffLimit)
                        timeDiffPass = false;
                    armEndPoseCloserIndexs.at(i) = closerIndex;
                }
                for(int i = 0; i < localizationPoseNames.size() && timeDiffPass; i++){
                    if(!localizationPoseToSyncs.at(i))
                        continue;
                    int closerIndex = 0;
                    double closerTimeDiff = INFINITY;
                    for(int j = 0; j < localizationPoseDataTimeSeries.at(i).size(); j++){
                        double timeDiff = fabs(localizationPoseDataTimeSeries.at(i).at(j).time - frameTime);
                        if(timeDiff < closerTimeDiff){
                            closerTimeDiff = timeDiff;
                            closerIndex = j;
                        }
                    }
                    if(closerTimeDiff > timeDiffLimit)
                        timeDiffPass = false;
                    localizationPoseCloserIndexs.at(i) = closerIndex;
                }
                for(int i = 0; i < gripperEncoderNames.size() && timeDiffPass; i++){
                    if(!gripperEncoderToSyncs.at(i))
                        continue;
                    int closerIndex = 0;
                    double closerTimeDiff = INFINITY;
                    for(int j = 0; j < gripperEncoderDataTimeSeries.at(i).size(); j++){
                        double timeDiff = fabs(gripperEncoderDataTimeSeries.at(i).at(j).time - frameTime);
                        if(timeDiff < closerTimeDiff){
                            closerTimeDiff = timeDiff;
                            closerIndex = j;
                        }
                    }
                    if(closerTimeDiff > timeDiffLimit)
                        timeDiffPass = false;
                    gripperEncoderCloserIndexs.at(i) = closerIndex;
                }
                for(int i = 0; i < imu9AxisNames.size() && timeDiffPass; i++){
                    if(!imu9AxisToSyncs.at(i))
                        continue;
                    int closerIndex = 0;
                    double closerTimeDiff = INFINITY;
                    for(int j = 0; j < imu9AxisDataTimeSeries.at(i).size(); j++){
                        double timeDiff = fabs(imu9AxisDataTimeSeries.at(i).at(j).time - frameTime);
                        if(timeDiff < closerTimeDiff){
                            closerTimeDiff = timeDiff;
                            closerIndex = j;
                        }
                    }
                    if(closerTimeDiff > timeDiffLimit)
                        timeDiffPass = false;
                    imu9AxisCloserIndexs.at(i) = closerIndex;
                }
                for(int i = 0; i < lidarPointCloudNames.size() && timeDiffPass; i++){
                    if(!lidarPointCloudToSyncs.at(i))
                        continue;
                    int closerIndex = 0;
                    double closerTimeDiff = INFINITY;
                    for(int j = 0; j < lidarPointCloudDataTimeSeries.at(i).size(); j++){
                        double timeDiff = fabs(lidarPointCloudDataTimeSeries.at(i).at(j).time - frameTime);
                        if(timeDiff < closerTimeDiff){
                            closerTimeDiff = timeDiff;
                            closerIndex = j;
                        }
                    }
                    if(closerTimeDiff > timeDiffLimit)
                        timeDiffPass = false;
                    lidarPointCloudCloserIndexs.at(i) = closerIndex;
                }
                for(int i = 0; i < robotBaseVelNames.size() && timeDiffPass; i++){
                    if(!robotBaseVelToSyncs.at(i))
                        continue;
                    int closerIndex = 0;
                    double closerTimeDiff = INFINITY;
                    for(int j = 0; j < robotBaseVelDataTimeSeries.at(i).size(); j++){
                        double timeDiff = fabs(robotBaseVelDataTimeSeries.at(i).at(j).time - frameTime);
                        if(timeDiff < closerTimeDiff){
                            closerTimeDiff = timeDiff;
                            closerIndex = j;
                        }
                    }
                    if(closerTimeDiff > timeDiffLimit)
                        timeDiffPass = false;
                    robotBaseVelCloserIndexs.at(i) = closerIndex;
                }
                for(int i = 0; i < liftMotorNames.size() && timeDiffPass; i++){
                    if(!liftMotorToSyncs.at(i))
                        continue;
                    int closerIndex = 0;
                    double closerTimeDiff = INFINITY;
                    for(int j = 0; j < liftMotorDataTimeSeries.at(i).size(); j++){
                        double timeDiff = fabs(liftMotorDataTimeSeries.at(i).at(j).time - frameTime);
                        if(timeDiff < closerTimeDiff){
                            closerTimeDiff = timeDiff;
                            closerIndex = j;
                        }
                    }
                    if(closerTimeDiff > timeDiffLimit)
                        timeDiffPass = false;
                    liftMotorCloserIndexs.at(i) = closerIndex;
                }
                if(!timeDiffPass)
                    continue;
                for(int i = 0; i < cameraColorNames.size(); i++){
                    if(!cameraColorToSyncs.at(i))
                        continue;
                    cameraColorDataTimeSeries.at(i).at(cameraColorCloserIndexs.at(i)).toSyncList();
                    cameraColorDataTimeSeries.at(i).erase(cameraColorDataTimeSeries.at(i).begin(), cameraColorDataTimeSeries.at(i).begin() + cameraColorCloserIndexs.at(i) + 1);
                }
                for(int i = 0; i < cameraDepthNames.size(); i++){
                    if(!cameraDepthToSyncs.at(i))
                        continue;
                    cameraDepthDataTimeSeries.at(i).at(cameraDepthCloserIndexs.at(i)).toSyncList();
                    cameraDepthDataTimeSeries.at(i).erase(cameraDepthDataTimeSeries.at(i).begin(), cameraDepthDataTimeSeries.at(i).begin() + cameraDepthCloserIndexs.at(i) + 1);
                }
                for(int i = 0; i < cameraPointCloudNames.size(); i++){
                    if(!cameraPointCloudToSyncs.at(i))
                        continue;
                    cameraPointCloudDataTimeSeries.at(i).at(cameraPointCloudCloserIndexs.at(i)).toSyncList();
                    cameraPointCloudDataTimeSeries.at(i).erase(cameraPointCloudDataTimeSeries.at(i).begin(), cameraPointCloudDataTimeSeries.at(i).begin() + cameraPointCloudCloserIndexs.at(i) + 1);
                }
                for(int i = 0; i < armJointStateNames.size(); i++){
                    if(!armJointStateToSyncs.at(i))
                        continue;
                    armJointStateDataTimeSeries.at(i).at(armJointStateCloserIndexs.at(i)).toSyncList();
                    armJointStateDataTimeSeries.at(i).erase(armJointStateDataTimeSeries.at(i).begin(), armJointStateDataTimeSeries.at(i).begin() + armJointStateCloserIndexs.at(i) + 1);
                }
                for(int i = 0; i < armEndPoseNames.size(); i++){
                    if(!armEndPoseToSyncs.at(i))
                        continue;
                    armEndPoseDataTimeSeries.at(i).at(armEndPoseCloserIndexs.at(i)).toSyncList();
                    armEndPoseDataTimeSeries.at(i).erase(armEndPoseDataTimeSeries.at(i).begin(), armEndPoseDataTimeSeries.at(i).begin() + armEndPoseCloserIndexs.at(i) + 1);
                }
                for(int i = 0; i < localizationPoseNames.size(); i++){
                    if(!localizationPoseToSyncs.at(i))
                        continue;
                    localizationPoseDataTimeSeries.at(i).at(localizationPoseCloserIndexs.at(i)).toSyncList();
                    localizationPoseDataTimeSeries.at(i).erase(localizationPoseDataTimeSeries.at(i).begin(), localizationPoseDataTimeSeries.at(i).begin() + localizationPoseCloserIndexs.at(i) + 1);
                }
                for(int i = 0; i < gripperEncoderNames.size(); i++){
                    if(!gripperEncoderToSyncs.at(i))
                        continue;
                    gripperEncoderDataTimeSeries.at(i).at(gripperEncoderCloserIndexs.at(i)).toSyncList();
                    gripperEncoderDataTimeSeries.at(i).erase(gripperEncoderDataTimeSeries.at(i).begin(), gripperEncoderDataTimeSeries.at(i).begin() + gripperEncoderCloserIndexs.at(i) + 1);
                }
                for(int i = 0; i < imu9AxisNames.size(); i++){
                    if(!imu9AxisToSyncs.at(i))
                        continue;
                    imu9AxisDataTimeSeries.at(i).at(imu9AxisCloserIndexs.at(i)).toSyncList();
                    imu9AxisDataTimeSeries.at(i).erase(imu9AxisDataTimeSeries.at(i).begin(), imu9AxisDataTimeSeries.at(i).begin() + imu9AxisCloserIndexs.at(i) + 1);
                }
                for(int i = 0; i < lidarPointCloudNames.size(); i++){
                    if(!lidarPointCloudToSyncs.at(i))
                        continue;
                    lidarPointCloudDataTimeSeries.at(i).at(lidarPointCloudCloserIndexs.at(i)).toSyncList();
                    lidarPointCloudDataTimeSeries.at(i).erase(lidarPointCloudDataTimeSeries.at(i).begin(), lidarPointCloudDataTimeSeries.at(i).begin() + lidarPointCloudCloserIndexs.at(i) + 1);
                }
                for(int i = 0; i < robotBaseVelNames.size(); i++){
                    if(!robotBaseVelToSyncs.at(i))
                        continue;
                    robotBaseVelDataTimeSeries.at(i).at(robotBaseVelCloserIndexs.at(i)).toSyncList();
                    robotBaseVelDataTimeSeries.at(i).erase(robotBaseVelDataTimeSeries.at(i).begin(), robotBaseVelDataTimeSeries.at(i).begin() + robotBaseVelCloserIndexs.at(i) + 1);
                }
                for(int i = 0; i < liftMotorNames.size(); i++){
                    if(!liftMotorToSyncs.at(i))
                        continue;
                    liftMotorDataTimeSeries.at(i).at(liftMotorCloserIndexs.at(i)).toSyncList();
                    liftMotorDataTimeSeries.at(i).erase(liftMotorDataTimeSeries.at(i).begin(), liftMotorDataTimeSeries.at(i).begin() + liftMotorCloserIndexs.at(i) + 1);
                }
                frameCount += 1;
            }
        }
        std::cout<<"sync frame num:"<<frameCount<<std::endl;
        if (frameCount == 0)
            checkDataAdequacy(true);
        for(int i = 0; i < cameraColorNames.size(); i++){
            std::ofstream file(cameraColorDirs.at(i) + "/sync.txt");
            for(int j = 0; j < cameraColorSyncTimeSeries.at(i).size(); j++)
                file<<std::to_string(cameraColorSyncTimeSeries.at(i).at(j).time)<<cameraColorExts.at(i)<<std::endl;
            file.close();
        }
        for(int i = 0; i < cameraDepthNames.size(); i++){
            std::ofstream file(cameraDepthDirs.at(i) + "/sync.txt");
            for(int j = 0; j < cameraDepthSyncTimeSeries.at(i).size(); j++)
                file<<std::to_string(cameraDepthSyncTimeSeries.at(i).at(j).time)<<cameraDepthExts.at(i)<<std::endl;
            file.close();
        }
        for(int i = 0; i < cameraPointCloudNames.size(); i++){
            std::ofstream file(cameraPointCloudDirs.at(i) + "/sync.txt");
            for(int j = 0; j < cameraPointCloudSyncTimeSeries.at(i).size(); j++)
                file<<std::to_string(cameraPointCloudSyncTimeSeries.at(i).at(j).time)<<cameraPointCloudExts.at(i)<<std::endl;
            file.close();
        }
        for(int i = 0; i < armJointStateNames.size(); i++){
            std::ofstream file(armJointStateDirs.at(i) + "/sync.txt");
            for(int j = 0; j < armJointStateSyncTimeSeries.at(i).size(); j++)
                file<<std::to_string(armJointStateSyncTimeSeries.at(i).at(j).time)<<armJointStateExts.at(i)<<std::endl;
            file.close();
        }
        for(int i = 0; i < armEndPoseNames.size(); i++){
            std::ofstream file(armEndPoseDirs.at(i) + "/sync.txt");
            for(int j = 0; j < armEndPoseSyncTimeSeries.at(i).size(); j++)
                file<<std::to_string(armEndPoseSyncTimeSeries.at(i).at(j).time)<<armEndPoseExts.at(i)<<std::endl;
            file.close();
        }
        for(int i = 0; i < localizationPoseNames.size(); i++){
            std::ofstream file(localizationPoseDirs.at(i) + "/sync.txt");
            for(int j = 0; j < localizationPoseSyncTimeSeries.at(i).size(); j++)
                file<<std::to_string(localizationPoseSyncTimeSeries.at(i).at(j).time)<<localizationPoseExts.at(i)<<std::endl;
            file.close();
        }
        for(int i = 0; i < gripperEncoderNames.size(); i++){
            std::ofstream file(gripperEncoderDirs.at(i) + "/sync.txt");
            for(int j = 0; j < gripperEncoderSyncTimeSeries.at(i).size(); j++)
                file<<std::to_string(gripperEncoderSyncTimeSeries.at(i).at(j).time)<<gripperEncoderExts.at(i)<<std::endl;
            file.close();
        }
        for(int i = 0; i < imu9AxisNames.size(); i++){
            std::ofstream file(imu9AxisDirs.at(i) + "/sync.txt");
            for(int j = 0; j < imu9AxisSyncTimeSeries.at(i).size(); j++)
                file<<std::to_string(imu9AxisSyncTimeSeries.at(i).at(j).time)<<imu9AxisExts.at(i)<<std::endl;
            file.close();
        }
        for(int i = 0; i < lidarPointCloudNames.size(); i++){
            std::ofstream file(lidarPointCloudDirs.at(i) + "/sync.txt");
            for(int j = 0; j < lidarPointCloudSyncTimeSeries.at(i).size(); j++)
                file<<std::to_string(lidarPointCloudSyncTimeSeries.at(i).at(j).time)<<lidarPointCloudExts.at(i)<<std::endl;
            file.close();
        }
        for(int i = 0; i < robotBaseVelNames.size(); i++){
            std::ofstream file(robotBaseVelDirs.at(i) + "/sync.txt");
            for(int j = 0; j < robotBaseVelSyncTimeSeries.at(i).size(); j++)
                file<<std::to_string(robotBaseVelSyncTimeSeries.at(i).at(j).time)<<robotBaseVelExts.at(i)<<std::endl;
            file.close();
        }
        for(int i = 0; i < liftMotorNames.size(); i++){
            std::ofstream file(liftMotorDirs.at(i) + "/sync.txt");
            for(int j = 0; j < liftMotorSyncTimeSeries.at(i).size(); j++)
                file<<std::to_string(liftMotorSyncTimeSeries.at(i).at(j).time)<<liftMotorExts.at(i)<<std::endl;
            file.close();
        }
    }

    double checkDataAdequacy(bool print=false){
        bool result = true;
        double time = INFINITY;
        for(int i = 0; i < cameraColorNames.size() && result; i++){
            if(cameraColorToSyncs.at(i)){
                if(cameraColorDataTimeSeries.at(i).size() == 0){
                    if(print){
                        std::cout<<"camera color "<<cameraColorNames.at(i)<<" has no data"<<std::endl;
                    }
                    result = false;
                }
                else
                    time = time < cameraColorDataTimeSeries.at(i).back().time ? time : cameraColorDataTimeSeries.at(i).back().time;
            }
        }
        for(int i = 0; i < cameraDepthNames.size() && result; i++){
            if(cameraDepthToSyncs.at(i)){
                if(cameraDepthDataTimeSeries.at(i).size() == 0){
                    if(print){
                        std::cout<<"camera depth "<<cameraDepthNames.at(i)<<" has no data"<<std::endl;
                    }
                    result = false;
                }
                else
                    time = time < cameraDepthDataTimeSeries.at(i).back().time ? time : cameraDepthDataTimeSeries.at(i).back().time;
            }
        }
        for(int i = 0; i < cameraPointCloudNames.size() && result; i++){
            if(cameraPointCloudToSyncs.at(i)){
                if(cameraPointCloudDataTimeSeries.at(i).size() == 0){
                    if(print){
                        std::cout<<"camera pointCloud "<<cameraPointCloudNames.at(i)<<" has no data"<<std::endl;
                    }
                    result = false;
                }
                else
                    time = time < cameraPointCloudDataTimeSeries.at(i).back().time ? time : cameraPointCloudDataTimeSeries.at(i).back().time;
            }
        }
        for(int i = 0; i < armJointStateNames.size() && result; i++){
            if(armJointStateToSyncs.at(i)){
                if(armJointStateDataTimeSeries.at(i).size() == 0){
                    if(print){
                        std::cout<<"arm jointState "<<armJointStateNames.at(i)<<" has no data"<<std::endl;
                    }
                    result = false;
                }
                else
                    time = time < armJointStateDataTimeSeries.at(i).back().time ? time : armJointStateDataTimeSeries.at(i).back().time;
            }
        }
        for(int i = 0; i < armEndPoseNames.size() && result; i++){
            if(armEndPoseToSyncs.at(i)){
                if(armEndPoseDataTimeSeries.at(i).size() == 0){
                    if(print){
                        std::cout<<"arm endPose "<<armEndPoseNames.at(i)<<" has no data"<<std::endl;
                    }
                    result = false;
                }
                else
                    time = time < armEndPoseDataTimeSeries.at(i).back().time ? time : armEndPoseDataTimeSeries.at(i).back().time;
            }
        }
        for(int i = 0; i < localizationPoseNames.size() && result; i++){
            if(localizationPoseToSyncs.at(i)){
                if(localizationPoseDataTimeSeries.at(i).size() == 0){
                    if(print){
                        std::cout<<"localization pose "<<localizationPoseNames.at(i)<<" has no data"<<std::endl;
                    }
                    result = false;
                }
                else
                    time = time < localizationPoseDataTimeSeries.at(i).back().time ? time : localizationPoseDataTimeSeries.at(i).back().time;
            }
        }
        for(int i = 0; i < gripperEncoderNames.size() && result; i++){
            if(gripperEncoderToSyncs.at(i)){
                if(gripperEncoderDataTimeSeries.at(i).size() == 0){
                    if(print){
                        std::cout<<"gripper encoder "<<gripperEncoderNames.at(i)<<" has no data"<<std::endl;
                    }
                    result = false;
                }
                else
                    time = time < gripperEncoderDataTimeSeries.at(i).back().time ? time : gripperEncoderDataTimeSeries.at(i).back().time;
            }
        }
        for(int i = 0; i < imu9AxisNames.size() && result; i++){
            if(imu9AxisToSyncs.at(i)){
                if(imu9AxisDataTimeSeries.at(i).size() == 0){
                    if(print){
                        std::cout<<"imu 9axis "<<imu9AxisNames.at(i)<<" has no data"<<std::endl;
                    }
                    result = false;
                }
                else
                    time = time < imu9AxisDataTimeSeries.at(i).back().time ? time : imu9AxisDataTimeSeries.at(i).back().time;
            }
        }
        for(int i = 0; i < lidarPointCloudNames.size() && result; i++){
            if(lidarPointCloudToSyncs.at(i)){
                if(lidarPointCloudDataTimeSeries.at(i).size() == 0){
                    if(print){
                        std::cout<<"lidar pointCloud "<<lidarPointCloudNames.at(i)<<" has no data"<<std::endl;
                    }
                    result = false;
                }
                else
                    time = time < lidarPointCloudDataTimeSeries.at(i).back().time ? time : lidarPointCloudDataTimeSeries.at(i).back().time;
            }
        }
        for(int i = 0; i < robotBaseVelNames.size() && result; i++){
            if(robotBaseVelToSyncs.at(i)){
                if(robotBaseVelDataTimeSeries.at(i).size() == 0){
                    if(print){
                        std::cout<<"robotBase vel "<<robotBaseVelNames.at(i)<<" has no data"<<std::endl;
                    }
                    result = false;
                }
                else
                    time = time < robotBaseVelDataTimeSeries.at(i).back().time ? time : robotBaseVelDataTimeSeries.at(i).back().time;
            }
        }
        for(int i = 0; i < liftMotorNames.size() && result; i++){
            if(liftMotorToSyncs.at(i)){
                if(liftMotorDataTimeSeries.at(i).size() == 0){
                    if(print){
                        std::cout<<"lift motor "<<liftMotorNames.at(i)<<" has no data"<<std::endl;
                    }
                    result = false;
                }
                else
                    time = time < liftMotorDataTimeSeries.at(i).back().time ? time : liftMotorDataTimeSeries.at(i).back().time;
            }
        }
        return result ? time : INFINITY;
    }

    int getFileInPath(std::string path, std::string ext, std::vector<TimeSeries>* dataList, std::vector<TimeSeries>* syncList){
        int count = 0;
        for (const auto& entry : boost::filesystem::directory_iterator(path)) {
            const auto& path = entry.path();
            if (path.extension() == ext) {
                try{
                    allTimeSeries.push_back(TimeSeries(std::stod(path.stem().string()), dataList, syncList));
                    count++;
                }catch(std::invalid_argument &ex){
                    continue;
                }
            }
        }
        return count;
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_sync");
    ros::NodeHandle nh;
    std::string datasetDir;
    int episodeIndex;
    double timeDiffLimit;
    nh.param<std::string>("datasetDir", datasetDir, "/home/agilex/data");
    nh.param<int>("episodeIndex", episodeIndex, 0);
    nh.param<double>("timeDiffLimit", timeDiffLimit, 0.1);
    ROS_INFO("\033[1;32m----> data sync Started.\033[0m");
    if(episodeIndex == -1){
        for (const auto& entry : boost::filesystem::directory_iterator(datasetDir)) {
            const auto& path = entry.path();
            std::string fileName = path.stem().string();
            if(fileName.substr(0, 7) == "episode" && fileName.substr(fileName.length() - 7, 7) != ".tar.gz"){
                std::cout<<fileName<<" processing"<<std::endl;
                fileName.replace(0, 7, "");
                DataSync dataSync(datasetDir, std::stoi(fileName), timeDiffLimit);
                dataSync.sync();
                std::cout<<fileName<<" done"<<std::endl;
            }
        }
    }else{
        DataSync dataSync(datasetDir, episodeIndex, timeDiffLimit);
        dataSync.sync();
    }
    std::cout<<"Done"<<std::endl;
    return 0;
}
