/*!
 * @author: yinchenhui
 * @date: 2019/04/27
 * 抽取雷达数据，将pcap文件数据提取,矫正后保存为pcd格式，并按时间戳命名
 * @param:
 *  inDir: direction contain pcap files
 *  outDir: direction to store pcd files
 *  calibrateFile: HDL64 calibration file
 *  poseFile: LiDAR pose file 
 */

#include <stdio.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/boost.h>
#include <pthread.h>
#include <iostream>
#include <string>
#include <vector>
#include "util/FileUtil.hpp"
#include "VelodyneReader.h"
//#include "Undistortion.h"
#include "util/TimeUtil.h"

#define HDL64       "HDL64"
#define VLP16       "VLP16"
#define VLP16_1     "VLP16_1"
#define VLP16_2     "VLP16_2"

#define MAX_THREAD_NUM  4
using namespace velodyne;

char usage[] = {
    " inDir outDir calibrateFile poseFile calibFile [startThreadId]\n"
    "brief: this pragram used to convert pcap into pcd file.\n"
    "PARAM:\n"
    "    inDir: direction contain pcap files\n"
    "    outDir: direction to store pcd files\n"
    "    correctFile: offical calibrate file likes VLP16.xml\n"
    //"    poseFile: OXTS pose file used for undistortion\n"
    //"    calibFile: calib file used for calculate LiDAR pose from other sensor\n"
    "    startThreadId: the first thread id used to run\n"
    "SAMPLE(with undistortion):\n"
    "    ./bin/ExtractPCAP /data/2018-07-21/rawdata/HDL64 /data/2018-07-21/HDL64"
    " Sensor/LiDAR/HDL-64.xml /data/2018-07-21/OXTS-pose.txt"
    " /data/2018-07-21/Calib/calib_imu_to_velo.txt\n"
    "SAMPLE(original pcd):\n"
    "    ./bin/ExtractPCAP /data/2018-07-21/rawdata/HDL64 /data/2018-07-21/HDL64"
    " Sensor/LiDAR/HDL-64.xml\n"
};

static std::string deviceName;
//The function to get data and file from pcap
void ExtractVelodyneData(const std::string& inDir, const std::string& correctFile,
    const std::string& outDir);
int ExtractVelodyneFile(const std::string& filename, const std::string& correctFile,
    const std::string& outDir, std::vector<std::pair<std::string, std::string>>& timePairList);

static int startThreadId = 1;
static bool isUndist = false;
//entry the main
int main(int argc, char** argv) {
    if (argc != 4 && argc != 6 && argc != 7) {
        std::cout << "usage:" << argv[0] << usage;
        return 1;
    }

    std::string inDir(argv[1]);
    std::string outDir(argv[2]);
    std::string correctFile(argv[3]);
    //std::string poseFile;
    //std::string calibFile;
//    if (argc >= 6) {
//        //poseFile = (std::string)(argv[4]);
//        calibFile = (std::string)(argv[5]);
//        isUndist = true;
//    }

    if (argc == 7) {
        startThreadId = stoi(std::string(argv[6]));
    } else {
        srand(time(NULL));
        startThreadId = rand() % static_cast<int>(omp_get_max_threads());
    }
    std::cout << "start thread id:" << startThreadId << std::endl;


//   std::map<std::string, Eigen::Affine3d> pose_map;
//        if (isUndist) {
//            if (!readPoseFile(poseFile, calibFile, pose_map)) {
//                std::cout << "Read pose file or calib file failed, exit." << std::endl;
//                return 1;
//            }
//        }

    ExtractVelodyneData(inDir, correctFile, outDir);

    return 0;
}
//
void ExtractVelodyneData(const std::string &inDir, const std::string& correctFile,
    const std::string &outDir) {
    // if in dir not exist, exit
    if (!UT::FileExists(inDir.c_str())) {
        std::cout << inDir << " not exist!" << std::endl;
        return;
    }
    // if out dir not exist, create dir
    if (!UT::FileExists(outDir.c_str())) {
        UT::DirectoryCreate(outDir.c_str());
    }

    // velodyne type
    deviceName = inDir.substr(inDir.rfind('/', inDir.size()-2)+1);
    deviceName = deviceName.substr(0, deviceName.rfind('/'));
    std::cout << "device name: " << deviceName << std::endl;
    // HDL64
    if (deviceName.find(HDL64) != std::string::npos) {
        std::cout << "velo type:HDL64" << std::endl;
    } else if (deviceName.find(VLP16) != std::string::npos) {   // VLP16
        std::cout << "velo type:VLP16" << std::endl;
    } else {
        std::cout << "invalid vel type." << std::endl;
        return;
    }
    std::cout << "correct file: " << correctFile << std::endl;



    // get all pcap files
    std::vector<std::string> pcapFiles;
    UT::GetFileNames(inDir, pcapFiles);
    std::cout << "get " << pcapFiles.size() << " pcap files." << std::endl;
    std::vector<std::vector<std::pair<std::string, std::string>>> allTimePairList(pcapFiles.size());

    int maxThreads = omp_get_max_threads();
    int numThreads = maxThreads;
    std::cout << "total " << maxThreads << " threads." << std::endl;
    if (numThreads > MAX_THREAD_NUM)
        numThreads = MAX_THREAD_NUM;
    std::cout << "set " << numThreads << " threads." << std::endl;
    omp_set_num_threads(numThreads);
    int sucCnt = 0;
    std::vector<bool> threadsBinded(numThreads, false);
#pragma omp parallel for reduction(+:sucCnt)
    for (int i = 0; i < pcapFiles.size(); ++i) {
        int threadId = omp_get_thread_num();
        if (!threadsBinded[threadId]) {
            std::cout << "bind thread:" << threadId << "-" << pthread_self() << std::endl;
            cpu_set_t cpuSet;
            CPU_ZERO(&cpuSet);
            CPU_SET((threadId+startThreadId) % maxThreads, &cpuSet);
            int ret = pthread_setaffinity_np(pthread_self(), sizeof(cpuSet), &cpuSet);
            if (ret < 0) {
                std::cout << "bind error!" << std::endl;
            }
            threadsBinded[threadId] = true;
        }
        const std::string pcapFile = inDir + "/" + pcapFiles[i];
        const std::string extName = pcapFile.substr(pcapFile.rfind('.'));
        if (extName != ".pcap") {
            std::cout << pcapFile << " not a valid pcap file" << std::endl;
            continue;
        }
        if (ExtractVelodyneFile(pcapFile, correctFile, outDir, allTimePairList[i])) {
        // if (ExtractVelodyneFile(pcapFile, correctFile, outDir, allTimePairList[i])) {
            ++sucCnt;
            std::cout << "extract " << pcapFile << " succeeded."<< std::endl;
        } else {
            std::cout << "extract " << pcapFile << " failed." << std::endl;
        }
    }

     //if(flag){

    int totalPCDFile = 0;
     //write all time pair into file
     const std::string veloTimeFileName =
         outDir.substr(0, outDir.rfind('/', outDir.size() - 2)+1) + "veloTimes.txt";
     std::ofstream fout(veloTimeFileName);
     if (!fout.is_open()) {
         std::cout << "open " << veloTimeFileName << " failed." << std::endl;
         return;
     }

     fout << "sys-time gps-time timeDiff" << std::endl;
     for (int i = 0; i < allTimePairList.size(); i++) {
         totalPCDFile += allTimePairList[i].size();
         std::vector<std::pair<std::string, std::string>> timePairList = allTimePairList[i];
         for (auto iter = timePairList.begin(); iter != timePairList.end(); ++iter) {
             double sysTs, gpsTs;
             timeString2timestamp(FormatSTDTimeString(iter->first), sysTs);
             //std::cout<<" iter->first"<<iter->first<<"systime"<<sysTs<<std::endl;
             timeString2timestamp(FormatSTDTimeString(iter->second.substr(0, 23)), gpsTs);
             fout << iter->first << " " << iter->second.substr(0, 23) << " " <<
                 static_cast<int>(sysTs - gpsTs) << std::endl;
         }
     }
     fout.close();

    if (isUndist) {
        std::cout << "extract " << sucCnt << " pcap files (" << totalPCDFile <<
            " PCD files) WITH undistortion successfully." << std::endl;
    } else {
        std::cout << "extract " << sucCnt << " pcap files (" << totalPCDFile <<
            " PCD files) WITHOUT undistortion successfully." << std::endl;
    }
}

int ExtractVelodyneFile(const std::string& filename, const std::string& correctFile,
    const std::string& outDir, std::vector<std::pair<std::string, std::string>>& timePairList) {
    boost::scoped_ptr<VelodyneReader> velodyneReader(new VelodyneReader(filename,
        correctFile));
    int numFrames = velodyneReader->readFrameInformation() - 1;
    if (numFrames < 0) {
        return 0;
    }

    std::cout << "Velodyne file: " << filename.c_str() <<
        " frame_num: " << numFrames << std::endl;
    for (int i = 0; i < numFrames; i++) {
        double time_stamp_ms = 0.0;
        // Frame的时间戳
        std::string gpsTimeStr;
        // 存每packet点云指针
        // std::vector<PointCloudI::Ptr> packetPointCloud;
        // std::vector<std::string> packetTimeStr;
        // PointCloudI::Ptr p_cloud = velodyneReader->getFrame(i, time_stamp_ms,
        //                                                 packetTimeStr, packetPointCloud);

        // // frame时间戳取第一个packet时间,原先为最后一packet时间
        // gpsTimeStr = packetTimeStr.front();
        // PointCloudI frame_pc;

        std::vector<PointCloudPtr> packet_cloud_ptr_vec;
        std::vector<std::string> packet_time_vec;
        PointCloudPtr pc_ptr = velodyneReader->getFrame(i, time_stamp_ms, packet_time_vec, packet_cloud_ptr_vec);
        // frame时间戳取第一个packet时间,原先为最后一packet时间
        //std::cout<<" output the information from Frame"<<i<<" getframe："<< time_stamp_ms <<std::endl;
        //完成时间戳到标准格式时间的转换
        timestamp2String(time_stamp_ms,gpsTimeStr);

        std::cout<<"From Frame"<<i<<"convert from timestampms:"<<std::fixed<<time_stamp_ms<<" to gpsTimeStr："<<gpsTimeStr<<std::endl;
        //

        //gpsTimeStr = packet_time_vec.front();
        //std::cout<<" output the information from Frame"<<i<<" gpsTimestr："<< gpsTimeStr.substr(0, 23)<<std::endl;
        PointCloudI frame_pc;

//
            for (int i = 0; i <  packet_cloud_ptr_vec.size(); i++) {
                frame_pc += *(packet_cloud_ptr_vec[i]);
            }
        //}
        // std::cout << time_str << " " << gpsTimeStr << std::endl;
        // use gps time
        std::string dst_file = outDir + "/" + deviceName + "_" + gpsTimeStr.substr(0, 23) + ".pcd";
        // std::string dst_file = outDir + "/" + deviceName + "_" + time_str + ".pcd";
        if(!frame_pc.empty()) {
            pcl::io::savePCDFileBinaryCompressed(dst_file, frame_pc);
        } /*else {
            LOG(ERROR) << "Frame point cloud " << gpsTimeStr.substr(0, 23) + ".pcd" << " is empty!";
        }*/

        // }

        std::string time_str;
        time_str = timestamp2String(time_stamp_ms);
        timePairList.emplace_back(std::pair<std::string, std::string>(time_str, gpsTimeStr));
    }
    velodyneReader->close();

    return numFrames;
}
