#ifndef COLLECT_CAR_SENSOR_LIDAR_VELODYNE_READER_H
#define COLLECT_CAR_SENSOR_LIDAR_VELODYNE_READER_H

#include <pcap.h>
#include <boost/foreach.hpp>
#include <boost/preprocessor.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
#include <string>
#include <vector>

// Some versions of libpcap do not have PCAP_NETMASK_UNKNOWN
#if !defined(PCAP_NETMASK_UNKNOWN)
#define PCAP_NETMASK_UNKNOWN 0xffffffff
#endif

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointI> PointCloudI;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// Add new type definations by Guorun Yang on 2018.10.09
typedef pcl::PointCloud<pcl::PointXYZI>::Ptr        PointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZI>::ConstPtr   PointCloudConstPtr;
typedef Eigen::Matrix<double, 4, 4>                 Matrix4d;


namespace velodyne {
// 参数依次为：
// 旋转角度？
// 每100中坐标点数目
// 最大的雷达线束
// 每包数据的发射数量
static const int HDL_NUM_ROT_ANGLES = 36001;
static const int HDL_LASER_PER_FIRING = 32;
static const int HDL_MAX_NUM_LASERS = 64;
static const int HDL_FIRING_PER_PKT = 12;

// 数据头
enum HDLBlock {
    BLOCK_0_TO_31 = 0xeeff,
    BLOCK_32_TO_63 = 0xddff
};
// 传感器类型
enum SensorType {
    HDL32E = 0x21,     // decimal: 33
    VLP16 = 0x22,      // decimal: 34
    VLP32AB = 0x23,    // decimal: 35
    VLP16HiRes = 0x24, // decimal: 36
    VLP32C = 0x28,     // decimal: 40

    // Work around : this is not defined by any specification
    // But it is usefull to define
    HDL64 = 0xa0, // decimal: 160
};
//传感器返回类型
enum DualReturnSensorMode {
    STRONGEST_RETURN = 0x37,
    LAST_RETURN = 0x38,
    DUAL_RETURN = 0x39,
};

struct HDLDate {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    double gpsTimestamp;
};
// 雷达的基本数据类型
// 结构体对齐的单位为1,便于强行转换的数据对齐
// 包含u16的距离和u8的信号强度，共3字节
#pragma pack(push, 1)
typedef struct HDLLaserReturn {
    uint16_t distance;
    uint8_t intensity;
} HDLLaserReturn;
// u16块标识位，u16旋转方位角，和32组LaserReturn 32*3 + 4 = 100
struct HDLFiringData {
    uint16_t blockIdentifier;
    uint16_t rotationalPosition;
    HDLLaserReturn laserReturns[HDL_LASER_PER_FIRING];

    inline bool isUpperBlock() const { return blockIdentifier == BLOCK_32_TO_63; }
};
// 12组 HDLFiringData u32时间戳 两个标识位 12*100 + 4 + 1 + 1 = 1206
struct HDLDataPacket {
    HDLFiringData firingData[HDL_FIRING_PER_PKT];
    uint32_t gpsTimestamp;
    uint8_t factoryField1;
    uint8_t factoryField2;
    SensorType getSensorType() const {
        if (isHDL64())
            return HDL64;
        return static_cast<SensorType>(factoryField2);
    }
    DualReturnSensorMode getDualReturnSensorMode() const {
        if (isHDL64())
            return isDualModeReturnHDL64() ? DUAL_RETURN : STRONGEST_RETURN;
        return static_cast<DualReturnSensorMode>(factoryField1);
    }
    static const unsigned int getDataByteLength() { return 1206; }
    static const unsigned int getPacketByteLength() { return getDataByteLength() + 42; }
    static inline bool isValidPacket(const unsigned char* data, unsigned int dataLength) {
        if (dataLength != getDataByteLength())
            return false;
        const HDLDataPacket* dataPacket = reinterpret_cast<const HDLDataPacket*>(data);
        return (dataPacket->firingData[0].blockIdentifier == BLOCK_0_TO_31 ||
            dataPacket->firingData[0].blockIdentifier == BLOCK_32_TO_63);
    }

    inline bool isHDL64() const { return firingData[1].isUpperBlock(); }

    inline bool isDualModeReturn() const { return isDualModeReturn(isHDL64()); }
    inline bool isDualModeReturn(const bool isHDL64) const {
        if (isHDL64)
            return isDualModeReturnHDL64();
        else
            return isDualModeReturn16Or32();
    }
    inline bool isDualModeReturn16Or32() const {
        return firingData[1].rotationalPosition == firingData[0].rotationalPosition;
    }
    inline bool isDualModeReturnHDL64() const {
        return firingData[2].rotationalPosition == firingData[0].rotationalPosition;
    }
    inline bool isDualReturnFiringBlock(const int firingBlock) {
        if (isHDL64())
            return isDualModeReturnHDL64() && isDualBlockOfDualPacket64(firingBlock);
        else
            return isDualModeReturn16Or32() && isDualBlockOfDualPacket16Or32(firingBlock);
    }

    inline static bool isDualBlockOfDualPacket(const bool isHDL64, const int firingBlock) {
        return isHDL64 ? isDualBlockOfDualPacket64(firingBlock)
                    : isDualBlockOfDualPacket16Or32(firingBlock);
    }
    inline static bool isDualBlockOfDualPacket64(const int firingBlock) {
        return (firingBlock % 4 >= 2);
    }
    inline static bool isDualBlockOfDualPacket16Or32(const int firingBlock) {
        return (firingBlock % 2 == 1);
    }
};

//校准雷达数据的自定义数据结构
struct HDLLaserCorrection {  // Internal representation of per-laser correction
  // In degrees
  double rotationalCorrection;
  double verticalCorrection;
  // In meters
  double distanceCorrection;
  double distanceCorrectionX;
  double distanceCorrectionY;

  double verticalOffsetCorrection;
  double horizontalOffsetCorrection;

  double focalDistance;
  double focalSlope;
  double closeSlope;  // Used in HDL-64 only

  short minIntensity;
  short maxIntensity;

  // precomputed values
  double sinRotationalCorrection;
  double cosRotationalCorrection;
  double sinVertCorrection;
  double cosVertCorrection;
  double sinVertOffsetCorrection;
  double cosVertOffsetCorrection;
};

#pragma pack(pop)

class VelodyneReader {
public:
    VelodyneReader(const std::string& filename, const std::string &correctFile);

    //回放相关函数
    bool open(std::string filename);
    bool close();

    int readFrameInformation();
    PointCloudI::Ptr getFrame(int frameNumber, double &timeStamp, std::vector<std::string> &packetTimeStr, std::vector<PointCloudI::Ptr> &packetPointCloud);
    void freeMemory();

    bool getWantIntensityCorrection() const;
    void setWantIntensityCorrection(bool value);

    bool getIgnoreZeroDistances() const;
    void setIgnoreZeroDistances(bool value);

protected:
    std::string fileName;

private:
    //回放相关参数
    pcap_t* pcapfile = 0;
    std::string lasterror;

    int lastAzimuth;
    unsigned int lastTimestamp;
    double timeAdjust;
    bool isHDL64Data = false;
    bool hasDualReturn = false;
    bool wantIntensityCorrection = false;
    bool ignoreZeroDistances = true;
    //校准相关参数
    std::vector<double> cos_lookup_table_;
    std::vector<double> sin_lookup_table_;
    HDLLaserCorrection laser_corrections_[HDL_MAX_NUM_LASERS];
    double XMLColorTable[HDL_MAX_NUM_LASERS][3];
    double distanceResolutionM;
    int calibrationReportedNumLasers;
    bool correctionsInitialized;


    //存储相关参数 filepositon:每一帧点云库在文件中的起始地址 skips 需要跳过的packet数目
    //标记一下帧头开始的位置，读取的时候将帧头之前的部分跳过，返回不全的数据
    std::vector<fpos_t> filePostions;
    std::vector<int> skips;
    int skip;
    //存储的点云数据
    PointCloudI pointCloud;
    PointCloudI packetCloud;
    PointCloudI::Ptr cloud = nullptr;
    std::vector<PointCloudI::Ptr> dataPointCloud;
    std::vector<PointCloudI::Ptr> dataPacketPointCloud;
    std::vector<uint32_t> dataTimeStamp;
    std::vector<double> dataDistanceM;
    //std::vector<std::string> dataPacketTime;
    HDLDate hdlDate;
    //回放相关函数
    void getFilePosition(fpos_t* position);
    void setFilePosition(fpos_t* position);
    bool nextPacket(const unsigned char*& data, unsigned int& dataLength,
                    pcap_pkthdr** headerReference = NULL);
    void loadCorrectionsFile(const std::string& correctionsFile);
    void unloadFrameData();
    bool processHDLPacket(unsigned char* data, std::size_t bytesReceived,
        HDLDate& hdlDate);
    void processFiring(HDLFiringData* firingData, int firingBlockLaserOffset,
        int firingBlock, int azimuthDiff,
        double timestamp, unsigned int rawtime, bool isThisFiringDualReturnData,
        bool isDualReturnPacket);
    void pushFiringData(const unsigned char laserId,
        const unsigned char rawLaserId, unsigned short azimuth, const double timestamp,
        const unsigned int rawtime, const HDLLaserReturn* laserReturn,
        const HDLLaserCorrection* correction, const bool isFiringDualReturnData);
    void computeCorrectedValues(const unsigned short azimuth,
        const HDLLaserReturn* laserReturn, const HDLLaserCorrection* correction,
        double pos[3], double& distanceM, short& intensity, bool correctIntensity);

    double computeTimestamp(unsigned int tohTime);
    double VLP16AdjustTimeStamp(int firingblock, int dsr, int firingwithinblock,
        const bool isDualReturnMode);
    double HDL64EAdjustTimeStamp(int firingblock, int dsr, const bool isDualReturnMode);
};
}  // namespace velodyne

#endif // COLLECT_CAR_SENSOR_LIDAR_VELODYNE_READER_H
