#include "VelodyneReader.h"
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include "util/TimeUtil.h"

velodyne::VelodyneReader::VelodyneReader(const std::string& filename,
    const std::string &correctFile) : fileName(filename) {
    this->ignoreZeroDistances = true;
    this->skip = 0;
    this->lastAzimuth = -1;
    this->lastTimestamp = 0;
    this->timeAdjust = 0;
    this->correctionsInitialized = false;
    this->isHDL64Data = false;
    this->distanceResolutionM = 0.002;
    this->calibrationReportedNumLasers = 64;
    this->wantIntensityCorrection = false;

    this->loadCorrectionsFile(correctFile);
}

bool velodyne::VelodyneReader::open(std::string filename) {
    if (filename != fileName) {
        std::cout << "file name error,open failed!" << std::endl;
        return false;
    }
    //打开回放文件
    char errbuf[PCAP_ERRBUF_SIZE];
    pcap_t* pcapFile = pcap_open_offline(this->fileName.c_str(), errbuf);
    if (!pcapFile) {
        this->lasterror = errbuf;
        std::cout << "open error:"<< this->lasterror.c_str() << std::endl;
        return false;
    }
    struct bpf_program filter;
    if (pcap_compile(pcapFile, &filter, "udp", 0, PCAP_NETMASK_UNKNOWN) == -1) {
        this->lasterror = pcap_geterr(pcapFile);
        std::cout <<"pcap compile error:" << this->lasterror.c_str() << std::endl;
        return false;
    }
    if (pcap_setfilter(pcapFile, &filter) == -1) {
        this->lasterror = pcap_geterr(pcapFile);
        std::cout <<"pcap filter error:" << this->lasterror.c_str() << std::endl;
        return false;
    }
    this->pcapfile = pcapFile;
    return true;
}

bool velodyne::VelodyneReader::close() {
    if (this->pcapfile) {
        pcap_close(this->pcapfile);
        this->pcapfile = 0;
    }
    return true;
}

int velodyne::VelodyneReader::readFrameInformation() {
    if (this->fileName.empty()) {
        return -1;
    }
    if (!this->correctionsInitialized) {
        return -1;
    }
    if (!open(this->fileName)) {
        std::cout <<"file open failed" << std::endl;
        return -1;
    }
    const unsigned char* data = 0;
    unsigned int dataLength = 0;
//    double timeSinceStart = 0;

    unsigned int lastAzimuth = 0;
    //unsigned int lastTimestamp = 0;

    std::vector<fpos_t> filePositions;
    std::vector<int> skips;
    unsigned long long numberOfFiringPackets = 0;

    fpos_t lastFilePosition;
    getFilePosition(&lastFilePosition);

    bool IsHDL64Data = false;
    bool isEmptyFrame = true;
    //to get complete date
    int counter = 32;
    while (nextPacket(data , dataLength)) {
        if (dataLength != 1206) {
            getFilePosition(&lastFilePosition);
            continue;
        }
        //通过强转使各数据归位
        const HDLDataPacket* dataPacket = reinterpret_cast<const HDLDataPacket*>(data);
        numberOfFiringPackets++;
        //calculate the diff time
        // unsigned int timeDiff = dataPacket->gpsTimestamp - lastTimestamp;
        // std::cout<< "timeDiff"<<timeDiff<<std::endl;
        // if (timeDiff > 600 && lastTimestamp != 0)
        // {
        //     printf("missed %d packets\n",  static_cast<int>(floor((timeDiff/553.0) + 0.5)));
        // }
        //处理每个数据包中的数据
        if(counter--){
            hdlDate.gpsTimestamp = dataPacket->gpsTimestamp;

            int val = static_cast<int>(dataPacket->factoryField2 & 0xFFFF);
            switch (dataPacket->factoryField1) {
                case 'Y': hdlDate.year = val; break;
                case 'N': hdlDate.month = val; break;
                case 'D': hdlDate.day = val; break;
                case 'H': hdlDate.hour = val + 8; break;
                case 'M': hdlDate.minute = val; break;
                case 'S': hdlDate.second = val; break;
                std::cout<<"hdlyear:"<<hdlDate.year<<"hdlhour:"<<val<<std::endl;
            }
        }


        for (int i = 0; i < HDL_FIRING_PER_PKT; ++i) {
            const HDLFiringData& firingData = dataPacket->firingData[i];

            IsHDL64Data |= (firingData.blockIdentifier == BLOCK_32_TO_63);

            // 根据是否忽略掉0距离的点来修正输出的点云
            if (this->ignoreZeroDistances) {
                for (int laserID = 0; laserID < HDL_LASER_PER_FIRING; laserID++) {
                    if (firingData.laserReturns[laserID].distance != 0) {
                        isEmptyFrame = false;
                        break;
                    }
                }
            } else {
                isEmptyFrame = false;
            }
            // 下一帧：当本帧的方位角<上一帧的方位角时
            // 记录该帧的位置指针、记录应该跳过的packet数
            if (firingData.rotationalPosition < lastAzimuth) {
                // Add file position if the frame is not empty
    //            if (!isEmptyFrame || !this->ignoreEmptyFrames)
                if (!isEmptyFrame) {
                    filePositions.push_back(lastFilePosition);
                    skips.push_back(i);
                }
    //            this->UpdateProgress(0.0);
                // We start a new frame, reinitialize the boolean
                isEmptyFrame = true;
            }
            lastAzimuth = firingData.rotationalPosition;
        }
        lastTimestamp = dataPacket->gpsTimestamp;
        getFilePosition(&lastFilePosition);
    }
    this->filePostions = filePositions;
    this->skips = skips;
    return filePositions.size();
}

PointCloudI::Ptr velodyne::VelodyneReader::getFrame(int frameNumber,
    double &timeStamp, std::vector<std::string> &packetTimeStr, std::vector<PointCloudI::Ptr> &packetPointCloud) {
    // 重置所有参数函数
    this->unloadFrameData();
    if (!this->pcapfile) {
        if (!this->open(this->fileName)) {
            std::cout <<"file open failed" << std::endl;
        }
    }
    assert(this->filePostions.size() == this->skips.size());
    const unsigned char* data = 0;
    unsigned int dataLength = 0;
    int idx = 0;
    // int temp_idx = 0;
    // 设置文件指针的位置和起始的列点
    this->setFilePosition(&this->filePostions[frameNumber]);
    this->skip = this->skips[frameNumber];//skip包中的
    pcap_pkthdr* pHeader = NULL;
    //HDLDate hdlDate;
    pointCloud.clear();
    this->dataPacketPointCloud.clear();
    // std::vector<std::string> v_buffer_second;

    while (this->nextPacket(data, dataLength, &pHeader)) {

        // 主要的处理函数
        if (!this->processHDLPacket(const_cast<unsigned char*>(data), dataLength, hdlDate)) {

            ++idx;
            std::string TimeStr;
            int us = static_cast<unsigned long long >(hdlDate.gpsTimestamp) % 1000;
            int ts = hdlDate.gpsTimestamp / 1000;
            int ms = ts % 1000; ts /= 1000;
            int ss = ts % 60;   ts /= 60;
            int mi = ts % 60;
            char buffer_first[16];
            char buffer_second[16];
            
            //snprintf(buffer, sizeof(buffer), "20%02d-%02d-%02d-%02d-%02d-%02d-%03d-%03d",
            //    hdlDate.year, hdlDate.month, hdlDate.day, hdlDate.hour, mi, ss, ms, us);
            snprintf(buffer_second, sizeof(buffer_second), "%02d-%02d-%03d-%03d", mi, ss, ms, us);
            // v_buffer_second.push_back(std::string(buffer_second));
            snprintf(buffer_first, sizeof(buffer_first), "20%02d-%02d-%02d-%02d-",
                    hdlDate.year, hdlDate.month, hdlDate.day, hdlDate.hour);
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%s%s", buffer_first, buffer_second);
            
            TimeStr = std::string(buffer);
            packetTimeStr.push_back(TimeStr);
            
            // if(idx != 0 && idx % 32 == 0)
            // {
            //     snprintf(buffer_first, sizeof(buffer_first), "20%02d-%02d-%02d-%02d-",
            //         hdlDate.year, hdlDate.month, hdlDate.day, hdlDate.hour);
            //     std::vector<std::string>::iterator buffer_second_iter = v_buffer_second.begin();
            //     for(int i = temp_idx; i < idx; ++i)
            //     {
            //         char buffer[32];
            //         snprintf(buffer, sizeof(buffer), "%s%s", buffer_first, (*buffer_second_iter).c_str());
            //         buffer_second_iter++;
            //         TimeStr = std::string(buffer);
            //         packetTimeStr.push_back(TimeStr);
                    
            //         temp_idx = idx;
            //     }
            //     v_buffer_second.clear();

            // }

        } else {
            if (pHeader) {
                timeval2timestamp(pHeader->ts, timeStamp);
             //   std::cout<<"timestamp"<<timeStamp<<std::endl;
            } else {
                timeStamp = 0.0;
            }
            //int ts = hdlDate.gpsTimestamp / 1000;
            //int ms = ts % 1000; ts /= 1000;
            //int ss = ts % 60;   ts /= 60;
            //int mi = ts % 60;
            //char buffer[32];
            //snprintf(buffer, sizeof(buffer), "20%02d-%02d-%02d-%02d-%02d-%02d-%03d",
            //    hdlDate.year, hdlDate.month, hdlDate.day, hdlDate.hour, mi, ss, ms);
            //gpsTimeStr = std::string(buffer);
            // return this->dataPointCloud.back();
            ++idx;
            std::string TimeStr;
            int us = static_cast<unsigned long long >(hdlDate.gpsTimestamp) % 1000;
            int ts = hdlDate.gpsTimestamp / 1000;
            int ms = ts % 1000; ts /= 1000;
            int ss = ts % 60;   ts /= 60;
            int mi = ts % 60;
            char buffer_first[16];
            char buffer_second[16];
            snprintf(buffer_first, sizeof(buffer_first), "20%02d-%02d-%02d-%02d-",
                    hdlDate.year, hdlDate.month, hdlDate.day, hdlDate.hour);
            //std::cout<<"20%02d-%02d-%02d-%02d-"<<buffer_first<<std::endl;

            //std::string s_buffer_first(buffer_first);
            snprintf(buffer_second, sizeof(buffer_second), "%02d-%02d-%03d-%03d", mi, ss, ms, us);
            // v_buffer_second.push_back(std::string(buffer_second));
            // std::vector<std::string>::iterator buffer_second_iter = v_buffer_second.begin();
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%s%s", buffer_first, buffer_second);
            TimeStr = std::string(buffer);
            packetTimeStr.push_back(TimeStr);



            // for(int i = temp_idx; i < idx; ++i)
            // {
            //     char buffer[32];
            //     snprintf(buffer, sizeof(buffer), "%s%s", buffer_first, (*buffer_second_iter).c_str());
            //     TimeStr = std::string(buffer);
            //     packetTimeStr.push_back(TimeStr);
                
            //     //temp_idx = idx;
            // }
            // v_buffer_second.clear();
            packetPointCloud = this->dataPacketPointCloud;

            return cloud;
       }
    }
    std::cout <<"incomplete frame" << std::endl;
    cloud = boost::shared_ptr<PointCloudI>(new PointCloudI(pointCloud));
    timeStamp = - 1.0;
    this->dataPointCloud.push_back(cloud);
    return this->dataPointCloud.back();
}

void velodyne::VelodyneReader::freeMemory() {
    this->sin_lookup_table_.clear();
    this->cos_lookup_table_.clear();
    this->filePostions.clear();
    this->skips.clear();
    this->dataDistanceM.clear();
}

bool velodyne::VelodyneReader::getWantIntensityCorrection() const {
    return this->wantIntensityCorrection;
}

void velodyne::VelodyneReader::setWantIntensityCorrection(bool value) {
    this->wantIntensityCorrection = value;
}

bool velodyne::VelodyneReader::getIgnoreZeroDistances() const {
    return this->wantIntensityCorrection;
}

void velodyne::VelodyneReader::setIgnoreZeroDistances(bool value) {
    this->ignoreZeroDistances = value;
}

void velodyne::VelodyneReader::getFilePosition(fpos_t *position) {
#ifdef _MSC_VER
    pcap_fgetpos(this->pcapfile, position);
#else
    FILE* f = pcap_file(this->pcapfile);
    fgetpos(f, position);
#endif
}

void velodyne::VelodyneReader::setFilePosition(fpos_t *position) {
#ifdef _MSC_VER
    pcap_fsetpos(this->pcapfile, position);
#else
    FILE* f = pcap_file(this->pcapfile);
    fsetpos(f, position);
#endif
}

bool velodyne::VelodyneReader::nextPacket(const unsigned char *&data,
    unsigned int &dataLength, pcap_pkthdr **headerReference) {
    if (!this->pcapfile) {
        std::cout <<"pcap out" << std::endl;
        return false;
    }
    struct pcap_pkthdr* header;
    int returnValue = pcap_next_ex(this->pcapfile, &header, &data);
    if (returnValue < 0) {
      this->close();
      return false;
    }

    if (headerReference != NULL) {
        *headerReference = header;
        // dataLength = header->len;
        // timeSinceStart = GetElapsedTime(header->ts, this->StartTime);
        // return true;
    }

    // The ethernet header is 42 bytes long; unnecessary
    const unsigned int bytesToSkip = 42;
    dataLength = header->len - bytesToSkip;
    data = data + bytesToSkip;
    return true;
}

void velodyne::VelodyneReader::loadCorrectionsFile(const std::string &correctionsFile) {
    boost::property_tree::ptree pt;
    try {
        read_xml(correctionsFile, pt, boost::property_tree::xml_parser::trim_whitespace);
    } catch (boost::exception const&) {
        std::cout << "LoadCorrectionsFile: error reading calibration file: " <<
            correctionsFile << std::endl;
        return;
    }
    // Read distLSB if provided
    BOOST_FOREACH(boost::property_tree::ptree::value_type& v,
        pt.get_child("boost_serialization.DB")) {
        if (v.first == "distLSB_") { // Stored in cm in xml
            distanceResolutionM = atof(v.second.data().c_str()) / 100.0;
        }
    }

    int i, j;
    i = 0;
    BOOST_FOREACH(boost::property_tree::ptree::value_type& p,
        pt.get_child("boost_serialization.DB.colors_")) {
        if (p.first == "item") {
            j = 0;
            BOOST_FOREACH(boost::property_tree::ptree::value_type& v,
                p.second.get_child("rgb"))

                    if (v.first == "item") {
                    std::stringstream ss;
                    double val;
                    ss << v.second.data();
                    ss >> val;

                    XMLColorTable[i][j] = val;
                    j++;
                }
            i++;
        }
    }

    int enabledCount = 0;
    BOOST_FOREACH(boost::property_tree::ptree::value_type& v,
        pt.get_child("boost_serialization.DB.enabled_")) {
        std::stringstream ss;
        if (v.first == "item") {
            ss << v.second.data();
            int test = 0;
            ss >> test;
            if (!ss.fail() && test == 1) {
                enabledCount++;
            }
        }
    }
    this->calibrationReportedNumLasers = enabledCount;

    // Getting min & max intensities from XML
    int laserId = 0;
    int minIntensity[HDL_MAX_NUM_LASERS], maxIntensity[HDL_MAX_NUM_LASERS];
    BOOST_FOREACH(boost::property_tree::ptree::value_type& v,
      pt.get_child("boost_serialization.DB.minIntensity_")) {
        std::stringstream ss;
        if (v.first == "item") {
            ss << v.second.data();
            ss >> minIntensity[laserId];
            laserId++;
        }
    }

    laserId = 0;
    BOOST_FOREACH(boost::property_tree::ptree::value_type& v,
      pt.get_child("boost_serialization.DB.maxIntensity_")) {
        std::stringstream ss;
        if (v.first == "item") {
            ss << v.second.data();
            ss >> maxIntensity[laserId];
            laserId++;
        }
    }

    BOOST_FOREACH(boost::property_tree::ptree::value_type& v,
        pt.get_child("boost_serialization.DB.points_")) {
        if (v.first == "item") {
            boost::property_tree::ptree points = v.second;
            BOOST_FOREACH(boost::property_tree::ptree::value_type& px, points) {
            if (px.first == "px") {
                boost::property_tree::ptree calibrationData = px.second;
                int index = -1;
                HDLLaserCorrection xmlData = { 0 };

                BOOST_FOREACH(boost::property_tree::ptree::value_type& item, calibrationData) {
                if (item.first == "id_")
                    index = atoi(item.second.data().c_str());
                if (item.first == "rotCorrection_")
                    xmlData.rotationalCorrection = atof(item.second.data().c_str());
                if (item.first == "vertCorrection_")
                    xmlData.verticalCorrection = atof(item.second.data().c_str());
                if (item.first == "distCorrection_")
                    xmlData.distanceCorrection = atof(item.second.data().c_str());
                if (item.first == "distCorrectionX_")
                    xmlData.distanceCorrectionX = atof(item.second.data().c_str());
                if (item.first == "distCorrectionY_")
                    xmlData.distanceCorrectionY = atof(item.second.data().c_str());
                if (item.first == "vertOffsetCorrection_")
                    xmlData.verticalOffsetCorrection = atof(item.second.data().c_str());
                if (item.first == "horizOffsetCorrection_")
                    xmlData.horizontalOffsetCorrection = atof(item.second.data().c_str());
                if (item.first == "focalDistance_")
                    xmlData.focalDistance = atof(item.second.data().c_str());
                if (item.first == "focalSlope_")
                    xmlData.focalSlope = atof(item.second.data().c_str());
                if (item.first == "closeSlope_")
                    xmlData.closeSlope = atof(item.second.data().c_str());
                }
                if (index != -1 && index < HDL_MAX_NUM_LASERS) {
                    laser_corrections_[index] = xmlData;
                    // Angles are already stored in degrees in xml
                    // Distances are stored in centimeters in xml, and we store meters.
                    laser_corrections_[index].distanceCorrection /= 100.0;
                    laser_corrections_[index].distanceCorrectionX /= 100.0;
                    laser_corrections_[index].distanceCorrectionY /= 100.0;
                    laser_corrections_[index].verticalOffsetCorrection /= 100.0;
                    laser_corrections_[index].horizontalOffsetCorrection /= 100.0;
                    laser_corrections_[index].focalDistance /= 100.0;
                    laser_corrections_[index].focalSlope /= 100.0;
                    laser_corrections_[index].closeSlope /= 100.0;
                    if (laser_corrections_[index].closeSlope == 0.0)
                        laser_corrections_[index].closeSlope = laser_corrections_[index].focalSlope;
                    laser_corrections_[index].minIntensity = minIntensity[index];
                    laser_corrections_[index].maxIntensity = maxIntensity[index];
                }
            }
            }
        }
    }

    int idx = 0;
    BOOST_FOREACH(boost::property_tree::ptree::value_type& v,
      pt.get_child("boost_serialization.DB.minIntensity_")) {
        std::stringstream ss;
        if (v.first == "item") {
            ss << v.second.data();
            int intensity = 0;
            ss >> intensity;
            if (!ss.fail() && idx < HDL_MAX_NUM_LASERS) {
                laser_corrections_[idx].minIntensity = intensity;
            }
            idx++;
        }
    }

    idx = 0;
    BOOST_FOREACH(boost::property_tree::ptree::value_type& v,
      pt.get_child("boost_serialization.DB.maxIntensity_")) {
        std::stringstream ss;
        if (v.first == "item") {
            ss << v.second.data();
            int intensity = 0;
            ss >> intensity;
            if (!ss.fail() && idx < HDL_MAX_NUM_LASERS) {
                laser_corrections_[idx].maxIntensity = intensity;
            }
            idx++;
        }
    }
    // initTrigonometricTables()函数中的内容，中的参数
    if (cos_lookup_table_.size() == 0 || sin_lookup_table_.size() == 0) {
        cos_lookup_table_.resize(HDL_NUM_ROT_ANGLES);
        sin_lookup_table_.resize(HDL_NUM_ROT_ANGLES);
        for (unsigned int i = 0; i < HDL_NUM_ROT_ANGLES; i++) {
            double rad = HDL_Grabber_toRadians(i / 100.0);
            cos_lookup_table_[i] = std::cos(rad);
            sin_lookup_table_[i] = std::sin(rad);
        }
    }
    // 预先计算64线的校正值
    for (int i = 0; i < HDL_MAX_NUM_LASERS; i++) {
        HDLLaserCorrection& correction = laser_corrections_[i];
        correction.cosVertCorrection =
            std::cos(HDL_Grabber_toRadians(correction.verticalCorrection));
        correction.sinVertCorrection =
            std::sin(HDL_Grabber_toRadians(correction.verticalCorrection));
        correction.cosRotationalCorrection =
            std::cos(HDL_Grabber_toRadians(correction.rotationalCorrection));
        correction.sinRotationalCorrection =
            std::sin(HDL_Grabber_toRadians(correction.rotationalCorrection));
        correction.sinVertOffsetCorrection =
            correction.verticalOffsetCorrection * correction.sinVertCorrection;
        correction.cosVertOffsetCorrection =
            correction.verticalOffsetCorrection * correction.cosVertCorrection;
    }
//    precomputeCorrectionCosSin();
    this->correctionsInitialized = true;
}

void velodyne::VelodyneReader::unloadFrameData() {
    this->lastAzimuth = -1;
    this->lastTimestamp = 0;
//    this->timeAdjust = std::numeric_limits<double>::quiet_NaN();
    this->timeAdjust = 0;
    this->hasDualReturn = false;
    this->isHDL64Data = false;
    this->dataDistanceM.clear();
    this->cloud.reset();
    this->dataPointCloud.clear();
    //this->dataPacketPointCloud.clear();
    this->dataTimeStamp.clear();
}

bool velodyne::VelodyneReader::processHDLPacket(unsigned char *data,
    size_t bytesReceived, HDLDate& hdlDate) {
    if (bytesReceived != 1206) {
        return false;
    }
    HDLDataPacket* dataPacket = reinterpret_cast<HDLDataPacket*>(data);

    const unsigned int rawtime = dataPacket->gpsTimestamp;
    uint32_t timestamp = dataPacket->gpsTimestamp;
    hdlDate.gpsTimestamp = dataPacket->gpsTimestamp;
    int val = static_cast<int>(dataPacket->factoryField2 & 0xFFFF);
    switch (dataPacket->factoryField1) {
        case 'Y': hdlDate.year = val; break;
        case 'N': hdlDate.month = val; break;
        case 'D': hdlDate.day = val; break;
        case 'H': hdlDate.hour = val + 8; break;
        case 'M': hdlDate.minute = val; break;
        case 'S': hdlDate.second = val; break;
    }

    int firingBlock = this->skip;
    this->skip = 0;

    this->packetCloud.clear();
    
    // Compute the total azimuth advanced during one full firing block
    std::vector<int> diffs(HDL_FIRING_PER_PKT - 1);
    for (int i = 0; i < HDL_FIRING_PER_PKT - 1; ++i) {
      int localDiff = (36000 + dataPacket->firingData[i + 1].rotationalPosition -
                        dataPacket->firingData[i].rotationalPosition) % 36000;
      diffs[i] = localDiff;
    }
    this->isHDL64Data |= dataPacket->isHDL64();
    std::sort(diffs.begin(), diffs.end());
    // Assume the median of the packet's rotationalPosition differences
    int azimuthDiff = diffs[HDL_FIRING_PER_PKT / 2];
    if (this->isHDL64Data) {
      azimuthDiff = diffs[HDL_FIRING_PER_PKT - 2];
    }
    assert(azimuthDiff > 0);

    // Add DualReturn-specific arrays if newly detected dual return packet
    if (dataPacket->isDualModeReturn(this->isHDL64Data) && !this->hasDualReturn) {
        this->hasDualReturn = true;
        // 只有当是双重返回时才添加下面三个字段
        // this->CurrentDataset->GetPointData()->AddArray(this->DistanceFlag.GetPointer());
        // this->CurrentDataset->GetPointData()->AddArray(this->IntensityFlag.GetPointer());
        // this->CurrentDataset->GetPointData()->AddArray(this->DualReturnMatching.GetPointer());
    }
    // 处理一packet中每一列数据
    for (; firingBlock < HDL_FIRING_PER_PKT; ++firingBlock) {
        HDLFiringData* firingData = &(dataPacket->firingData[firingBlock]);
        int multiBlockLaserIdOffset = (firingData->blockIdentifier == BLOCK_0_TO_31) ? 0 :
                                (firingData->blockIdentifier == BLOCK_32_TO_63 ? 32 : 0);

        if (firingData->rotationalPosition < this->lastAzimuth) {
            // this->splitFrame();
            //cloud = boost::shared_ptr<PointCloudI>(new PointCloudI(pointCloud));
            
            PointCloudI::Ptr cloudPtr(new PointCloudI);
            cloudPtr = boost::shared_ptr<PointCloudI>(new PointCloudI(packetCloud));
            this->dataPacketPointCloud.push_back(cloudPtr);

            // 将时间戳和点云存入vector
            // this->dataTimeStamp.push_back(timestamp);
            // this->dataPointCloud.push_back(cloud);
            // pointCloud.clear();
            this->lastAzimuth = -1;
            return true;
        }

        // 测试用timestamp
        processFiring(firingData, multiBlockLaserIdOffset, firingBlock, azimuthDiff, timestamp,
            rawtime, this->hasDualReturn && dataPacket->isDualBlockOfDualPacket(this->isHDL64Data, firingBlock),
            this->hasDualReturn);
        this->lastAzimuth = firingData->rotationalPosition;
    }

    //存每packet数据
    PointCloudI::Ptr cloudPtr(new PointCloudI);
    cloudPtr = boost::shared_ptr<PointCloudI>(new PointCloudI(packetCloud));
    this->dataPacketPointCloud.push_back(cloudPtr);

    return false;
}

static int ROUND(float f) {
    return static_cast<int>(f + (f >= 0.0 ? 0.5 : -0.5));
}
static int ROUND(double f) {
    return static_cast<int>(f + (f >= 0.0 ? 0.5 : -0.5));
}

void velodyne::VelodyneReader::processFiring(velodyne::HDLFiringData *firingData,
    int firingBlockLaserOffset, int firingBlock, int azimuthDiff, double timestamp,
    unsigned int rawtime, bool isThisFiringDualReturnData, bool isDualReturnPacket) {
    for (int dsr = 0; dsr < HDL_LASER_PER_FIRING; dsr++) {
        // 当一列数据的前两个字节为0xDDFF时，为64线雷达的下面32线的数据，所以laseID需要+32
        // 然而当非64线时，此处为firingBlockLaserOffset为0，不用加
        const unsigned char rawLaserId = static_cast<unsigned char>(dsr + firingBlockLaserOffset);
        unsigned char laserId = rawLaserId;
        const unsigned short azimuth = firingData->rotationalPosition;

        // Detect VLP-16 data and adjust laser id if necessary
        int firingWithinBlock = 0;

        // 当数据为16线数据时的处理情况
        if (this->calibrationReportedNumLasers == 16) {
            // 此处为选16线的校准文件但是数据为64线的情况，本程序可以直接避免这种情况，所以注释掉
    //        if (firingBlockLaserOffset != 0)
    //        {
    //          if (!this->alreadyWarnedForIgnoredHDL64FiringPacket)
    //          {
    //            vtkGenericWarningMacro("Error: Received a HDL-64 UPPERBLOCK firing packet "
    //                                   "with a VLP-16 calibration file. Ignoring the firing.");
    //            this->alreadyWarnedForIgnoredHDL64FiringPacket = true;
    //          }
    //          return;
    //        }
            // 16线数据中一列数据包含两个方位角的数据
            if (laserId >= 16) {
                laserId -= 16;
                firingWithinBlock = 1;
            }
        }

        // Interpolate azimuths and timestamps per laser within firing blocks
        double timestampadjustment = 0;
        int azimuthadjustment = 0;
        // 修正timestamp和azimuth
        // if (this->UseIntraFiringAdjustment)
        if (true) {
            double blockdsr0 = 0, nextblockdsr0 = 1;
            switch (this->calibrationReportedNumLasers) {
                case 64: {
                    timestampadjustment =
                        -HDL64EAdjustTimeStamp(firingBlock, dsr, isDualReturnPacket);
                    nextblockdsr0 =
                        -HDL64EAdjustTimeStamp(firingBlock +
                            (isDualReturnPacket ? 4 : 2), 0, isDualReturnPacket);
                    blockdsr0 =
                        -HDL64EAdjustTimeStamp(firingBlock, 0, isDualReturnPacket);
                    break;
                }
                //  case 32:{
                //     if (this->ReportedSensor == VLP32AB || this->ReportedSensor == VLP32C) {
                //         timestampadjustment = VLP32AdjustTimeStamp(firingBlock, dsr, isDualReturnPacket);
                //         nextblockdsr0 = VLP32AdjustTimeStamp(
                //         firingBlock + (isDualReturnPacket ? 2 : 1), 0, isDualReturnPacket);
                //         blockdsr0 = VLP32AdjustTimeStamp(firingBlock, 0, isDualReturnPacket);
                //     } else {
                //         timestampadjustment = HDL32AdjustTimeStamp(firingBlock, dsr, isDualReturnPacket);
                //         nextblockdsr0 = HDL32AdjustTimeStamp(
                //         firingBlock + (isDualReturnPacket ? 2 : 1), 0, isDualReturnPacket);
                //         blockdsr0 = HDL32AdjustTimeStamp(firingBlock, 0, isDualReturnPacket);
                //     }
                //     break;
                //  }
                case 16: {
                    timestampadjustment =
                        VLP16AdjustTimeStamp(firingBlock, laserId, firingWithinBlock, isDualReturnPacket);
                    nextblockdsr0 =
                        VLP16AdjustTimeStamp(firingBlock + (isDualReturnPacket ? 2 : 1),
                            0, 0, isDualReturnPacket);
                    blockdsr0 =
                        VLP16AdjustTimeStamp(firingBlock, 0, 0, isDualReturnPacket);
                    break;
                }
                default: {
                    timestampadjustment = 0.0;
                    blockdsr0 = 0.0;
                    nextblockdsr0 = 1.0;
                }
            }
            azimuthadjustment = ROUND(azimuthDiff * ((timestampadjustment - blockdsr0) /
                (nextblockdsr0 - blockdsr0)));
            timestampadjustment = ROUND(timestampadjustment);
        }
        if (!this->ignoreZeroDistances || firingData->laserReturns[dsr].distance != 0.0) {
            pushFiringData(laserId, rawLaserId, azimuth + azimuthadjustment,
                timestamp + timestampadjustment, rawtime + static_cast<unsigned int>(timestampadjustment),
                &(firingData->laserReturns[dsr]), &(laser_corrections_[dsr + firingBlockLaserOffset]),
                isThisFiringDualReturnData);
        }
    }
}

void velodyne::VelodyneReader::pushFiringData(const unsigned char laserId,
    const unsigned char rawLaserId, unsigned short azimuth, const double timestamp,
    const unsigned int rawtime, const velodyne::HDLLaserReturn *laserReturn,
    const velodyne::HDLLaserCorrection *correction, const bool isFiringDualReturnData) {
    azimuth %= 36000;
    short intensity = laserReturn->intensity;

    double distanceM;
    double pos[3];
    //是否对 信号值进行修正
    bool applyIntensityCorrection = this->wantIntensityCorrection && this->isHDL64Data;
    computeCorrectedValues(azimuth, laserReturn, correction, pos, distanceM,
        intensity, applyIntensityCorrection);
    PointI pointTemp;
    pointTemp.x = pos[0];
    pointTemp.y = pos[1];
    pointTemp.z = pos[2];
    pointTemp.intensity = intensity;
    // 如果双重返回的第二次的值和第一次一样，那么这个点不加入pointcloud
    if (isFiringDualReturnData) {
        if (pointCloud.size() && dataDistanceM.back() == distanceM) {
            return;
        }
    }
    packetCloud.push_back(pointTemp);
    pointCloud.push_back(pointTemp);
    dataDistanceM.push_back(distanceM);
}

void velodyne::VelodyneReader::computeCorrectedValues(const unsigned short azimuth,
    const velodyne::HDLLaserReturn *laserReturn, const velodyne::HDLLaserCorrection *correction,
    double pos[], double &distanceM, short &intensity, bool correctIntensity) {
    intensity = laserReturn->intensity;
    double cosAzimuth, sinAzimuth;
    if (correction->rotationalCorrection == 0) {
      cosAzimuth = this->cos_lookup_table_[azimuth];
      sinAzimuth = this->sin_lookup_table_[azimuth];
    } else {
        // realAzimuth = azimuth/100 - rotationalCorrection
        // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
        // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
        cosAzimuth = this->cos_lookup_table_[azimuth] * correction->cosRotationalCorrection +
            this->sin_lookup_table_[azimuth] * correction->sinRotationalCorrection;
        sinAzimuth = this->sin_lookup_table_[azimuth] * correction->cosRotationalCorrection -
            this->cos_lookup_table_[azimuth] * correction->sinRotationalCorrection;
    }
    // Compute the distance in the xy plane (w/o accounting for rotation)
    /**the new term of 'vert_offset * sin_vert_angle'
     * was added to the expression due to the mathemathical
     * model we used.
     */
    double distanceMRaw = laserReturn->distance * this->distanceResolutionM;
    distanceM = distanceMRaw + correction->distanceCorrection;
    double xyDistance = distanceM * correction->cosVertCorrection - correction->sinVertOffsetCorrection;

    pos[0] = xyDistance * sinAzimuth - correction->horizontalOffsetCorrection * cosAzimuth;
    pos[1] = xyDistance * cosAzimuth + correction->horizontalOffsetCorrection * sinAzimuth;
    pos[2] = distanceM * correction->sinVertCorrection + correction->verticalOffsetCorrection;
    if (correctIntensity && (correction->minIntensity < correction->maxIntensity)) {
        // Compute corrected intensity

        /* Please refer to the manual:
            "Velodyne, Inc. ©2013  63‐HDL64ES3 REV G" Appendix F. Pages 45-46
            PLease note: in the manual, focalDistance is in centimeters, distance is the raw short from
            the laser
            & the graph is in meter */

        // Casting the input values to double for the computation

        double computedIntensity = static_cast<double>(intensity);
        double minIntensity = static_cast<double>(correction->minIntensity);
        double maxIntensity = static_cast<double>(correction->maxIntensity);

        // Rescale the intensity between 0 and 255
        computedIntensity = (computedIntensity - minIntensity) / (maxIntensity - minIntensity) * 255.0;

        if (computedIntensity < 0) {
            computedIntensity = 0;
        }

        double focalOffset = 256 * pow(1.0 - correction->focalDistance / 131.0, 2);
        double insideAbsValue = std::abs(
            focalOffset - 256 * pow(1.0 - static_cast<double>(laserReturn->distance) / 65535.0f, 2));

        if (insideAbsValue > 0) {
            computedIntensity = computedIntensity + correction->focalSlope * insideAbsValue;
        } else {
            computedIntensity = computedIntensity + correction->closeSlope * insideAbsValue;
        }
        computedIntensity = std::max(std::min(computedIntensity, 255.0), 1.0);

        intensity = static_cast<short>(computedIntensity);
    }
}

double velodyne::VelodyneReader::computeTimestamp(unsigned int tohTime) {
    static const double hourInMilliseconds = 3600.0 * 1e6;
    //整点判断
    if (tohTime < this->lastTimestamp) {
        this->timeAdjust += hourInMilliseconds;
        std::cout <<"whole clock" << std::endl;
    }
    this->lastTimestamp = tohTime;
    return static_cast<double>(tohTime) + this->timeAdjust;
}

double velodyne::VelodyneReader::VLP16AdjustTimeStamp(int firingblock, int dsr,
    int firingwithinblock, const bool isDualReturnMode) {
    if (!isDualReturnMode) {
      return (firingblock * 110.592) + (dsr * 2.304) + (firingwithinblock * 55.296);
    } else {
      return (firingblock / 2 * 110.592) + (dsr * 2.304) + (firingwithinblock * 55.296);
    }
}

double velodyne::VelodyneReader::HDL64EAdjustTimeStamp(int firingblock, int dsr,
    const bool isDualReturnMode) {
    const int dsrReversed = HDL_LASER_PER_FIRING - dsr - 1;
    const int firingblockReversed = HDL_FIRING_PER_PKT - firingblock - 1;
    if (!isDualReturnMode) {
        const double TimeOffsetMicroSec[4] = { 2.34, 3.54, 4.74, 6.0 };
        return (std::floor(static_cast<double>(firingblockReversed) / 2.0) * 48.0) +
            TimeOffsetMicroSec[(dsrReversed % 4)] + (dsrReversed / 4) * TimeOffsetMicroSec[3];
    } else {
        const double TimeOffsetMicroSec[4] = { 3.5, 4.7, 5.9, 7.2 };
        return (std::floor(static_cast<double>(firingblockReversed) / 4.0) * 57.6) +
            TimeOffsetMicroSec[(dsrReversed % 4)] + (dsrReversed / 4) * TimeOffsetMicroSec[3];
    }
}








