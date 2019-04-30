*************************
# editor: yinchenhui
# goal：for VLP16 and other type Lidar to get .pcd file from .pcap file 
# 时间戳提取与文件类型转换
*************************
# 方式选择
#MATLAB
![优点] 依赖底层函数velodyneFileReader()，hasFrame(),readFrame()，可以方便解析velodyne雷达各个型号的pcap数据包，
![缺点] 只有velodyne雷达数据的解析方式；函数固定无法develop；无法输出具体时间戳信息；其他函数甚至不支持pcap文件的读取 **（排除）**
#python
![优点] 与winpcap\libpcap结合
![缺点] 不同平台上有不同的使用方法和依赖；兼容系性差
#其余软件：wireshark，Pcap-Analyzer，Xplico
![优缺点] 可视化，解析度，大多无法同时兼顾，而且不是针对雷达点云设计，无法深度数据解析
#基于原C++方法
![优缺点] 兼容性好，可开发用于多种雷达pcap文件解析，仅依赖PCL原生数据库，便于开发复用，因此在原代码基础上develop.
**********************
# 程序运行###数据及xml标定文件放入build文件夹###
$ cd Pcap2pcd
$ mkdir build 
$ cd build 
$ cmake ..
$ make 
$ ./ExtractCloud pcap.dir savepcd.dir correctfile
SAMPLE:
$ ./ExtractCloud VLP16/ VLP16pcddata/2019-04-20-22-07/ VLP-16.xml
*******************
### 数据存储格式
* **pcap**: 一种通用的数据流格式,可从网络特定端口抓取数据存储。
* **pcd**: Point Cloud Data，一种存储点云数据的文件格式。包括至少（X,Y,Z,I） 4种数据类型
* **datapacket**: 雷达原始udp数据包。用pcap文件存储。  
总长度为1248 = 42(头部)+1206(数据)+4(gpstimestamp)+1(status type)+1(status value) 
因为数据包涉及特殊的格式，和网络通信协议，因此直接暴力进制转换的解析方法不太现实，因此才开发出了大量解析软件。如：wireshark，Pcap-Analyzer，Xplico等。

### 时间戳构成:  
* **目标格式**： YYYY-MM-DD-HH-MI-SS-MS
1. **Status type & Status value**(1-byte & 1-byte): 每个数据包中只存储一个类型：Y/N/D/H/M/S 和其对应的值。  
  * 在此把Status type 所对应的值整合为**datetime**，表示为年-月-日-时-分-秒-毫秒:   
    |Y=year | N=month | D=day | H=hour | M=minute | S=second | 
  * Status type & Status value 共16组，即循环16个数据包才可以取到所需完整信息。
### 数据位置

