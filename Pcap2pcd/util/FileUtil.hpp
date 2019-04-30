/**
 * @brief This file include some file system utility functions.
 * 
 * @file FileUtil.hpp
 * @author liuyumoney
 * @date 2018-08-09
 */

#ifndef COLLECT_CAR_UTILITY_UTILITY_HPP
#define COLLECT_CAR_UTILITY_UTILITY_HPP

#include <limits.h>
#include <stdarg.h>
#include <float.h>
#ifdef _WIN32
#include <io.h>
#include <windows.h>
#include <direct.h>
#else
#include <unistd.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#endif
#include <vector>
#include <list>
#include <string>
#include <algorithm>

#define UT_STRING_MAX_LENGTH    512
#define UT_FACTOR_RAD_TO_DEG    57.295779505601046646705075978956f
#define UT_FACTOR_DEG_TO_RAD    0.01745329252222222222222222222222f

#define UT_E                2.71828182845904523536f         // e
#define UT_LOG2E            1.44269504088896340736f         // log2(e)
#define UT_LOG10E           0.434294481903251827651f        // log10(e)
#define UT_LN2              0.693147180559945309417f        // ln(2)
#define UT_LN10             2.30258509299404568402f         // ln(10)
#define UT_PI               3.14159265358979323846f         // pi
#define UT_2PI              6.28318530717958647692f         // pi*2
#define UT_PI_2             1.57079632679489661923f         // pi/2
#define UT_PI_4             0.785398163397448309616f        // pi/4
#define UT_1_PI             0.318309886183790671538f        // 1/pi
#define UT_1_2PI            0.159154943091895335769f        // 1/(pi*2)
#define UT_2_PI             0.636619772367581343076f        // 2/pi
#define UT_2_SQRTPI         1.1283791670955125739f          // 2/sqrt(pi)
#define UT_SQRT2            1.4142135623730950488f          // sqrt(2)
#define UT_1_SQRT2          0.7071067811865475244f          // 1/sqrt(2)
#define UT_1_SQRT2PI        0.39894228040143267794f         // 1/sqrt(2pi)

#define UT_GET_STRING(format, str) {\
    if (format) {\
        va_list args;\
        va_start(args, format);\
        vsprintf(str, format, args);\
        va_end(args);\
    } else {\
        str[0] = 0;\
    }\
}\

#define UT_SWAP(a, b) {\
    const auto t = (a);\
    (a) = (b);\
    (b) = (t);\
}\

#define UT_CLAMP(v, vMin, vMax) std::min(std::max((v), (vMin)), (vMax))

#define UT_DOT_TO_ANGLE(d) acos(UT_CLAMP((d), -1.0f, 1.0f))

#define UT_ASSERT(expression) UT::Assert(expression, #expression)

#define UT_FLT_EPSILON_MIN            1.0e-05f
#define UT_FLT_EPSILON_MAX            1.0e-03f
#define UT_FLT_EPSILON_RATIO_MIN    1.0e-03f
#define UT_FLT_EPSILON_RATIO_MAX    0.01f

#define UT_DBL_EPSILON_MIN            DBL_EPSILON
#define UT_DBL_EPSILON_MAX            1.0e-06
#define UT_DBL_EPSILON_RATIO_MIN    1.0e-05
#define UT_DBL_EPSILON_RATIO_MAX    0.001

#ifndef _WIN32
#ifndef _isnan
inline int _isnan(double x) { return std::isnan(x); }
#endif  // _isnan

#ifndef _isnanf
#define _isnanf isnanf
#endif

#ifndef _access
#define _access access
#endif

#endif  // _WIN32

#ifdef _WIN32
#define MKDIR(dir) _mkdir((dir))
#elif __linux__
#define MKDIR(dir) mkdir((dir), 0777)
#endif

// UT: short for UTility
namespace UT {
inline void PrintSeparator(const char c = '-') {
    printf("\r");
    for (int i = 0; i < 80; i++) {
        printf("%c", c);
    }
    printf("\n");
}

inline void SaveSeparator(FILE *fp, const char c = '-') {
    printf("\r");
    for (int i = 0; i < 79; ++i) {
        printf("%c", c);
    }
    printf("\n");
}
inline void PrintLoaded(const char *fileName) {
    printf("Loaded \'%s\'\n", fileName);
}
inline void PrintLoaded(const std::string fileName) {
    PrintLoaded(fileName.c_str());
}
inline void PrintSaved(const char *fileName) {
    printf("Saved \'%s\'\n", fileName);
}
inline void PrintSaved(const std::string fileName) {
    PrintSaved(fileName.c_str());
}
template<typename TYPE> inline void PrintValue(const TYPE v,
        const bool e = false) {
    printf("%d", v);
}
template<> inline void PrintValue<float>(const float v, const bool e) {
    if (e) {
        printf("%e", v);
    } else {
        printf("%f", v);
    }
}
template<> inline void PrintValue<double>(const double v, const bool e) {
    if (e) {
        printf("%e", v);
    } else {
        printf("%f", v);
    }
}
template<typename TYPE> inline void PrintError(const TYPE v1, const TYPE v2,
        const bool e = false) {
    printf("|");
    PrintValue<TYPE>(v1, e);
    printf(" - ");
    PrintValue<TYPE>(v2, e);
    printf("| = ");
    PrintValue<TYPE>(v1 >= v2 ? v1 - v2 : v2 - v1, e);
    printf("\n");
}

inline void Assert(const bool expression, const char *format, ...) {
    if (expression) {
        return;
    }
    char str[UT_STRING_MAX_LENGTH];
    UT_GET_STRING(format, str);
    printf("%s", str);
    exit(0);
}
inline void Error(const char *format = NULL, ...) {
    PrintSeparator('!');
    char str[UT_STRING_MAX_LENGTH];
    UT_GET_STRING(format, str);
    printf("ERROR: %s", str);
    exit(0);
}
inline void Check(const char *format = NULL, ...) {
#ifdef UT_PRINT_CHECK
    PrintSeparator('!');
    char str[UT_STRING_MAX_LENGTH];
    UT_GET_STRING(format, str);
    printf("CHECK: %s", str);
#endif
}

inline void DebugStart() {
}
inline bool Debugging() {
    return true;
}
inline void DebugStop();

template<typename TYPE> inline TYPE Random();
// todo: under iOS, the value returned by rand() could bigger then 32767!
template<> inline int Random<int>() {
    return rand() & 0xFFFF;    // [0, 32767]
}
template<> inline bool Random<bool>() {
    return (Random<int>() & 1) == 0;
}

template<> inline float Random<float>() {
    return Random<int>() / 32767.0f;
}

template<typename TYPE> inline TYPE Random(const TYPE vMax);

template<> inline int Random<int>(const int vMax) {
    return Random<int>() % vMax;
}
template<> inline float Random<float>(const float vMax) {
    return Random<float>() * vMax;
}

template<typename TYPE> inline TYPE Random(const TYPE vMin, const TYPE vMax) {
    return TYPE(Random<float>() * (vMax - vMin) + vMin);
}

template<typename TYPE> inline void Random(TYPE *v, const int N) {
    for (int i = 0; i < N; ++i) {
        v[i] = Random<TYPE>();
    }
}
template<typename TYPE> inline void Random(TYPE *v, const int N,
        const TYPE vMax) {
    for (int i = 0; i < N; ++i) {
        v[i] = Random<TYPE>(vMax);
    }
}
template<typename TYPE> inline void Random(TYPE *v, const int N,
        const TYPE vMin, const TYPE vMax) {
    for (int i = 0; i < N; ++i) {
        v[i] = Random<TYPE>(vMin, vMax);
    }
}

template<typename TYPE> inline TYPE Epsilon();
template<> inline float Epsilon<float>() {
    return FLT_EPSILON;
}
template<> inline double Epsilon<double>() {
    return DBL_EPSILON;
}
template<typename TYPE> inline TYPE Invalid();

template<> inline int Invalid<int>() {
    return INT_MAX;
}
template<> inline float Invalid<float>() {
    return FLT_MAX;
}
template<> inline double Invalid<double>() {
    return DBL_MAX;
}

template<typename TYPE> inline bool IsNAN(const TYPE v);
template<> inline bool IsNAN<float>(const float v) {
    return _isnanf(v) != 0;
}
template<> inline bool IsNAN<double>(const double v) {
    return _isnan(v) != 0;
}

template<class TYPE> inline void SaveB(const TYPE &v, FILE *fp) {
    fwrite(&v, sizeof(TYPE), 1, fp);
}
template<class TYPE> inline void SaveB(const TYPE *v, const int N, FILE *fp) {
    if (N > 0) {
        fwrite(v, sizeof(TYPE), N, fp);
    }
}
template<class TYPE> inline void LoadB(TYPE &v, FILE *fp) {
    fread(&v, sizeof(TYPE), 1, fp);
}
template<class TYPE> inline void LoadB(TYPE *v, const int N, FILE *fp) {
    if (N > 0) {
        fread(v, sizeof(TYPE), N, fp);
    }
}
template<typename TYPE> inline TYPE LoadB(FILE *fp) {
    TYPE v;
    LoadB<TYPE>(v, fp);
    return v;
}
template<typename TYPE> inline bool Load(TYPE &v, FILE *fp);
template<> inline bool Load<float>(float &v, FILE *fp) {
    return fscanf(fp, "%f", &v) == 1;
}
template<> inline bool Load<double>(double &v, FILE *fp) {
    return fscanf(fp, "%lf", &v) == 1;
}
template<typename TYPE> inline TYPE Load(FILE *fp) {
    TYPE v;
    if (Load<TYPE>(v, fp)) {
        return v;
    } else {
        return Invalid<TYPE>();
    }
}

template<class TYPE> inline float MemoryMB(const int N = 1) {
    return sizeof(TYPE) * N / 1024.0f / 1024.0f;
}

inline void StringSaveB(const std::string &str, FILE *fp) {
    char buf[UT_STRING_MAX_LENGTH];
    sprintf(buf, "%s\n", str.c_str());  // NOLINT
    fwrite(buf, 1, strlen(buf), fp);
}

inline void StringLoadB(std::string &str, FILE *fp) {
    char buf[UT_STRING_MAX_LENGTH];
    fgets(buf, UT_STRING_MAX_LENGTH, fp);
    const int len = static_cast<int>(strlen(buf));
    if (buf[len - 1] == 10) {
        buf[len - 1] = 0;
    }
    str = buf;
}
inline void StringsSaveB(const std::vector<std::string> &strs, FILE *fp) {
    const int size = static_cast<int>(strs.size());
    SaveB<int>(size, fp);
    for (int i = 0; i < size; ++i) {
        StringSaveB(strs[i], fp);
    }
}

inline void StringsLoadB(std::vector<std::string> &strs, FILE *fp) {
    const int size = LoadB<int>(fp);
    strs.resize(size);
    for (int i = 0; i < size; ++i) {
        StringLoadB(strs[i], fp);
    }
}

inline std::string String(const char* format, ...) {
    char buffer[UT_STRING_MAX_LENGTH];
    UT_GET_STRING(format, buffer);
    std::string str(buffer);
    return str;
}

inline std::string StringInput() {
    char buffer[UT_STRING_MAX_LENGTH];
    fgets(buffer, sizeof(buffer), stdin);
    std::string input(buffer);
    return input;
}

inline std::string StringReplace(const std::string str,
    const std::string strSrc,
    const std::string strDst) {
    std::string::size_type pos;
    std::string res = str;
    while (1) {
        if ((pos = res.find(strSrc)) == std::string::npos) {
            break;
        }
        res.replace(pos, strSrc.length(), strDst);
    }
    return res;
}

inline float StringMemoryMB(const std::string &str) {
    return MemoryMB<char>(static_cast<int>(str.size()));
}

inline std::vector<std::string> Strings(const std::string str0) {
    std::vector<std::string> strs(1);
    strs[0] = str0;
    return strs;
}

inline std::vector<std::string> Strings(const std::string str0,
    const std::string str1) {
    std::vector<std::string> strs(2);
    strs[0] = str0;
    strs[1] = str1;
    return strs;
}

inline std::vector<std::string> Strings(const std::string str0, const std::string str1,
    const std::string str2) {
    std::vector<std::string> strs(3);
    strs[0] = str0;
    strs[1] = str1;
    strs[2] = str2;
    return strs;
}

inline std::vector<std::string> Strings(const std::string str0, const std::string str1,
    const std::string str2, const std::string str3) {
    std::vector<std::string> strs(4);
    strs[0] = str0;
    strs[1] = str1;
    strs[2] = str2;
    strs[3] = str3;
    return strs;
}

inline std::vector<std::string> Strings(const std::string str0, const std::string str1,
    const std::string str2, const std::string str3,
    const std::string str4) {
    std::vector<std::string> strs(5);
    strs[0] = str0;
    strs[1] = str1;
    strs[2] = str2;
    strs[3] = str3;
    strs[4] = str4;
    return strs;
}

inline std::vector<std::string> Strings(const std::string str0, const std::string str1,
    const std::string str2, const std::string str3,
    const std::string str4, const std::string str5) {
    std::vector<std::string> strs(6);
    strs[0] = str0;
    strs[1] = str1;
    strs[2] = str2;
    strs[3] = str3;
    strs[4] = str4;
    strs[5] = str5;
    return strs;
}

inline std::vector<std::string> Strings(const std::string *strs, const int N) {
    std::vector<std::string> _strs(N);
    for (int i = 0; i < N; ++i) {
        _strs[i] = strs[i];
    }
    return _strs;
}

inline std::string FileNameExtractDirectory(const std::string fileName) {
    const std::string::size_type i1 = fileName.rfind('/'),
                                 i2 = fileName.rfind('\\');
    if (i1 == std::string::npos && i2 == std::string::npos) {
        return std::string();
    } else if (i1 != std::string::npos && i2 == std::string::npos) {
        return fileName.substr(0, i1 + 1);
    } else if (i1 == std::string::npos && i2 != std::string::npos) {
        return fileName.substr(0, i2 + 1);
    } else if (i1 > i2) {
        return fileName.substr(0, i1 + 1);
    } else {
        return fileName.substr(0, i2 + 1);
    }
}
inline std::string FileNameRemoveDirectory(const std::string fileName) {
    const std::string::size_type i1 = fileName.rfind('/'),
                                 i2 = fileName.rfind('\\');
    if (i1 == std::string::npos && i2 == std::string::npos) {
        return fileName;
    } else if (i1 != std::string::npos && i2 == std::string::npos) {
        return fileName.substr(i1 + 1, fileName.size());
    } else if (i1 == std::string::npos && i2 != std::string::npos) {
        return fileName.substr(i2 + 1, fileName.size());
    } else if (i1 > i2) {
        return fileName.substr(i1 + 1, fileName.size());
    } else {
        return fileName.substr(i2 + 1, fileName.size());
    }
}
inline std::string FileNameExtractExtension(const std::string fileName) {
    const std::string::size_type i = fileName.rfind('.');
    if (i == std::string::npos) {
        return std::string();
    } else {
        return fileName.substr(i + 1, fileName.size());
    }
}
inline std::string FileNameRemoveExtension(const std::string fileName) {
    const std::string::size_type i = fileName.rfind('.');
    if (i == std::string::npos) {
        return fileName;
    } else {
        return fileName.substr(0, i);
    }
}
inline std::string FileNameRemoveDirectoryExtension(const std::string fileName) {
    return FileNameRemoveDirectory(FileNameRemoveExtension(fileName));
}

inline std::string FileNameRemovePrefix(const std::string fileName,
                                 const std::string prefix) {
    if (fileName == "" || fileName.find(prefix) != 0) {
        return fileName;
    } else {
        return fileName.substr(prefix.length(), fileName.length());
    }
}

inline std::string FileNameAppendSuffix(const std::string fileName,
                                 const std::string suffix) {
    return FileNameRemoveExtension(fileName) + suffix + "." +
           FileNameExtractExtension(fileName);
}

inline std::string FileNameAppendSuffix(const std::string fileName, const int suffix) {
    char buf[UT_STRING_MAX_LENGTH];
    sprintf(buf, "%d", suffix); // NOLINT
    return FileNameAppendSuffix(fileName, buf);
}

inline std::string FileNameReplaceDirectory(const std::string fileName,
                                     const std::string dirSrc, const std::string dirDst) {
    if (fileName == "" || dirSrc != "" && fileName.find(dirSrc) != 0) {
        return fileName;
    } else if (fileName[0] == '.') {
        return dirDst + fileName;
    } else {
        return dirDst + fileName.substr(dirSrc.length(), fileName.length());
    }
}

template<typename TYPE>    inline TYPE FileNameExtractSuffix(
    const std::string fileName);
template<> inline int FileNameExtractSuffix<int>(const std::string fileName) {
    int i2 = static_cast<int>(fileName.length());
    while (--i2 >= 0 && !isdigit(fileName[i2])) { }
    int i1 = ++i2;
    while (--i1 >= 0 && isdigit(fileName[i1])) { }
    if (++i1 == i2) {
        return -1;
    } else {
        return atoi(fileName.substr(i1, i2 - i1).c_str());
    }
}
template<> inline double FileNameExtractSuffix<double>(const std::string
        fileName) {
    const std::string _fileName = FileNameRemoveDirectoryExtension(
                                      fileName).c_str();
    const int N = static_cast<int>(_fileName.length());
    for (int i = 0; i < N; ++i) {
        if (isdigit(_fileName[i])) {
            return atof(_fileName.substr(i, N - i).c_str());
        }
    }
    return DBL_MAX;
}

inline std::string FileNameIncreaseSuffix(const std::string fileName, const int incr) {
    const int len = static_cast<int>(fileName.length());
    int i2 = len;
    while (--i2 >= 0 && !isdigit(fileName[i2])) {}
    int i1 = ++i2;
    while (--i1 >= 0 && isdigit(fileName[i1])) { }
    const int number = ++i1 == i2 ? incr : atoi(fileName.substr(i1,
                       i2 - i1).c_str()) + incr;
    const int width1 = i2 - i1, width2 = static_cast<int>(log10f(static_cast<float>(number)));  // NOLINT
    const int width = width1 > width2 ? width1 : width2;

    char buf[10];
    switch (width) {
    case 2:
        sprintf(buf, "%.2d", number);   // NOLINT
        break;
    case 3:
        sprintf(buf, "%.3d", number);   // NOLINT
        break;
    case 4:
        sprintf(buf, "%.4d", number);   // NOLINT
        break;
    case 5:
        sprintf(buf, "%.5d", number);   // NOLINT
        break;
    case 6:
        sprintf(buf, "%.6d", number);   // NOLINT
        break;
    case 7:
        sprintf(buf, "%.7d", number);   // NOLINT
        break;
    case 8:
        sprintf(buf, "%.8d", number);   // NOLINT
        break;
    case 9:
        sprintf(buf, "%.9d", number);   // NOLINT
        break;
    case 10:
        sprintf(buf, "%.10d", number);   // NOLINT
        break;
    default:
        sprintf(buf, "%d", number);     // NOLINT
        break;
    }
    return fileName.substr(0, i1) + buf + fileName.substr(i2, len - i2);
}

inline bool FileExists(const char *filename) {
    return _access(filename, 0) == 0;
}

inline bool FileExists(const std::string fileName) {
    return FileExists(fileName.c_str());
}

inline bool FileCopy(const std::string fileNameSrc, const std::string fileNameDst) {
    if (FileExists(fileNameDst)) {
        return false;
    }
    char command[UT_STRING_MAX_LENGTH];
    sprintf(command, "copy %s %s", StringReplace(fileNameSrc, "/", "\\").c_str(),   // NOLINT
        StringReplace(fileNameDst, "/", "\\").c_str());
    printf("%s\n", command);
#ifdef _WIN32
    system(command);
#endif
    return true;
}
inline bool FileDelete(const std::string fileName, const bool check = true,
    const bool verbose = true) {
#ifdef CFG_DEBUG
    UT_ASSERT(FileExists(fileName));
#endif
    char command[UT_STRING_MAX_LENGTH];
    sprintf(command, "del %s", fileName.c_str());   // NOLINT
    if (check) {
        char input[UT_STRING_MAX_LENGTH];
        printf("Delete \'%s\'? (Y/N) ", fileName.c_str());
        scanf("%s", input);
        if (strlen(input) != 1 || input[0] != 'Y' && input[0] != 'y') {
            return false;
        }
    }
#ifdef WIN32
    if (verbose) {
        printf("Deleted \'%s\'\n", fileName.c_str());
    }
    system(command);
#endif
    return true;
}

inline std::vector<std::string> FilesSearch(const std::string fileNameFirst,
    const int iStart = 0, const int iStep = 1,
    const int iEnd = INT_MAX,
    const bool verbose = true) {
    std::vector<std::string> fileNames;
#ifdef _WIN32
    if (FileNameRemoveDirectoryExtension(fileNameFirst) == "*") {
        const std::string dir = FileNameExtractDirectory(fileNameFirst);
        _finddata_t fd;
        intptr_t fh = _findfirst(fileNameFirst.c_str(), &fd);
        if (fh == -1) {
            return fileNames;
        }
        do {
            const std::string fileName = dir + fd.name;
            fileNames.push_back(fileName);
            if (verbose) {
                printf("\rFound \'%s\'", fileName.c_str());
            }
        } while (_findnext(fh, &fd) == 0);

        int i, j;
        const int N = static_cast<int>(fileNames.size());
        for (i = iStart, j = 0; i < N && i <= iEnd; i += iStep) {
            fileNames[j++] = fileNames[i];
        }
        fileNames.resize(j);
    } else {
        int i = iStart;
        while (i <= iEnd) {
            const std::string fileName = FileNameIncreaseSuffix(fileNameFirst, i);
            if (!FileExists(fileName)) {
                break;
            }
            fileNames.push_back(fileName);
            if (verbose) {
                printf("\rFound \'%s\'", fileName.c_str());
            }
            if (iStep == 0) {
                break;
            }
            i += iStep;
        }
    }
    if (verbose) {
        if (fileNames.empty()) {
            printf("Not found \'%s\'", fileNameFirst.c_str());
        }
        printf("\n");
    }
#endif
    return fileNames;
}
inline void FilesStartSaving(const std::string fileNameFirst, const bool check = true,
    const bool verbose = true) {
#ifdef _WIN32
    const std::string dir = FileNameExtractDirectory(fileNameFirst);
    if (FileExists(dir)) {
        const std::vector<std::string> fileNames = FilesSearch(fileNameFirst, 0, 1,
            INT_MAX, verbose);
        const int N = static_cast<int>(fileNames.size());
        for (int i = 0; i < N; ++i) {
            if (!FileDelete(fileNames[i], check && i == 0, verbose)) {
                break;
            }
        }
    } else {
        CreateDirectory(dir.c_str(), 0);
    }
#endif
}

inline void GetFileNames(std::string path, std::vector<std::string> &filenames) {
    if (path.empty()) {
        std::cout << "path is NULL" << std::endl;
        return;
    }
#ifdef _WIN32
    intptr_t hFile = 0;
    struct _finddata_t fileinfo;
    std::string p;
    if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1) {
        do {
            //如果是目录,迭代之（即文件夹内还有文件夹）
            if ((fileinfo.attrib &  _A_SUBDIR)) {
                //文件名不等于"."&&文件名不等于".."
                //.表示当前目录
                //..表示当前目录的父目录
                //判断时，两者都要忽略，不然就无限递归跳不出去了！
                // if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
                //     getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
            } else {
                // 文件名
                filenames.push_back(fileinfo.name);
            }
        } while (_findnext(hFile, &fileinfo) == 0);
        // _findclose函数结束查找
        _findclose(hFile);
    }
#else   // linux
    DIR *dp;
    struct dirent *dirp;
    std::string file;

    // check if dir is valid
    struct stat s;
    lstat(path.c_str(), &s);
    if (!S_ISDIR(s.st_mode)) {
        std::cout << "dir is not valid" << std::endl;
        return;
    }

    if ((dp = opendir(path.c_str())) == NULL) {
        std::cout << "can't open " << path << std::endl;
        return;
    }

    while ((dirp = readdir(dp)) != NULL) {
        if (dirp->d_type == 8) {
            file = std::string(dirp->d_name);
            // 文件名
            filenames.push_back(file);
        }
    }

    closedir(dp);
#endif
    // sort with file name ASCII
    std::sort(filenames.begin(), filenames.end());
}

inline bool DirectoryCreate(const char* dir) {
    std::string path(dir);
    if (path.empty())
        return false;
    if (path.back() != '\\' && path.back() != '/')
        path.push_back('/');
    for (int i = 1; i < path.size(); i++) {
        if (path[i] == '\\' || path[i] == '/' || i == path.size()-1) {
            path[i] = '\0';
            if (!FileExists(path)) {
                if (MKDIR(path.c_str()) == -1)
                    return false;
            }
            path[i] = '/';
        }
    }
    printf("create dir %s succeed.\n", dir);
    return true;
}

template<class TYPE_SRC, class TYPE_DST>
inline std::vector<const TYPE_DST *> Pointers(const std::vector<TYPE_SRC> &V) {
    std::vector<const TYPE_DST *> Vptr;
    const int N = static_cast<int>(V.size());
    Vptr.resize(N);
    for (int i = 0; i < N; ++i) {
        Vptr[i] = &V[i];
    }
    return Vptr;
}
template<class TYPE_SRC_1, class TYPE_SRC_2, class TYPE_DST>
inline std::vector<const TYPE_DST *> Pointers(const std::vector<TYPE_SRC_1> &V1,
        const std::vector<TYPE_SRC_2> &V2) {
    std::vector<const TYPE_DST *> Vptr;
    const int N1 = static_cast<int>(V1.size());
    for (int i = 0; i < N1; ++i) {
        Vptr.push_back(&V1[i]);
    }
    const int N2 = static_cast<int>(V2.size());
    for (int i = 0; i < N2; ++i) {
        Vptr.push_back(&V2[i]);
    }
    return Vptr;
}
inline void SaveValues(const char *fileName, const std::vector<float> &vals) {
    FILE *fp = fopen(fileName, "w");
    const int N = static_cast<int>(vals.size());
    for (int i = 0; i < N; ++i) {
        fprintf(fp, "%f\n", vals[i]);
    }
    fclose(fp);
}

inline void SaveHistogram(const char *fileName, const std::vector<float> &vals,
                   const int nBins) {
    int i;
    float val, valMin = FLT_MAX, valMax = -FLT_MAX;
    const int N = static_cast<int>(vals.size());
    for (i = 0; i < N; ++i) {
        val = vals[i];
        if (val < valMin) {
            valMin = val;
        }
        if (val > valMax) {
            valMax = val;
        }
    }
    const float binWidth = (valMax - valMin) / (nBins - 1);

    int iBin;
    std::vector<int> hist(N, 0);
    for (i = 0; i < N; ++i) {
        iBin = static_cast<int>((vals[i] - valMin) / binWidth);
        ++hist[iBin];
    }

    FILE *fp = fopen(fileName, "w");
    for (iBin = 0, val = valMin; iBin < nBins; ++iBin, val += binWidth) {
        fprintf(fp, "%f %d\n", val, hist[iBin]);
    }
    fclose(fp);
}

template<typename TYPE> inline bool Equal(const TYPE v1, const TYPE v2,
        const TYPE eps = 0) {
    return eps == 0 && v1 == v2 || eps > 0 && (
               v1 > v2 && v1 - v2 <= eps || v1 < v2 && v2 - v1 <= eps);
}
template<> inline bool Equal<float>(const float v1, const float v2,
                                    const float eps) {
    const float e = fabs(v1 - v2), va1 = fabs(v1), va2 = fabs(v2),
                vMin = std::min(va1, va2), vMax = std::max(va1, va2);
    const float vd = vMin >= UT_FLT_EPSILON_MIN ? vMin : vMax, er = e / vd;
    return e < eps || e <= UT_FLT_EPSILON_MIN || er <= UT_FLT_EPSILON_RATIO_MIN
           || e <= UT_FLT_EPSILON_MAX && er <= UT_FLT_EPSILON_RATIO_MAX;
}
template<> inline bool Equal<double>(const double v1, const double v2,
                                     const double eps) {
    const double e = fabs(v1 - v2), va1 = fabs(v1), va2 = fabs(v2),
                 vMin = std::min(va1, va2), vMax = std::max(va1, va2);
    const double vd = vMin >= UT_DBL_EPSILON_MIN ? vMin : vMax, er = e / vd;
    return e < eps || e <= UT_DBL_EPSILON_MIN || er <= UT_DBL_EPSILON_RATIO_MIN
           || e <= UT_DBL_EPSILON_MAX && er <= UT_DBL_EPSILON_RATIO_MAX;
}

template<typename TYPE> inline bool AssertEqual(const TYPE v1, const TYPE v2,
        const int verbose = 1, const TYPE eps = 0) {
    if (Equal<TYPE>(v1, v2, eps)) {
        return true;
    }
    if (verbose > 0) {
        PrintSeparator();
        PrintError<TYPE>(v1, v2, verbose > 1);
    }
    return Equal(v1, v2, eps);
}

template<class TYPE> inline void VectorMakeZero(std::vector<TYPE> &V) {
    memset(V.data(), 0, sizeof(TYPE) * V.size());
}
template<class TYPE> inline void VectorMakeZero(std::vector<TYPE> &V,
        const int i1, const int i2) {
    memset(V.data() + i1, 0, sizeof(TYPE) * (i2 - i1));
}
template<class TYPE_1, class TYPE_2> inline bool VectorEqual(const TYPE_1 *v1,
        const TYPE_2 *v2, const int N) {
    bool equal = true;
    for (int i = 0; i < N && equal; ++i) {
        equal = v1[i] == v2[i];
    }
    return equal;
}
template<class TYPE_1, class TYPE_2> inline bool VectorEqual(
    const std::vector<TYPE_1> &V1, const std::vector<TYPE_2> &V2) {
    return V1.size() == V2.size()
           && VectorEqual<TYPE_1, TYPE_2>(V1.data(), V2.data(),
                                          static_cast<int>(V1.size()));
}

template<typename TYPE> inline bool VectorAssertEqual(const TYPE *v1,
        const TYPE *v2, const int N, const int verbose = 1, const TYPE eps = 0) {
    bool equal = true;
    for (int i = 0; i < N && equal; ++i) {
        equal = AssertEqual<TYPE>(v1[i], v2[i], 1, eps);
    }
    if (!equal && verbose > 0) {
        const bool e = verbose > 1;
        PrintSeparator();
        for (int i = 0; i < N; ++i) {
            PrintError<TYPE>(v1[i], v2[i], e);
        }
    }
    return equal;
}
template<class TYPE_1, class TYPE_2>
inline void VectorAssertEqual(
    const std::vector<TYPE_1> &V1, const std::vector<TYPE_2> &V2) {
    UT_ASSERT(VectorEqual(V1, V2));
}
template<class TYPE>
inline std::vector<TYPE> VectorRepeat(
    const std::vector<TYPE> &V, const int N) {
    if (N <= 0) {
        return std::vector<TYPE>();
    } else if (N == 1) {
        return V;
    }
    std::vector<TYPE> _V;
    for (int i = 0; i < N; ++i) {
        _V.insert(_V.end(), V.begin(), V.end());
    }
    return _V;
}
template<class TYPE> inline void VectorSaveB(const std::vector<TYPE> &V,
        FILE *fp) {
    const int N = static_cast<int>(V.size());
    SaveB<int>(N, fp);
    SaveB<TYPE>(V.data(), N, fp);
}
template<class TYPE> inline bool VectorSaveB(const std::vector<TYPE> &V,
        const char *fileName) {
    FILE *fp = fopen(fileName, "wb");
    if (!fp) {
        return false;
    }
    VectorSaveB<TYPE>(V, fp);
    fclose(fp);
    PrintSaved(fileName);
    return true;
}
template<class TYPE> inline int VectorLoadB(std::vector<TYPE> &V, FILE *fp) {
    const int N = LoadB<int>(fp);
    V.resize(N);
    LoadB<TYPE>(V.data(), N, fp);
    return N;
}
template<class TYPE> inline bool VectorLoadB(std::vector<TYPE> &V,
        const char *fileName) {
    FILE *fp = fopen(fileName, "rb");
    if (!fp) {
        return false;
    }
    VectorLoadB<TYPE>(V, fp);
    fclose(fp);
    PrintLoaded(fileName);
    return true;
}
template<class TYPE> inline void VectorsSaveB(const
        std::vector<std::vector<TYPE> > &Vs, FILE *fp) {
    const int N = static_cast<int>(Vs.size());
    SaveB<int>(N, fp);
    for (int i = 0; i < N; ++i) {
        VectorSaveB(Vs[i], fp);
    }
}
template<class TYPE> inline void VectorsLoadB(std::vector<std::vector<TYPE> >
        &Vs, FILE *fp) {
    const int N = LoadB<int>(fp);
    Vs.resize(N);
    for (int i = 0; i < N; ++i) {
        VectorLoadB(Vs[i], fp);
    }
}
template<class TYPE> inline float VectorMemoryMB(const std::vector<TYPE> &V) {
    return MemoryMB<TYPE>(static_cast<int>(V.capacity()));
}
template<class TYPE> inline float VectorsMemoryMB(const
        std::vector<std::vector<TYPE> > &Vs) {
    float sum = 0.0f;
    const int N = static_cast<int>(Vs.size());
    for (int i = 0; i < N; ++i) {
        sum = VectorMemoryMB(Vs[i]) + sum;
    }
    return sum;
}

template<class TYPE> inline void ListSaveB(const std::list<TYPE> &L, FILE *fp) {
    const int N = static_cast<int>(L.size());
    SaveB<int>(N, fp);
    for (typename std::list<TYPE>::const_iterator it = L.begin(); it != L.end();
            ++it) {
        SaveB<TYPE>(*it, fp);
    }
}
template<class TYPE> inline void ListLoadB(std::list<TYPE> &L, FILE *fp) {
    const int N = LoadB<int>(fp);
    L.resize(N);
    for (typename std::list<TYPE>::iterator it = L.begin(); it != L.end(); ++it) {
        LoadB<TYPE>(*it, fp);
    }
}

enum CornerType {
    CORNER_LEFT_TOP,
    CORNER_LEFT_DOWN,
    CORNER_RIGHT_DOWN,
    CORNER_RIGHT_TOP,
    CORNER_TYPES
};

template<class TYPE> inline bool CheckReduction(const TYPE &e1, const TYPE &e2,
        const int verbose = 1, const float eMin = FLT_EPSILON,
        const float eMax = FLT_MAX) {
    const float e1_2 = e1.SquaredLength();
    if (e1_2 < eMin * eMin) {
        return true;
    }
    if (e1_2 > eMax * eMax) {
        return false;
    }
    const float e2_2 = e2.SquaredLength();
    if (e1_2 >= e2_2) {
        return true;
    }
    if (verbose > 0) {
        PrintSeparator();
        if (verbose > 1) {
            printf("    %e: ", e1_2);
            e1.printf("%s", true);
            printf("--> %e: ", e2_2);
            e2.printf("%s", true);
        } else {
            printf("    %f: ", e1_2);
            e1.printf("%s", false);
            printf("--> %f: ", e2_2);
            e2.printf("%s", false);
        }
    }
    return false;
}
template<> inline bool CheckReduction<float>(const float &e1, const float &e2,
        const int verbose, const float eMin, const float eMax) {
    const float e1_a = fabs(e1);
    if (e1_a < eMin) {
        return true;
    }
    if (e1_a > eMax) {
        return false;
    }
    const float e2_a = fabs(e2);
    if (e1_a >= e2_a) {
        return true;
    }
    if (verbose > 0) {
        PrintSeparator();
        if (verbose > 1) {
            printf("    %e --> %e\n", e1, e2);
        } else {
            printf("    %f --> %f\n", e1, e2);
        }
    }
    return false;
}

template<typename TYPE> inline TYPE Input();
template<> inline std::string Input<std::string>() {
    std::string input;
    sscanf("%s", input.c_str());
    return input;
}
template<> inline int Input<int>() {
    const std::string input = Input<std::string>();
    int _input;
    sscanf(input.c_str(), "%d", &_input);
    return _input;
}
template<> inline char Input<char>() {
    const std::string input = Input<std::string>();
    char _input;
    sscanf(input.c_str(), "%c", &_input);
    return _input;
}
template<typename TYPE> inline TYPE Input(const char *str) {
    printf("%s << ", str);
    return Input<TYPE>();
}

template<typename TYPE> inline float ESSquaredLength(const TYPE &e) {
    return e.SquaredLength();
}
template<> inline float ESSquaredLength<float>(const float &e) {
    return e * e;
}
template<typename TYPE> inline void ESErrorPrint(const TYPE &e,
        const bool l = true) {
    e.Print(l);
}
template<> inline void ESErrorPrint<float>(const float &e, const bool l) {
    printf("%f", e);
}
template<class INDEX> inline void ESIndexPrint(const INDEX &idx) {
    idx.Print();
}
template<> inline void ESIndexPrint<int>(const int &idx) {
    if (idx != -1) {
        printf(" [%d]", idx);
    }
}

template<typename TYPE, typename INDEX> class ES {
public:
    inline void Initialize() {
        m_Se2 = m_Swe2 = 0.0f;
        m_SN = 0;
        m_e2Max = m_we2Max = -1.0f;
    }
    void Accumulate(const TYPE &e, const float we2 = -1.0f,
                           const INDEX idx = -1) {
        const float e2 = ESSquaredLength<TYPE>(e);
        if (e2 == -1.0f) {
            m_Se2 = -1.0f;
        } else {
            m_Se2 = e2 + m_Se2;
        }
        if (we2 == -1.0f) {
            m_Swe2 = -1.0f;
        } else {
            m_Swe2 = we2 + m_Swe2;
        }
        ++m_SN;
        if (we2 == -1.0f && e2 > m_e2Max || we2 != -1.0f && we2 > m_we2Max) {
            m_e2Max = e2;
            m_we2Max = we2;
            m_eMax = e;
            m_idxMax = idx;
        }
    }
    inline bool Valid() const {
        return m_SN > 0;
    }
    inline float Average() const {
        return m_SN == 0 ? 0.0f : sqrt(m_Se2 / m_SN);
    }
    inline void Print(const std::string str = "", const bool l = true,
                       const bool n = true) const {
        // if(!Valid())
        //     return;
        printf("%s", str.c_str());
        if (m_Swe2 != -1.0f) {
            if (l) {
                printf("%e", m_Swe2);
            } else {
                printf("%.2e", m_Swe2);
            }
        }
        if (m_Se2 != -1.0f) {
            if (m_Swe2 != -1.0f) {
                printf(" <-- ");
            }
            const float eAvg = Average();
            if (l) {
                printf("%f", eAvg);
            } else {
                printf("%.2f", eAvg);
            }
        }
        printf(" <= ");
        ESErrorPrint(m_eMax, l);
        if (m_we2Max != -1.0f) {
            printf(" --> ");
            if (l) {
                printf("%e", m_we2Max);
            } else {
                printf("%.2e", m_we2Max);
            }
        }
        ESIndexPrint<INDEX>(m_idxMax);
        if (n) {
            printf("\n");
        }
    }

public:
    float m_Se2, m_Swe2;
    int m_SN;
    float m_e2Max, m_we2Max;
    TYPE m_eMax;
    INDEX m_idxMax;
};
}  // namespace UT

#endif  // COLLECT_CAR_UTILITY_UTILITY_HPP
