/*!
 * @author: liuyumoney
 * @date: 2018/06/20
 * 时间相关的函数
 * timeval:包含了秒和微秒两个部分
 * timestamp：一般时间戳的单位是秒，这里不加说明都是到毫秒
 * timeStr：精确到毫秒的时间
 */

#ifndef COLLECT_CAR_UTIL_TIME_UTIL_H
#define COLLECT_CAR_UTIL_TIME_UTIL_H

#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <string>
#include <vector>

// get current system time
void GetLocalTime(timeval &tv);

// convert into std format(yyyy-mm-dd hh:mi:ss.ms)
// support yyyy-mm-dd-hh-mi-ss.ms or yyyymmddhhmissms
std::string FormatSTDTimeString(const std::string& inStr);

// convert time string into timeval
// input time string format:yyyy-mm-dd hh-mi-ss[?]ms
bool timeString2timeval(const std::string& timeStr, timeval& tv);

// convert timeval into string with format:yyyy-mm-dd-HH-Mi-SS-ms
void timeval2String(const timeval& tv, std::string& timeStr);
std::string timeval2String(const timeval& tv);

// convert timestamp(ms unit) into string with format:yyyy-mm-dd-HH-Mi-SS-ms
template<typename T>
void timestamp2String(const T& ts, std::string &timeStr);
template<typename T>
std::string timestamp2String(const T&ts);

// convert time string into timestamp(ms unit)
// input time string format:yyyy-mm-dd hh:mi:ss.ms
template<typename T>
bool timeString2timestamp(const std::string& timeStr, T& ts);

// convert timeval into timestamp(ms unit)
template<typename T>
void timeval2timestamp(const timeval& tv, T& ts);

// convert timestamp into timeval
template<typename T>
void timestamp2timeval(const T& ts, timeval& tv);
template<typename T>
timeval timestamp2timeval(const T& ts);

/////// template function definition should in the same file with declare //////

// convert timestamp(ms unit) into string with format:yyyy-mm-dd-HH-Mi-SS-ms
template<typename T>
void timestamp2String(const T& ts, std::string &timeStr) {
    timeval tv;
    timestamp2timeval(ts, tv);
    timeval2String(tv, timeStr);
}

template<typename T>
std::string timestamp2String(const T&ts) {
    std::string str;
    timestamp2String(ts, str);
    return str;
}

// convert time string into timestamp(ms unit)
template<typename T>
bool timeString2timestamp(const std::string& timeStr, T& ts) {
    timeval tv;
    if (!timeString2timeval(timeStr, tv))
        return false;
    timeval2timestamp(tv, ts);
    return true;
}

// convert timeval into timestamp(ms unit)
// T should be double or int64
template<typename T>
void timeval2timestamp(const timeval& tv, T& ts) {
    ts = static_cast<T>(tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

// convert timestamp into timeval
template<typename T>
void timestamp2timeval(const T& ts, timeval& tv) {
    tv.tv_sec = static_cast<__time_t>(ts / 1000);
    tv.tv_usec = static_cast<__suseconds_t>((ts - tv.tv_sec * 1000) * 1000);
}

template<typename T>
timeval timestamp2timeval(const T& ts) {
    timeval tv;
    timestamp2timeval(ts, tv);
    return tv;
}

#endif  // COLLECT_CAR_UTIL_TIME_UTIL_H
