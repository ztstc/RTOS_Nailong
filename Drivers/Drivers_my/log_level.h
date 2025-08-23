// log_level.h
#ifndef LOG_LEVEL_H
#define LOG_LEVEL_H

#include <stdio.h>

// 定义日志等级
typedef enum {
    LOG_LEVEL_NONE = 0,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG
} LogLevel;

// 声明全局变量
extern LogLevel current_log_level;

// 定义日志宏
#define LOG_ERROR(fmt, ...) if (current_log_level >= LOG_LEVEL_ERROR) { printf("ERROR: " fmt "\n", ##__VA_ARGS__); }
#define LOG_INFO(fmt, ...)  if (current_log_level >= LOG_LEVEL_INFO)  { printf("INFO: " fmt "\n", ##__VA_ARGS__); }
#define LOG_DEBUG(fmt, ...) if (current_log_level >= LOG_LEVEL_DEBUG) { printf("DEBUG: " fmt "\n", ##__VA_ARGS__); }

#endif
/*
先设置日志等级
 LOG_DEBUG("This is a debug message");
 LOG_INFO("This is an info message");
 LOG_ERROR("This is an error message");
*/

