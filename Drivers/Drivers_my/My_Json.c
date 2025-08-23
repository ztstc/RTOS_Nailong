// My_Json.c
#include "My_Json.h"
#include <jansson.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

void parseJson(const char *json) {
    // 空字符串保护
    if (json == NULL || json[0] == '\0') {
        LOG_ERROR("parseJson input is empty!\n");
        return;
    }
    // 调试：打印收到的字符串
    LOG_DEBUG("parseJson input: %s\n", json);

    // 解析 JSON 字符串
    json_error_t error;
    json_t *root = json_loads(json, 0, &error);

    if (!root) {
        LOG_ERROR("Failed to parse JSON: %s\n", error.text);
        return;
    }

    if (!json_is_object(root) && (current_log_level<LOG_LEVEL_NONE)) {
        fprintf(stderr, "JSON is not an object. Type: %d\n", json_typeof(root));
        json_decref(root);
        return;
    }

    // 遍历 JSON 对象的所有键值对
    const char *key;
    json_t *value;
    json_object_foreach(root, key, value) {
        const char *value_str = NULL;

        if (json_is_string(value)) {
            value_str = json_string_value(value);
        } else if (json_is_integer(value)) {
            char buffer[32];
            sprintf(buffer, "%lld", json_integer_value(value));
            value_str = buffer;
        } else if (json_is_real(value)) {
            char buffer[32];
            sprintf(buffer, "%.1f", json_real_value(value));
            value_str = buffer;
        } else if (json_is_boolean(value)) {
            value_str = json_boolean_value(value) ? "true" : "false";
        } else if (json_is_null(value)) {
            value_str = "null";
        } else {
            fprintf(stderr, "Unsupported value type for key: %s\n", key);
            continue;
        }

        if (strcmp(key, "motorL") == 0) {
            speedL = atof(value_str);
        } else if (strcmp(key, "motorR") == 0) {
            speedR = atof(value_str);
        } else if (strcmp(key, "Servo0") == 0) {
            requestservo[0] = atoi(value_str);
        } else if (strcmp(key, "Servo1") == 0) {
            requestservo[1] = atoi(value_str);
        }
    }
    LOG_INFO("motorL: %.1f\n", speedL);
    LOG_INFO("motorR: %.1f\n", speedR);
    LOG_INFO("Servo0: %d\n", requestservo[0]);
    LOG_INFO("Servo1: %d\n", requestservo[1]);

    json_decref(root);
}
