// My_Json.h
#ifndef __MYJSON_H
#define __MYJSON_H

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include "log_level.h"
#include <jansson.h>

extern uint16_t requestservo[10];
extern float speedL;
extern float speedR;

void parseJson(const char *json);


#endif
