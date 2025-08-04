#ifndef APP_H
#define APP_H
#include "mosfet_driver.h"
#include "digital_PI_rI_V2.h"
#include "stdint.h"
#include "stdbool.h"
void excute();
void handle();
void app_init();
uint8_t duty;
float vout[3];
#endif
