// OV2640 camera pin assignments for the Seeed XIAO ESP32-S3 Sense.
// Reference: Seeed wiki "XIAO ESP32-S3 Sense — Camera Usage" — these are
// the on-board hardwired connections to the OV2640 module on the Sense
// daughterboard. Don't edit unless the board changes.

#pragma once

#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   10
#define SIOD_GPIO_NUM   40   // I2C SDA to OV2640 SCCB
#define SIOC_GPIO_NUM   39   // I2C SCL to OV2640 SCCB

#define Y9_GPIO_NUM     48
#define Y8_GPIO_NUM     11
#define Y7_GPIO_NUM     12
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     16
#define Y4_GPIO_NUM     18
#define Y3_GPIO_NUM     17
#define Y2_GPIO_NUM     15

#define VSYNC_GPIO_NUM  38
#define HREF_GPIO_NUM   47
#define PCLK_GPIO_NUM   13
