// Copyright (c) 2023-2025 TANGAIR
// SPDX-License-Identifier: Apache-2.0
#ifndef LINGZU_USB_CAN_SDK_H
#define LINGZU_USB_CAN_SDK_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define STANDARD 0
#define EXTENDED 1
#pragma pack(push, 1)
typedef struct FrameInfo {
    uint32_t canID;
    uint8_t frameType;
    uint8_t dataLength;
} FrameInfo;
#pragma pack(pop)
int32_t openUSBCAN(const char *devName);
int32_t closeUSBCAN(int32_t dev);
// Checked classic transport: complete fixed packet = 17; failure = -1.
int32_t sendUSBCAN(int32_t dev, uint8_t channel, FrameInfo* info, uint8_t *data);
// timeout in microseconds; complete CRC-validated frame = 0, otherwise -1.
int32_t readUSBCAN(int32_t dev, uint8_t *channel, FrameInfo* info, uint8_t *data, int32_t timeout);
#ifdef __cplusplus
}
#endif
#endif
