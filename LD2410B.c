
/* BEGIN Header */
/**
 ******************************************************************************
 * \file            LD2410B.c
 * \author          Andrea Vivani
 * \brief           Encoder / decoder for LD2410-B radar sensor frames
 ******************************************************************************
 * Copyright 2025 Andrea Vivani
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the “Software”), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************
 */
/* END Header */

/* Includes ------------------------------------------------------------------*/
#include "LD2410B.h"
#include <string.h>

/* Macros --------------------------------------------------------------------*/

#define LD2410B_FRAME_HEADER_CMD           0xFAFBFCFDU
#define LD2410B_FRAME_FOOTER_CMD           0x01020304U
#define LD2410B_FRAME_HEADER_DATA          0xF1F2F3F4U
#define LD2410B_FRAME_FOOTER_DATA          0xF5F6F7F8U

#define LD2410B_CMD_ENABLE_CONFIG          0x00FFU
#define LD2410B_CMD_END_CONFIG             0x00FEU
#define LD2410B_CMD_WRITE_MAX_DISTANCE     0x0060U
#define LD2410B_CMD_READ_PARAMS            0x0061U
#define LD2410B_CMD_ENABLE_ENGINEERING     0x0062U
#define LD2410B_CMD_DISABLE_ENGINEERING    0x0063U
#define LD2410B_CMD_WRITE_GATE_SENSITIVITY 0x0064U
#define LD2410B_CMD_READ_VERSION           0x00A0U
#define LD2410B_CMD_SET_BAUDRATE           0x00A1U
#define LD2410B_CMD_FACTORY_RESET          0x00A2U
#define LD2410B_CMD_RESTART                0x00A3U
#define LD2410B_CMD_BLUETOOTH_CONTROL      0x00A4U
#define LD2410B_CMD_GET_MAC                0x00A5U
#define LD2410B_CMD_BT_GET_PERMISSION      0x00A8U
#define LD2410B_CMD_BT_SET_PASSWORD        0x00A9U
#define LD2410B_CMD_SET_DISTANCE_RES       0x00AAU
#define LD2410B_CMD_QUERY_DISTANCE_RES     0x00ABU
#define LD2410B_CMD_AUX_CONTROL_SET        0x00ADU
#define LD2410B_CMD_AUX_CONTROL_QUERY      0x00AEU

/* Private Functions ---------------------------------------------------------*/

static inline uint8_t writeU32Le(uint8_t* buf, uint32_t val) {
    buf[0] = (uint8_t)(val & 0xFFU);
    buf[1] = (uint8_t)((val >> 8) & 0xFFU);
    buf[2] = (uint8_t)((val >> 16) & 0xFFU);
    buf[3] = (uint8_t)((val >> 24) & 0xFFU);

    return sizeof(uint32_t);
}

static inline uint8_t writeU16Le(uint8_t* buf, uint16_t val) {
    buf[0] = (uint8_t)(val & 0xFFU);
    buf[1] = (uint8_t)((val >> 8) & 0xFFU);

    return sizeof(uint16_t);
}

static inline uint8_t readU32Le(const uint8_t* buf, uint32_t* val) {
    *val = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);

    return sizeof(uint32_t);
}

static inline uint8_t readU16Le(const uint8_t* buf, uint16_t* val) {
    *val = (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));

    return sizeof(uint16_t);
}

/* Build command header; returns status */
static LD2410B_Status_t buildHeader(uint8_t* buffer, uint16_t size, uint16_t dataLen) {
    if (size < dataLen + 10U) { /* 6 header bytes + 4 footer bytes */
        return LD2410B_ERROR_BUFFER_TOO_SMALL;
    }

    writeU32Le(buffer, LD2410B_FRAME_HEADER_CMD);
    writeU16Le(buffer + 4, dataLen);
    return LD2410B_SUCCESS;
}

static LD2410B_Status_t verifyCmdFrame(const uint8_t* buffer, uint16_t size) {
    if (size < 10U) {
        return LD2410B_ERROR_INVALID_LENGTH;
    }

    uint32_t header, footer;
    readU32Le(buffer, &header);
    if (header != LD2410B_FRAME_HEADER_CMD) {
        return LD2410B_ERROR_INVALID_HEADER;
    }

    uint16_t dataLen;
    readU16Le(buffer + 4, &dataLen);
    if (size < dataLen + 10U) {
        return LD2410B_ERROR_INVALID_SIZE;
    }
    if (dataLen < 4U) {
        return LD2410B_ERROR_INVALID_LENGTH;
    }

    readU32Le(buffer + dataLen + 6U, &footer);
    if (footer != LD2410B_FRAME_FOOTER_CMD) {
        return LD2410B_ERROR_INVALID_FOOTER;
    }

    return LD2410B_SUCCESS;
}

/* Functions -----------------------------------------------------------------*/

uint8_t LD2410B_buildSetConfigOn(uint8_t* buffer, uint16_t size) {
    if (buildHeader(buffer, size, 4) != LD2410B_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U; /* Starting offset, after header */
    off += writeU16Le(buffer + off, LD2410B_CMD_ENABLE_CONFIG);
    off += writeU16Le(buffer + off, 0x0001U);
    off += writeU32Le(buffer + off, LD2410B_FRAME_FOOTER_CMD);

    return off;
}

uint8_t LD2410B_buildSetConfigOff(uint8_t* buffer, uint16_t size) {
    if (buildHeader(buffer, size, 2) != LD2410B_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U; /* Starting offset, after header */
    off += writeU16Le(buffer + off, LD2410B_CMD_END_CONFIG);
    off += writeU32Le(buffer + off, LD2410B_FRAME_FOOTER_CMD);

    return off;
}

uint8_t LD2410B_buildSetMaxDistance(uint8_t* buffer, uint16_t size, uint8_t maxMotionGate, uint8_t maxStaticGate, uint16_t timeoutSeconds) {
    if (maxMotionGate < 2U || maxMotionGate > LD2410B_MAX_GATE) {
        return 0;
    }
    if (maxStaticGate < 2U || maxStaticGate > LD2410B_MAX_GATE) {
        return 0;
    }

    if (buildHeader(buffer, size, 20) != LD2410B_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U; /* Starting offset, after header */
    off += writeU16Le(buffer + off, LD2410B_CMD_WRITE_MAX_DISTANCE);
    off += writeU16Le(buffer + off, 0x0000U);
    off += writeU32Le(buffer + off, maxMotionGate);
    off += writeU16Le(buffer + off, 0x0001U);
    off += writeU32Le(buffer + off, maxStaticGate);
    off += writeU16Le(buffer + off, 0x0002U);
    off += writeU32Le(buffer + off, timeoutSeconds);
    off += writeU32Le(buffer + off, LD2410B_FRAME_FOOTER_CMD);

    return off;
}

uint8_t LD2410B_buildGetParams(uint8_t* buffer, uint16_t size) {
    if (buildHeader(buffer, size, 2) != LD2410B_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U; /* Starting offset, after header */
    off += writeU16Le(buffer + off, LD2410B_CMD_READ_PARAMS);
    off += writeU32Le(buffer + off, LD2410B_FRAME_FOOTER_CMD);

    return off;
}

uint8_t LD2410B_buildSetEngineeringOn(uint8_t* buffer, uint16_t size) {
    if (buildHeader(buffer, size, 2) != LD2410B_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U; /* Starting offset, after header */
    off += writeU16Le(buffer + off, LD2410B_CMD_ENABLE_ENGINEERING);
    off += writeU32Le(buffer + off, LD2410B_FRAME_FOOTER_CMD);

    return off;
}

uint8_t LD2410B_buildSetEngineeringOff(uint8_t* buffer, uint16_t size) {
    if (buildHeader(buffer, size, 2) != LD2410B_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U; /* Starting offset, after header */
    off += writeU16Le(buffer + off, LD2410B_CMD_DISABLE_ENGINEERING);
    off += writeU32Le(buffer + off, LD2410B_FRAME_FOOTER_CMD);

    return off;
}

uint8_t LD2410B_buildSetSensitivity(uint8_t* buffer, uint16_t size, uint16_t gate, uint8_t motion_sensitivity, uint8_t static_sensitivity) {
    if (gate > LD2410B_MAX_GATE && gate != LD2410B_ALL_GATES) {
        return 0;
    }

    if (buildHeader(buffer, size, 20) != LD2410B_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U; /* Starting offset, after header */
    off += writeU16Le(buffer + off, LD2410B_CMD_WRITE_GATE_SENSITIVITY);
    /* Use spec layout: gate word + gate value + motion word + motion value + static word + static value */
    off += writeU16Le(buffer + off, 0x0000U); /* distance gate word */
    off += writeU32Le(buffer + off, gate);
    off += writeU16Le(buffer + off, 0x0001U); /* motion sensitivity word */
    off += writeU32Le(buffer + off, motion_sensitivity);
    off += writeU16Le(buffer + off, 0x0002U); /* static sensitivity word */
    off += writeU32Le(buffer + off, static_sensitivity);
    off += writeU32Le(buffer + off, LD2410B_FRAME_FOOTER_CMD);

    return off;
}

uint8_t LD2410B_buildGetFWVersion(uint8_t* buffer, uint16_t size) {
    if (buildHeader(buffer, size, 2) != LD2410B_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U; /* Starting offset, after header */
    off += writeU16Le(buffer + off, LD2410B_CMD_READ_VERSION);
    off += writeU32Le(buffer + off, LD2410B_FRAME_FOOTER_CMD);

    return off;
}

uint8_t LD2410B_buildSetBaudrate(uint8_t* buffer, uint16_t size, LD2410B_Baudrate_t baudrate) {
    if (buildHeader(buffer, size, 4) != LD2410B_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U; /* Starting offset, after header */
    off += writeU16Le(buffer + off, LD2410B_CMD_SET_BAUDRATE);
    off += writeU16Le(buffer + off, (uint16_t)baudrate);
    off += writeU32Le(buffer + off, LD2410B_FRAME_FOOTER_CMD);

    return off;
}

uint8_t LD2410B_buildSetFactoryReset(uint8_t* buffer, uint16_t size) {
    if (buildHeader(buffer, size, 2) != LD2410B_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U; /* Starting offset, after header */
    off += writeU16Le(buffer + off, LD2410B_CMD_FACTORY_RESET);
    off += writeU32Le(buffer + off, LD2410B_FRAME_FOOTER_CMD);

    return off;
}

uint8_t LD2410B_buildSetRestart(uint8_t* buffer, uint16_t size) {
    if (buildHeader(buffer, size, 2) != LD2410B_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U; /* Starting offset, after header */
    off += writeU16Le(buffer + off, LD2410B_CMD_RESTART);
    off += writeU32Le(buffer + off, LD2410B_FRAME_FOOTER_CMD);

    return off;
}

uint8_t LD2410B_buildSetBluetoothControl(uint8_t* buffer, uint16_t size, LD2410B_BluetoothControl_t enable) {
    if (buildHeader(buffer, size, 4) != LD2410B_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U; /* Starting offset, after header */
    off += writeU16Le(buffer + off, LD2410B_CMD_BLUETOOTH_CONTROL);
    off += writeU16Le(buffer + off, (enable == LD2410B_BT_ENABLE) ? 1U : 0U);
    off += writeU32Le(buffer + off, LD2410B_FRAME_FOOTER_CMD);

    return off;
}

uint8_t LD2410B_buildGetMacAddress(uint8_t* buffer, uint16_t size) {
    if (buildHeader(buffer, size, 4) != LD2410B_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U; /* Starting offset, after header */
    off += writeU16Le(buffer + off, LD2410B_CMD_GET_MAC);
    off += writeU16Le(buffer + off, 0x0001U);
    off += writeU32Le(buffer + off, LD2410B_FRAME_FOOTER_CMD);

    return off;
}

uint8_t LD2410B_buildObtainBTPermission(uint8_t* buffer, uint16_t size, const uint8_t password[6]) {
    if (password == NULL) {
        return 0;
    }

    if (buildHeader(buffer, size, 8) != LD2410B_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U; /* Starting offset, after header */
    off += writeU16Le(buffer + off, LD2410B_CMD_BT_GET_PERMISSION);
    /* Password is 6 bytes */
    memcpy(buffer + off, password, 6);
    off += 6U;
    off += writeU32Le(buffer + off, LD2410B_FRAME_FOOTER_CMD);

    return off;
}

uint8_t LD2410B_buildSetBTPassword(uint8_t* buffer, uint16_t size, const uint8_t password[6]) {
    if (password == NULL) {
        return 0;
    }

    if (buildHeader(buffer, size, 8) != LD2410B_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U; /* Starting offset, after header */
    off += writeU16Le(buffer + off, LD2410B_CMD_BT_SET_PASSWORD);
    /* Password is 6 bytes */
    memcpy(buffer + off, password, 6);
    off += 6U;
    off += writeU32Le(buffer + off, LD2410B_FRAME_FOOTER_CMD);

    return off;
}

uint8_t LD2410B_buildSetDistanceResolution(uint8_t* buffer, uint16_t size, LD2410B_DistanceResolution_t res) {
    if (buildHeader(buffer, size, 4) != LD2410B_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U; /* Starting offset, after header */
    off += writeU16Le(buffer + off, LD2410B_CMD_SET_DISTANCE_RES);
    off += writeU16Le(buffer + off, (res == LD2410B_RES_075M) ? 0U : 1U);
    off += writeU32Le(buffer + off, LD2410B_FRAME_FOOTER_CMD);

    return off;
}

uint8_t LD2410B_buildGetDistanceResolution(uint8_t* buffer, uint16_t size) {
    if (buildHeader(buffer, size, 2) != LD2410B_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U; /* Starting offset, after header */
    off += writeU16Le(buffer + off, LD2410B_CMD_QUERY_DISTANCE_RES);
    off += writeU32Le(buffer + off, LD2410B_FRAME_FOOTER_CMD);

    return off;
}

uint8_t LD2410B_buildSetAuxControl(uint8_t* buffer, uint16_t size, const LD2410B_LightSense_t lightSense, const uint8_t threshold, const LD2410B_OutputPin_t outputPin) {
    if (buildHeader(buffer, size, 6) != LD2410B_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U; /* Starting offset, after header */
    off += writeU16Le(buffer + off, LD2410B_CMD_AUX_CONTROL_SET);
    buffer[off++] = (uint8_t)lightSense;
    buffer[off++] = threshold;
    buffer[off++] = (uint8_t)outputPin;
    buffer[off++] = 0x00U;
    off += writeU32Le(buffer + off, LD2410B_FRAME_FOOTER_CMD);

    return off;
}

uint8_t LD2410B_buildGetAuxControl(uint8_t* buffer, uint16_t size) {
    if (buildHeader(buffer, size, 2) != LD2410B_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U; /* Starting offset, after header */
    off += writeU16Le(buffer + off, LD2410B_CMD_AUX_CONTROL_QUERY);
    off += writeU32Le(buffer + off, LD2410B_FRAME_FOOTER_CMD);

    return off;
}

LD2410B_Status_t LD2410B_checkSetACK(const uint8_t* txBuffer, uint16_t txSize, const uint8_t* rxBuffer, uint16_t rxSize) {
    if ((txBuffer == NULL) || (rxBuffer == NULL)) {
        return LD2410B_ERROR_NULL_PTR;
    }

    if (txSize < 12U) {
        return LD2410B_ERROR_INVALID_LENGTH;
    }

    LD2410B_Status_t status = verifyCmdFrame(rxBuffer, rxSize);
    if (status != LD2410B_SUCCESS) {
        return status;
    }

    uint16_t txCmd, rxCmd;
    readU16Le(txBuffer + 6, &txCmd);
    readU16Le(rxBuffer + 6, &rxCmd);
    if (rxCmd != (txCmd | 0x0100U)) {
        return LD2410B_ERROR_INVALID_DATA;
    }

    /* FIXED: Check ACK status byte at offset 8 */
    uint16_t ackStatus;
    readU16Le(rxBuffer + 8, &ackStatus);
    if (ackStatus != 0x0000U) {
        return LD2410B_ERROR_INVALID_DATA;
    }

    return LD2410B_SUCCESS;
}

LD2410B_Status_t LD2410B_parseGetParams(const uint8_t* buffer, uint16_t size, LD2410B_ConfigParam_t* config) {
    if (config == NULL) {
        return LD2410B_ERROR_NULL_PTR;
    }

    if (size < 38U) {
        return LD2410B_ERROR_INVALID_LENGTH;
    }

    LD2410B_Status_t status = verifyCmdFrame(buffer, size);
    if (status != LD2410B_SUCCESS) {
        return status;
    }

    uint8_t off = 6U;
    uint16_t rxCmd;
    off += readU16Le(buffer + off, &rxCmd);
    if (rxCmd != (LD2410B_CMD_READ_PARAMS | 0x0100U)) {
        return LD2410B_ERROR_INVALID_DATA;
    }
    off += 2U; /* skip ACK status bytes */

    if (buffer[off++] != 0xAAU) {
        return LD2410B_ERROR_INVALID_DATA;
    }

    config->maxDistGate = buffer[off++];
    config->maxMotionGate = buffer[off++];
    config->maxStaticGate = buffer[off++];

    /* Validate gate values before using as loop bounds */
    if (config->maxMotionGate > LD2410B_MAX_GATE || config->maxStaticGate > LD2410B_MAX_GATE) {
        return LD2410B_ERROR_INVALID_DATA;
    }

    for (uint8_t ii = 0; ii <= config->maxMotionGate; ii++) {
        config->gateSensitivity[ii].motion = buffer[off++];
    }
    for (uint8_t ii = 0; ii <= config->maxStaticGate; ii++) {
        config->gateSensitivity[ii].stationary = buffer[off++];
    }
    readU16Le(buffer + off, &(config->timeoutSeconds));

    return LD2410B_SUCCESS;
}

LD2410B_Status_t LD2410B_parseGetFWVersion(const uint8_t* buffer, uint16_t size, LD2410B_Version_t* version) {
    if (version == NULL) {
        return LD2410B_ERROR_NULL_PTR;
    }

    if (size < 22U) {
        return LD2410B_ERROR_INVALID_LENGTH;
    }

    LD2410B_Status_t status = verifyCmdFrame(buffer, size);
    if (status != LD2410B_SUCCESS) {
        return status;
    }

    uint8_t off = 6U;
    uint16_t rxCmd;
    off += readU16Le(buffer + off, &rxCmd);
    if (rxCmd != (LD2410B_CMD_READ_VERSION | 0x0100U)) {
        return LD2410B_ERROR_INVALID_DATA;
    }
    off += 2U; /* skip ACK status bytes */

    off += readU16Le(buffer + off, &(version->type));
    off += readU16Le(buffer + off, &(version->major));
    off += readU32Le(buffer + off, &(version->minor));

    return LD2410B_SUCCESS;
}

LD2410B_Status_t LD2410B_parseGetMacAddress(const uint8_t* buffer, uint16_t size, uint8_t mac[6]) {
    if (mac == NULL) {
        return LD2410B_ERROR_NULL_PTR;
    }

    if (size < 20U) {
        return LD2410B_ERROR_INVALID_LENGTH;
    }

    LD2410B_Status_t status = verifyCmdFrame(buffer, size);
    if (status != LD2410B_SUCCESS) {
        return status;
    }

    uint8_t off = 6U;
    uint16_t rxCmd;
    off += readU16Le(buffer + off, &rxCmd);
    if (rxCmd != (LD2410B_CMD_GET_MAC | 0x0100U)) {
        return LD2410B_ERROR_INVALID_DATA;
    }
    off += 2U; /* skip ACK status bytes */

    memcpy(mac, buffer + off, 6);

    return LD2410B_SUCCESS;
}

LD2410B_Status_t LD2410B_parseGetDistanceResolution(const uint8_t* buffer, uint16_t size, LD2410B_DistanceResolution_t* res) {
    if (res == NULL) {
        return LD2410B_ERROR_NULL_PTR;
    }

    if (size < 16U) {
        return LD2410B_ERROR_INVALID_LENGTH;
    }

    LD2410B_Status_t status = verifyCmdFrame(buffer, size);
    if (status != LD2410B_SUCCESS) {
        return status;
    }

    uint8_t off = 6U;
    uint16_t rxCmd;
    off += readU16Le(buffer + off, &rxCmd);
    if (rxCmd != (LD2410B_CMD_QUERY_DISTANCE_RES | 0x0100U)) {
        return LD2410B_ERROR_INVALID_DATA;
    }
    off += 2U; /* skip ACK status bytes */

    uint16_t resVal;
    readU16Le(buffer + off, &resVal);
    *res = (resVal == 0U) ? LD2410B_RES_075M : LD2410B_RES_020M;

    return LD2410B_SUCCESS;
}

LD2410B_Status_t LD2410B_parseGetAuxControl(const uint8_t* buffer, uint16_t size, LD2410B_LightSense_t* lightSense, uint8_t* threshold, LD2410B_OutputPin_t* outputPin) {
    if (size < 18U) {
        return LD2410B_ERROR_INVALID_LENGTH;
    }

    LD2410B_Status_t status = verifyCmdFrame(buffer, size);
    if (status != LD2410B_SUCCESS) {
        return status;
    }

    uint8_t off = 6U;
    uint16_t rxCmd;
    off += readU16Le(buffer + off, &rxCmd);
    if (rxCmd != (LD2410B_CMD_AUX_CONTROL_QUERY | 0x0100U)) {
        return LD2410B_ERROR_INVALID_DATA;
    }
    off += 2U; /* skip ACK status bytes */

    if (lightSense != NULL) {
        *lightSense = (LD2410B_LightSense_t)buffer[off];
    }
    off++;

    if (threshold != NULL) {
        *threshold = buffer[off];
    }
    off++;

    if (outputPin != NULL) {
        *outputPin = (LD2410B_OutputPin_t)buffer[off];
    }

    return LD2410B_SUCCESS;
}

LD2410B_Status_t LD2410B_parseRadarData(const uint8_t* buffer, uint16_t size, LD2410B_Target_t* target) {
    if (buffer == NULL) {
        return LD2410B_ERROR_NULL_PTR;
    }

    if (target == NULL) {
        return LD2410B_ERROR_NULL_PTR;
    }

    if (size < 10U) {
        return LD2410B_ERROR_INVALID_LENGTH;
    }

    uint8_t off = 0;

    uint32_t header, footer;
    off += readU32Le(buffer + off, &header);

    if (header != LD2410B_FRAME_HEADER_DATA) {
        return LD2410B_ERROR_INVALID_HEADER;
    }

    uint16_t dataLen;
    off += readU16Le(buffer + off, &dataLen);

    if (size < dataLen + 10U) {
        return LD2410B_ERROR_INVALID_SIZE;
    }

    readU32Le(buffer + dataLen + 6U, &footer);
    if (footer != LD2410B_FRAME_FOOTER_DATA) {
        return LD2410B_ERROR_INVALID_FOOTER;
    }

    uint8_t data_type = buffer[off++];
    if (!(data_type == 0x01U || data_type == 0x02U)) {
        return LD2410B_ERROR_INVALID_DATA;
    }

    if (buffer[off++] != 0xAAU) {
        return LD2410B_ERROR_INVALID_DATA;
    }

    target->state = (LD2410B_TargetState_t)buffer[off++];
    off += readU16Le(buffer + off, &(target->motion_distance_cm));
    target->motion_energy = buffer[off++];
    off += readU16Le(buffer + off, &(target->static_distance_cm));
    target->static_energy = buffer[off++];
    off += readU16Le(buffer + off, &(target->detection_distance_cm));

    if (data_type == 0x01U) {
        /* Engineering mode */
        target->maxMotionGate = buffer[off++];
        target->maxStaticGate = buffer[off++];

        if (target->maxMotionGate > LD2410B_MAX_GATE || target->maxStaticGate > LD2410B_MAX_GATE) {
            return LD2410B_ERROR_INVALID_DATA;
        }

        /* Check buffer bounds before reading gate energies */
        uint16_t requiredSize = (uint16_t)(off + target->maxMotionGate + 1U + target->maxStaticGate + 1U + 2U);
        if (size < requiredSize) {
            return LD2410B_ERROR_INVALID_SIZE;
        }

        for (uint8_t ii = 0; ii <= target->maxMotionGate; ii++) {
            target->gateEnergy[ii].motion = buffer[off++];
        }
        for (uint8_t ii = 0; ii <= target->maxStaticGate; ii++) {
            target->gateEnergy[ii].stationary = buffer[off++];
        }
    } else {
        /* Basic mode - zero out engineering data */
        target->maxMotionGate = 0;
        target->maxStaticGate = 0;
    }

    target->light_value = buffer[off++];
    target->out_state = buffer[off++];

    return LD2410B_SUCCESS;
}