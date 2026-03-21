/* BEGIN Header */
/**
 ******************************************************************************
 * \file            LD2410S.c
 * \author          Andrea Vivani
 * \brief           Encoder / decoder for HLK-LD2410S radar sensor frames
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
#include "LD2410S.h"
#include <string.h>

/* Macros --------------------------------------------------------------------*/

/* Command frames: header FD FC FB FA (little-endian u32 = 0xFAFBFCFD), footer 04 03 02 01 */
#define LD2410S_FRAME_HEADER_CMD 0xFAFBFCFDU
#define LD2410S_FRAME_FOOTER_CMD 0x01020304U

/* Standard report frames: header F4 F3 F2 F1 (u32=0xF1F2F3F4), footer F8 F7 F6 F5 (u32=0xF5F6F7F8) */
#define LD2410S_FRAME_HEADER_STD 0xF1F2F3F4U
#define LD2410S_FRAME_FOOTER_STD 0xF5F6F7F8U

/* Minimal report frames: head 0x6E, tail 0x62 */
#define LD2410S_FRAME_HEAD_MIN 0x6EU
#define LD2410S_FRAME_TAIL_MIN 0x62U

/* Commands (HLK-LD2410S serial communication protocol v1.00) */
#define LD2410S_CMD_READ_FW_VERSION    0x0000U
#define LD2410S_CMD_ENABLE_CONFIG      0x00FFU
#define LD2410S_CMD_END_CONFIG         0x00FEU
#define LD2410S_CMD_WRITE_SN           0x0010U
#define LD2410S_CMD_READ_SN            0x0011U
#define LD2410S_CMD_SET_COMMON_PARAMS  0x0070U
#define LD2410S_CMD_GET_COMMON_PARAMS  0x0071U
#define LD2410S_CMD_SET_TRIG_THRESH    0x0072U
#define LD2410S_CMD_GET_TRIG_THRESH    0x0073U
#define LD2410S_CMD_SET_HOLD_THRESH    0x0076U
#define LD2410S_CMD_GET_HOLD_THRESH    0x0077U
#define LD2410S_CMD_SET_OUTPUT_MODE    0x007AU

/* Common parameter words (Table 2-2) */
#define LD2410S_PW_FARTHEST_GATE        0x0005U
#define LD2410S_PW_NEAREST_GATE         0x000AU
#define LD2410S_PW_UNATTENDED_DELAY_S   0x0006U
#define LD2410S_PW_STATUS_REPORT_FREQ   0x0002U
#define LD2410S_PW_DISTANCE_REPORT_FREQ 0x000CU
#define LD2410S_PW_RESPONSE_SPEED       0x000BU

/* Private helpers -----------------------------------------------------------*/

static inline uint8_t writeU32Le(uint8_t* buf, uint32_t val) {
    buf[0] = (uint8_t)(val & 0xFFU);
    buf[1] = (uint8_t)((val >> 8) & 0xFFU);
    buf[2] = (uint8_t)((val >> 16) & 0xFFU);
    buf[3] = (uint8_t)((val >> 24) & 0xFFU);
    return (uint8_t)sizeof(uint32_t);
}

static inline uint8_t writeU16Le(uint8_t* buf, uint16_t val) {
    buf[0] = (uint8_t)(val & 0xFFU);
    buf[1] = (uint8_t)((val >> 8) & 0xFFU);
    return (uint8_t)sizeof(uint16_t);
}

static inline uint8_t readU32Le(const uint8_t* buf, uint32_t* val) {
    *val = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    return (uint8_t)sizeof(uint32_t);
}

static inline uint8_t readU16Le(const uint8_t* buf, uint16_t* val) {
    *val = (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));
    return (uint8_t)sizeof(uint16_t);
}

static LD2410S_Status_t buildHeader(uint8_t* buffer, uint16_t size, uint16_t dataLen) {
    if (!buffer) {
        return LD2410S_ERROR_NULL_PTR;
    }
    if (size < (uint16_t)(dataLen + 10U)) {
        return LD2410S_ERROR_BUFFER_TOO_SMALL;
    }
    writeU32Le(buffer, LD2410S_FRAME_HEADER_CMD);
    writeU16Le(buffer + 4, dataLen);
    return LD2410S_SUCCESS;
}

static LD2410S_Status_t verifyCmdFrame(const uint8_t* buffer, uint16_t size, uint16_t* outDataLen) {
    if (!buffer) {
        return LD2410S_ERROR_NULL_PTR;
    }
    if (size < 10U) {
        return LD2410S_ERROR_INVALID_LENGTH;
    }

    uint32_t header;
    readU32Le(buffer, &header);
    if (header != LD2410S_FRAME_HEADER_CMD) {
        return LD2410S_ERROR_INVALID_HEADER;
    }

    uint16_t dataLen;
    readU16Le(buffer + 4, &dataLen);
    if (outDataLen) {
        *outDataLen = dataLen;
    }

    if (dataLen < 2U) {
        return LD2410S_ERROR_INVALID_LENGTH;
    }
    if (size < (uint16_t)(dataLen + 10U)) {
        return LD2410S_ERROR_INVALID_SIZE;
    }

    uint32_t footer;
    readU32Le(buffer + dataLen + 6U, &footer);
    if (footer != LD2410S_FRAME_FOOTER_CMD) {
        return LD2410S_ERROR_INVALID_FOOTER;
    }
    return LD2410S_SUCCESS;
}

/**
 * Parse ACK for LD2410S commands.
 *
 * ACK format (typical):
 * - cmd word ACK = (cmd | 0x0100)
 * - 2-byte ACK status (0 = success, 1 = failed)
 * - optional return data
 */
static LD2410S_Status_t parseAck(const uint8_t* buffer,
                                 uint16_t size,
                                 uint16_t expectedCmd,
                                 uint16_t* outAckStatus,
                                 const uint8_t** outPayload,
                                 uint16_t* outPayloadLen) {
    uint16_t dataLen = 0;
    LD2410S_Status_t st = verifyCmdFrame(buffer, size, &dataLen);
    if (st != LD2410S_SUCCESS) {
        return st;
    }

    uint16_t cmd;
    readU16Le(buffer + 6, &cmd);
    if (cmd != (uint16_t)(expectedCmd | 0x0100U)) {
        return LD2410S_ERROR_INVALID_DATA;
    }

    if (dataLen < 4U) {
        return LD2410S_ERROR_INVALID_LENGTH;
    }

    uint16_t ack;
    readU16Le(buffer + 8, &ack);
    if (outAckStatus) {
        *outAckStatus = ack;
    }

    if (outPayload) {
        *outPayload = buffer + 10;
    }
    if (outPayloadLen) {
        *outPayloadLen = (uint16_t)(dataLen - 4U);
    }

    return (ack == 0U) ? LD2410S_SUCCESS : LD2410S_ERROR_CMD_FAILED;
}

static uint8_t buildSimpleCmd(uint8_t* buffer, uint16_t size, uint16_t cmdWord) {
    if (buildHeader(buffer, size, 2U) != LD2410S_SUCCESS) {
        return 0;
    }
    uint8_t off = 6U;
    off += writeU16Le(buffer + off, cmdWord);
    off += writeU32Le(buffer + off, LD2410S_FRAME_FOOTER_CMD);
    return off;
}

/* Public API ----------------------------------------------------------------*/

uint8_t LD2410S_buildSetConfigOn(uint8_t* buffer, uint16_t size) {
    if (buildHeader(buffer, size, 4U) != LD2410S_SUCCESS) {
        return 0;
    }
    uint8_t off = 6U;
    off += writeU16Le(buffer + off, LD2410S_CMD_ENABLE_CONFIG);
    off += writeU16Le(buffer + off, 0x0001U);
    off += writeU32Le(buffer + off, LD2410S_FRAME_FOOTER_CMD);
    return off;
}

uint8_t LD2410S_buildSetConfigOff(uint8_t* buffer, uint16_t size) {
    return buildSimpleCmd(buffer, size, LD2410S_CMD_END_CONFIG);
}

uint8_t LD2410S_buildGetFWVersion(uint8_t* buffer, uint16_t size) {
    return buildSimpleCmd(buffer, size, LD2410S_CMD_READ_FW_VERSION);
}

uint8_t LD2410S_buildSetOutputMode(uint8_t* buffer, uint16_t size, LD2410S_OutputMode_t mode) {
    /* command value: 6 bytes
     * Standard: 00 00 01 00 00 00
     * Minimal : 00 00 00 00 00 00
     */
    if (buildHeader(buffer, size, 8U) != LD2410S_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U;
    off += writeU16Le(buffer + off, LD2410S_CMD_SET_OUTPUT_MODE);

    /* fixed first two bytes 0x0000 */
    off += writeU16Le(buffer + off, 0x0000U);
    off += writeU16Le(buffer + off, (mode == LD2410S_OUTPUT_STANDARD) ? 0x0001U : 0x0000U);
    off += writeU16Le(buffer + off, 0x0000U);

    off += writeU32Le(buffer + off, LD2410S_FRAME_FOOTER_CMD);
    return off;
}

uint8_t LD2410S_buildWriteSerialNumber(uint8_t* buffer, uint16_t size, const uint8_t sn8[8]) {
    if (!buffer || !sn8) {
        return 0;
    }
    /* dataLen: cmd(2) + snLen(2) + sn(8) */
    if (buildHeader(buffer, size, 12U) != LD2410S_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U;
    off += writeU16Le(buffer + off, LD2410S_CMD_WRITE_SN);
    off += writeU16Le(buffer + off, 0x0008U);
    memcpy(buffer + off, sn8, 8U);
    off += 8U;
    off += writeU32Le(buffer + off, LD2410S_FRAME_FOOTER_CMD);
    return off;
}

uint8_t LD2410S_buildGetSerialNumber(uint8_t* buffer, uint16_t size) {
    return buildSimpleCmd(buffer, size, LD2410S_CMD_READ_SN);
}

uint8_t LD2410S_buildSetCommonParams(uint8_t* buffer, uint16_t size, const LD2410S_CommonParams_t* params) {
    if (!buffer || !params) {
        return 0;
    }

    /* 6 parameters: each (word 2) + (value 4) => 36 bytes + cmd(2) = 38 */
    const uint16_t dataLen = 38U;
    if (buildHeader(buffer, size, dataLen) != LD2410S_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U;
    off += writeU16Le(buffer + off, LD2410S_CMD_SET_COMMON_PARAMS);

    /* farthest gate */
    off += writeU16Le(buffer + off, LD2410S_PW_FARTHEST_GATE);
    off += writeU32Le(buffer + off, (uint32_t)params->farthest_gate);

    /* nearest gate */
    off += writeU16Le(buffer + off, LD2410S_PW_NEAREST_GATE);
    off += writeU32Le(buffer + off, (uint32_t)params->nearest_gate);

    /* unattended delay */
    off += writeU16Le(buffer + off, LD2410S_PW_UNATTENDED_DELAY_S);
    off += writeU32Le(buffer + off, (uint32_t)params->unattended_delay_s);

    /* status report frequency */
    off += writeU16Le(buffer + off, LD2410S_PW_STATUS_REPORT_FREQ);
    off += writeU32Le(buffer + off, (uint32_t)params->status_report_hz_x10);

    /* distance report frequency */
    off += writeU16Le(buffer + off, LD2410S_PW_DISTANCE_REPORT_FREQ);
    off += writeU32Le(buffer + off, (uint32_t)params->distance_report_hz_x10);

    /* response speed */
    off += writeU16Le(buffer + off, LD2410S_PW_RESPONSE_SPEED);
    off += writeU32Le(buffer + off, (uint32_t)params->response);

    off += writeU32Le(buffer + off, LD2410S_FRAME_FOOTER_CMD);
    return off;
}

uint8_t LD2410S_buildGetCommonParams(uint8_t* buffer, uint16_t size) {
    /* command value: (2-byte parameter word)*N
     * per datasheet example N=6
     */
    if (buildHeader(buffer, size, 14U) != LD2410S_SUCCESS) {
        return 0;
    }
    uint8_t off = 6U;
    off += writeU16Le(buffer + off, LD2410S_CMD_GET_COMMON_PARAMS);
    off += writeU16Le(buffer + off, LD2410S_PW_FARTHEST_GATE);
    off += writeU16Le(buffer + off, LD2410S_PW_NEAREST_GATE);
    off += writeU16Le(buffer + off, LD2410S_PW_UNATTENDED_DELAY_S);
    off += writeU16Le(buffer + off, LD2410S_PW_STATUS_REPORT_FREQ);
    off += writeU16Le(buffer + off, LD2410S_PW_DISTANCE_REPORT_FREQ);
    off += writeU16Le(buffer + off, LD2410S_PW_RESPONSE_SPEED);
    off += writeU32Le(buffer + off, LD2410S_FRAME_FOOTER_CMD);
    return off;
}

uint8_t LD2410S_buildSetTriggerThresholds(uint8_t* buffer, uint16_t size, const uint32_t thresholds[16]) {
    if (!buffer || !thresholds) {
        return 0;
    }
    /* cmd(2) + 16*(param(2)+val(4)) = 2 + 96 = 98 */
    if (buildHeader(buffer, size, 98U) != LD2410S_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U;
    off += writeU16Le(buffer + off, LD2410S_CMD_SET_TRIG_THRESH);
    for (uint16_t i = 0; i < 16U; i++) {
        off += writeU16Le(buffer + off, (uint16_t)i);
        off += writeU32Le(buffer + off, (uint32_t)thresholds[i]);
    }
    off += writeU32Le(buffer + off, LD2410S_FRAME_FOOTER_CMD);
    return off;
}

uint8_t LD2410S_buildGetTriggerThresholds(uint8_t* buffer, uint16_t size) {
    /* cmd(2) + 16*(param(2)) = 34 */
    if (buildHeader(buffer, size, 34U) != LD2410S_SUCCESS) {
        return 0;
    }
    uint8_t off = 6U;
    off += writeU16Le(buffer + off, LD2410S_CMD_GET_TRIG_THRESH);
    for (uint16_t i = 0; i < 16U; i++) {
        off += writeU16Le(buffer + off, (uint16_t)i);
    }
    off += writeU32Le(buffer + off, LD2410S_FRAME_FOOTER_CMD);
    return off;
}

uint8_t LD2410S_buildSetHoldThresholds(uint8_t* buffer, uint16_t size, const uint32_t thresholds[16]) {
    if (!buffer || !thresholds) {
        return 0;
    }
    /* cmd(2) + 16*(param(2)+val(4)) = 98 */
    if (buildHeader(buffer, size, 98U) != LD2410S_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U;
    off += writeU16Le(buffer + off, LD2410S_CMD_SET_HOLD_THRESH);
    for (uint16_t i = 0; i < 16U; i++) {
        off += writeU16Le(buffer + off, (uint16_t)i);
        off += writeU32Le(buffer + off, (uint32_t)thresholds[i]);
    }
    off += writeU32Le(buffer + off, LD2410S_FRAME_FOOTER_CMD);
    return off;
}

uint8_t LD2410S_buildGetHoldThresholds(uint8_t* buffer, uint16_t size) {
    /* cmd(2) + 16*(param(2)) = 34 */
    if (buildHeader(buffer, size, 34U) != LD2410S_SUCCESS) {
        return 0;
    }
    uint8_t off = 6U;
    off += writeU16Le(buffer + off, LD2410S_CMD_GET_HOLD_THRESH);
    for (uint16_t i = 0; i < 16U; i++) {
        off += writeU16Le(buffer + off, (uint16_t)i);
    }
    off += writeU32Le(buffer + off, LD2410S_FRAME_FOOTER_CMD);
    return off;
}

LD2410S_Status_t LD2410S_checkSetACK(const uint8_t* txBuffer, uint16_t txSize, const uint8_t* rxBuffer, uint16_t rxSize) {
    if (!txBuffer || !rxBuffer) {
        return LD2410S_ERROR_NULL_PTR;
    }
    if (txSize < 8U) {
        return LD2410S_ERROR_INVALID_LENGTH;
    }
    uint16_t txCmd;
    readU16Le(txBuffer + 6, &txCmd);

    uint16_t ackStatus;
    LD2410S_Status_t st = parseAck(rxBuffer, rxSize, txCmd, &ackStatus, NULL, NULL);
    (void)ackStatus;
    return st;
}

LD2410S_Status_t LD2410S_parseGetFWVersion(const uint8_t* buffer, uint16_t size, LD2410S_Version_t* version) {
    if (!version) {
        return LD2410S_ERROR_NULL_PTR;
    }

    uint16_t ack;
    const uint8_t* payload;
    uint16_t plen;
    LD2410S_Status_t st = parseAck(buffer, size, LD2410S_CMD_READ_FW_VERSION, &ack, &payload, &plen);
    if (st != LD2410S_SUCCESS) {
        return st;
    }
    if (plen < 6U) {
        return LD2410S_ERROR_INVALID_LENGTH;
    }
    readU16Le(payload + 0, &version->major);
    readU16Le(payload + 2, &version->minor);
    readU16Le(payload + 4, &version->patch);
    return LD2410S_SUCCESS;
}

LD2410S_Status_t LD2410S_parseGetSerialNumber(const uint8_t* buffer, uint16_t size, uint8_t sn8[8]) {
    if (!sn8) {
        return LD2410S_ERROR_NULL_PTR;
    }

    uint16_t ack;
    const uint8_t* payload;
    uint16_t plen;
    LD2410S_Status_t st = parseAck(buffer, size, LD2410S_CMD_READ_SN, &ack, &payload, &plen);
    if (st != LD2410S_SUCCESS) {
        return st;
    }
    /* payload: snLen(2) + sn(8) */
    if (plen < 10U) {
        return LD2410S_ERROR_INVALID_LENGTH;
    }
    uint16_t snLen;
    readU16Le(payload, &snLen);
    if (snLen != 8U) {
        return LD2410S_ERROR_INVALID_DATA;
    }
    memcpy(sn8, payload + 2, 8U);
    return LD2410S_SUCCESS;
}

LD2410S_Status_t LD2410S_parseGetCommonParams(const uint8_t* buffer, uint16_t size, LD2410S_CommonParams_t* params) {
    if (!params) {
        return LD2410S_ERROR_NULL_PTR;
    }

    uint16_t ack;
    const uint8_t* payload;
    uint16_t plen;
    LD2410S_Status_t st = parseAck(buffer, size, LD2410S_CMD_GET_COMMON_PARAMS, &ack, &payload, &plen);
    if (st != LD2410S_SUCCESS) {
        return st;
    }
    /* 6 values of 4 bytes each */
    if (plen < 24U) {
        return LD2410S_ERROR_INVALID_LENGTH;
    }

    uint32_t v;

    readU32Le(payload + 0, &v);
    params->farthest_gate = (uint8_t)v;

    readU32Le(payload + 4, &v);
    params->nearest_gate = (uint8_t)v;

    readU32Le(payload + 8, &v);
    params->unattended_delay_s = (uint16_t)v;

    readU32Le(payload + 12, &v);
    params->status_report_hz_x10 = (uint16_t)v;

    readU32Le(payload + 16, &v);
    params->distance_report_hz_x10 = (uint16_t)v;

    readU32Le(payload + 20, &v);
    params->response = (LD2410S_ResponseSpeed_t)(uint16_t)v;

    return LD2410S_SUCCESS;
}

static LD2410S_Status_t parseThresholds(const uint8_t* buffer, uint16_t size, uint16_t cmd, uint32_t thresholds[16]) {
    if (!thresholds) {
        return LD2410S_ERROR_NULL_PTR;
    }

    uint16_t ack;
    const uint8_t* payload;
    uint16_t plen;
    LD2410S_Status_t st = parseAck(buffer, size, cmd, &ack, &payload, &plen);
    if (st != LD2410S_SUCCESS) {
        return st;
    }
    if (plen < 64U) {
        return LD2410S_ERROR_INVALID_LENGTH;
    }

    for (uint16_t i = 0; i < 16U; i++) {
        uint32_t v;
        readU32Le(payload + (uint16_t)(i * 4U), &v);
        thresholds[i] = v;
    }
    return LD2410S_SUCCESS;
}

LD2410S_Status_t LD2410S_parseGetTriggerThresholds(const uint8_t* buffer, uint16_t size, uint32_t thresholds[16]) {
    return parseThresholds(buffer, size, LD2410S_CMD_GET_TRIG_THRESH, thresholds);
}

LD2410S_Status_t LD2410S_parseGetHoldThresholds(const uint8_t* buffer, uint16_t size, uint32_t thresholds[16]) {
    return parseThresholds(buffer, size, LD2410S_CMD_GET_HOLD_THRESH, thresholds);
}

LD2410S_Status_t LD2410S_parseRadarData(const uint8_t* buffer, uint16_t size, LD2410S_Report_t* report) {
    if (!buffer || !report) {
        return LD2410S_ERROR_NULL_PTR;
    }

    /* Minimal report: head(1)=0x6E, state(1), dist(2), tail(1)=0x62 */
    if (size >= 5U && buffer[0] == LD2410S_FRAME_HEAD_MIN && buffer[4] == LD2410S_FRAME_TAIL_MIN) {
        report->data_type = 0x00U;
        report->target_state = (LD2410S_TargetState_t)buffer[1];
        readU16Le(buffer + 2, &report->object_distance_cm);
        return LD2410S_SUCCESS;
    }

    /* Standard report: header(4) + len(2) + dataType(1) + state(1) + dist(2) + reserved(2) + energy(64) + footer(4) */
    if (size < (4U + 2U + 1U + 1U + 2U + 2U + 64U + 4U)) {
        return LD2410S_ERROR_INVALID_LENGTH;
    }

    uint32_t header;
    readU32Le(buffer, &header);
    if (header != LD2410S_FRAME_HEADER_STD) {
        return LD2410S_ERROR_INVALID_HEADER;
    }

    uint16_t dataLen;
    readU16Le(buffer + 4, &dataLen);
    if (dataLen < (1U + 1U + 2U + 2U + 64U)) {  /* data_type + state + dist + reserved + energy = 70 */
        return LD2410S_ERROR_INVALID_LENGTH;
    }
    if (size < (uint16_t)(dataLen + 10U)) {
        return LD2410S_ERROR_INVALID_SIZE;
    }

    uint32_t footer;
    readU32Le(buffer + dataLen + 6U, &footer);
    if (footer != LD2410S_FRAME_FOOTER_STD) {
        return LD2410S_ERROR_INVALID_FOOTER;
    }

    report->data_type = buffer[6];
    report->target_state = (LD2410S_TargetState_t)buffer[7];
    readU16Le(buffer + 8, &report->object_distance_cm);
    report->reserved[0] = buffer[10];
    report->reserved[1] = buffer[11];
    memcpy(report->energy, buffer + 12, 64U);

    return LD2410S_SUCCESS;
}
