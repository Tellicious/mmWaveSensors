/* BEGIN Header */
/**
 ******************************************************************************
 * \file            LD2420.c
 * \author          Andrea Vivani
 * \brief           Encoder / decoder for HLK-LD2420 radar sensor frames
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
#include "LD2420.h"
#include <string.h>

/* Macros --------------------------------------------------------------------*/

/* Command frames: header FD FC FB FA (u32=0xFAFBFCFD), footer 04 03 02 01 */
#define LD2420_FRAME_HEADER_CMD 0xFAFBFCFDU
#define LD2420_FRAME_FOOTER_CMD 0x01020304U

/* Upload-mode report frames: header F4 F3 F2 F1 (u32=0xF1F2F3F4), footer F8 F7 F6 F5 (u32=0xF5F6F7F8) */
#define LD2420_FRAME_HEADER_UP  0xF1F2F3F4U
#define LD2420_FRAME_FOOTER_UP  0xF5F6F7F8U

/* Commands (HLK-LD2420 user manual v1.3.2) */
#define LD2420_CMD_READ_FW_VERSION 0x0000U
#define LD2420_CMD_ENABLE_CONFIG   0x00FFU
#define LD2420_CMD_END_CONFIG      0x00FEU
#define LD2420_CMD_SET_PARAMS      0x0007U
#define LD2420_CMD_GET_PARAMS      0x0008U

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

static LD2420_Status_t buildHeader(uint8_t* buffer, uint16_t size, uint16_t dataLen) {
    if (!buffer) {
        return LD2420_ERROR_NULL_PTR;
    }
    if (size < (uint16_t)(dataLen + 10U)) {
        return LD2420_ERROR_BUFFER_TOO_SMALL;
    }
    writeU32Le(buffer, LD2420_FRAME_HEADER_CMD);
    writeU16Le(buffer + 4, dataLen);
    return LD2420_SUCCESS;
}

static LD2420_Status_t verifyCmdFrame(const uint8_t* buffer, uint16_t size, uint16_t* outDataLen) {
    if (!buffer) {
        return LD2420_ERROR_NULL_PTR;
    }
    if (size < 10U) {
        return LD2420_ERROR_INVALID_LENGTH;
    }

    uint32_t header;
    readU32Le(buffer, &header);
    if (header != LD2420_FRAME_HEADER_CMD) {
        return LD2420_ERROR_INVALID_HEADER;
    }

    uint16_t dataLen;
    readU16Le(buffer + 4, &dataLen);
    if (outDataLen) {
        *outDataLen = dataLen;
    }

    if (dataLen < 2U) {
        return LD2420_ERROR_INVALID_LENGTH;
    }
    if (size < (uint16_t)(dataLen + 10U)) {
        return LD2420_ERROR_INVALID_SIZE;
    }

    uint32_t footer;
    readU32Le(buffer + dataLen + 6U, &footer);
    if (footer != LD2420_FRAME_FOOTER_CMD) {
        return LD2420_ERROR_INVALID_FOOTER;
    }
    return LD2420_SUCCESS;
}

/**
 * Parse ACK for LD2420 commands.
 *
 * ACK format per examples:
 * - cmd word ACK = (cmd | 0x0100)
 * - 2-byte ACK status (0 = success, 1 = failed)
 * - optional return data
 */
static LD2420_Status_t parseAck(const uint8_t* buffer,
                                uint16_t size,
                                uint16_t expectedCmd,
                                uint16_t* outAckStatus,
                                const uint8_t** outPayload,
                                uint16_t* outPayloadLen) {
    uint16_t dataLen = 0;
    LD2420_Status_t st = verifyCmdFrame(buffer, size, &dataLen);
    if (st != LD2420_SUCCESS) {
        return st;
    }

    uint16_t cmd;
    readU16Le(buffer + 6, &cmd);
    if (cmd != (uint16_t)(expectedCmd | 0x0100U)) {
        return LD2420_ERROR_INVALID_DATA;
    }

    if (dataLen < 4U) {
        return LD2420_ERROR_INVALID_LENGTH;
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

    return (ack == 0U) ? LD2420_SUCCESS : LD2420_ERROR_CMD_FAILED;
}

static uint8_t buildSimpleCmd(uint8_t* buffer, uint16_t size, uint16_t cmdWord) {
    if (buildHeader(buffer, size, 2U) != LD2420_SUCCESS) {
        return 0;
    }
    uint8_t off = 6U;
    off += writeU16Le(buffer + off, cmdWord);
    off += writeU32Le(buffer + off, LD2420_FRAME_FOOTER_CMD);
    return off;
}

/* Public API ----------------------------------------------------------------*/

uint8_t LD2420_buildSetConfigOn(uint8_t* buffer, uint16_t size) {
    if (buildHeader(buffer, size, 4U) != LD2420_SUCCESS) {
        return 0;
    }
    uint8_t off = 6U;
    off += writeU16Le(buffer + off, LD2420_CMD_ENABLE_CONFIG);
    off += writeU16Le(buffer + off, 0x0001U);
    off += writeU32Le(buffer + off, LD2420_FRAME_FOOTER_CMD);
    return off;
}

uint8_t LD2420_buildSetConfigOff(uint8_t* buffer, uint16_t size) {
    return buildSimpleCmd(buffer, size, LD2420_CMD_END_CONFIG);
}

uint8_t LD2420_buildGetFWVersion(uint8_t* buffer, uint16_t size) {
    return buildSimpleCmd(buffer, size, LD2420_CMD_READ_FW_VERSION);
}

uint8_t LD2420_buildSetParams(uint8_t* buffer, uint16_t size, const uint16_t* ids, const uint32_t* values, uint8_t n) {
    if (!buffer || !ids || !values) {
        return 0;
    }
    if (n == 0U) {
        return 0;
    }

    /* dataLen = cmd(2) + n*(id(2) + value(4)) */
    uint16_t dataLen = (uint16_t)(2U + (uint16_t)n * 6U);
    if (buildHeader(buffer, size, dataLen) != LD2420_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U;
    off += writeU16Le(buffer + off, LD2420_CMD_SET_PARAMS);
    for (uint8_t i = 0; i < n; i++) {
        off += writeU16Le(buffer + off, ids[i]);
        off += writeU32Le(buffer + off, values[i]);
    }
    off += writeU32Le(buffer + off, LD2420_FRAME_FOOTER_CMD);
    return off;
}

uint8_t LD2420_buildGetParams(uint8_t* buffer, uint16_t size, const uint16_t* ids, uint8_t n) {
    if (!buffer || !ids) {
        return 0;
    }
    if (n == 0U) {
        return 0;
    }

    /* dataLen = cmd(2) + n*(id(2)) */
    uint16_t dataLen = (uint16_t)(2U + (uint16_t)n * 2U);
    if (buildHeader(buffer, size, dataLen) != LD2420_SUCCESS) {
        return 0;
    }

    uint8_t off = 6U;
    off += writeU16Le(buffer + off, LD2420_CMD_GET_PARAMS);
    for (uint8_t i = 0; i < n; i++) {
        off += writeU16Le(buffer + off, ids[i]);
    }
    off += writeU32Le(buffer + off, LD2420_FRAME_FOOTER_CMD);
    return off;
}

LD2420_Status_t LD2420_checkSetACK(const uint8_t* txBuffer, uint16_t txSize, const uint8_t* rxBuffer, uint16_t rxSize) {
    if (!txBuffer || !rxBuffer) {
        return LD2420_ERROR_NULL_PTR;
    }
    if (txSize < 8U) {
        return LD2420_ERROR_INVALID_LENGTH;
    }
    uint16_t txCmd;
    readU16Le(txBuffer + 6, &txCmd);

    uint16_t ackStatus;
    LD2420_Status_t st = parseAck(rxBuffer, rxSize, txCmd, &ackStatus, NULL, NULL);
    (void)ackStatus;
    return st;
}

LD2420_Status_t LD2420_parseGetFWVersion(const uint8_t* buffer, uint16_t size, LD2420_Version_t* version) {
    if (!version) {
        return LD2420_ERROR_NULL_PTR;
    }

    uint16_t ack;
    const uint8_t* payload;
    uint16_t plen;
    LD2420_Status_t st = parseAck(buffer, size, LD2420_CMD_READ_FW_VERSION, &ack, &payload, &plen);
    if (st != LD2420_SUCCESS) {
        return st;
    }
    if (plen < 6U) {
        return LD2420_ERROR_INVALID_LENGTH;
    }
    readU16Le(payload + 0, &version->major);
    readU16Le(payload + 2, &version->minor);
    readU16Le(payload + 4, &version->patch);
    return LD2420_SUCCESS;
}

LD2420_Status_t LD2420_parseGetParams(const uint8_t* buffer, uint16_t size, uint32_t* values, uint8_t n) {
    if (!values) {
        return LD2420_ERROR_NULL_PTR;
    }
    if (n == 0U) {
        return LD2420_ERROR_INVALID_DATA;
    }

    uint16_t ack;
    const uint8_t* payload;
    uint16_t plen;
    LD2420_Status_t st = parseAck(buffer, size, LD2420_CMD_GET_PARAMS, &ack, &payload, &plen);
    if (st != LD2420_SUCCESS) {
        return st;
    }

    /* Each value is 4 bytes */
    if (plen < (uint16_t)((uint16_t)n * 4U)) {
        return LD2420_ERROR_INVALID_LENGTH;
    }

    for (uint8_t i = 0; i < n; i++) {
        uint32_t v;
        readU32Le(payload + (uint16_t)i * 4U, &v);
        values[i] = v;
    }
    return LD2420_SUCCESS;
}

LD2420_Status_t LD2420_parseRadarData(const uint8_t* buffer, uint16_t size, LD2420_Target_t* target) {
    if (!buffer || !target) {
        return LD2420_ERROR_NULL_PTR;
    }

    /* Upload-mode frame (Table 5-7):
     * header(4) + length(2) + detect(1) + dist(2) + energies(32) + footer(4)
     */
    const uint16_t minSize = (uint16_t)(4U + 2U + 1U + 2U + 32U + 4U);
    if (size < minSize) {
        return LD2420_ERROR_INVALID_LENGTH;
    }

    uint32_t header;
    readU32Le(buffer, &header);
    if (header != LD2420_FRAME_HEADER_UP) {
        return LD2420_ERROR_INVALID_HEADER;
    }

    uint16_t dataLen;
    readU16Le(buffer + 4, &dataLen);
    if (size < (uint16_t)(dataLen + 10U)) {
        return LD2420_ERROR_INVALID_SIZE;
    }

    uint32_t footer;
    readU32Le(buffer + dataLen + 6U, &footer);
    if (footer != LD2420_FRAME_FOOTER_UP) {
        return LD2420_ERROR_INVALID_FOOTER;
    }

    /* dataLen is the sum of (detect result + target distance + gate energies), per Table 5-7 */
    if (dataLen < (uint16_t)(1U + 2U + 32U)) {
        return LD2420_ERROR_INVALID_LENGTH;
    }

    target->presence = buffer[6];
    readU16Le(buffer + 7, &target->target_distance_cm);

    const uint8_t* p = buffer + 9;
    for (uint8_t i = 0; i < 16U; i++) {
        uint16_t e;
        readU16Le(p + (uint16_t)i * 2U, &e);
        target->gate_energy[i] = e;
    }

    return LD2420_SUCCESS;
}
