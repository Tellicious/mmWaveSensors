/* BEGIN Header */
/**
 ******************************************************************************
 * \file            LD2410S.h
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

#ifndef __LD2410S_H__
#define __LD2410S_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Typedefs ------------------------------------------------------------------*/

/**
 * Return status
 */
typedef enum {
    LD2410S_SUCCESS = 0,
    LD2410S_ERROR_NULL_PTR,
    LD2410S_ERROR_BUFFER_TOO_SMALL,
    LD2410S_ERROR_INVALID_DATA,
    LD2410S_ERROR_INVALID_LENGTH,
    LD2410S_ERROR_INVALID_HEADER,
    LD2410S_ERROR_INVALID_FOOTER,
    LD2410S_ERROR_INVALID_SIZE,
    LD2410S_ERROR_UNSUPPORTED,
    LD2410S_ERROR_CMD_FAILED, /**< ACK status indicates failure */
} LD2410S_Status_t;

/**
 * Output mode (command 0x007A)
 */
typedef enum {
    LD2410S_OUTPUT_MINIMAL = 0,
    LD2410S_OUTPUT_STANDARD = 1,
} LD2410S_OutputMode_t;

/**
 * Response speed values (common parameter word 0x000B)
 */
typedef enum {
    LD2410S_RESPONSE_NORMAL = 5,
    LD2410S_RESPONSE_FAST = 10,
} LD2410S_ResponseSpeed_t;

/**
 * Common parameters (commands 0x0070 / 0x0071)
 *
 * Note about frequency representation:
 * The protocol transports values as 32-bit little-endian integers. In the
 * official examples, 0.5 Hz is returned as value 5, suggesting a scale of
 * 10 (Hz * 10).
 */
typedef struct {
    uint8_t farthest_gate;            /**< Parameter word 0x0005, range 1..16 */
    uint8_t nearest_gate;             /**< Parameter word 0x000A, range 0..16 */
    uint16_t unattended_delay_s;      /**< Parameter word 0x0006, range 10..120, unit seconds */
    uint16_t status_report_hz_x10;    /**< Parameter word 0x0002, 0.5..8.0 Hz in 0.5 steps, value = Hz*10 */
    uint16_t distance_report_hz_x10;  /**< Parameter word 0x000C, 0.5..8.0 Hz in 0.5 steps, value = Hz*10 */
    LD2410S_ResponseSpeed_t response; /**< Parameter word 0x000B, 5 (Normal) or 10 (Fast) */
} LD2410S_CommonParams_t;

/**
 * Firmware version information (command 0x0000)
 */
typedef struct {
    uint16_t major;
    uint16_t minor;
    uint16_t patch;
} LD2410S_Version_t;

/**
 * Target state in report frames
 *
 * Spec: 0/1 indicates no one; 2/3 indicates someone.
 */
typedef enum {
    LD2410S_TARGET_NONE_0 = 0,
    LD2410S_TARGET_NONE_1 = 1,
    LD2410S_TARGET_PRESENT_2 = 2,
    LD2410S_TARGET_PRESENT_3 = 3,
} LD2410S_TargetState_t;

/**
 * Parsed target report (standard output mode)
 *
 * The datasheet defines a 64-byte energy blob. The vendor document does not
 * explicitly state the per-gate element width in that blob, therefore the
 * library exposes it as raw bytes.
 */
typedef struct {
    uint8_t data_type; /**< 0x01 for standard report */
    LD2410S_TargetState_t target_state;
    uint16_t object_distance_cm;
    uint8_t reserved[2];
    uint8_t energy[64];
} LD2410S_Report_t;

/* Function prototypes -------------------------------------------------------*/

/**
 * \brief           Build command to enable configuration mode (0x00FF)
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410S_buildSetConfigOn(uint8_t* buffer, uint16_t size);

/**
 * \brief           Build command to end configuration mode (0x00FE)
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410S_buildSetConfigOff(uint8_t* buffer, uint16_t size);

/**
 * \brief           Build command to read firmware version (0x0000)
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410S_buildGetFWVersion(uint8_t* buffer, uint16_t size);

/**
 * \brief           Build command to switch module output mode (0x007A)
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 * \param[in]       mode: Output mode to set
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410S_buildSetOutputMode(uint8_t* buffer, uint16_t size, LD2410S_OutputMode_t mode);

/**
 * \brief           Build command to write serial number (0x0010)
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 * \param[in]       sn8: 8-byte serial number (ASCII)
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410S_buildWriteSerialNumber(uint8_t* buffer, uint16_t size, const uint8_t sn8[8]);

/**
 * \brief           Build command to read serial number (0x0011)
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410S_buildGetSerialNumber(uint8_t* buffer, uint16_t size);

/**
 * \brief           Build command to set common parameters (0x0070)
 *
 * The parameter words are fixed by the protocol and will be sent in the order:
 * farthest gate (0x0005), nearest gate (0x000A), unattended delay (0x0006),
 * status freq (0x0002), distance freq (0x000C), response speed (0x000B).
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 * \param[in]       params: Common parameters to set
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410S_buildSetCommonParams(uint8_t* buffer, uint16_t size, const LD2410S_CommonParams_t* params);

/**
 * \brief           Build command to read common parameters (0x0071)
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410S_buildGetCommonParams(uint8_t* buffer, uint16_t size);

/**
 * \brief           Build command to trigger automatic thresholds determination
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 * \param[in]       triggerFactor: sensitivity factor (suggested 2)
 * \param[in]       holdFactor: retention/hold factor (suggested 1)
 * \param[in]       scanTime: scan duration in seconds (suggested 78)
 * 
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410S_buildSetAutoThresholds(uint8_t* buffer, uint16_t size, uint16_t triggerFactor, uint16_t holdFactor, uint16_t scanTime);

/**
 * \brief           Build command to set trigger thresholds (0x0072)
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 * \param[in]       thresholds: 16 trigger threshold values, one per gate
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410S_buildSetTriggerThresholds(uint8_t* buffer, uint16_t size, const uint32_t thresholds[16]);

/**
 * \brief           Build command to read trigger thresholds (0x0073)
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410S_buildGetTriggerThresholds(uint8_t* buffer, uint16_t size);

/**
 * \brief           Build command to set hold thresholds (0x0076)
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 * \param[in]       thresholds: 16 hold threshold values, one per gate
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410S_buildSetHoldThresholds(uint8_t* buffer, uint16_t size, const uint32_t thresholds[16]);

/**
 * \brief           Build command to read hold thresholds (0x0077)
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410S_buildGetHoldThresholds(uint8_t* buffer, uint16_t size);

/**
 * \brief           Check acknowledgment response to a set command
 *
 * \param[in]       txBuffer: Transmitted command buffer
 * \param[in]       txSize: Size of transmitted command
 * \param[in]       rxBuffer: Received response buffer
 * \param[in]       rxSize: Size of received response
 *
 * \return          LD2410S_SUCCESS if ACK is valid, error code otherwise
 */
LD2410S_Status_t LD2410S_checkSetACK(const uint8_t* txBuffer, uint16_t txSize, const uint8_t* rxBuffer, uint16_t rxSize);

/**
 * \brief           Parse firmware version response
 *
 * \param[in]       buffer: Buffer containing response frame
 * \param[in]       size: Size of the provided buffer
 * \param[out]      version: Parsed firmware version
 *
 * \return          LD2410S_SUCCESS on success, error code otherwise
 */
LD2410S_Status_t LD2410S_parseGetFWVersion(const uint8_t* buffer, uint16_t size, LD2410S_Version_t* version);

/**
 * \brief           Parse serial number response
 *
 * \param[in]       buffer: Buffer containing response frame
 * \param[in]       size: Size of the provided buffer
 * \param[out]      sn8: 8-byte serial number (ASCII)
 *
 * \return          LD2410S_SUCCESS on success, error code otherwise
 */
LD2410S_Status_t LD2410S_parseGetSerialNumber(const uint8_t* buffer, uint16_t size, uint8_t sn8[8]);

/**
 * \brief           Parse common parameters response
 *
 * \param[in]       buffer: Buffer containing response frame
 * \param[in]       size: Size of the provided buffer
 * \param[out]      params: Parsed common parameters
 *
 * \return          LD2410S_SUCCESS on success, error code otherwise
 */
LD2410S_Status_t LD2410S_parseGetCommonParams(const uint8_t* buffer, uint16_t size, LD2410S_CommonParams_t* params);

/**
 * \brief           Parse trigger thresholds response
 *
 * \param[in]       buffer: Buffer containing response frame
 * \param[in]       size: Size of the provided buffer
 * \param[out]      thresholds: 16 trigger threshold values, one per gate
 *
 * \return          LD2410S_SUCCESS on success, error code otherwise
 */
LD2410S_Status_t LD2410S_parseGetTriggerThresholds(const uint8_t* buffer, uint16_t size, uint32_t thresholds[16]);

/**
 * \brief           Parse hold thresholds response
 *
 * \param[in]       buffer: Buffer containing response frame
 * \param[in]       size: Size of the provided buffer
 * \param[out]      thresholds: 16 hold threshold values, one per gate
 *
 * \return          LD2410S_SUCCESS on success, error code otherwise
 */
LD2410S_Status_t LD2410S_parseGetHoldThresholds(const uint8_t* buffer, uint16_t size, uint32_t thresholds[16]);

/**
 * \brief           Parse radar report data frame
 *
 * Supports:
 * - Minimal data report frames (0x6E .. 0x62)
 * - Standard data report frames (F4 F3 F2 F1 .. F8 F7 F6 F5)
 *
 * \param[in]       buffer: Buffer containing report frame
 * \param[in]       size: Size of the provided buffer
 * \param[out]      report: Parsed report (standard mode only). For minimal mode,
 *                  energy[] is left unchanged and only target_state/object_distance_cm are set.
 *
 * \return          LD2410S_SUCCESS on success, error code otherwise
 */
LD2410S_Status_t LD2410S_parseRadarData(const uint8_t* buffer, uint16_t size, LD2410S_Report_t* report);

#ifdef __cplusplus
}
#endif

#endif /* __LD2410S_H__ */
