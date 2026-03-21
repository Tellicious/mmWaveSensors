/* BEGIN Header */
/**
 ******************************************************************************
 * \file            LD2420.h
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

#ifndef __LD2420_H__
#define __LD2420_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Macros --------------------------------------------------------------------*/

/** LD2420 exposes 0..15 distance gates. */
#define LD2420_MAX_GATE 15u

/* Typedefs ------------------------------------------------------------------*/

/**
 * \brief           Return status
 */
typedef enum {
    LD2420_SUCCESS = 0,
    LD2420_ERROR_NULL_PTR,
    LD2420_ERROR_BUFFER_TOO_SMALL,
    LD2420_ERROR_INVALID_DATA,
    LD2420_ERROR_INVALID_LENGTH,
    LD2420_ERROR_INVALID_HEADER,
    LD2420_ERROR_INVALID_FOOTER,
    LD2420_ERROR_INVALID_SIZE,
    LD2420_ERROR_CMD_FAILED, /**< ACK status indicates failure */
} LD2420_Status_t;

/**
 * \brief           Firmware version information (command 0x0000)
 */
typedef struct {
    uint16_t major;
    uint16_t minor;
    uint16_t patch;
} LD2420_Version_t;

/**
 * \brief           Target report in upload mode (Table 5-7)
 */
typedef struct {
    uint8_t presence;              /**< 0x00 no one, 0x01 someone */
    uint16_t target_distance_cm;   /**< Distance of detected target (cm) */
    uint16_t gate_energy[16];      /**< Energy for gates 0..15 */
} LD2420_Target_t;

/**
 * \brief           Parameter IDs for command 0x0007 / 0x0008 (Table 5-5)
 */
typedef enum {
    LD2420_PARAM_MIN_GATE = 0x0000,          /**< Minimum distance gate (0..15) */
    LD2420_PARAM_MAX_GATE = 0x0001,          /**< Maximum distance gate (0..15) */
    LD2420_PARAM_DISAPPEAR_DELAY_S = 0x0004, /**< Target disappear delay (seconds) */
    /* Trigger thresholds: 0x0010..0x001F (gate 0..15) */
    /* Hold thresholds:    0x0020..0x002F (gate 0..15) */
} LD2420_ParamId_t;

/* Function prototypes -------------------------------------------------------*/

/**
 * \brief           Build command to enable configuration mode (0x00FF)
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2420_buildSetConfigOn(uint8_t* buffer, uint16_t size);

/**
 * \brief           Build command to end configuration mode (0x00FE)
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2420_buildSetConfigOff(uint8_t* buffer, uint16_t size);

/**
 * \brief           Build command to read firmware version (0x0000)
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2420_buildGetFWVersion(uint8_t* buffer, uint16_t size);

/**
 * \brief           Build command to set one or more parameters (0x0007)
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 * \param[in]       ids: Parameter IDs array (2 bytes each)
 * \param[in]       values: Parameter values array (4 bytes each)
 * \param[in]       n: Number of (id,value) pairs
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2420_buildSetParams(uint8_t* buffer, uint16_t size, const uint16_t* ids, const uint32_t* values, uint8_t n);

/**
 * \brief           Build command to get one or more parameters (0x0008)
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 * \param[in]       ids: Parameter IDs array
 * \param[in]       n: Number of IDs
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2420_buildGetParams(uint8_t* buffer, uint16_t size, const uint16_t* ids, uint8_t n);

/**
 * \brief           Check acknowledgment response to a set command
 *
 * \param[in]       txBuffer: Transmitted command buffer
 * \param[in]       txSize: Size of transmitted command
 * \param[in]       rxBuffer: Received response buffer
 * \param[in]       rxSize: Size of received response
 *
 * \return          LD2420_SUCCESS if ACK is valid, error code otherwise
 */
LD2420_Status_t LD2420_checkSetACK(const uint8_t* txBuffer, uint16_t txSize, const uint8_t* rxBuffer, uint16_t rxSize);

/**
 * \brief           Parse firmware version response
 *
 * \param[in]       buffer: Buffer containing response frame
 * \param[in]       size: Size of the provided buffer
 * \param[out]      version: Parsed firmware version
 *
 * \return          LD2420_SUCCESS on success, error code otherwise
 */
LD2420_Status_t LD2420_parseGetFWVersion(const uint8_t* buffer, uint16_t size, LD2420_Version_t* version);

/**
 * \brief           Parse get-parameters response (0x0008)
 *
 * \param[in]       buffer: Buffer containing ACK frame
 * \param[in]       size: Size of the provided buffer
 * \param[out]      values: Output array for parameter values (4 bytes each)
 * \param[in]       n: Number of values expected
 *
 * \return          LD2420_SUCCESS on success, error code otherwise
 */
LD2420_Status_t LD2420_parseGetParams(const uint8_t* buffer, uint16_t size, uint32_t* values, uint8_t n);

/**
 * \brief           Parse upload-mode report data frame (Table 5-7)
 *
 * \param[in]       buffer: Buffer containing report frame
 * \param[in]       size: Size of the provided buffer
 * \param[out]      target: Parsed target report
 *
 * \return          LD2420_SUCCESS on success, error code otherwise
 */
LD2420_Status_t LD2420_parseRadarData(const uint8_t* buffer, uint16_t size, LD2420_Target_t* target);

#ifdef __cplusplus
}
#endif

#endif /* __LD2420_H__ */
