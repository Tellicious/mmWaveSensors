/* BEGIN Header */
/**
 ******************************************************************************
 * \file            LD2410B.h
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

#ifndef __LD2410B_H__
#define __LD2410B_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Macros --------------------------------------------------------------------*/

#define LD2410B_MAX_GATE  8u
#define LD2410B_ALL_GATES 0xFFFFU

/* Typedefs ------------------------------------------------------------------*/

/**
 * Return status
 */
typedef enum {
    LD2410B_SUCCESS = 0,
    LD2410B_ERROR_NULL_PTR,
    LD2410B_ERROR_BUFFER_TOO_SMALL,
    LD2410B_ERROR_INVALID_DATA,
    LD2410B_ERROR_INVALID_LENGTH,
    LD2410B_ERROR_INVALID_HEADER,
    LD2410B_ERROR_INVALID_FOOTER,
    LD2410B_ERROR_INVALID_SIZE,
} LD2410B_Status_t;

/**
* Baud rate settings
*/
typedef enum {
    LD2410B_BAUD_9600 = 1,
    LD2410B_BAUD_19200 = 2,
    LD2410B_BAUD_38400 = 3,
    LD2410B_BAUD_57600 = 4,
    LD2410B_BAUD_115200 = 5,
    LD2410B_BAUD_230400 = 6,
    LD2410B_BAUD_256000 = 7,
    LD2410B_BAUD_460800 = 8
} LD2410B_Baudrate_t;

/**
 * Bluetooth control settings
 */
typedef enum {
    LD2410B_BT_DISABLE = 0,
    LD2410B_BT_ENABLE = 1,
} LD2410B_BluetoothControl_t;

/**
 * Distance resolution settings
 */
typedef enum { LD2410B_RES_075M = 0, LD2410B_RES_020M = 1 } LD2410B_DistanceResolution_t;

/**
 * Light sense configuration
 */
typedef enum {
    LD2410B_LIGHT_SENSE_OFF = 0,
    LD2410B_LIGHT_SENSE_LOW_THRESHOLD = 1,
    LD2410B_LIGHT_SENSE_HIGH_THRESHOLD = 2,
} LD2410B_LightSense_t;

/**
 * Output pin configuration
 */
typedef enum {
    LD2410B_AUX_OUT_DEFAULT_LOW = 0,
    LD2410B_AUX_OUT_DEFAULT_HIGH = 1,
} LD2410B_OutputPin_t;

/**
 * Gate data
 */
typedef struct {
    uint8_t motion;
    uint8_t stationary;
} LD2410B_GateData_t;

/**
 * Configuration parameters 
 */
typedef struct {
    uint8_t maxDistGate;                                      /* Maximum detection distance gate (2-8) */
    uint8_t maxMotionGate;                                    /* Maximum motion detection distance gate (2-8) */
    uint8_t maxStaticGate;                                    /* Maximum static detection distance gate (2-8) */
    LD2410B_GateData_t gateSensitivity[LD2410B_MAX_GATE + 1]; /* Sensitivity per gate */
    uint16_t timeoutSeconds;                                  /* Detection timeout in seconds (0-65535) */
} LD2410B_ConfigParam_t;

/**
 * Firmware version
 */
typedef struct {
    uint16_t type; /* should be 0x0001 per spec */
    uint16_t major;
    uint32_t minor; /* YYMMDDVV */
} LD2410B_Version_t;

/**
 * Target state
 */
typedef enum { LD2410B_TARGET_STATE_NONE = 0, LD2410B_TARGET_STATE_MOVING = 1, LD2410B_TARGET_STATE_STILL = 2, LD2410B_TARGET_STATE_BOTH = 3 } LD2410B_TargetState_t;

/**
 * Target data
 */
typedef struct {
    LD2410B_TargetState_t state;
    uint16_t motion_distance_cm;
    uint8_t motion_energy;
    uint16_t static_distance_cm;
    uint8_t static_energy;
    uint16_t detection_distance_cm;
    uint8_t maxMotionGate;
    uint8_t maxStaticGate;
    LD2410B_GateData_t gateEnergy[LD2410B_MAX_GATE + 1]; /* Motion and static energy per gate */
    uint8_t light_value;                                 /* photodiode 0-255 */
    uint8_t out_state;                                   /* 0=no one, 1=someone */
} LD2410B_Target_t;

/* Function prototypes -------------------------------------------------------*/

/**
 * \brief           Build command to enable configuration mode
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410B_buildSetConfigOn(uint8_t* buffer, uint16_t size);

/**
 * \brief           Build command to end configuration mode
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410B_buildSetConfigOff(uint8_t* buffer, uint16_t size);

/**
 * \brief           Build command to set maximum detection distances
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 * \param[in]       maxMotionGate: Maximum motion gate (2-8)
 * \param[in]       maxStaticGate: Maximum static gate (2-8)
 * \param[in]       timeoutSeconds: Timeout in seconds (0-65535)
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410B_buildSetMaxDistance(uint8_t* buffer, uint16_t size, uint8_t maxMotionGate, uint8_t maxStaticGate, uint16_t timeoutSeconds);

/**
 * \brief           Build command to read current parameters
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410B_buildGetParams(uint8_t* buffer, uint16_t size);

/**
 * \brief           Build command to enable engineering mode
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410B_buildSetEngineeringOn(uint8_t* buffer, uint16_t size);

/**
 * \brief           Build command to disable engineering mode
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410B_buildSetEngineeringOff(uint8_t* buffer, uint16_t size);

/**
 * \brief           Build command to set sensitivity for a gate
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 * \param[in]       gate: Gate index (0-8) or LD2410B_ALL_GATES
 * \param[in]       motion_sensitivity: Motion sensitivity value (0-255)
 * \param[in]       static_sensitivity: Static sensitivity value (0-255)
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410B_buildSetSensitivity(uint8_t* buffer, uint16_t size, uint16_t gate, uint8_t motion_sensitivity, uint8_t static_sensitivity);

/**
 * \brief           Build command to read firmware version
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410B_buildGetFWVersion(uint8_t* buffer, uint16_t size);

/**
 * \brief           Build command to set baud rate
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 * \param[in]       baudrate: Baud rate setting
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410B_buildSetBaudrate(uint8_t* buffer, uint16_t size, LD2410B_Baudrate_t baudrate);

/**
 * \brief           Build command to perform factory reset
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410B_buildSetFactoryReset(uint8_t* buffer, uint16_t size);

/**
 * \brief           Build command to restart the device
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410B_buildSetRestart(uint8_t* buffer, uint16_t size);

/**
 * \brief           Build command to control Bluetooth
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 * \param[in]       enable: Bluetooth control setting (enable/disable)
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410B_buildSetBluetoothControl(uint8_t* buffer, uint16_t size, LD2410B_BluetoothControl_t enable);

/**
 * \brief           Build command to read MAC address
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410B_buildGetMacAddress(uint8_t* buffer, uint16_t size);

/**
 * \brief           Build command to obtain Bluetooth permission
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 * \param[in]       password: 6-byte password
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410B_buildObtainBTPermission(uint8_t* buffer, uint16_t size, const uint8_t password[6]);

/**
 * \brief           Build command to set Bluetooth password
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 * \param[in]       password: 6-byte password
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410B_buildSetBTPassword(uint8_t* buffer, uint16_t size, const uint8_t password[6]);

/**
 * \brief           Build command to set distance resolution
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 * \param[in]       res: Distance resolution setting
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410B_buildSetDistanceResolution(uint8_t* buffer, uint16_t size, LD2410B_DistanceResolution_t res);

/**
 * \brief           Build command to read distance resolution
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410B_buildGetDistanceResolution(uint8_t* buffer, uint16_t size);

/**
 * \brief           Build command to set auxiliary control settings
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 * \param[in]       lightSense: Light sense configuration
 * \param[in]       threshold: Light threshold value (0-255)
 * \param[in]       outputPin: Output pin default state
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410B_buildSetAuxControl(uint8_t* buffer, uint16_t size, const LD2410B_LightSense_t lightSense, const uint8_t threshold, const LD2410B_OutputPin_t outputPin);

/**
 * \brief           Build command to read auxiliary control settings
 *
 * \param[out]      buffer: Buffer to write command frame to
 * \param[in]       size: Max size of the provided buffer
 *
 * \return          Number of bytes written to buffer, 0 on error
 */
uint8_t LD2410B_buildGetAuxControl(uint8_t* buffer, uint16_t size);

/**
 * \brief           Check acknowledgment response to a set command
 *
 * \param[in]       txBuffer: Transmitted command buffer
 * \param[in]       txSize: Size of transmitted command
 * \param[in]       rxBuffer: Received response buffer
 * \param[in]       rxSize: Size of received response
 *
 * \return          LD2410B_SUCCESS if ACK is valid, error code otherwise
 */
LD2410B_Status_t LD2410B_checkSetACK(const uint8_t* txBuffer, uint16_t txSize, const uint8_t* rxBuffer, uint16_t rxSize);

/**
 * \brief           Parse configuration parameters response
 *
 * \param[in]       buffer: Buffer containing response frame
 * \param[in]       size: Size of the provided buffer
 * \param[out]      config: Parsed configuration parameters
 *
 * \return          LD2410B_SUCCESS on success, error code otherwise
 */
LD2410B_Status_t LD2410B_parseGetParams(const uint8_t* buffer, uint16_t size, LD2410B_ConfigParam_t* config);

/**
 * \brief           Parse firmware version response
 *
 * \param[in]       buffer: Buffer containing response frame
 * \param[in]       size: Size of the provided buffer
 * \param[out]      version: Parsed firmware version
 *
 * \return          LD2410B_SUCCESS on success, error code otherwise
 */
LD2410B_Status_t LD2410B_parseGetFWVersion(const uint8_t* buffer, uint16_t size, LD2410B_Version_t* version);

/**
 * \brief           Parse distance resolution response
 *
 * \param[in]       buffer: Buffer containing response frame
 * \param[in]       size: Size of the provided buffer
 * \param[out]      res: Parsed distance resolution setting
 *
 * \return          LD2410B_SUCCESS on success, error code otherwise
 */
LD2410B_Status_t LD2410B_parseGetDistanceResolution(const uint8_t* buffer, uint16_t size, LD2410B_DistanceResolution_t* res);

/**
 * \brief           Parse MAC address response
 *
 * \param[in]       buffer: Buffer containing response frame
 * \param[in]       size: Size of the provided buffer
 * \param[out]      mac: 6-byte MAC address array
 *
 * \return          LD2410B_SUCCESS on success, error code otherwise
 */
LD2410B_Status_t LD2410B_parseGetMacAddress(const uint8_t* buffer, uint16_t size, uint8_t* mac);

/**
 * \brief           Parse auxiliary control settings response
 *
 * \param[in]       buffer: Buffer containing response frame
 * \param[in]       size: Size of the provided buffer
 * \param[out]      lightSense: Parsed light sense configuration (can be NULL)
 * \param[out]      threshold: Parsed light threshold value (can be NULL)
 * \param[out]      outputPin: Parsed output pin default state (can be NULL)
 *
 * \return          LD2410B_SUCCESS on success, error code otherwise
 */
LD2410B_Status_t LD2410B_parseGetAuxControl(const uint8_t* buffer, uint16_t size, LD2410B_LightSense_t* lightSense, uint8_t* threshold, LD2410B_OutputPin_t* outputPin);

/**
  * \brief           Parse radar data frame
  * 
  * \param[in]       buffer: Buffer containing data frame
  * \param[in]       size: Size of the provided buffer
  * \param[out]      target: Parsed target data
  * 
  * \return          LD2410B_SUCCESS on success, error code otherwise
  */
LD2410B_Status_t LD2410B_parseRadarData(const uint8_t* buffer, uint16_t size, LD2410B_Target_t* target);

#ifdef __cplusplus
}
#endif

#endif /* __LD2410B_H__ */