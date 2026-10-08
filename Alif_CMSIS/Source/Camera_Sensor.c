/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     Camera_Sensor.c
 * @author   Chandra Bhushan Singh
 * @email    chandrabhushan.singh@alifsemi.com
 * @version  V1.1.0
 * @date     30-September-2026
 * @brief    Camera Sensors instance fetch helper and I2C mux GPIO.
 ******************************************************************************/

#include "stddef.h"
#include "Camera_Sensor.h"
#include "Driver_Common.h"

#if CAMERA_DUAL_SENSOR_SUPPORT
#include "Driver_IO.h"
#include "board_config.h"
#include "sys_utils.h"

extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CAMERA_I2C_MUX_GPIO_PORT);
static ARM_DRIVER_GPIO *GPIO_Driver_SWITCH_CAM =
    &ARM_Driver_GPIO_(BOARD_CAMERA_I2C_MUX_GPIO_PORT);
static uint8_t camera_sensor_i2c_mux_initialized;
#endif

CAMERA_SENSOR_DEVICE *Get_Camera_Sensor_0(void);
CAMERA_SENSOR_DEVICE *Get_Camera_Sensor_1(void);

/**
 * \fn        uint8_t Camera_Sensor_GetCount(void)
 * \brief     Get the number of camera sensor instances supported.
 *
 * The count is fixed at compile time by CAMERA_SENSOR_MAX_COUNT
 * (1 for single camera, 2 when CAMERA_DUAL_SENSOR_SUPPORT is enabled).
 *
 * \return    Number of camera sensor instances.
 */
uint8_t Camera_Sensor_GetCount(void)
{
    return (uint8_t) CAMERA_SENSOR_MAX_COUNT;
}

/**
 * \fn        CAMERA_SENSOR_DEVICE *Camera_Sensor_Get(uint8_t instance)
 * \brief     Get the camera sensor device for a given instance.
 *
 * Instance 0 is always available. Instance 1 is available only when
 * CAMERA_DUAL_SENSOR_SUPPORT is enabled.
 *
 * \param[in] instance  Camera sensor instance (CAMERA_SENSOR_INSTANCE_0 or
 *                      CAMERA_SENSOR_INSTANCE_1).
 * \return    Pointer to the sensor device, or NULL if the instance is invalid
 *            or not supported.
 */
CAMERA_SENSOR_DEVICE *Camera_Sensor_Get(uint8_t instance)
{
    switch (instance) {
    case CAMERA_SENSOR_INSTANCE_0:
        return Get_Camera_Sensor_0();

#if CAMERA_DUAL_SENSOR_SUPPORT
    case CAMERA_SENSOR_INSTANCE_1:
        return Get_Camera_Sensor_1();
#endif

    default:
        return NULL;
    }
}

/**
 * \fn        void Camera_Sensor_I2C_Mux_Switch(uint8_t instance)
 * \brief     Route the camera I2C bus to the selected sensor.
 *
 * Drives the I2C C1/C2 analog switch GPIO:
 *   - LOW  : instance 0 (selfie camera, CSI2 native DPHY)
 *   - HIGH : instance 1 (standard camera, DSI DPHY used as RX)
 * Waits 2 ms after the change for the switch to settle.
 * Does nothing if the mux GPIO driver is unavailable, and is a no-op
 * when CAMERA_DUAL_SENSOR_SUPPORT is disabled.
 *
 * \param[in] instance  Camera sensor instance to connect to the I2C bus.
 */
void Camera_Sensor_I2C_Mux_Switch(uint8_t instance)
{
#if CAMERA_DUAL_SENSOR_SUPPORT
    if (GPIO_Driver_SWITCH_CAM == NULL) {
        return;
    }

    GPIO_Driver_SWITCH_CAM->SetValue(BOARD_CAMERA_I2C_MUX_GPIO_PIN,
                                     (instance == CAMERA_SENSOR_INSTANCE_1)
                                         ? GPIO_PIN_OUTPUT_STATE_HIGH
                                         : GPIO_PIN_OUTPUT_STATE_LOW);
    sys_busy_loop_us(2000);
#else
    (void) instance;
#endif
}

/**
 * \fn        int32_t Camera_Sensor_I2C_Mux_Initialize(void)
 * \brief     Initialize the GPIO that controls the camera I2C mux.
 *
 * Initializes the mux GPIO, powers it on, configures it as an output and
 * selects instance 0 as the default. Safe to call more than once; later
 * calls return ARM_DRIVER_OK without doing anything. Returns
 * ARM_DRIVER_OK without doing anything when CAMERA_DUAL_SENSOR_SUPPORT is
 * disabled.
 *
 * \return    \ref execution_status
 */
int32_t Camera_Sensor_I2C_Mux_Initialize(void)
{
#if CAMERA_DUAL_SENSOR_SUPPORT
    int32_t ret;

    if (camera_sensor_i2c_mux_initialized) {
        return ARM_DRIVER_OK;
    }

    ret = GPIO_Driver_SWITCH_CAM->Initialize(BOARD_CAMERA_I2C_MUX_GPIO_PIN, NULL);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_SWITCH_CAM->PowerControl(BOARD_CAMERA_I2C_MUX_GPIO_PIN, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_SWITCH_CAM->SetDirection(BOARD_CAMERA_I2C_MUX_GPIO_PIN,
                                               GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    Camera_Sensor_I2C_Mux_Switch(CAMERA_SENSOR_INSTANCE_0);
    camera_sensor_i2c_mux_initialized = 1;
    return ARM_DRIVER_OK;
#else
    return ARM_DRIVER_OK;
#endif
}

/**
 * \fn        void Camera_Sensor_I2C_Mux_Uninitialize(void)
 * \brief     Release the GPIO that controls the camera I2C mux.
 *
 * Powers off and uninitializes the mux GPIO. Does nothing if the mux was
 * not initialized, and is a no-op when CAMERA_DUAL_SENSOR_SUPPORT is
 * disabled.
 */
void Camera_Sensor_I2C_Mux_Uninitialize(void)
{
#if CAMERA_DUAL_SENSOR_SUPPORT
    if (!camera_sensor_i2c_mux_initialized) {
        return;
    }

    GPIO_Driver_SWITCH_CAM->PowerControl(BOARD_CAMERA_I2C_MUX_GPIO_PIN, ARM_POWER_OFF);
    GPIO_Driver_SWITCH_CAM->Uninitialize(BOARD_CAMERA_I2C_MUX_GPIO_PIN);
    camera_sensor_i2c_mux_initialized = 0;
#endif
}
