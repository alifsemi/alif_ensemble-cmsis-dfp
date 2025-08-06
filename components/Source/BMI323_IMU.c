/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 */

/******************************************************************************
 * @file     BMI323_IMU.c
 * @author   Shreehari H K
 * @email    shreehari.hk@alifsemi.com
 * @version  V1.0.0
 * @date     28-Oct-2024
 * @brief    Driver for Inertial Measurement Unit BMI323
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#include "RTE_Components.h"
#include "sys_utils.h"
/* Pinmux Driver */
#include "pinconf.h"

/* IO Driver */
#include "Driver_IO.h"

/* IMU Driver */
#include "Driver_IMU.h"

/* I3C Driver */
#include "Driver_I3C.h"

#if defined(RTE_Drivers_BMI323)

/* BMI Interrupt pin control IO port */
extern ARM_DRIVER_GPIO  ARM_Driver_GPIO_(RTE_BMI323_INT_IO_PORT);
static ARM_DRIVER_GPIO *IO_Driver_INT = &ARM_Driver_GPIO_(RTE_BMI323_INT_IO_PORT);

#define ARM_IMU_DRV_VERSION            ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)

/* Timeout in Microsec */
#define IMU_I3C_TIMEOUT_US             (100000)

/* BMI323 dummy bytes*/
#define BMI323_INITIAL_DUMMY_BYTES_LEN (2)

/* Register offest */
#define BMI323_ADDR_REG_IDX_SIZE       (1)

/* BMI323 Driver status */
#define BMI323_DRIVER_INITIALIZED      (1 << 0U)
#define BMI323_DRIVER_POWERED          (1 << 1U)

/* Target Slave Address */
#define BMI323_DEFAULT_ADDR            (0x69)

/* Target Slave's Chip ID Reg addr and it's value */
#define BMI323_CHIP_ID_REG             (0x0)
#define BMI323_CHIP_ID_SIZE            (0x02 + BMI323_INITIAL_DUMMY_BYTES_LEN)
#define BMI323_CHIP_ID_VAL             (0x43)
#define BMI323_CHIP_ID_OFFSET          (0x2)

/*  Data Registers */
#define BMI323_TEMP_DATA_REG           (0x09)
#define BMI323_TEMP_DATA_SIZE          (0x02 + BMI323_INITIAL_DUMMY_BYTES_LEN)
#define BMI323_ACCEL_DATA_REG          (0x03)
#define BMI323_ACCEL_DATA_SIZE         (0x06 + BMI323_INITIAL_DUMMY_BYTES_LEN)
#define BMI323_GYRO_DATA_REG           (0x06)
#define BMI323_GYRO_DATA_SIZE          (0x06 + BMI323_INITIAL_DUMMY_BYTES_LEN)

/* Data registers offset */
#define BMI323_TEMP_DATA_OFFSET        (0x00 + BMI323_INITIAL_DUMMY_BYTES_LEN)
#define BMI323_ACCEL_DATA_X_OFFSET     (0x00 + BMI323_INITIAL_DUMMY_BYTES_LEN)
#define BMI323_ACCEL_DATA_Y_OFFSET     (0x02 + BMI323_INITIAL_DUMMY_BYTES_LEN)
#define BMI323_ACCEL_DATA_Z_OFFSET     (0x04 + BMI323_INITIAL_DUMMY_BYTES_LEN)
#define BMI323_GYRO_DATA_X_OFFSET      (0x00 + BMI323_INITIAL_DUMMY_BYTES_LEN)
#define BMI323_GYRO_DATA_Y_OFFSET      (0x02 + BMI323_INITIAL_DUMMY_BYTES_LEN)
#define BMI323_GYRO_DATA_Z_OFFSET      (0x04 + BMI323_INITIAL_DUMMY_BYTES_LEN)

/* Configuration Registers */
#define BMI323_CONFIG_REG_SIZE         (0x02)
#define BMI323_ACCEL_CONFIG_REG        (0x20)
#define BMI323_ACCEL_CONFIG_VAL        (0x4037) /* Enable Normal mode */
#define BMI323_GYRO_CONFIG_REG         (0x21)
#define BMI323_GYRO_CONFIG_VAL         (0x404B) /* Enable Normal mode */

/* Interrupt config, status and src register */
#define BMI323_IO_INT_CTRL_REG         (0x38)
#define BMI323_IO_INT_CTRL_VAL         (0x4) /* Enable interrupt output enable */

#define BMI323_INT_MAP2_REG            (0x3B)  /* Interrupt configuration Register */
#define BMI323_INT_MAP2_VAL            (0x540) /* Route Acc, Gyr, Temp data ready intr to INT1 */

/* Interrupt status macro*/
#define BMI323_INT1_STATUS_REG         (0x0D)
#define BMI323_INT1_STATUS_SIZE        (0x02 + BMI323_INITIAL_DUMMY_BYTES_LEN)

/* Data ready status macros */
#define BMI323_INT1_STATUS_TEMP_DRDY   (0x800)  /* Temperature data is ready   */
#define BMI323_INT1_STATUS_GYRO_DRDY   (0x1000) /* Gyroscope data is ready     */
#define BMI323_INT1_STATUS_ACCEL_DRDY  (0x2000) /* Accelerometer data is ready */

#define BMI323_ACCEL_CALIB_VAL         (2048U) /*Calibration for full scale output selection of +-16g */
#define BMI323_GYRO_CALIB_VAL          (16.384) /*Calibration for full scale output selection of +-2kdps */

#define BMI323_ACCEL_VAL(x)            ((x * 1000) / (BMI323_ACCEL_CALIB_VAL)) /* Acceleration value in mg*/
#define BMI323_GYRO_VAL(x)             ((x * 1000) / (BMI323_GYRO_CALIB_VAL)) /* Gyro value in mdps */
#define BMI323_TEMPERATURE(x)          ((x / 512.0) + 23)                     /* Temp value in C */

#define BMI323_CFG_DELAY_US            10
#define DELAY_1US                      1

/* BMI323 driver Info variable */
static struct BMI323_DRV_INFO {
    uint8_t                 state;         /* Driver state                   */
    uint8_t                 target_addr;   /* Target slave's dynamic address */
    uint16_t                reserved;      /* Reserved                       */
    volatile uint32_t       imu_i3c_event; /* I3C Event status               */
    volatile ARM_IMU_STATUS status;        /* Driver status                  */
} bmi323_drv_info;

/* Driver version*/
static const ARM_DRIVER_VERSION DriverVersion = {ARM_IMU_API_VERSION, ARM_IMU_DRV_VERSION};

/* I3C driver */
extern ARM_DRIVER_I3C  Driver_I3C;
static ARM_DRIVER_I3C *I3C_Driver                    = &Driver_I3C;

/* Driver Capabilities */
static const ARM_IMU_CAPABILITIES DriverCapabilities = {
    1, /* Supports Accelerometer data */
    1, /* Supports Gyroscope data */
    0, /* Doesn't support Magnetometer data */
    1, /* Supports Temperature sens data */
    0  /* reserved (must be zero)*/
};

/* Gets data ready status */
static uint8_t IMU_GetDataStatus(void);

/**
  \fn          ARM_DRIVER_VERSION ARM_IMU_GetVersion(void)
  \brief       Gets IMU driver version.
  \return      \ref ARM_DRIVER_VERSION.
*/
static ARM_DRIVER_VERSION ARM_IMU_GetVersion(void)
{
    return DriverVersion;
}

/**
  \fn          ARM_IMU_CAPABILITIES ARM_IMU_GetCapabilities(void)
  \brief       Gets IMU driver capabilities.
  \return      \ref ARM_IMU_CAPABILITIES.
*/
static ARM_IMU_CAPABILITIES ARM_IMU_GetCapabilities(void)
{
    return DriverCapabilities;
}

/**
  \fn          ARM_IMU_STATUS ARM_IMU_GetStatus(void)
  \brief       Gets IMU driver status.
  \return      \ref ARM_IMU_STATUS.
*/
static ARM_IMU_STATUS ARM_IMU_GetStatus(void)
{
    if (bmi323_drv_info.status.data_rcvd) {
        bmi323_drv_info.status.drdy_status = IMU_GetDataStatus();
    }

    return bmi323_drv_info.status;
}

/**
  \fn           int32_t ARM_IMU_IntEnable(bool enable)
  \brief        BMI323 INT line IO control.
  \param[in]    enable : true or false.
  \return       \ref execution_status.
  */
static int32_t ARM_IMU_IntEnable(bool enable)
{
    uint32_t arg = ARM_GPIO_IRQ_POLARITY_LOW | ARM_GPIO_IRQ_SENSITIVE_LEVEL;
    int32_t  ret;

    if (enable) {
        /* Enable interrupt */
        ret = IO_Driver_INT->Control(RTE_BMI323_INT_PIN_NO, ARM_GPIO_ENABLE_INTERRUPT, &arg);
    } else {
        /* Disable interrupt */
        ret = IO_Driver_INT->Control(RTE_BMI323_INT_PIN_NO, ARM_GPIO_DISABLE_INTERRUPT, NULL);
    }

    return ret;
}

/**
  \fn           void ARM_IMU_IO_CB(uint32_t event)
  \brief        IMU driver I3C callback event.
  \param[in]    event: I3C Event.
  \return       None
*/
void ARM_IMU_IO_CB(uint32_t event)
{
    ARG_UNUSED(event);

    /* Disable IMU interrupt */
    (void) ARM_IMU_IntEnable(false);

    /* Sets Data Rcvd to true */
    bmi323_drv_info.status.data_rcvd = true;
}

/**
  \fn           void IMU_I3CCallBack(uint32_t event)
  \brief        IMU driver I3C callback event.
  \param[in]    event: I3C Event.
  \return       None
*/
void IMU_I3CCallBack(uint32_t event)
{
    SET_BIT(bmi323_drv_info.imu_i3c_event, event);
}

/**
  \fn           int32_t IMU_ResetDynAddr(void)
  \brief        Resets all slaves' dynamic address
  \return       \ref execution_status.
*/
static int32_t IMU_ResetDynAddr(void)
{
    int32_t ret;

    ARM_I3C_CMD i3c_cmd = {0};

    /* Reset slave address */
    i3c_cmd.rw          = 0U;
    i3c_cmd.cmd_id      = I3C_CCC_RSTDAA(true);
    i3c_cmd.len         = 0U;
    i3c_cmd.addr        = 0;

    ret                 = I3C_Driver->MasterSendCommand(&i3c_cmd);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* wait for callback event. */
    while (!((bmi323_drv_info.imu_i3c_event & ARM_I3C_EVENT_TRANSFER_DONE) ||
             (bmi323_drv_info.imu_i3c_event & ARM_I3C_EVENT_TRANSFER_ERROR)));

    if (bmi323_drv_info.imu_i3c_event & ARM_I3C_EVENT_TRANSFER_ERROR) {
        return ARM_DRIVER_ERROR;
    }

    bmi323_drv_info.imu_i3c_event = IMU_EVENT_NONE;

    return ARM_DRIVER_OK;
}

/**
  \fn           int32_t IMU_SetDynAddr(void)
  \brief        Assigns Dynamic address to IMU.
  \return       \ref execution_status.
*/
static int32_t IMU_SetDynAddr(void)
{
    int32_t ret;

    /* I3C CCC (Common Command Codes) */
    ARM_I3C_CMD i3c_cmd = {0};

    i3c_cmd.rw          = 0U;
    i3c_cmd.cmd_id      = I3C_CCC_SETDASA;
    i3c_cmd.len         = 1U;

    /* Assign IMU's Static address */
    i3c_cmd.addr        = BMI323_DEFAULT_ADDR;

    /* Assign Dynamic address */
    ret                 = I3C_Driver->MasterAssignDA(&i3c_cmd);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* wait for callback event. */
    while (!((bmi323_drv_info.imu_i3c_event & ARM_I3C_EVENT_TRANSFER_DONE) ||
             (bmi323_drv_info.imu_i3c_event & ARM_I3C_EVENT_TRANSFER_ERROR)));

    if (bmi323_drv_info.imu_i3c_event & ARM_I3C_EVENT_TRANSFER_ERROR) {
        return ARM_DRIVER_ERROR;
    }

    bmi323_drv_info.imu_i3c_event = IMU_EVENT_NONE;

    /* Fetch the assigned dynamic address */
    ret = I3C_Driver->GetSlaveDynAddr(BMI323_DEFAULT_ADDR, &bmi323_drv_info.target_addr);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t IMU_Init(void)
  \brief       Initializes IMU driver.
               This function will
                 - initialize i3c instance.
  \return      \ref Execution status.
*/
static int32_t IMU_Init(void)
{
    int32_t            ret;
    ARM_DRIVER_VERSION version;

    /* Get i3c driver version. */
    version = I3C_Driver->GetVersion();

    if ((version.api < ARM_DRIVER_VERSION_MAJOR_MINOR(7U, 0U)) ||
        (version.drv < ARM_DRIVER_VERSION_MAJOR_MINOR(7U, 0U))) {
        return ARM_DRIVER_ERROR;
    }

    ret = I3C_Driver->Initialize(IMU_I3CCallBack);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = I3C_Driver->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Initialize IO driver */
    ret = IO_Driver_INT->Initialize(RTE_BMI323_INT_PIN_NO, &ARM_IMU_IO_CB);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Power-up IO driver */
    ret = IO_Driver_INT->PowerControl(RTE_BMI323_INT_PIN_NO, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t IMU_Write(uint8_t tar_addr, uint16_t reg_addr,
                                 uint8_t *reg_data, uint8_t len)
  \brief       Writes data to IMU registers.
  \param[in]   tar_addr : I3C slave address.
  \param[in]   reg_addr : Register address.
  \param[in]   reg_data : pointer to source data buf.
  \param[in]   len      : Number of bytes to write.
  \return      \ref Execution status.
*/
static int32_t IMU_Write(uint8_t tar_addr, uint16_t reg_addr, uint8_t *reg_data, uint8_t len)

{
    int32_t  ret     = 0U;
    uint8_t  iter    = 0U;
    uint32_t counter = 0U;
    __ALIGNED(4) uint8_t tx_buf[4];

    /* Store register's address in 0th index */
    tx_buf[0] = reg_addr;

    if (len) {
        for (iter = 0U; iter < len; iter++) {
            tx_buf[iter + BMI323_ADDR_REG_IDX_SIZE] = reg_data[iter];
        }
    }

    bmi323_drv_info.imu_i3c_event = IMU_EVENT_NONE;

    /* Send msg to slave */
    ret = I3C_Driver->MasterTransmit(tar_addr, tx_buf, (len + BMI323_ADDR_REG_IDX_SIZE));
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* wait for callback event. */
    while (!((bmi323_drv_info.imu_i3c_event & ARM_I3C_EVENT_TRANSFER_DONE) ||
             (bmi323_drv_info.imu_i3c_event & ARM_I3C_EVENT_TRANSFER_ERROR))) {
        if (counter++ < IMU_I3C_TIMEOUT_US) {
            sys_busy_loop_us(DELAY_1US);
        } else {
            return ARM_DRIVER_ERROR;
        }
    }
    if (bmi323_drv_info.imu_i3c_event & ARM_I3C_EVENT_TRANSFER_ERROR) {
        return ARM_DRIVER_ERROR;
    }

    bmi323_drv_info.imu_i3c_event = IMU_EVENT_NONE;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t IMU_Read(uint8_t tar_addr, uint16_t reg_addr,
                                uint8_t *reg_data, uint8_t len)
  \brief       Reads data from IMU registers.
  \param[in]   tar_addr : I3C slave address.
  \param[in]   reg_addr : Register address.
  \param[in]   reg_data : pointer to destination buf.
  \param[in]   len      : Number of bytes to read.
  \return      \ref Execution status.
*/
static int32_t IMU_Read(uint8_t tar_addr, uint16_t reg_addr, uint8_t *reg_data, uint8_t len)
{
    int32_t  ret;
    uint32_t counter = 0U;

    /* Send register address */
    ret              = IMU_Write(tar_addr, reg_addr, NULL, 0U);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Receive data from slave */
    ret = I3C_Driver->MasterReceive(tar_addr, reg_data, len);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* wait for callback event. */
    while (!((bmi323_drv_info.imu_i3c_event & ARM_I3C_EVENT_TRANSFER_DONE) ||
             (bmi323_drv_info.imu_i3c_event & ARM_I3C_EVENT_TRANSFER_ERROR))) {
        if (counter++ < IMU_I3C_TIMEOUT_US) {
            sys_busy_loop_us(DELAY_1US);
        } else {
            return ARM_DRIVER_ERROR;
        }
    }
    if (bmi323_drv_info.imu_i3c_event & ARM_I3C_EVENT_TRANSFER_ERROR) {
        return ARM_DRIVER_ERROR;
    }
    bmi323_drv_info.imu_i3c_event = IMU_EVENT_NONE;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t IMU_AccelConfig(void)
  \brief       Configures Accelerometer of IMU driver.
  \return      \ref Execution status.
*/
static int32_t IMU_AccelConfig(void)
{
    __ALIGNED(4) uint16_t data[BMI323_CONFIG_REG_SIZE];

    /* Configure Accelerometer */
    data[0] = BMI323_ACCEL_CONFIG_VAL;
    IMU_Write(bmi323_drv_info.target_addr,
              BMI323_ACCEL_CONFIG_REG,
              (uint8_t *) data,
              BMI323_CONFIG_REG_SIZE);

    sys_busy_loop_us(BMI323_CFG_DELAY_US);

    data[0] = 0;
    IMU_Read(bmi323_drv_info.target_addr,
             BMI323_ACCEL_CONFIG_REG,
             (uint8_t *) data,
             (BMI323_CONFIG_REG_SIZE + BMI323_INITIAL_DUMMY_BYTES_LEN));

    if (data[1] != BMI323_ACCEL_CONFIG_VAL) {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t IMU_GyroConfig(void)
  \brief       Configures Gyroscope of IMU driver.
  \return      \ref Execution status.
*/
static int32_t IMU_GyroConfig(void)
{
    __ALIGNED(4) uint16_t data[BMI323_CONFIG_REG_SIZE];

    /* Configure Gyroscope */
    data[0] = BMI323_GYRO_CONFIG_VAL;
    IMU_Write(bmi323_drv_info.target_addr,
              BMI323_GYRO_CONFIG_REG,
              (uint8_t *) data,
              BMI323_CONFIG_REG_SIZE);

    sys_busy_loop_us(BMI323_CFG_DELAY_US);

    data[0] = 0;
    IMU_Read(bmi323_drv_info.target_addr,
             BMI323_GYRO_CONFIG_REG,
             (uint8_t *) data,
             (BMI323_CONFIG_REG_SIZE + BMI323_INITIAL_DUMMY_BYTES_LEN));

    if (data[1] != BMI323_GYRO_CONFIG_VAL) {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}
/**
  \fn          int32_t IMU_INTConfig(void)
  \brief       Configures Interrupt of IMU.
  \return      \ref Execution status.
*/
static int32_t IMU_INTConfig(void)
{
    __ALIGNED(4) uint16_t data[BMI323_CONFIG_REG_SIZE];

    /* Below code is for for configuring Interrupt */
    data[0] = BMI323_INT_MAP2_VAL;
    IMU_Write(bmi323_drv_info.target_addr,
              BMI323_INT_MAP2_REG,
              (uint8_t *) data,
              BMI323_CONFIG_REG_SIZE);

    sys_busy_loop_us(BMI323_CFG_DELAY_US);

    /* Below code is for for Enabling Interrupt */
    data[0] = BMI323_IO_INT_CTRL_VAL;
    IMU_Write(bmi323_drv_info.target_addr,
              BMI323_IO_INT_CTRL_REG,
              (uint8_t *) data,
              BMI323_CONFIG_REG_SIZE);

    sys_busy_loop_us(BMI323_CFG_DELAY_US);

    /* Read back the Interrupt configuration  */
    data[0] = 0;
    IMU_Read(bmi323_drv_info.target_addr,
             BMI323_INT_MAP2_REG,
             (uint8_t *) data,
             (BMI323_CONFIG_REG_SIZE + BMI323_INITIAL_DUMMY_BYTES_LEN));

    if (data[1] != BMI323_INT_MAP2_VAL) {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t IMU_Config(void)
  \brief       Configures IMU driver.
  \return      \ref Execution status.
*/
static int32_t IMU_Config(void)
{
    int32_t ret;

    /* Invokes Accelerometer configuration */
    ret = IMU_AccelConfig();
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Invokes Gyroscope configuration */
    ret = IMU_GyroConfig();
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Invokes INT configuration */
    ret = IMU_INTConfig();
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t IMU_Setup(void)
  \brief       Sets up IMU driver.
               This function will
                 - Sets up i3c instance.
  \return      \ref Execution status.
*/
static int32_t IMU_Setup(void)
{
    int32_t ret;
    __ALIGNED(4) uint8_t data[BMI323_CHIP_ID_SIZE];

    /* Initializes I3C master */
    ret = I3C_Driver->Control(I3C_MASTER_INIT, 0);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* i3c Speed Mode Configuration: Slow mode*/
    ret = I3C_Driver->Control(I3C_MASTER_SET_BUS_MODE, I3C_BUS_SLOW_MODE);

    /* Rejects Hot-Join request */
    ret = I3C_Driver->Control(I3C_MASTER_SETUP_HOT_JOIN_ACCEPTANCE, 0);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Rejects Master request */
    ret = I3C_Driver->Control(I3C_MASTER_SETUP_MR_ACCEPTANCE, 0);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Rejects Slave Interrupt request */
    ret = I3C_Driver->Control(I3C_MASTER_SETUP_SIR_ACCEPTANCE, 0);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Resets IMU's address */
    ret = IMU_ResetDynAddr();
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Dynamic Addresses the IMU */
    ret = IMU_SetDynAddr();
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* i3c Speed Mode Configuration: Normal mode*/
    ret = I3C_Driver->Control(I3C_MASTER_SET_BUS_MODE, I3C_BUS_NORMAL_MODE);

    /* Reads Chip ID */
    IMU_Read(bmi323_drv_info.target_addr, BMI323_CHIP_ID_REG, data, BMI323_CHIP_ID_SIZE);

    if (data[BMI323_CHIP_ID_OFFSET] != BMI323_CHIP_ID_VAL) {
        return ARM_DRIVER_ERROR;
    }

    /* Configures the slave */
    ret = IMU_Config();
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Disable interrupt */
    ret = ARM_IMU_IntEnable(false);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = IO_Driver_INT->SetDirection(RTE_BMI323_INT_PIN_NO, GPIO_PIN_DIRECTION_INPUT);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Enable interrupt */
    ret = ARM_IMU_IntEnable(true);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          uint8_t IMU_GetDataStatus(void)
  \brief       Gets data ready status
  \return      \ref data ready status
*/
static uint8_t IMU_GetDataStatus(void)
{
    __ALIGNED(4) uint16_t buf[2];
    uint16_t status = 0;

    /* Reads data ready status */
    IMU_Read(bmi323_drv_info.target_addr,
             BMI323_INT1_STATUS_REG,
             (uint8_t *) buf,
             BMI323_INT1_STATUS_SIZE);

    /* Set status as per data availabilty */
    if (buf[1] & BMI323_INT1_STATUS_TEMP_DRDY) {
        status = IMU_TEMPERATURE_DATA_READY;
    }

    if (buf[1] & BMI323_INT1_STATUS_GYRO_DRDY) {
        status |= IMU_GYRO_DATA_READY;
    }

    if (buf[1] & BMI323_INT1_STATUS_ACCEL_DRDY) {
        status |= IMU_ACCELEROMETER_DATA_READY;
    }

    return status;
}

/**
  \fn          int32_t IMU_GetAccelData(ARM_IMU_COORDINATES *accel_data)
  \brief       Gets Accelerometer data from IMU driver.
  \param[in]   accel_data : Accelerometer data
  \return      \ref Execution status.
*/
static int32_t IMU_GetAccelData(ARM_IMU_COORDINATES *accel_data)
{
    ARM_IMU_COORDINATES data;
    __ALIGNED(4) uint8_t buf[BMI323_ACCEL_DATA_SIZE];

    /* Reads Acceleromter data */
    IMU_Read(bmi323_drv_info.target_addr, BMI323_ACCEL_DATA_REG, buf, BMI323_ACCEL_DATA_SIZE);

    /* Processes Accelerometer data */
    data.x        = buf[BMI323_ACCEL_DATA_X_OFFSET];
    data.y        = buf[BMI323_ACCEL_DATA_Y_OFFSET];
    data.z        = buf[BMI323_ACCEL_DATA_Z_OFFSET];

    accel_data->x = BMI323_ACCEL_VAL(data.x);
    accel_data->y = BMI323_ACCEL_VAL(data.y);
    accel_data->z = BMI323_ACCEL_VAL(data.z);

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t IMU_GetGyroData(ARM_IMU_COORDINATES *gyro_data)
  \brief       Gets Gyroscope data from IMU driver.
  \param[in]   gyro_data : Gyroscope data
  \return      \ref Execution status.
*/
static int32_t IMU_GetGyroData(ARM_IMU_COORDINATES *gyro_data)
{
    ARM_IMU_COORDINATES data;
    __ALIGNED(4) uint8_t buf[BMI323_GYRO_DATA_SIZE];

    /* Reads Gyroscope data */
    IMU_Read(bmi323_drv_info.target_addr, BMI323_GYRO_DATA_REG,
             buf, BMI323_GYRO_DATA_SIZE);

    /* Processes Gyroscope data */
    data.x       = buf[BMI323_GYRO_DATA_X_OFFSET];
    data.y       = buf[BMI323_GYRO_DATA_X_OFFSET];
    data.z       = buf[BMI323_GYRO_DATA_X_OFFSET];

    gyro_data->x = BMI323_GYRO_VAL(data.x);
    gyro_data->y = BMI323_GYRO_VAL(data.y);
    gyro_data->z = BMI323_GYRO_VAL(data.z);

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t IMU_GetTempData(float *temp_data)
  \brief       Gets Temperature sensor data from IMU driver.
  \param[in]   temp_data : Temperature sensor data
  \return      \ref Execution status.
*/
static int32_t IMU_GetTempData(float *temp_data)
{
    int16_t ltemp;
    __ALIGNED(4) uint8_t buf[BMI323_TEMP_DATA_SIZE];

    /* Reads Temperature Sensor data */
    IMU_Read(bmi323_drv_info.target_addr, BMI323_TEMP_DATA_REG,
             buf, BMI323_TEMP_DATA_SIZE);

    /* Processes Temp data */
    ltemp      = buf[BMI323_TEMP_DATA_OFFSET];
    *temp_data = BMI323_TEMPERATURE(ltemp);

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t IMU_Deinit(void)
  \brief       De-initializes IMU driver.
  \return      \ref Execution status.
*/
static int32_t IMU_Deinit(void)
{
    int32_t ret;

    ret = I3C_Driver->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = I3C_Driver->Uninitialize();
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t  ARM_IMU_Initialize (void)
  \brief       Initializes IMU Interface.
  \param[in]   none.
  \return      \ref execution_status.
*/
static int32_t ARM_IMU_Initialize(void)
{
    int32_t ret;

    if (bmi323_drv_info.state & BMI323_DRIVER_INITIALIZED) {
        return ARM_DRIVER_OK;
    }

    ret = IMU_Init();
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Initializes driver state and data received status */
    bmi323_drv_info.state            = BMI323_DRIVER_INITIALIZED;
    bmi323_drv_info.status.data_rcvd = false;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t  ARM_IMU_Uninitialize(void)
  \brief       Uninitializes IMU Interface.
  \param[in]   none.
  \return      \ref execution_status.
*/
static int32_t ARM_IMU_Uninitialize(void)
{
    int32_t ret;

    ret = IMU_Deinit();
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Resets driver state and data received status */
    bmi323_drv_info.state            = 0U;
    bmi323_drv_info.status.data_rcvd = false;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t  ARM_IMU_PowerControl(ARM_POWER_STATE state)
  \brief       Controls power for IMU Interface.
  \param[in]   state : Power state
  \return      \ref execution_status.
*/
static int32_t ARM_IMU_PowerControl(ARM_POWER_STATE state)
{
    int32_t ret;

    /* Returns error if not initialized */
    if (!(bmi323_drv_info.state & BMI323_DRIVER_INITIALIZED)) {
        return ARM_DRIVER_ERROR;
    }

    switch (state) {
    case ARM_POWER_OFF:
        if (!(bmi323_drv_info.state & BMI323_DRIVER_POWERED)) {
            return ARM_DRIVER_OK;
        }

        /* Disable IO interrupt */
        ret = ARM_IMU_IntEnable(false);
        if (ret != ARM_DRIVER_OK) {
            return ret;
        }

        /* Sets state to Powered Down */
        bmi323_drv_info.state &= ~BMI323_DRIVER_POWERED;
        break;

    case ARM_POWER_FULL:
        if (bmi323_drv_info.state & BMI323_DRIVER_POWERED) {
            return ARM_DRIVER_OK;
        }

        /* Sets up IMU */
        ret = IMU_Setup();
        if (ret != ARM_DRIVER_OK) {
            return ret;
        }

        /* Sets state to Powered Up */
        bmi323_drv_info.state |= BMI323_DRIVER_POWERED;
        break;

    case ARM_POWER_LOW:
    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return ARM_DRIVER_OK;
}

/**
  \fn           int32_t ARM_IMU_Control(uint32_t control, uint32_t arg)
  \brief        Control BMI323 Slave
  \param[in]    control  : Operation
  \param[in]    arg      : Argument of operation
  \return       \ref execution_status
*/
static int32_t ARM_IMU_Control(uint32_t control, uint32_t arg)
{
    void  *ptr;
    float *temp_data;

    switch (control) {
    case IMU_GET_ACCELEROMETER_DATA:
        if (!arg) {
            return ARM_DRIVER_ERROR;
        }
        ptr = (ARM_IMU_COORDINATES *) arg;

        /* Gets Accelerometer data */
        IMU_GetAccelData(ptr);

        /* Resets data status */
        CLEAR_BIT(bmi323_drv_info.status.drdy_status, IMU_ACCELEROMETER_DATA_READY);
        bmi323_drv_info.status.data_rcvd = false;
        break;

    case IMU_GET_GYROSCOPE_DATA:
        if (!arg) {
            return ARM_DRIVER_ERROR;
        }
        ptr = (ARM_IMU_COORDINATES *) arg;

        /* Gets Gyroscope data */
        IMU_GetGyroData(ptr);

        /* Resets data status */
        CLEAR_BIT(bmi323_drv_info.status.drdy_status, IMU_GYRO_DATA_READY);
        bmi323_drv_info.status.data_rcvd = false;
        break;

    case IMU_GET_TEMPERATURE_DATA:
        if (!arg) {
            return ARM_DRIVER_ERROR;
        }
        temp_data = (float *) arg;

        /* Gets Temperature data */
        IMU_GetTempData(temp_data);

        /* Resets data status */
        CLEAR_BIT(bmi323_drv_info.status.drdy_status, IMU_TEMPERATURE_DATA_READY);
        bmi323_drv_info.status.data_rcvd = false;
        break;

    case IMU_SET_INTERRUPT:
        if (arg) {
            /* Enable IMU interrupt */
            (void) ARM_IMU_IntEnable(true);
        } else {
            /* Disable IMU interrupt */
            (void) ARM_IMU_IntEnable(false);
        }
        break;

    case IMU_GET_MAGNETOMETER_DATA:
    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

extern ARM_DRIVER_IMU BMI323;

ARM_DRIVER_IMU BMI323 = {
    ARM_IMU_GetVersion,
    ARM_IMU_GetCapabilities,
    ARM_IMU_GetStatus,
    ARM_IMU_Initialize,
    ARM_IMU_Uninitialize,
    ARM_IMU_PowerControl,
    ARM_IMU_Control
};

#endif /* defined (RTE_Drivers_BMI323) */
