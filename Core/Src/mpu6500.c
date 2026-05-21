#include "mpu6500.h"

#include "FreeRTOS.h"
#include "main.h"
#include "task.h"

extern I2C_HandleTypeDef hi2c1;

#define MPU6500_I2C_ADDR        (0x68U << 1)
#define MPU6500_REG_CONFIG      0x1AU
#define MPU6500_REG_GYRO_CONFIG 0x1BU
#define MPU6500_REG_PWR_MGMT_1  0x6BU
#define MPU6500_REG_PWR_MGMT_2  0x6CU
#define MPU6500_REG_GYRO_ZOUT_H 0x47U
#define MPU6500_REG_WHO_AM_I    0x75U
#define MPU6500_WHO_AM_I_VALUE  0x70U

static bool s_mpu_initialized;
static Mpu6500State_t s_mpu_state;

static bool Mpu6500_ReadRegister(uint8_t reg, uint8_t *value);
static bool Mpu6500_WriteRegister(uint8_t reg, uint8_t value);

bool Mpu6500_Init(void)
{
  uint8_t who_am_i = 0U;

  if (s_mpu_initialized)
  {
    return s_mpu_state.ready;
  }

  s_mpu_initialized = true;

  if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6500_I2C_ADDR, 3U, 50U) != HAL_OK)
  {
    s_mpu_state.ready = false;
    return false;
  }

  if (!Mpu6500_ReadRegister(MPU6500_REG_WHO_AM_I, &who_am_i))
  {
    s_mpu_state.ready = false;
    return false;
  }

  s_mpu_state.who_am_i = who_am_i;

  if (who_am_i != MPU6500_WHO_AM_I_VALUE)
  {
    s_mpu_state.ready = false;
    return false;
  }

  if (!Mpu6500_WriteRegister(MPU6500_REG_PWR_MGMT_1, 0x80U))
  {
    s_mpu_state.ready = false;
    return false;
  }
  HAL_Delay(100U);

  if (!Mpu6500_WriteRegister(MPU6500_REG_PWR_MGMT_1, 0x01U) ||
      !Mpu6500_WriteRegister(MPU6500_REG_PWR_MGMT_2, 0x00U) ||
      !Mpu6500_WriteRegister(MPU6500_REG_CONFIG, 0x03U) ||
      !Mpu6500_WriteRegister(MPU6500_REG_GYRO_CONFIG, 0x00U))
  {
    s_mpu_state.ready = false;
    return false;
  }

  s_mpu_state.ready = true;
  s_mpu_state.gyro_z_raw = 0;
  s_mpu_state.gyro_z_dps_x100 = 0;
  return true;
}

void Mpu6500_Update(void)
{
  uint8_t raw[2] = {0U, 0U};
  int16_t gyro_z_raw;
  int32_t gyro_z_dps_x100;

  if (!s_mpu_state.ready)
  {
    return;
  }

  if (HAL_I2C_Mem_Read(&hi2c1, MPU6500_I2C_ADDR, MPU6500_REG_GYRO_ZOUT_H, I2C_MEMADD_SIZE_8BIT, raw, 2U, 100U) != HAL_OK)
  {
    taskENTER_CRITICAL();
    s_mpu_state.ready = false;
    taskEXIT_CRITICAL();
    return;
  }

  gyro_z_raw = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
  gyro_z_dps_x100 = ((int32_t)gyro_z_raw * 100) / 131;

  taskENTER_CRITICAL();
  s_mpu_state.gyro_z_raw = gyro_z_raw;
  s_mpu_state.gyro_z_dps_x100 = gyro_z_dps_x100;
  taskEXIT_CRITICAL();
}

bool Mpu6500_GetState(Mpu6500State_t *out_state)
{
  if (out_state == NULL)
  {
    return false;
  }

  taskENTER_CRITICAL();
  *out_state = s_mpu_state;
  taskEXIT_CRITICAL();

  return s_mpu_initialized;
}

static bool Mpu6500_ReadRegister(uint8_t reg, uint8_t *value)
{
  return (HAL_I2C_Mem_Read(&hi2c1, MPU6500_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, value, 1U, 100U) == HAL_OK);
}

static bool Mpu6500_WriteRegister(uint8_t reg, uint8_t value)
{
  return (HAL_I2C_Mem_Write(&hi2c1, MPU6500_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1U, 100U) == HAL_OK);
}
