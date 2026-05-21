#ifndef MPU6500_H
#define MPU6500_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  bool ready;
  uint8_t who_am_i;
  int16_t gyro_z_raw;
  int32_t gyro_z_dps_x100;
} Mpu6500State_t;

bool Mpu6500_Init(void);
void Mpu6500_Update(void);
bool Mpu6500_GetState(Mpu6500State_t *out_state);

#endif /* MPU6500_H */
