#ifndef SLAM_NAV_H
#define SLAM_NAV_H

#include <stdbool.h>
#include <stdint.h>

#include "lidar_pipeline.h"

typedef enum
{
  SLAM_NAV_STATE_IDLE = 0,
  SLAM_NAV_STATE_PLAN,
  SLAM_NAV_STATE_TURN,
  SLAM_NAV_STATE_DRIVE,
  SLAM_NAV_STATE_REPLAN,
  SLAM_NAV_STATE_DONE,
  SLAM_NAV_STATE_NO_PATH
} SlamNavState_t;

bool SlamNav_Init(void);
void SlamNav_StartExplore(void);
void SlamNav_StartReturnTo(int32_t goal_x_mm, int32_t goal_y_mm);
void SlamNav_Stop(void);
bool SlamNav_IsActive(void);
SlamNavState_t SlamNav_GetState(void);
void SlamNav_SetControlConfig(uint16_t drive_pwm_permille,
                              uint16_t turn_pwm_permille,
                              uint16_t safe_distance_mm);
void SlamNav_ObserveLidarPoint(const LidarPoint_t *point);
const char *SlamNav_StateName(SlamNavState_t state);

#endif /* SLAM_NAV_H */
