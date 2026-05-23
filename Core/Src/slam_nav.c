#include "slam_nav.h"

#include <stdio.h>
#include <string.h>

#include "astar_planner.h"
#include "bluetooth_control.h"
#include "FreeRTOS.h"
#include "main.h"
#include "mapping_grid.h"
#include "motor_control.h"
#include "queue.h"
#include "task.h"

#define SLAM_NAV_TASK_STACK_WORDS       640U
#define SLAM_NAV_TASK_PRIORITY          (tskIDLE_PRIORITY + 2U)
#define SLAM_NAV_COMMAND_QUEUE_LENGTH   4U
#define SLAM_NAV_TASK_PERIOD_MS         20U
#define SLAM_NAV_DEFAULT_DRIVE_PWM      450U
#define SLAM_NAV_DEFAULT_TURN_PWM       450U
#define SLAM_NAV_DEFAULT_SAFE_MM        350U
#define SLAM_NAV_MAX_PWM                1000U
#define SLAM_NAV_MIN_SAFE_MM            350U
#define SLAM_NAV_MIN_DRIVE_PWM          380U
#define SLAM_NAV_MIN_TURN_PWM           380U
#define SLAM_NAV_FRONT_SECTOR_CDEG      3000U
#define SLAM_NAV_FRONT_BLOCK_HOLD_MS    600U
#define SLAM_NAV_TURN_TOL_CDEG          1200L
#define SLAM_NAV_DRIVE_HEADING_TOL_CDEG 2600L
#define SLAM_NAV_TARGET_RADIUS_MM       40L
#define SLAM_NAV_ROBOT_FREE_RADIUS      1U
#define SLAM_NAV_HEARTBEAT_INTERVAL_MS  500U
#define SLAM_NAV_PATH_CHUNK_CELLS       10U

typedef enum
{
  SLAM_NAV_COMMAND_START_EXPLORE = 0,
  SLAM_NAV_COMMAND_START_RETURN,
  SLAM_NAV_COMMAND_STOP
} SlamNavCommand_t;

typedef enum
{
  SLAM_NAV_MODE_EXPLORE = 0,
  SLAM_NAV_MODE_RETURN
} SlamNavMode_t;

typedef struct
{
  SlamNavCommand_t command;
  int32_t goal_x_mm;
  int32_t goal_y_mm;
} SlamNavCommandMessage_t;

static StaticTask_t s_nav_task_struct;
static StackType_t s_nav_task_stack[SLAM_NAV_TASK_STACK_WORDS];
static TaskHandle_t s_nav_task_handle;

static StaticQueue_t s_command_queue_struct;
static uint8_t s_command_queue_storage[SLAM_NAV_COMMAND_QUEUE_LENGTH * sizeof(SlamNavCommandMessage_t)];
static QueueHandle_t s_command_queue;

static MappingGridSnapshot_t s_snapshot;
static AstarPlannerPath_t s_path;

static bool s_initialized;
static bool s_active;
static SlamNavMode_t s_mode = SLAM_NAV_MODE_EXPLORE;
static SlamNavState_t s_state = SLAM_NAV_STATE_IDLE;
static uint16_t s_drive_pwm_permille = SLAM_NAV_DEFAULT_DRIVE_PWM;
static uint16_t s_turn_pwm_permille = SLAM_NAV_DEFAULT_TURN_PWM;
static uint16_t s_safe_distance_mm = SLAM_NAV_DEFAULT_SAFE_MM;
static uint32_t s_front_blocked_until_ms;
static uint16_t s_front_min_distance_mm;
static uint16_t s_path_index;
static AstarPlannerCell_t s_current_target_cell;
static int32_t s_target_x_mm;
static int32_t s_target_y_mm;
static int32_t s_target_heading_cdeg;
static uint32_t s_plan_seq;
static uint32_t s_last_heartbeat_tick_ms;
static int32_t s_return_goal_x_mm;
static int32_t s_return_goal_y_mm;

static void SlamNav_Task(void *argument);
static void SlamNav_HandleCommand(const SlamNavCommandMessage_t *message);
static void SlamNav_Update(void);
static void SlamNav_StartInternal(SlamNavMode_t mode, int32_t goal_x_mm, int32_t goal_y_mm);
static void SlamNav_StopInternal(const char *reason, bool send_status);
static void SlamNav_UpdatePlan(void);
static void SlamNav_UpdateTurn(void);
static void SlamNav_UpdateDrive(void);
static bool SlamNav_GetPoseAndCell(MappingGridPose_t *out_pose, uint8_t *out_x, uint8_t *out_y);
static bool SlamNav_SetTargetFromPath(uint16_t path_index);
static bool SlamNav_IsFrontBlocked(void);
static bool SlamNav_IsFrontAngle(uint16_t angle_cdeg);
static int32_t SlamNav_NormalizeHeadingCdeg(int32_t heading_cdeg);
static int32_t SlamNav_SignedHeadingErrorCdeg(int32_t target_cdeg, int32_t current_cdeg);
static int32_t SlamNav_Abs32(int32_t value);
static uint16_t SlamNav_ClampPwm(uint16_t value);
static uint16_t SlamNav_ActivePwm(uint16_t configured_pwm, uint16_t fallback_pwm, uint16_t minimum_pwm);
static int32_t SlamNav_CellHeadingCdeg(const AstarPlannerCell_t *from, const AstarPlannerCell_t *to);
static void SlamNav_SendStatus(const char *state, const char *reason);
static void SlamNav_SendHeartbeatIfDue(void);
static void SlamNav_SendPath(void);

bool SlamNav_Init(void)
{
  if (s_initialized)
  {
    return true;
  }

  s_command_queue = xQueueCreateStatic(
      SLAM_NAV_COMMAND_QUEUE_LENGTH,
      sizeof(SlamNavCommandMessage_t),
      s_command_queue_storage,
      &s_command_queue_struct);
  configASSERT(s_command_queue != NULL);

  s_nav_task_handle = xTaskCreateStatic(
      SlamNav_Task,
      "slamNav",
      SLAM_NAV_TASK_STACK_WORDS,
      NULL,
      SLAM_NAV_TASK_PRIORITY,
      s_nav_task_stack,
      &s_nav_task_struct);
  configASSERT(s_nav_task_handle != NULL);

  s_initialized = true;
  return true;
}

void SlamNav_StartExplore(void)
{
  SlamNavCommandMessage_t message;

  if (!s_initialized)
  {
    (void)SlamNav_Init();
  }

  message.command = SLAM_NAV_COMMAND_START_EXPLORE;
  message.goal_x_mm = 0L;
  message.goal_y_mm = 0L;

  if (s_command_queue != NULL)
  {
    (void)xQueueSend(s_command_queue, &message, 0U);
  }
}

void SlamNav_StartReturnTo(int32_t goal_x_mm, int32_t goal_y_mm)
{
  SlamNavCommandMessage_t message;

  if (!s_initialized)
  {
    (void)SlamNav_Init();
  }

  message.command = SLAM_NAV_COMMAND_START_RETURN;
  message.goal_x_mm = goal_x_mm;
  message.goal_y_mm = goal_y_mm;

  if (s_command_queue != NULL)
  {
    (void)xQueueSend(s_command_queue, &message, 0U);
  }
}

void SlamNav_Stop(void)
{
  SlamNav_StopInternal("STOP", false);
}

bool SlamNav_IsActive(void)
{
  bool active;

  taskENTER_CRITICAL();
  active = s_active;
  taskEXIT_CRITICAL();
  return active;
}

SlamNavState_t SlamNav_GetState(void)
{
  SlamNavState_t state;

  taskENTER_CRITICAL();
  state = s_state;
  taskEXIT_CRITICAL();
  return state;
}

void SlamNav_SetControlConfig(uint16_t drive_pwm_permille,
                              uint16_t turn_pwm_permille,
                              uint16_t safe_distance_mm)
{
  taskENTER_CRITICAL();
  s_drive_pwm_permille = SlamNav_ClampPwm(drive_pwm_permille);
  s_turn_pwm_permille = SlamNav_ClampPwm(turn_pwm_permille);
  s_safe_distance_mm = (safe_distance_mm < SLAM_NAV_MIN_SAFE_MM) ? SLAM_NAV_MIN_SAFE_MM : safe_distance_mm;
  taskEXIT_CRITICAL();
}

void SlamNav_ObserveLidarPoint(const LidarPoint_t *point)
{
  uint16_t safe_mm;

  if ((point == NULL) ||
      !s_active ||
      (point->quality == 0U) ||
      (point->distance_mm == 0U) ||
      !SlamNav_IsFrontAngle(LidarPipeline_LidarToRobotAngleU16(point->angle_cdeg)))
  {
    return;
  }

  taskENTER_CRITICAL();
  safe_mm = s_safe_distance_mm;
  taskEXIT_CRITICAL();

  if (point->distance_mm <= safe_mm)
  {
    taskENTER_CRITICAL();
    s_front_blocked_until_ms = HAL_GetTick() + SLAM_NAV_FRONT_BLOCK_HOLD_MS;
    s_front_min_distance_mm = point->distance_mm;
    taskEXIT_CRITICAL();
  }
}

const char *SlamNav_StateName(SlamNavState_t state)
{
  switch (state)
  {
    case SLAM_NAV_STATE_IDLE:    return "IDLE";
    case SLAM_NAV_STATE_PLAN:    return "PLAN";
    case SLAM_NAV_STATE_TURN:    return "TURN";
    case SLAM_NAV_STATE_DRIVE:   return "DRIVE";
    case SLAM_NAV_STATE_REPLAN:  return "REPLAN";
    case SLAM_NAV_STATE_DONE:    return "DONE";
    case SLAM_NAV_STATE_NO_PATH: return "NO_PATH";
    default:                     return "UNKNOWN";
  }
}

static void SlamNav_Task(void *argument)
{
  SlamNavCommandMessage_t message;

  (void)argument;

  for (;;)
  {
    while ((s_command_queue != NULL) &&
           (xQueueReceive(s_command_queue, &message, 0U) == pdPASS))
    {
      SlamNav_HandleCommand(&message);
    }

    SlamNav_Update();
    vTaskDelay(pdMS_TO_TICKS(SLAM_NAV_TASK_PERIOD_MS));
  }
}

static void SlamNav_HandleCommand(const SlamNavCommandMessage_t *message)
{
  if (message == NULL)
  {
    return;
  }

  if (message->command == SLAM_NAV_COMMAND_START_EXPLORE)
  {
    SlamNav_StartInternal(SLAM_NAV_MODE_EXPLORE, 0L, 0L);
  }
  else if (message->command == SLAM_NAV_COMMAND_START_RETURN)
  {
    SlamNav_StartInternal(SLAM_NAV_MODE_RETURN, message->goal_x_mm, message->goal_y_mm);
  }
  else
  {
    SlamNav_StopInternal("STOP", true);
  }
}

static void SlamNav_Update(void)
{
  if (!s_active)
  {
    return;
  }

  SlamNav_SendHeartbeatIfDue();

  if ((s_state == SLAM_NAV_STATE_DRIVE) && SlamNav_IsFrontBlocked())
  {
    MotorControl_Stop();
    s_state = SLAM_NAV_STATE_REPLAN;
    SlamNav_SendStatus("REPLAN", "BLOCKED");
  }

  switch (s_state)
  {
    case SLAM_NAV_STATE_PLAN:
    case SLAM_NAV_STATE_REPLAN:
      SlamNav_UpdatePlan();
      break;
    case SLAM_NAV_STATE_TURN:
      SlamNav_UpdateTurn();
      break;
    case SLAM_NAV_STATE_DRIVE:
      SlamNav_UpdateDrive();
      break;
    case SLAM_NAV_STATE_IDLE:
    case SLAM_NAV_STATE_DONE:
    case SLAM_NAV_STATE_NO_PATH:
    default:
      break;
  }
}

static void SlamNav_StartInternal(SlamNavMode_t mode, int32_t goal_x_mm, int32_t goal_y_mm)
{
  taskENTER_CRITICAL();
  s_active = true;
  s_mode = mode;
  s_state = SLAM_NAV_STATE_PLAN;
  s_front_blocked_until_ms = 0U;
  s_front_min_distance_mm = 0U;
  s_path_index = 0U;
  s_plan_seq = 0U;
  s_last_heartbeat_tick_ms = 0U;
  s_return_goal_x_mm = goal_x_mm;
  s_return_goal_y_mm = goal_y_mm;
  taskEXIT_CRITICAL();

  MotorControl_Stop();
  SlamNav_SendStatus("START", (mode == SLAM_NAV_MODE_RETURN) ? "RETURN" : "FRONTIER");
}

static void SlamNav_StopInternal(const char *reason, bool send_status)
{
  bool was_active;

  taskENTER_CRITICAL();
  was_active = s_active;
  s_active = false;
  s_mode = SLAM_NAV_MODE_EXPLORE;
  s_state = SLAM_NAV_STATE_IDLE;
  s_front_blocked_until_ms = 0U;
  s_front_min_distance_mm = 0U;
  taskEXIT_CRITICAL();

  if (was_active)
  {
    MotorControl_Stop();
  }

  if (send_status && was_active)
  {
    SlamNav_SendStatus("STOP", reason);
  }
}

static void SlamNav_UpdatePlan(void)
{
  MappingGridPose_t pose;
  uint8_t start_x;
  uint8_t start_y;
  uint8_t goal_x = 0U;
  uint8_t goal_y = 0U;
  AstarPlannerStatus_t status;
  SlamNavMode_t mode;
  int32_t return_goal_x_mm;
  int32_t return_goal_y_mm;

  if (!SlamNav_GetPoseAndCell(&pose, &start_x, &start_y))
  {
    s_state = SLAM_NAV_STATE_NO_PATH;
    s_active = false;
    MotorControl_Stop();
    SlamNav_SendStatus("NO_PATH", "BAD_POSE");
    return;
  }

  MappingGrid_MarkRobotFree(&pose, SLAM_NAV_ROBOT_FREE_RADIUS);
  if (!MappingGrid_CopySnapshot(&s_snapshot))
  {
    s_state = SLAM_NAV_STATE_NO_PATH;
    s_active = false;
    MotorControl_Stop();
    SlamNav_SendStatus("NO_PATH", "SNAPSHOT");
    return;
  }

  taskENTER_CRITICAL();
  mode = s_mode;
  return_goal_x_mm = s_return_goal_x_mm;
  return_goal_y_mm = s_return_goal_y_mm;
  taskEXIT_CRITICAL();

  if (mode == SLAM_NAV_MODE_RETURN)
  {
    if (!MappingGrid_WorldToCell(return_goal_x_mm, return_goal_y_mm, &goal_x, &goal_y))
    {
      s_state = SLAM_NAV_STATE_NO_PATH;
      s_active = false;
      MotorControl_Stop();
      SlamNav_SendStatus("NO_PATH", "RETURN_GOAL");
      return;
    }

    if ((start_x == goal_x) && (start_y == goal_y))
    {
      s_state = SLAM_NAV_STATE_DONE;
      s_active = false;
      MotorControl_Stop();
      s_path.length = 1U;
      s_path.target.x = goal_x;
      s_path.target.y = goal_y;
      s_current_target_cell = s_path.target;
      SlamNav_SendStatus("DONE", "RETURN_HOME");
      return;
    }

    status = AstarPlanner_PlanToGoal(&s_snapshot, start_x, start_y, goal_x, goal_y, &s_path);
  }
  else
  {
    status = AstarPlanner_PlanToFrontier(&s_snapshot, start_x, start_y, &s_path);
  }

  s_plan_seq++;
  if (((status != ASTAR_PLANNER_STATUS_OK) &&
       (status != ASTAR_PLANNER_STATUS_PATH_TRUNCATED)) ||
      (s_path.length < 2U))
  {
    s_state = SLAM_NAV_STATE_NO_PATH;
    s_active = false;
    MotorControl_Stop();
    SlamNav_SendStatus("NO_PATH", AstarPlanner_StatusName(status));
    return;
  }

  if (!SlamNav_SetTargetFromPath(1U))
  {
    s_state = SLAM_NAV_STATE_NO_PATH;
    s_active = false;
    MotorControl_Stop();
    SlamNav_SendStatus("NO_PATH", "TARGET");
    return;
  }

  s_state = SLAM_NAV_STATE_TURN;
  SlamNav_SendPath();
  SlamNav_SendStatus((mode == SLAM_NAV_MODE_RETURN) ? "RETURN" : "PLAN", AstarPlanner_StatusName(status));
}

static void SlamNav_UpdateTurn(void)
{
  MappingGridPose_t pose;
  int32_t error_cdeg;
  uint16_t turn_pwm;

  if (!MappingGrid_GetPose(&pose))
  {
    s_state = SLAM_NAV_STATE_REPLAN;
    return;
  }

  error_cdeg = SlamNav_SignedHeadingErrorCdeg(s_target_heading_cdeg, pose.heading_cdeg);
  if (SlamNav_Abs32(error_cdeg) <= SLAM_NAV_TURN_TOL_CDEG)
  {
    MotorControl_Stop();
    s_state = SLAM_NAV_STATE_DRIVE;
    return;
  }

  turn_pwm = SlamNav_ActivePwm(s_turn_pwm_permille, SLAM_NAV_DEFAULT_TURN_PWM, SLAM_NAV_MIN_TURN_PWM);
  if (error_cdeg > 0L)
  {
    MotorControl_SetTurnLeft(turn_pwm);
  }
  else
  {
    MotorControl_SetTurnRight(turn_pwm);
  }
}

static void SlamNav_UpdateDrive(void)
{
  MappingGridPose_t pose;
  uint8_t current_x;
  uint8_t current_y;
  int32_t dx_mm;
  int32_t dy_mm;
  int32_t distance_sq;
  int32_t error_cdeg;

  if (!SlamNav_GetPoseAndCell(&pose, &current_x, &current_y))
  {
    s_state = SLAM_NAV_STATE_REPLAN;
    return;
  }

  if (MappingGrid_GetCell(s_current_target_cell.x, s_current_target_cell.y) != MAPPING_GRID_CELL_FREE)
  {
    MotorControl_Stop();
    s_state = SLAM_NAV_STATE_REPLAN;
    SlamNav_SendStatus("REPLAN", "TARGET_BLOCKED");
    return;
  }

  dx_mm = s_target_x_mm - pose.x_mm;
  dy_mm = s_target_y_mm - pose.y_mm;
  distance_sq = (dx_mm * dx_mm) + (dy_mm * dy_mm);
  if (((current_x == s_current_target_cell.x) && (current_y == s_current_target_cell.y)) ||
      (distance_sq <= (SLAM_NAV_TARGET_RADIUS_MM * SLAM_NAV_TARGET_RADIUS_MM)))
  {
    MotorControl_Stop();
    s_state = SLAM_NAV_STATE_PLAN;
    SlamNav_SendStatus("CELL", "REACHED");
    return;
  }

  error_cdeg = SlamNav_SignedHeadingErrorCdeg(s_target_heading_cdeg, pose.heading_cdeg);
  if (SlamNav_Abs32(error_cdeg) > SLAM_NAV_DRIVE_HEADING_TOL_CDEG)
  {
    MotorControl_Stop();
    s_state = SLAM_NAV_STATE_TURN;
    return;
  }

  MotorControl_SetForward(SlamNav_ActivePwm(s_drive_pwm_permille, SLAM_NAV_DEFAULT_DRIVE_PWM, SLAM_NAV_MIN_DRIVE_PWM));
}

static bool SlamNav_GetPoseAndCell(MappingGridPose_t *out_pose, uint8_t *out_x, uint8_t *out_y)
{
  MappingGridPose_t pose;

  if ((out_pose == NULL) || (out_x == NULL) || (out_y == NULL))
  {
    return false;
  }

  if (!MappingGrid_GetPose(&pose))
  {
    return false;
  }

  if (!MappingGrid_WorldToCell(pose.x_mm, pose.y_mm, out_x, out_y))
  {
    return false;
  }

  *out_pose = pose;
  return true;
}

static bool SlamNav_SetTargetFromPath(uint16_t path_index)
{
  AstarPlannerCell_t from;
  AstarPlannerCell_t to;

  if ((path_index == 0U) || (path_index >= s_path.length))
  {
    return false;
  }

  from = s_path.cells[path_index - 1U];
  to = s_path.cells[path_index];
  if (!MappingGrid_CellToWorld(to.x, to.y, &s_target_x_mm, &s_target_y_mm))
  {
    return false;
  }

  s_path_index = path_index;
  s_current_target_cell = to;
  s_target_heading_cdeg = SlamNav_CellHeadingCdeg(&from, &to);
  return true;
}

static bool SlamNav_IsFrontBlocked(void)
{
  bool blocked;
  uint32_t now = HAL_GetTick();

  taskENTER_CRITICAL();
  blocked = ((s_front_blocked_until_ms != 0U) &&
             ((int32_t)(now - s_front_blocked_until_ms) < 0L));
  taskEXIT_CRITICAL();
  return blocked;
}

static bool SlamNav_IsFrontAngle(uint16_t angle_cdeg)
{
  return ((angle_cdeg <= SLAM_NAV_FRONT_SECTOR_CDEG) ||
          (angle_cdeg >= (uint16_t)(36000U - SLAM_NAV_FRONT_SECTOR_CDEG)));
}

static int32_t SlamNav_NormalizeHeadingCdeg(int32_t heading_cdeg)
{
  while (heading_cdeg < 0L)
  {
    heading_cdeg += 36000L;
  }

  while (heading_cdeg >= 36000L)
  {
    heading_cdeg -= 36000L;
  }

  return heading_cdeg;
}

static int32_t SlamNav_SignedHeadingErrorCdeg(int32_t target_cdeg, int32_t current_cdeg)
{
  int32_t error = SlamNav_NormalizeHeadingCdeg(target_cdeg) -
      SlamNav_NormalizeHeadingCdeg(current_cdeg);

  if (error > 18000L)
  {
    error -= 36000L;
  }
  else if (error < -18000L)
  {
    error += 36000L;
  }

  return error;
}

static int32_t SlamNav_Abs32(int32_t value)
{
  return (value < 0L) ? -value : value;
}

static uint16_t SlamNav_ClampPwm(uint16_t value)
{
  return (value > SLAM_NAV_MAX_PWM) ? SLAM_NAV_MAX_PWM : value;
}

static uint16_t SlamNav_ActivePwm(uint16_t configured_pwm, uint16_t fallback_pwm, uint16_t minimum_pwm)
{
  uint16_t pwm = (configured_pwm > 0U) ? configured_pwm : fallback_pwm;

  if ((pwm > 0U) && (pwm < minimum_pwm))
  {
    pwm = minimum_pwm;
  }

  return pwm;
}

static int32_t SlamNav_CellHeadingCdeg(const AstarPlannerCell_t *from, const AstarPlannerCell_t *to)
{
  if ((from == NULL) || (to == NULL))
  {
    return 0L;
  }

  if (to->x > from->x)
  {
    return 0L;
  }

  if (to->x < from->x)
  {
    return 18000L;
  }

  if (to->y < from->y)
  {
    return 9000L;
  }

  return 27000L;
}

static void SlamNav_SendStatus(const char *state, const char *reason)
{
  char line[128];

  if (state == NULL)
  {
    state = SlamNav_StateName(s_state);
  }

  if (reason == NULL)
  {
    reason = "NONE";
  }

  (void)snprintf(
      line,
      sizeof(line),
      "SLAM state=%s reason=%s seq=%lu target=%u,%u path=%u front=%u\r\n",
      state,
      reason,
      (unsigned long)s_plan_seq,
      (unsigned int)s_current_target_cell.x,
      (unsigned int)s_current_target_cell.y,
      (unsigned int)s_path.length,
      (unsigned int)s_front_min_distance_mm);
  (void)BluetoothControl_SendText(line);
}

static void SlamNav_SendHeartbeatIfDue(void)
{
  char line[128];
  uint32_t now = HAL_GetTick();
  uint32_t revision = MappingGrid_GetRevision();

  if ((now - s_last_heartbeat_tick_ms) < SLAM_NAV_HEARTBEAT_INTERVAL_MS)
  {
    return;
  }
  s_last_heartbeat_tick_ms = now;

  (void)snprintf(
      line,
      sizeof(line),
      "SLAM HB state=%s seq=%lu target=%u,%u path_i=%u path_len=%u front=%u rev=%lu\r\n",
      SlamNav_StateName(s_state),
      (unsigned long)s_plan_seq,
      (unsigned int)s_current_target_cell.x,
      (unsigned int)s_current_target_cell.y,
      (unsigned int)s_path_index,
      (unsigned int)s_path.length,
      (unsigned int)s_front_min_distance_mm,
      (unsigned long)revision);
  (void)BluetoothControl_SendText(line);
}

static void SlamNav_SendPath(void)
{
  char line[128];
  uint16_t index = 0U;
  uint32_t revision = MappingGrid_GetRevision();

  (void)snprintf(
      line,
      sizeof(line),
      "PATH BEGIN seq=%lu rev=%lu len=%u target=%u,%u\r\n",
      (unsigned long)s_plan_seq,
      (unsigned long)revision,
      (unsigned int)s_path.length,
      (unsigned int)s_path.target.x,
      (unsigned int)s_path.target.y);
  (void)BluetoothControl_SendText(line);

  while (index < s_path.length)
  {
    uint16_t chunk_start = index;
    uint16_t used;

    (void)snprintf(
        line,
        sizeof(line),
        "PATH CHUNK seq=%lu idx=%u data=",
        (unsigned long)s_plan_seq,
        (unsigned int)chunk_start);
    used = (uint16_t)strlen(line);

    while ((index < s_path.length) &&
           ((index - chunk_start) < SLAM_NAV_PATH_CHUNK_CELLS) &&
           (used < (sizeof(line) - 10U)))
    {
      int written = snprintf(
          &line[used],
          sizeof(line) - used,
          "%u,%u;",
          (unsigned int)s_path.cells[index].x,
          (unsigned int)s_path.cells[index].y);

      if (written <= 0)
      {
        break;
      }

      used = (uint16_t)(used + (uint16_t)written);
      index++;
    }

    if (used < (sizeof(line) - 2U))
    {
      line[used++] = '\r';
      line[used++] = '\n';
      line[used] = '\0';
    }
    else
    {
      line[sizeof(line) - 3U] = '\r';
      line[sizeof(line) - 2U] = '\n';
      line[sizeof(line) - 1U] = '\0';
    }

    (void)BluetoothControl_SendText(line);
  }

  (void)snprintf(
      line,
      sizeof(line),
      "PATH END seq=%lu\r\n",
      (unsigned long)s_plan_seq);
  (void)BluetoothControl_SendText(line);
}
