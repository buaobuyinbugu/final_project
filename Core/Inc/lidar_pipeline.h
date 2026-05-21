#ifndef LIDAR_PIPELINE_H
#define LIDAR_PIPELINE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  LIDAR_DMA_EVENT_HALF = 0,
  LIDAR_DMA_EVENT_FULL = 1
} LidarDmaEvent_t;

typedef struct
{
  uint16_t offset;
  uint16_t length;
  uint32_t block_id;
  uint32_t tick_count;
  LidarDmaEvent_t event;
} LidarDmaBlockDescriptor_t;

typedef struct
{
  uint32_t block_id;
  uint32_t tick_count;
  LidarDmaEvent_t event;
  uint16_t block_bytes;
  uint32_t total_bytes;
  uint16_t valid_nodes;
  uint16_t invalid_nodes;
  uint16_t scan_start_nodes;
  uint16_t last_distance_mm;
  uint16_t last_quality;
  uint16_t min_distance_mm;
  uint16_t first_angle_cdeg;
  uint16_t last_angle_cdeg;
  uint8_t descriptor_seen;
  uint8_t parser_pending_bytes;
  uint32_t uart_error_count;
  uint32_t dma_queue_drops;
  uint32_t result_queue_drops;
  uint32_t point_queue_drops;
} LidarParseResult_t;

#define LIDAR_POINT_FLAG_SCAN_START 0x01U

typedef struct
{
  uint32_t tick_count;
  uint16_t angle_cdeg;
  uint16_t distance_mm;
  uint8_t quality;
  uint8_t flags;
} LidarPoint_t;

bool LidarPipeline_Init(void);
bool LidarPipeline_GetLatestResult(LidarParseResult_t *out_result);
bool LidarPipeline_TakePoint(LidarPoint_t *out_point);
uint32_t LidarPipeline_GetPointQueueDrops(void);

#endif /* LIDAR_PIPELINE_H */
