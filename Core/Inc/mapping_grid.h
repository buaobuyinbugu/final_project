#ifndef MAPPING_GRID_H
#define MAPPING_GRID_H

#include <stdbool.h>
#include <stdint.h>

#define MAPPING_GRID_WIDTH_CELLS   80U
#define MAPPING_GRID_HEIGHT_CELLS  80U
#define MAPPING_GRID_CELL_SIZE_MM  50U
#define MAPPING_GRID_MAX_RANGE_MM  8000U

typedef enum
{
  MAPPING_GRID_CELL_UNKNOWN = 0,
  MAPPING_GRID_CELL_FREE,
  MAPPING_GRID_CELL_OCCUPIED
} MappingGridCellState_t;

typedef struct
{
  int32_t x_mm;
  int32_t y_mm;
  int32_t heading_cdeg;
} MappingGridPose_t;

typedef struct
{
  uint16_t unknown_cells;
  uint16_t free_cells;
  uint16_t occupied_cells;
  uint32_t inserted_points;
  uint32_t rejected_points;
  uint32_t free_ray_updates;
  uint32_t occupied_updates;
  uint32_t clipped_rays;
  uint32_t revision;
} MappingGridStats_t;

void MappingGrid_Init(void);
void MappingGrid_Reset(void);
void MappingGrid_SetPose(const MappingGridPose_t *pose);
bool MappingGrid_GetPose(MappingGridPose_t *out_pose);
bool MappingGrid_InsertPolarPoint(uint16_t angle_cdeg, uint16_t distance_mm, uint8_t quality);
bool MappingGrid_InsertPolarPointAtPose(const MappingGridPose_t *pose,
                                        uint16_t angle_cdeg,
                                        uint16_t distance_mm,
                                        uint8_t quality);
bool MappingGrid_WorldToCell(int32_t x_mm, int32_t y_mm, uint8_t *out_x, uint8_t *out_y);
MappingGridCellState_t MappingGrid_GetCell(uint8_t x, uint8_t y);
bool MappingGrid_FormatRow(uint8_t row, char *out_text, uint16_t out_size);
bool MappingGrid_GetStats(MappingGridStats_t *out_stats);
uint32_t MappingGrid_GetRevision(void);

#endif /* MAPPING_GRID_H */
