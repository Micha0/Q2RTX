#pragma once

typedef struct bsp_collision_s bsp_collision_t;
typedef struct bsp_s bsp_t;
typedef struct edict_s edict_t;
typedef struct cplane_s cplane_t;

#ifdef EXPORT
#undef EXPORT
#endif

#ifdef __cplusplus
#define EXPORT extern "C"
#define EXTERN extern "C"
#else
#define EXPORT
#define EXTERN extern
#endif

EXTERN void COL_Init(bsp_t* bsp);
EXTERN void COL_Think();
EXTERN void COL_Destroy(bsp_t* bsp);
EXTERN void COL_SpawnEntity(edict_t* ent);
EXPORT void COL_LoadComplete();
EXTERN void COL_DebugDraw();
EXTERN void COL_SpawnConstraint(edict_t* ent);
EXTERN void COL_GetPlaneCorners(unsigned short planeId, int* outSize, float** outFloat);
EXTERN void COL_DebugVisualizePlane(unsigned short planeId);
