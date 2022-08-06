#pragma once

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

typedef struct bsp_s bsp_t;
typedef struct mnode_s mnode_t;
typedef struct mleaf_s mleaf_t;
typedef struct mmodel_s mmodel_t;
typedef struct mbrush_s mbrush_t;
typedef struct cplane_s cplane_t;

EXPORT typedef void (*BSP_ReceiveLeafsFPtr) (bsp_t* bsp, mleaf_t* leaf, void* context);
EXPORT typedef void (*BSP_ReceiveBrushLeafFPtr) (bsp_t* bsp, mmodel_t* leaf, mbrush_t* brush, void* context);
EXPORT typedef void (*BSP_ReceiveBrushPlaneFPtr) (bsp_t* bsp, mmodel_t* leaf, mbrush_t* brush, cplane_t* plane, float plane_nx, float plane_ny, float plane_nz, float plane_d, void* context);

EXPORT void BSP_RecurseLeafs(bsp_t* bsp, mnode_t* node, void* context, BSP_ReceiveLeafsFPtr callback);
EXPORT void BSP_RecurseBrushes(bsp_t* bsp, mmodel_t* root, int brushMask, void* context, BSP_ReceiveBrushLeafFPtr callback);
EXPORT void BSP_RecurseBrushPlanes(bsp_t* bsp, void* context, int brushMask /* = (CONTENTS_SOLID|CONTENTS_WINDOW)*/, BSP_ReceiveBrushPlaneFPtr callback);

/*
Game DLL needs to know when a map has been loaded and destroyed
But there's a hitch:
Only Game DLL (baseq2) and server knows about
spawn_temp_t, game_export_t, game_import_t, level_locals_t, game_locals_t, gi or ge

So we make our own exports instead. 
*/
EXTERN void (*G_Passthrough_BSP_Loaded)(bsp_t* bsp);
EXTERN void (*G_Passthrough_BSP_Destroy)(bsp_t* bsp);
