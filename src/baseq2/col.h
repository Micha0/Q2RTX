typedef struct bsp_collision_s bsp_collision_t;
typedef struct bsp_s bsp_t;
typedef struct edict_s edict_t;

#ifdef __cplusplus__
#define EXPORT extern "C"
#else
#define EXPORT
#endif

EXPORT void COL_Init(bsp_t* bsp);
EXPORT void COL_Think();
EXPORT void COL_Destroy(bsp_t* bsp);
EXPORT void COL_SpawnEntity(edict_t* ent);
EXPORT void COL_DebugDraw();
