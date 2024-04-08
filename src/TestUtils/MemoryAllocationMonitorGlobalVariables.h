#ifndef BIPEDAL_LOCOMOTION_MEMORY_ALLOCATION_MONITOR_GLOBAL_VARIABLE_H
#define BIPEDAL_LOCOMOTION_MEMORY_ALLOCATION_MONITOR_GLOBAL_VARIABLE_H

#include <stdint.h>

// We do this to ensure that this variable always have C mangling
#ifdef __cplusplus
extern "C" 
{
#endif

/** 1 if the monitor is enabled, 0 otherwise */
extern int32_t g_blf_mam_monitorEnabled;

/** Number of calloc operations in this monitor */
extern int32_t g_blf_mam_numberOfCallocOperations;

/** Number of malloc operations in this monitor */
extern int32_t g_blf_mam_numberOfMallocOperations;

/** Number of free operation in this monitor */
extern int32_t g_blf_mam_numberOfFreeOperations;

/** Number of realloc operation in this monitor */
extern int32_t g_blf_mam_numberOfReallocOperations;

/** Number of memalign operation in this monitor */
extern int32_t g_blf_mam_numberOfMemalignOperations;

#ifdef __cplusplus
}
#endif



#endif

