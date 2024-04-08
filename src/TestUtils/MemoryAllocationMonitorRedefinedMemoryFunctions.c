
// Based on https://stackoverflow.com/questions/6083337/overriding-malloc-using-the-ld-preload-mechanism

#define _GNU_SOURCE

#include <stdio.h>
#include <dlfcn.h>

#include "MemoryAllocationMonitorGlobalVariables.h"

static void * (*g_blf_mam_real_calloc)(size_t nmemb, size_t size);
static void * (*g_blf_mam_real_malloc)(size_t size);
static void   (*g_blf_mam_real_free)(void *ptr);
static void * (*g_blf_mam_real_realloc)(void *ptr, size_t size);
static void * (*g_blf_mam_real_memalign)(size_t blocksize, size_t bytes);

static void blf_mam_init(void)
{
    g_blf_mam_real_calloc = dlsym(RTLD_NEXT, "calloc");
    g_blf_mam_real_malloc = dlsym(RTLD_NEXT, "malloc");
    g_blf_mam_real_free = dlsym(RTLD_NEXT, "free");
    g_blf_mam_real_realloc = dlsym(RTLD_NEXT, "realloc");
    g_blf_mam_real_memalign = dlsym(RTLD_NEXT, "memalign");

    if (NULL == g_blf_mam_real_calloc ||
        NULL == g_blf_mam_real_malloc ||
        NULL == g_blf_mam_real_free ||
        NULL == g_blf_mam_real_realloc ||
        NULL == g_blf_mam_real_memalign) {
        fprintf(stderr, "BipedalLocomotionFramework::MemoryAllocationMonitor: FATAL ERROR IN dlsym`: %s\n", dlerror());
    }

}

void *calloc(size_t nmemb, size_t size)
{
    if(g_blf_mam_real_calloc==NULL) {
        blf_mam_init();
    }

    if (g_blf_mam_monitorEnabled) {
        g_blf_mam_numberOfCallocOperations++;
    }

    return g_blf_mam_real_calloc(nmemb, size);
}

void *malloc(size_t size)
{
    if(g_blf_mam_real_malloc==NULL) {
        blf_mam_init();
    }

    if (g_blf_mam_monitorEnabled) {
        g_blf_mam_numberOfMallocOperations++;
    }

    return g_blf_mam_real_malloc(size);
}

void free(void *ptr)
{
    if(g_blf_mam_real_free==NULL) {
        blf_mam_init();
    }

    if (g_blf_mam_monitorEnabled) {
        g_blf_mam_numberOfFreeOperations++;
    }

    g_blf_mam_real_free(ptr);
    return;
}

void *realloc(void *ptr, size_t size)
{
    if(g_blf_mam_real_realloc==NULL) {
        blf_mam_init();
    }

    if (g_blf_mam_monitorEnabled) {
        g_blf_mam_numberOfReallocOperations++;
    }

    return g_blf_mam_real_realloc(ptr,size);
}

void *memalign(size_t blocksize, size_t bytes)
{
    if(g_blf_mam_real_memalign==NULL) {
        blf_mam_init();
    }

    if (g_blf_mam_monitorEnabled) {
        g_blf_mam_numberOfMemalignOperations++;
    }

    return g_blf_mam_real_memalign(blocksize,bytes);
}
