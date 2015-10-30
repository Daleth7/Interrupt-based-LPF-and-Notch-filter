#ifndef UTILITYES_HDR238838777______7477______
#define UTILITYES_HDR238838777______7477______

#include "extended_types.h"

UINT32 find_lsob(UINT32 target);    // Find least significant ON bit
UINT32 map32(
    UINT32 orig,
    UINT32 old_min, UINT32 old_max,
    UINT32 new_min, UINT32 new_max
    );
float mapf(
    float orig,
    float old_min, float old_max,
    float new_min, float new_max
    );

#endif