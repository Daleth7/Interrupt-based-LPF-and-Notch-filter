#include "utilities.h"

UINT32 find_lsob(UINT32 target){
    UINT32 toreturn = 0u;
    while(!(target & 0x1)){
        ++toreturn;
        target >>= 1;
    }
    return toreturn;
}

UINT32 map32(
    UINT32 orig,
    UINT32 old_min, UINT32 old_max,
    UINT32 new_min, UINT32 new_max
    )
{
    return ((orig-old_min)*(new_max-new_min))/(old_max-new_min) + new_min;
}

float mapf(
    float orig,
    float old_min, float old_max,
    float new_min, float new_max
    )
{
    return ((orig-old_min)*(new_max-new_min))/(old_max-new_min) + new_min;
}