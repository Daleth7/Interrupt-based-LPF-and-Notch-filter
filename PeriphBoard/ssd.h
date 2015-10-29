#ifndef SEVEN_SEGMENT_DIISPLAY_HDR8293784838888888______
#define SEVEN_SEGMENT_DIISPLAY_HDR8293784838888888______

#include "extended_types.h"

void configure_ssd_ports(void);
void display_dig(
    UINT32 add_delay, UINT8 num, UINT8 select,
    BOOLEAN__ show_dot, BOOLEAN__ show_sign
);
void turn_off_ssd(void);

#endif