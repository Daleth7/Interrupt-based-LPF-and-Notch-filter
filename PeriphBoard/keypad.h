#ifndef KEYPAD_HEADER_FILEEE823794872938______
#define KEYPAD_HEADER_FILEEE823794872938______

#include "extended_types.h"

void configure_keypad_ports(void);
void check_key(UINT8* row_dest, UINT8* col_dest);
UINT8 debounce_keypress(void);

#endif