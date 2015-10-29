#ifndef TYPE_ALIASES___________
#define TYPE_ALIASES___________

    // Common type aliases and inline expressions

#include <stdint.h>

#define INT32   int32_t
#define UINT32  uint32_t
#define UINT16  uint16_t
#define UINT8   uint8_t

#define BOOLEAN__     UINT8
#define TRUE__        1
#define FALSE__       0

    // Short macro functions for inlining common expressions
#define IS_NULL(P) (P == NULL)

#endif