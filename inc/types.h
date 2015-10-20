#ifndef TYPES_H
#define TYPES_H

typedef signed char             int8;
typedef signed short            int16;
typedef signed long             int32;

typedef signed char const       cint8;
typedef signed short const      cint16;
typedef signed long const       cint32;

typedef unsigned char           uint8;
typedef unsigned short          uint16;
typedef unsigned long           uint32;

typedef unsigned char const     ucint8;
typedef unsigned short const    ucint16;
typedef unsigned long const     ucint32;

typedef unsigned char   bool;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef NULL
#define NULL 0
#endif

#ifndef BV
#define BV(n)      (1 << (n))
#endif

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)


#define INT8_MIN    -128
#define INT16_MIN   -32768
#define INT32_MIN   -2147483648

#define INT8_MAX    127
#define INT16_MAX   32767
#define INT32_MAX   2147483647

#define UINT8_MAX   255
#define UINT16_MAX  65535u
#define UINT32_MAX  4294967295uL

#endif
