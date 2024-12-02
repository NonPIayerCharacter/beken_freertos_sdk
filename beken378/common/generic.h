#ifndef _GENERIC_H_
#define _GENERIC_H_
#include <stdbool.h>
#include "include.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*FUNCPTR)(void);
typedef void (*FUNC_1PARAM_PTR)(void *ctxt);
typedef void (*FUNC_2PARAM_PTR)(void *arg, unsigned char vif_idx);
typedef int (*FUNC_2PARAM_CB)(uint32_t larg, uint32_t rarg);

#ifndef _MAX
#define _MAX(x, y)                  (((x) > (y)) ? (x) : (y))
#endif

#ifndef _MIN
#define _MIN(x, y)                  (((x) < (y)) ? (x) : (y))
#endif

#ifndef _max
#define _max(x, y)                  (((x) > (y)) ? (x) : (y))
#endif

#ifndef _min
#define _min(x, y)                  (((x) < (y)) ? (x) : (y))
#endif

extern void bk_printf(const char *fmt, ...);
#define as_printf (bk_printf("%s:%d\r\n",__FUNCTION__,__LINE__))

#if (0 == CFG_RELEASE_FIRMWARE)
#define ASSERT_EQ(a, b)                             \
{                                                   \
    if ((a) != (b))                                 \
    {                                               \
        bk_printf("%s:%d %d!=%d\r\n",__FUNCTION__,__LINE__, (a), (b)); \
        while(1);                                   \
    }                                               \
}
#define ASSERT_NE(a, b)                             \
{                                                   \
    if ((a) == (b))                                 \
    {                                               \
        bk_printf("%s:%d %d==%d\r\n",__FUNCTION__,__LINE__, (a), (b)); \
        while(1);                                   \
    }                                               \
}
#define ASSERT_GT(a, b)                             \
{                                                   \
    if ((a) <= (b))                                 \
    {                                               \
        bk_printf("%s:%d %d<=%d\r\n",__FUNCTION__,__LINE__, (a), (b)); \
        while(1);                                   \
    }                                               \
}
#define ASSERT_GE(a, b)                             \
{                                                   \
    if ((a) < (b))                                 \
    {                                               \
        bk_printf("%s:%d %d<%d\r\n",__FUNCTION__,__LINE__, (a), (b)); \
        while(1);                                   \
    }                                               \
}
#define ASSERT_LT(a, b)                             \
{                                                   \
    if ((a) >= (b))                                 \
    {                                               \
        bk_printf("%s:%d %d>=%d\r\n",__FUNCTION__,__LINE__, (a), (b)); \
        while(1);                                   \
    }                                               \
}
#define ASSERT_LE(a, b)                             \
{                                                   \
    if ((a) > (b))                                 \
    {                                               \
        bk_printf("%s:%d %d>%d\r\n",__FUNCTION__,__LINE__, (a), (b)); \
        while(1);                                   \
    }                                               \
}
#define ASSERT(exp)                                 \
{                                                   \
    if ( !(exp) )                                   \
    {                                               \
    	as_printf;							     	\
        while(1);                                   \
    }                                               \
} 
#else
#define ASSERT_EQ(exp)
#define ASSERT_NE(exp)
#define ASSERT_GT(exp)
#define ASSERT_GE(exp)
#define ASSERT_LT(exp)
#define ASSERT_LE(exp)
#define ASSERT(exp)
#endif

#define BUG_ON(exp)               ASSERT(!(exp))

#ifndef NULL
#define NULL                     (0L)
#endif

#ifndef NULLPTR
#define NULLPTR                  ((void *)0)
#endif

#define BIT(i)                   (1UL << (i))

static inline unsigned short __bswap16_bk( unsigned short _x)
{

	return (( unsigned short)((_x >> 8) | ((_x << 8) & 0xff00)));
}

static inline  unsigned int __bswap32_bk(unsigned int _x)
{

	return ((unsigned int)((_x >> 24) | ((_x >> 8) & 0xff00) |
	    ((_x << 8) & 0xff0000) | ((_x << 24) & 0xff000000)));
}

static inline unsigned long long  __bswap64_bk(unsigned long long _x)
{

	return ((unsigned long long)((_x >> 56) | ((_x >> 40) & 0xff00) |
	    ((_x >> 24) & 0xff0000) | ((_x >> 8) & 0xff000000) |
	    ((_x << 8) & ((unsigned long long)0xff << 32)) |
	    ((_x << 24) & ((unsigned long long)0xff << 40)) |
	    ((_x << 40) & ((unsigned long long)0xff << 48)) | ((_x << 56))));
}

#define __swab16(x) __bswap16_bk((unsigned char *)&(x))
#define __swab32(x) __bswap32_bk((unsigned char *)&(x))

#define cpu_to_le16(x)   (x)
#define cpu_to_le32(x)   (x)

#define __cpu_to_be32(x) __swab32((x))
#define __be32_to_cpu(x) __swab32((x))
#define __cpu_to_be16(x) __swab16((x))
#define __be16_to_cpu(x) __swab16((x))

#ifndef __htonl
#define	__htonl(_x)	__bswap32_bk(_x)
#endif

#ifndef __htons
#define	__htons(_x)	__bswap16_bk(_x)
#endif

#ifndef __ntohl
#define	__ntohl(_x)	__bswap32_bk(_x)
#endif

#ifndef __ntohs
#define	__ntohs(_x)	__bswap16_bk(_x)
#endif

#define ___htonl(x) __cpu_to_be32(x)
#define ___htons(x) __cpu_to_be16(x)
#define ___ntohl(x) __be32_to_cpu(x)
#define ___ntohs(x) __be16_to_cpu(x)

#if (!CFG_SUPPORT_RTT)
#define htons(x) __htons(x)
#define ntohs(x) __ntohs(x)
#define htonl(x) __htonl(x)
#define ntohl(x) __ntohl(x)
#endif

extern size_t strnlen(const char * s, size_t len);

#ifdef __cplusplus
}
#endif

#endif // _GENERIC_H_

