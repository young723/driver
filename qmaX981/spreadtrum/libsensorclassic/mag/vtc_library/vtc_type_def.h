/****************************************************************
*
* @file vtc_type_def.h
*
* Copyright(C), 2016, Voltafield Corporation
*
****************************************************************/

#ifndef _VTC_TYPE_DEF_H
#define _VTC_TYPE_DEF_H

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */
//================================================================

#if 1
/* type definitions */
#ifndef UINT8
  typedef unsigned char                                       UINT8;
#endif
#ifndef UINT16
  typedef unsigned short  			 	                            UINT16;
#endif
#ifndef UINT32
  typedef unsigned int                                        UINT32;
#endif
#ifndef INT8
  typedef signed char                                         INT8;
#endif
#ifndef INT16
  typedef signed short                                        INT16;
#endif
#ifndef INT32
  typedef signed int                                          INT32;
#endif
#ifndef UINT64
  typedef unsigned long long                                  UINT64;
#endif
#ifndef INT64
  typedef signed long long                                    INT64;
#endif

#ifndef BOOL
  typedef unsigned int                                        BOOL;
#endif

#ifndef FALSE
  #define FALSE                                               ((BOOL)0)
#endif

#ifndef TRUE
  #define TRUE                                                ((BOOL)1)
#endif
#endif

typedef float                                               FLOAT;
typedef double                                              DOUBLE;
typedef char                                                CHAR;
//================================================================

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */

#endif /* _VTC_TYPE_DEF_H */

