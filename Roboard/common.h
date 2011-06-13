#ifndef __COMMON_H
#define __COMMON_H

#include <stdlib.h>
#include "defines.h"


#ifdef __cplusplus
extern "C" {
#endif


#define	     ERR_NOERROR    (0)
RBAPI(int)   roboio_GetErrCode(void);
RBAPI(char*) roboio_GetErrMsg(void);

RBAPI(bool) err_SetLogFile(char* logfile);
RBAPI(void) err_CloseLogFile(void);


RBAPI(bool) roboio_SetRBVer(int ver);
//-- values for the "ver" argument
     #define RB_100b1       (98)    //early RB-100 with 32-channel PWM (& TTL COM1) for interal use
     #define RB_100b2       (99)    //early RB-100 with RTS-controlled COM4
     #define RB_100         (100)   //officially released RB-100
     #define RB_110         (110)
     #define RB_050         (50)
//-- if the above function returns false, roboio_GetErrCode() may return:
     #define ERROR_RBVER_UNMATCH    (ERR_NOERROR + 800)
     #define ERROR_RBVER_UNKNOWN    (ERR_NOERROR + 801)


#ifdef __cplusplus
}
#endif 


#if defined(USE_COMMON)
#ifdef __cplusplus
extern "C" {
#endif

    RBAPI(int)  roboio_GetRBVer(void);

    RBAPI(bool) roboio_CheckRBVer(void);
    //-- if the above function returns false, roboio_GetErrCode() may return:
    //   #define ERROR_RBVER_UNMATCH    (ERR_NOERROR + 800)
    //   #define ERROR_RBVER_UNKNOWN    (ERR_NOERROR + 801)


    //ERROR MESSAGE:
    _RBAPI_C(void) errmsg(char* fmt, ...);
    _RBAPI_C(void) errmsg_exit(char* fmt, ...);


    //common blackboard for ERROR responses
    _RBAPI_C(void) err_SetMsg(int errtype, char* fmt, ...);


    //TIMER:
    RBAPI(unsigned long) timer_nowtime(void);
    RBAPI(void) delay_ms(unsigned long delaytime);
    RBAPI(void) delay_us(unsigned long delaytime);


    //MEMORY:
    RBAPI(void*) mem_alloc(size_t size);
    RBAPI(void*) mem_realloc(void* pointer, size_t size);

#ifdef __cplusplus
}
#endif 
#endif

#endif

