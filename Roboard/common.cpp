#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include "defines.h"
#include "io.h"

#define  USE_COMMON
#include "common.h"

#if defined(RB_MSVC_WIN32) || defined(RB_MSVC_WINCE)
	#include <windows.h>
#elif defined(RB_LINUX)
    #include <unistd.h>
    #include <sys/times.h>
#elif defined(RB_BC_DOS)
	#include <dos.h>
#elif defined(RB_DJGPP)
    //#include <bios.h>
    #include <time.h>
#elif defined(RB_WATCOM)
    #include <i86.h>
    #include <time.h>
#endif



static int RBVer = RB_100;

RBAPI(int) roboio_GetRBVer(void) {
    return RBVer;
}

RBAPI(bool) roboio_CheckRBVer(void) {  // called only by io_Init(); the caller must ensure that I/O ports can be accessed by this function
    int  cpuid = io_CpuID();

    switch (roboio_GetRBVer())
    {
        case RB_100b1:
        case RB_100b2:
            if (cpuid == CPU_VORTEX86DX_1) return true;
            break;
        case RB_100:
            if ((cpuid == CPU_VORTEX86DX_1) || (cpuid == CPU_VORTEX86DX_3)) return true;
            break;
        case RB_110:
        case RB_050:
            if (cpuid == CPU_VORTEX86DX_3) return true;
            break;
        default:
	       err_SetMsg(ERROR_RBVER_UNKNOWN, "unknown RoBoard version");
	       return false;
    } //end switch (ver)

	err_SetMsg(ERROR_RBVER_UNMATCH, "unmatched RoBoard version");
	return false;
}

RBAPI(bool) roboio_SetRBVer(int ver) {
    switch (ver)
    {
        case RB_100b1:
        case RB_100b2:
        case RB_100:
        case RB_110:
        case RB_050:
            RBVer = ver;
            break;
        default:
	       err_SetMsg(ERROR_RBVER_UNKNOWN, "unknown RoBoard version");
	       return false;
    } //end switch (ver)

    return true;
}



/******************  Error Message Functions  ******************/
static int   ERR_Type;
static char  ERR_MsgBuf[512] = {'\0'};

RBAPI(int) roboio_GetErrCode(void) {
    return ERR_Type;
}

RBAPI(char*) roboio_GetErrMsg(void) {
    return &(ERR_MsgBuf[0]);
}

static FILE* ERR_outputDevice = stderr;

RBAPI(bool) err_SetLogFile(char* logfile) {
    err_CloseLogFile();

	if (logfile == NULL)
	{
		ERR_outputDevice = NULL;
		return true;
	}

	if ((ERR_outputDevice = fopen(logfile, "w")) != NULL) return true;

	ERR_outputDevice = stderr;
    return false;
}

RBAPI(void) err_CloseLogFile(void) {
    if ((ERR_outputDevice != stderr) && (ERR_outputDevice != NULL))
        fclose(ERR_outputDevice);

	ERR_outputDevice = stderr;
}

#ifdef _MANAGED
	#pragma managed(push, off)
#endif
_RBAPI_C(void) errmsg(char* fmt, ...) {
    va_list args;

    if (ERR_outputDevice == NULL) return;

	va_start(args, fmt);
	vfprintf(ERR_outputDevice, fmt, args);
	va_end(args);

    fflush(ERR_outputDevice);
}

_RBAPI_C(void) errmsg_exit(char* fmt, ...) {
    va_list args;

    if (ERR_outputDevice != NULL)
    {
	    va_start(args, fmt);
	    vfprintf(ERR_outputDevice, fmt, args);
	    va_end(args);

        fflush(ERR_outputDevice);
    }

	exit(2);
}

_RBAPI_C(void) err_SetMsg(int errtype, char* fmt, ...) {
    char buf[512];
    va_list args;

	va_start(args, fmt);
	vsprintf(buf, fmt, args);
	va_end(args);

	strcpy(ERR_MsgBuf, buf);
	ERR_Type = errtype;
}
#ifdef _MANAGED
	#pragma managed(pop)
#endif
/*-------------- end of Error Message Functions ---------------*/



/******************  Common Timer Functions  *******************/
RBAPI(unsigned long) timer_nowtime(void) { //in ms
#if defined(RB_MSVC_WIN32) || defined(RB_MSVC_WINCE)
	return GetTickCount();
#elif defined(RB_LINUX)
    static bool usetimer = false;
    static unsigned long long inittime;
    struct tms cputime;

    if (usetimer == false)
    {
        inittime  = (unsigned long long)times(&cputime);
        usetimer = true;
    }

    return (unsigned long)((times(&cputime) - inittime)*1000UL/sysconf(_SC_CLK_TCK));
#elif defined(RB_BC_DOS)
    static bool usetimer = false;
    static unsigned long far* timeraddr;
    static unsigned long inittime;
    
    if (usetimer == false)
    {
        timeraddr = (unsigned long far*)MK_FP(0x40,0x6c);
        inittime  = *timeraddr;
        usetimer = true;
    }
    
    return ((*timeraddr) - inittime) * 55UL;
#elif defined(RB_DJGPP)
    static bool usetimer = false;
    static uclock_t inittime;
    
    if (usetimer == false)
    {
        //inittime  = biostime(0, 0);
        inittime = uclock();
        usetimer = true;
    }
    
    //return (biostime(0, 0) - inittime) * 55UL;
    return (unsigned long)((uclock() - inittime)*1000UL/UCLOCKS_PER_SEC);
#elif defined(RB_WATCOM)
    static bool usetimer = false;
    static clock_t inittime;
    
    if (usetimer == false)
    {
        inittime = clock();
        usetimer = true;
    }
    
    return (unsigned long)((clock() - inittime)*1000UL/CLOCKS_PER_SEC);
#else
    #error timer_nowtime() does not support the target system!
#endif
}


RBAPI(void) delay_ms(unsigned long delaytime) { //delay in ms
#if defined(RB_BC_DOS) || defined(RB_WATCOM)
    while (delaytime > 0xffffUL)
    {
        delay(0xffff);
        delaytime = delaytime - 0xffffUL;
    }

    if (delaytime > 0UL) delay((unsigned)delaytime);
#else
	delaytime = delaytime + timer_nowtime();
	while (timer_nowtime() < delaytime);
#endif
}


static unsigned long getclocks(void) {
    unsigned long nowclocks;
 
#if defined(RB_MSVC_WIN32) || defined(RB_MSVC_WINCE) || defined(RB_WATCOM)
    _asm {
        push eax
        push edx        
        rdtsc  //  not yet tested on older Watcom C++
        mov DWORD PTR nowclocks, eax
        //mov DWORD PTR nowclocks_msb, edx
        pop edx
        pop eax        
    };
#elif defined(RB_BC_DOS)
    __emit__(0x66); _asm push ax;
    __emit__(0x66); _asm push dx;

    __emit__(0x0f, 0x31);
    __emit__(0x66); _asm mov WORD PTR nowclocks, ax;
    //__emit__(0x66); _asm mov WORD PTR nowclocks_msb, dx;

    __emit__(0x66); _asm pop dx;
    __emit__(0x66); _asm pop ax;
#elif defined(RB_DJGPP) || defined(RB_LINUX)
    __asm__ __volatile__ (
        ".byte 0x0f; .byte 0x31"
        : "=a" (nowclocks) //, "=d"(nowclocks_msb)
        : //no input registers
        : "edx");
#else
	errmsg_exit("ERROR: %s() isn't supported due to unrecognized target system!\n", __FUNCTION__);
#endif

    return nowclocks;
}

RBAPI(void) delay_us(unsigned long delaytime) { //delay in us
    unsigned long nowclocks;

    #define CLOCKS_PER_MICROSEC (999UL) //only for RoBoard (RB-100)
    nowclocks = getclocks();
    while ((getclocks() - nowclocks)/CLOCKS_PER_MICROSEC < delaytime);
}
/*------------------ end of Timer Functions -------------------*/



/******************  Common Memory Functions  ******************/
RBAPI(void*) mem_alloc(size_t size) {
	void* p;

	if ((p = malloc(size)) == NULL)
	   errmsg_exit("ERROR: no enough memory to allocate %u bytes!", size);

	return p;
}

RBAPI(void*) mem_realloc(void* pointer, size_t size) {
	void* p;

	if ((p = realloc(pointer, size)) == NULL)
	   errmsg_exit("ERROR: no enough memory to reallocate %u bytes!", size);

	return p;
}
/*------------------- end of Memory Functions -----------------*/

