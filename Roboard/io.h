#ifndef __IO_H
#define __IO_H

#include "defines.h"

#if defined(RB_MSVC_WIN32)  // currently used in io.cpp & rcservo.cpp
    #define USE_WINIO       // use Yariv Kaplan's WinIO library  (http://www.internals.com/)
    //#define USE_PCIDEBUG  // use Kasiwano's PciDebug library   (http://www.otto.to/~kasiwano/)
    //#define USE_PHYMEM    // use cyb70289's PhyMem library     (http://www.codeproject.com/KB/system/phymem.aspx)
#endif

#ifdef __cplusplus
extern "C" {
#endif

RBAPI(bool) io_InUse(void);

RBAPI(int)  io_Init(void);
RBAPI(bool) io_init(void);  // for backward compatibility to RoBoIO v1.0
//-- if return -1 or false, roboio_GetErrCode() may return:
//   #define ERROR_RBVER_UNKNOWN    (ERR_NOERROR + 801) //need include <common.h>
//   #define ERROR_RBVER_UNMATCH    (ERR_NOERROR + 800) //need include <common.h>
     #define ERROR_IOINITFAIL		(ERR_NOERROR + 100)
     #define ERROR_IOSECTIONFULL	(ERR_NOERROR + 101)
     #define ERROR_CPUUNSUPPORTED	(ERR_NOERROR + 102)

RBAPI(void) io_Close(int section);
RBAPI(void) io_close(void);  // for backward compatibility to RoBoIO v1.0


RBAPI(void) io_outpdw(unsigned short addr, unsigned long val);
RBAPI(unsigned long) io_inpdw(unsigned short addr);

RBAPI(void) io_outpw(unsigned short addr, unsigned short val);
RBAPI(unsigned short) io_inpw(unsigned short addr);

RBAPI(void) io_outpb(unsigned short addr, unsigned char val);
RBAPI(unsigned char) io_inpb(unsigned short addr);


RBAPI(void) io_SetISASpeed(int mode);
//-- values for the "mode" argument
     #define ISAMODE_NORMAL	           (0)
     #define ISAMODE_HIGHSPEED	       (1)

RBAPI(int) io_CpuID(void);
//-- return-values of io_CpuID()
     #define CPU_UNSUPPORTED	       (0)
     #define CPU_VORTEX86SX		       (10)
     #define CPU_VORTEX86DX_1	       (21)
     #define CPU_VORTEX86DX_2	       (22)
     #define CPU_VORTEX86DX_3	       (23)
     #define CPU_VORTEX86DX_UNKNOWN    (24)
     #define CPU_VORTEX86MX            (30)
     #define CPU_VORTEX86MX_PLUS       (33)


RBAPI(void) io_EnableIRQ(void);
RBAPI(void) io_DisableIRQ(void);

RBAPI(void) io_outpcidw(unsigned long addr, unsigned long val);
RBAPI(unsigned long) io_inpcidw(unsigned long addr);

RBAPI(void) io_outpciw(unsigned long addr, unsigned short val);
RBAPI(unsigned short) io_inpciw(unsigned long addr);

RBAPI(void) io_outpcib(unsigned long addr, unsigned char val);
RBAPI(unsigned char) io_inpcib(unsigned long addr);

#ifdef __cplusplus
}
#endif


/****************************  Inline Functions  **************************/
#ifdef ROBOIO_DLL //use no inline functions for DLL
#ifdef __cplusplus
extern "C" {
#endif
    RBAPI(unsigned char) read_sb_regb(unsigned char idx);
    RBAPI(unsigned short) read_sb_regw(unsigned char idx);
    RBAPI(unsigned long) read_sb_reg(unsigned char idx);
    RBAPI(void) write_sb_regb(unsigned char idx, unsigned char val);
    RBAPI(void) write_sb_regw(unsigned char idx, unsigned short val);  //idx must = 0 (mod 2)
    RBAPI(void) write_sb_reg(unsigned char idx, unsigned long val);    //idx must = 0 (mod 4)

    RBAPI(unsigned char) read_nb_regb(unsigned char idx);
    RBAPI(unsigned short) read_nb_regw(unsigned char idx);
    RBAPI(unsigned long) read_nb_reg(unsigned char idx);
    RBAPI(void) write_nb_regb(unsigned char idx, unsigned char val);
    RBAPI(void) write_nb_regw(unsigned char idx, unsigned short val);  //idx must = 0 (mod 2)
    RBAPI(void) write_nb_reg(unsigned char idx, unsigned long val);    //idx must = 0 (mod 4)
#ifdef __cplusplus
}
#endif
#endif

#if !defined(ROBOIO_DLL) || defined(__IO_LIB)
    // read south bridge register
    RB_INLINE RBAPI(unsigned char) read_sb_regb(unsigned char idx) {
        return io_inpcib(0x80003800L+(unsigned long)idx);
    }

    RB_INLINE RBAPI(unsigned short) read_sb_regw(unsigned char idx) {
        return io_inpciw(0x80003800L+(unsigned long)idx);
    }

    RB_INLINE RBAPI(unsigned long) read_sb_reg(unsigned char idx) {
        return io_inpcidw(0x80003800L+(unsigned long)idx);
    }

    // write south bridge register
    RB_INLINE RBAPI(void) write_sb_regb(unsigned char idx, unsigned char val) {
        io_outpcib(0x80003800L+(unsigned long)idx, val);
    }

    RB_INLINE RBAPI(void) write_sb_regw(unsigned char idx, unsigned short val) {
        io_outpciw(0x80003800L+(unsigned long)idx, val);
    }

    RB_INLINE RBAPI(void) write_sb_reg(unsigned char idx, unsigned long val) {
        io_outpcidw(0x80003800L+(unsigned long)idx, val);
    }


    // read north bridge register
    RB_INLINE RBAPI(unsigned char) read_nb_regb(unsigned char idx) {
        return io_inpcib(0x80000000L+(unsigned long)idx);
    }

    RB_INLINE RBAPI(unsigned short) read_nb_regw(unsigned char idx) {
        return io_inpciw(0x80000000L+(unsigned long)idx);
    }

    RB_INLINE RBAPI(unsigned long) read_nb_reg(unsigned char idx) {
        return io_inpcidw(0x80000000L+(unsigned long)idx);
    }

    // write north bridge register
    RB_INLINE RBAPI(void) write_nb_regb(unsigned char idx, unsigned char val) {
        io_outpcib(0x80000000L+(unsigned long)idx, val);
    }

    RB_INLINE RBAPI(void) write_nb_regw(unsigned char idx, unsigned short val) {
        io_outpciw(0x80000000L+(unsigned long)idx, val);
    }

    RB_INLINE RBAPI(void) write_nb_reg(unsigned char idx, unsigned long val) {
        io_outpcidw(0x80000000L+(unsigned long)idx, val);
     }
#endif
/*-----------------------  end of Inline Functions  ----------------------*/

#endif

