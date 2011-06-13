#ifndef __SPI_H
#define __SPI_H

#include "defines.h"
#include "spidx.h"


#ifdef __cplusplus
extern "C" {
#endif

RBAPI(bool) spi_InUse(void);

/***************************  SPI Hardware Function  **************************/
// note: SPI lib assumes that I/O pins for GPIO3/SPI have functioned as GPIO3 or SPI pins;
//       this is BIOS default setting in general.
RBAPI(bool) spi_Init2(unsigned baseaddr, int clkmode);
//-- values for the "clkmode" argument
     #define SPICLK_21400KHZ              (0)
     #define SPICLK_18750KHZ              (1)
     #define SPICLK_15000KHZ              (2)
     #define SPICLK_12500KHZ              (3)
     #define SPICLK_10000KHZ              (4)
     #define SPICLK_10714KHZ              (5)
     #define SPICLK_11538KHZ              (6)
     #define SPICLK_13636KHZ              (7)
     #define SPICLK_16666KHZ              (8)
     #define SPICLK_25000KHZ              (9)
     #define SPICLK_30000KHZ              (10)
     #define SPICLK_37000KHZ              (11)
     #define SPICLK_50000KHZ              (12)
     #define SPICLK_75000KHZ              (13)
     #define SPICLK_150000KHZ             (14)
//-- if the above function return false, roboio_GetErrCode() may return:
//   #define ERROR_RBVER_UNKNOWN          (ERR_NOERROR + 801) //need include <common.h>
//   #define ERROR_RBVER_UNMATCH          (ERR_NOERROR + 800) //need include <common.h>
//   #define ERROR_IOINITFAIL             (ERR_NOERROR + 100) //need include <io.h>
//   #define ERROR_IOSECTIONFULL          (ERR_NOERROR + 101) //need include <io.h>
//   #define ERROR_CPUUNSUPPORTED         (ERR_NOERROR + 102) //need include <io.h>
     #define ERROR_SPI_INUSE              (ERR_NOERROR + 200)
     #define ERROR_SPI_INITFAIL           (ERR_NOERROR + 202)

RBAPI(void) spi_Close(void);
/*---------------------  end of SPI Hardware Function  -----------------------*/



/**********************  SPI Software-Simulated Function  *********************/
RBAPI(bool) spi_InitSW(int mode, unsigned long clkdelay);
//-- values for the "mode" argument (note that different values can be added to use mixed modes)
	 #define SPIMODE_CPOL0                (0x00)
	 #define SPIMODE_CPOL1                (0x04)
	 #define SPIMODE_SMOD0                (0x00)
	 #define SPIMODE_SMOD1                (0x02)
	 #define SPIMODE_CPHA0                (0x00)
	 #define SPIMODE_CPHA1                (0x01)
//-- if the above functions return false, roboio_GetErrCode() may return:
//   #define ERROR_RBVER_UNKNOWN          (ERR_NOERROR + 801) //need include <common.h>
//   #define ERROR_RBVER_UNMATCH          (ERR_NOERROR + 800) //need include <common.h>
//   #define ERROR_IOINITFAIL             (ERR_NOERROR + 100) //need include <io.h>
//   #define ERROR_IOSECTIONFULL          (ERR_NOERROR + 101) //need include <io.h>
//   #define ERROR_CPUUNSUPPORTED         (ERR_NOERROR + 102) //need include <io.h>
//   #define ERROR_SPI_INUSE              (ERR_NOERROR + 200)
     #define ERROR_SPISW_INVALIDMODE      (ERR_NOERROR + 230)

RBAPI(void) spi_CloseSW(void);
/*----------------  end of SPI Software-Simulated Function  ------------------*/



/***************************  SPI Common Functions  ***************************/
RBAPI(bool) spi_Write(unsigned char val);
RBAPI(bool) spi_WriteFlush(unsigned char val);
//-- if the above functions return false, roboio_GetErrCode() may return:
//   #define ERROR_SPIFIFOFULL            (ERR_NOERROR + 214) //need include <spidx.h>
//   #define ERROR_SPIFIFOFAIL            (ERR_NOERROR + 215) //need include <spidx.h>
//   #define ERROR_SPISW_INVALIDMODE      (ERR_NOERROR + 230) //need include <spidx.h>

RBAPI(bool) spi_FIFOFlush(void);
//-- if the above function returns false, roboio_GetErrCode() may return:
//   #define ERROR_SPIFIFOFAIL            (ERR_NOERROR + 215) //need include <spidx.h>
//   #define ERROR_SPISW_INVALIDMODE      (ERR_NOERROR + 230) //need include <spidx.h>

RBAPI(unsigned) spi_Read(void);
//-- if the above function returns 0xffff, roboio_GetErrCode() may return:
//   #define ERROR_SPIFIFOFAIL            (ERR_NOERROR + 215) //need include <spidx.h>
//   #define ERROR_SPIREADFAIL            (ERR_NOERROR + 219) //need include <spidx.h>
//   #define ERROR_SPISW_INVALIDMODE      (ERR_NOERROR + 230) //need include <spidx.h>

RBAPI(unsigned) spi_Read2(void);
RBAPI(unsigned) spi_ReadStrict(void);
//-- if the above functions return 0xffff, roboio_GetErrCode() may return:
//   #define ERROR_SPIREADFAIL            (ERR_NOERROR + 219) //need include <spidx.h>
//   #define ERROR_SPISW_INVALIDMODE      (ERR_NOERROR + 230) //need include <spidx.h>

RBAPI(unsigned) spi_Exchange(unsigned char val);
//-- if the above functions return 0xffff, roboio_GetErrCode() may return:
//   #define ERROR_SPIFIFOFULL            (ERR_NOERROR + 214) //need include <spidx.h>
//   #define ERROR_SPIFIFOFAIL            (ERR_NOERROR + 215) //need include <spidx.h>
//   #define ERROR_SPISW_INVALIDMODE      (ERR_NOERROR + 230) //need include <spidx.h>
/*----------------------  end of SPI Common Functions  -----------------------*/



RBAPI(bool) spi_EnableSS(void);
RBAPI(bool) spi_DisableSS(void);
//-- if the above functions return false, roboio_GetErrCode() may return:
//   #define ERROR_SPIFIFOFAIL            (ERR_NOERROR + 215)
     #define ERROR_SPI_UNSUPPORTED        (ERR_NOERROR + 220)

#ifdef __cplusplus
}
#endif



/****************************  Inline Functions  **************************/
#ifdef ROBOIO_DLL //use no inline functions for DLL
#ifdef __cplusplus
extern "C" {
#endif
    RBAPI(bool) spi_Init(int clkmode);

    #ifdef ROBOIO
        // for compatibility to RoBoIO 1.61
        RBAPI(bool) spi_Initialize(int clkmode);
        RBAPI(bool) spi_InitializeSW(int mode, unsigned long clkdelay);
    #endif

#ifdef __cplusplus
}
#endif
#endif

#if !defined(ROBOIO_DLL) || defined(__SPI_LIB)
    RB_INLINE RBAPI(bool) spi_Init(int clkmode) {
        return spi_Init2(0xffff, clkmode);
    }

    #ifdef ROBOIO
        // for compatibility to RoBoIO 1.61
        RB_INLINE RBAPI(bool) spi_Initialize(int clkmode) {
            return spi_Init(clkmode);
        }
        RB_INLINE RBAPI(bool) spi_InitializeSW(int mode, unsigned long clkdelay) {
            return spi_InitSW(mode, clkdelay);
        }
    #endif

#endif
/*-----------------------  end of Inline Functions  ----------------------*/

#endif

