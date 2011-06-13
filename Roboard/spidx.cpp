/*******************************************************************************
  Update List

   2010/05/18:

    - [by acen] rename the following functions' names from "spi_..." to "spidx_..."

	            spi_DisableCS(), spi_EnableCS(), spi_Write(), spi_WriteFlush(),
				spi_FIFOFlush(), spi_Read(), spi_Read2(), spi_ReadStrict()

	- [by acen] Add sw_dataout(), sw_datain(), spisw_Exchange() for software-simulated SPI
				
*******************************************************************************/

#define __SPIDX_LIB

#define  USE_COMMON
#include "common.h"
#include "io.h"
#include "spidx.h"

#define SPI_TIMEOUT     (500L)  // 500 ms


#define NB_SPI_BASEADDR_REG   (0x40)
/**********  North Bridge External SPI BaseAddr Setting REG: 0x40~0x43  **********
 *    The user can write NB_SPI_BASEADDR_REG in North Bridge to chanage
 *    the base I/O address of the external SPI interface of Vertex86DX.
 *    
 *    For RoBoard, SPI base address has been settled to 0xfc00, and
 *    the user should avoid to change it.
 *********************************************************************************/

#define SB_MULTIFUNC_REG      (0xc0)
/************  South Bridge Multifunction Pin Control REG: 0xC0~0xC3  ************
 *    To use SPI interface, the user should set PINS0 (bit 0 of SB_MULTIFUNC_REG)
 *    to 1. (PINS0 = 1: SPI; PINS0 = 0: GPIO3[3:0])
 *********************************************************************************/

static unsigned BaseAddress = 0xfc00;

RBAPI(void) spi_SetBaseAddress(unsigned baseaddr) {
    // note that memory-space base address is not supported
    write_nb_reg(NB_SPI_BASEADDR_REG, (unsigned long)(baseaddr & 0xfff0) + 1L);
    BaseAddress = baseaddr;
}

RBAPI(unsigned) spi_SetDefaultBaseAddress(void) {
    unsigned long spi_nbreg = read_nb_reg(NB_SPI_BASEADDR_REG);
    
    if ((spi_nbreg & 3L) == 1L)
    {
        BaseAddress = (unsigned)(spi_nbreg & 0xfff0L);
        return BaseAddress;
    }

    return 0xffff;  // the default setting is not supported
}



#define SPI_CTRLREG     (BaseAddress + 0x0a)
/****************  External SPI Control Register: 0x0a  **************
 *    bit 7-5:   reserved
 *    bit 4:     =1 enable FIFO mode
 *    bit 3-0:   SPICLK_DIV (must != 0)
 *               SPI clock rate = DRAM clock rate / (2 * SPICLK_DIV)
 *********************************************************************/

RBAPI(bool) spi_SetControlREG(unsigned char ctrlreg) {
    unsigned long timeout;

    if (spi_Busy() == false)  // tricks for performance; timer_nowtime() is very time-consuming in some OS
    {
        io_outpb(SPI_CTRLREG, ctrlreg);
        return true;
    }

    timeout = timer_nowtime();
    while ((spi_Busy() == true) && ((timer_nowtime() - timeout) <= SPI_TIMEOUT));

    if (spi_Busy() == false)
    {
        io_outpb(SPI_CTRLREG, ctrlreg);
        return true;
    }
    else
    {
        err_SetMsg(ERROR_SPIBUSY, "fail to write SPI Control Register because SPI Controller is busy");
        return false;
    }
}

RBAPI(unsigned char) spi_ReadControlREG(void) {
    return io_inpb(SPI_CTRLREG);
}

RBAPI(bool) spi_EnableFIFO(void) {
    return spi_SetControlREG(spi_ReadControlREG() | 0x10);
}

RBAPI(bool) spi_DisableFIFO(void) {
    return spi_SetControlREG(spi_ReadControlREG() & 0xef);
}

RBAPI(bool) spi_SetClockDivisor(unsigned char clkdiv) {
    if ((clkdiv < 1) || (clkdiv > 15))
    {
        err_SetMsg(ERROR_SPIWRONGCLOCK, "invalid clock divisor");
        return false;
    }

    return spi_SetControlREG((spi_ReadControlREG() & 0xf0) + clkdiv);
}



#define SPI_STATREG     (BaseAddress + 0x0b)
/******************  External SPI Status Register: 0x0b  **************
 *    bit 7:     =1 --> SPI controller busy
 *    bit 6:     =1 --> FIFO full (16 bytes)
 *    bit 5:     =1 --> input data has been ready
 *    bit 4:     =1 --> output has completed (FIFO empty)
 *    bit 3-0:   reserved
 *********************************************************************/

RBAPI(unsigned char) spi_ReadStatusREG(void) {
    return io_inpb(SPI_STATREG);
}

RBAPI(bool) spi_InputReady(void) {
    if ((io_inpb(SPI_STATREG) & 0x20) != 0) return true; else return false;
}
RBAPI(bool) spi_OutputFIFOEmpty(void) {
    if ((io_inpb(SPI_STATREG) & 0x10) != 0) return true; else return false;
}
RBAPI(bool) spi_OutputFIFOFull(void) {
    if ((io_inpb(SPI_STATREG) & 0x40) != 0) return true; else return false;
}
RBAPI(bool) spi_Busy(void) {
    if ((io_inpb(SPI_STATREG) & 0x80) != 0) return true; else return false;
}

#define SPI_CSREG       (BaseAddress + 0x0c)
/********************  External SPI CS Register: 0x0b  ***************
 *    bit 7-1:   reserved
 *    bit 0:     SPI CS
 *********************************************************************/

static void spi_SetCSREG(unsigned char csreg) {
    io_outpb(SPI_CSREG, csreg);
}

RBAPI(unsigned char) spi_ReadCSREG(void) {
    return io_inpb(SPI_CSREG);
}

void spidx_DisableCS(void) {
    spi_SetCSREG(0x01);
}

// note: when enabling CS, SPI controller will always be busy
void spidx_EnableCS(void) {
    spi_SetCSREG(0x00);
}



#define SPI_ERRORREG    (BaseAddress + 0x0d)
/******************  External SPI Error Register: 0x0b  **************
 *    bit 7-5:   reserved
 *    bit 4:     =1 --> write SPI_CTRLREG when SPI controller is busy
 *    bit 3:     =1 --> unread input data is overwrited by new input
 *    bit 2:     =1 --> FIFO underflow (input is read before ready)
 *    bit 1:     =1 --> FIFO overflow  (FIFO is written when full)
 *    bit 0:     reserved
 *    Note: each bit must be written with 1 to clear
 *********************************************************************/

RBAPI(void) spi_SetErrorREG(unsigned char errreg) {
    io_outpb(SPI_ERRORREG, errreg);
}

RBAPI(unsigned char) spi_ReadErrorREG(void) {
    return io_inpb(SPI_ERRORREG);
}

RBAPI(bool) spierror_SPIBusy(void) {
    if ((spi_ReadErrorREG() & 0x10) != 0) return true; else return false;
}
RBAPI(bool) spierror_InputOverlap(void) {
    if ((spi_ReadErrorREG() & 0x08) != 0) return true; else return false;
}
RBAPI(bool) spierror_FIFOUnderrun(void) {
    if ((spi_ReadErrorREG() & 0x04) != 0) return true; else return false;
}
RBAPI(bool) spierror_FIFOOverrun(void) {
    if ((spi_ReadErrorREG() & 0x02) != 0) return true; else return false;
}


RBAPI(void) spi_ClearErrors(void) {
    spi_SetErrorREG(0x1e);
}

RBAPI(void) spierror_ClearSPIBusy(void) {
    spi_SetErrorREG(0x10);
}
RBAPI(void) spierror_ClearInputOverlap(void) {
    spi_SetErrorREG(0x08);
}
RBAPI(void) spierror_ClearFIFOUnderrun(void) {
    spi_SetErrorREG(0x04);
}
RBAPI(void) spierror_ClearFIFOOverrun(void) {
    spi_SetErrorREG(0x02);
}



#define SPI_OUTREG      (BaseAddress + 0x08)

RBAPI(bool) spidx_Write(unsigned char val) {
    unsigned long timeout;

    if (spi_OutputFIFOFull() == false)  // tricks for performance; timer_nowtime() is very time-consuming in some OS
    {
        io_outpb(SPI_OUTREG, val);
        return true;
    }

    timeout = timer_nowtime();
    while ((spi_OutputFIFOFull() == true) && ((timer_nowtime() - timeout) <= SPI_TIMEOUT));

    if (spi_OutputFIFOFull() == false)
    {
        io_outpb(SPI_OUTREG, val);
        return true;
    }
    else
    {
        err_SetMsg(ERROR_SPIFIFOFULL, "fail to write SPI because SPI output-FIFO remains to be full");
        return false;
    }
}

RBAPI(bool) spidx_WriteFlush(unsigned char val) {
    if (spidx_Write(val)  == false)  return false;
    if (spidx_FIFOFlush() == false)  return false;
    return true;
}

RBAPI(bool) spidx_FIFOFlush(void) {
    unsigned long timeout;

    if (spi_OutputFIFOEmpty() == true)  // tricks for performance; timer_nowtime() is very time-consuming in some OS
        return true;

    timeout = timer_nowtime();
    while ((spi_OutputFIFOEmpty() == false) && ((timer_nowtime() - timeout) <= SPI_TIMEOUT));

    if (spi_OutputFIFOEmpty() == true)
        return true;
    else
    {
        err_SetMsg(ERROR_SPIFIFOFAIL, "fail to clear SPI output-FIFO");
        return false;
    }
}



#define SPI_INREG       (BaseAddress + 0x09)

RBAPI(unsigned) spidx_Read(void) {
    // avoid read-after-write timing issue
    if (spi_OutputFIFOEmpty() == false) //&&
    if (spidx_FIFOFlush() == false)
        return false;
    
    return spidx_Read2();
}

RBAPI(unsigned) spidx_Read2(void) {
    unsigned long timeout, i;
    
    if (spi_InputReady() == false)    // no data is in the input buffer
    {
        io_outpb(SPI_INREG, 0x00);    // write SPI_INREG to triggle SPI reading function
        
        for (i=0; i<100000L; i++)     // tricks for performance; timer_nowtime() is very time-consuming in some OS
            if (spi_InputReady() == true) return (unsigned)io_inpb(SPI_INREG);

        timeout = timer_nowtime();
        while ((spi_InputReady() == false) && ((timer_nowtime() - timeout) <= SPI_TIMEOUT));
    }
    
    if (spi_InputReady() == true)
        return (unsigned)io_inpb(SPI_INREG);
    else
    {
        err_SetMsg(ERROR_SPIREADFAIL, "fail to read SPI input because waiting no data");
        return 0xffff;
    }
}

RBAPI(unsigned) spidx_ReadStrict(void) {
	int i;

	for (i=0; i<20; i++)  // in fact, Vortex86DX's SPI H/W has no input FIFO, but I am lazy to change this code:p
	{
		if (spi_InputReady() == false) break;
		if (spidx_Read() == 0xffff) return 0xffff;
	}

    if (spi_InputReady() == true)
	{
		err_SetMsg(ERROR_SPIREADFAIL, "can't clear overdue data in SPI input buffer");
		return 0xffff;
	}

	return spidx_Read();
}

