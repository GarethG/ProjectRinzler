#define __SPI_LIB

//#include <math.h>

#define  USE_COMMON
#include "common.h"
#include "io.h"
#include "spi.h"
void spidx_EnableCS(void);
void spidx_DisableCS(void);



static bool SPISW_active = false;

#define GPIO3_DATA   (0x7b)
#define GPIO3_DIR    (0x9b)



/***************************  SPI Hardware Function  **************************/
static unsigned char SPI_CLKMODE[15] =
    {
        7,  // 21.4Mbps; assume RoBoard's DRAM clock is 300MHZ
        8,  // 18.75Mbps
        10, // 15Mbps
        12, // 12.5Mbps
        15, // 10Mbps
        14, // 10.7Mbps
        13, // 11.5Mbps
        11, // 13.6Mbps
        9,  // 16.6Mbps
        6,  // 25Mbps
        5,  // 30Mbps
        4,  // 37Mbps
        3,  // 50Mbps
        2,  // 75Mbps
        1   // 150Mbps
    };


static int SPI_ioSection = -1;

RBAPI(bool) spi_InUse(void) {
    if (SPI_ioSection == -1) return false; else return true;
}


#define SB_MULTIFUNC_REG      (0xc0)
/************** South Bridge Multifunction Pin Control REG: 0xC0~0xC3 ************
 *    To use SPI interface, the user should set PINS0 (bit 0 of SB_MULTIFUNC_REG)
 *    to 1. (PINS0 = 1: SPI; PINS0 = 0: GPIO3[3:0])
 *********************************************************************************/
static unsigned long OLD_SPIGPIO3FLAG = 0xffffffffL;

RBAPI(bool) spi_Init2(unsigned baseaddr, int clkmode) {
    int i;

	if(spi_InUse() == true)
	{
		err_SetMsg(ERROR_SPI_INUSE, "SPI was already opened");
		return false;
	}
	
	if ((SPI_ioSection = io_Init()) == -1) return false;

	//NOTE: base address should be selected carefully to avoid conflicts with other devices!
	if (baseaddr != 0xffff)
        spi_SetBaseAddress(baseaddr);
	else
	{
        baseaddr = spi_SetDefaultBaseAddress();
        if ((baseaddr == 0x0000) || (baseaddr == 0xffff)) spi_SetBaseAddress(0xfc00);
    }

    spidx_DisableCS();
    spi_ClearErrors();

    // enable FIFO and set clock divisor
    if (spi_SetControlREG(0x10 + SPI_CLKMODE[clkmode]) == false)
    {
        spi_Close();
        err_SetMsg(ERROR_SPI_INITFAIL, "fail to write SPI Control Register");
        return false;
    }
    
    // clear the input buffer if it is not empty
    for (i=0; i<20; i++)
    {
        if (spi_InputReady() == false) break;
        if (spidx_Read() == 0xffff)
        {
            spi_Close();
            err_SetMsg(ERROR_SPI_INITFAIL, "fail to clear SPI input buffer");
            return false;
        }
    }

    // switch GPIO3[3:0] to external SPI interface
    OLD_SPIGPIO3FLAG = read_sb_reg(SB_MULTIFUNC_REG);
    write_sb_reg(SB_MULTIFUNC_REG, OLD_SPIGPIO3FLAG | 1L);
    OLD_SPIGPIO3FLAG = OLD_SPIGPIO3FLAG & 1L;

    return true;
}

RBAPI(void) spi_Close(void) {
    if (SPISW_active == true) return;

	if (SPI_ioSection == -1) return;

    spidx_DisableCS();
    spi_ClearErrors();

    // restore GPIO3[3:0]/external SPI switch setting
    if (OLD_SPIGPIO3FLAG != 0xffffffffL)
        write_sb_reg(SB_MULTIFUNC_REG, (read_sb_reg(SB_MULTIFUNC_REG) & 0xfffffffeL) + OLD_SPIGPIO3FLAG);

	io_Close(SPI_ioSection);
	SPI_ioSection = -1;
	OLD_SPIGPIO3FLAG = 0xffffffffL;
}
/*---------------------  end of SPI Hardware Function  -----------------------*/



/**********************  SPI Software-Simulated Function  *********************/

// GPIO3[3:0] will be used to simulate SPI port for SPI S/W mode
// (GPIO30 = SPI_CS, GPIO31 = SPI_CPOL, GPIO32 = SPI_MOSI, GPIO33 = SPI_MISO)

static unsigned char OLD_SPIGPIODIR;
static unsigned char OLD_SPIGPIODATA;

static unsigned long SPISW_delay = 0L;
static int           SPISW_mode  = SPIMODE_CPOL0 + SPIMODE_CPHA1;

RBAPI(bool) spi_InitSW(int mode, unsigned long clkdelay) {
	if(spi_InUse() == true)
	{
		err_SetMsg(ERROR_SPI_INUSE, "SPI was already opened");
		return false;
	}
	
	if ((SPI_ioSection = io_Init()) == -1) return false;

	SPISW_mode  = mode;
	SPISW_delay = clkdelay;


    // set the initial state of SPI GPIO pins
	OLD_SPIGPIODIR  = io_inpb(GPIO3_DIR);
	OLD_SPIGPIODATA = io_inpb(GPIO3_DATA);

	io_outpb(GPIO3_DIR,  (OLD_SPIGPIODIR & 0xf0) | 0x07);
	if ((SPISW_mode & SPIMODE_CPOL1) != 0)
	   io_outpb(GPIO3_DATA, (OLD_SPIGPIODATA & 0xf0) + 0x03);  // SPI_CLK = 1, SPI_CS = 1
	else
	   io_outpb(GPIO3_DATA, (OLD_SPIGPIODATA & 0xf0) + 0x01);  // SPI_CLK = 0, SPI_CS = 1

	OLD_SPIGPIODIR  = OLD_SPIGPIODIR  & 0x0f;
	OLD_SPIGPIODATA = OLD_SPIGPIODATA & 0x0f;


	// switch to GPIO interface
	OLD_SPIGPIO3FLAG = read_sb_reg(SB_MULTIFUNC_REG);
    write_sb_reg(SB_MULTIFUNC_REG, OLD_SPIGPIO3FLAG & 0xfffffffeL);
    OLD_SPIGPIO3FLAG = OLD_SPIGPIO3FLAG & 1L;

	SPISW_active = true;

	return true;
}

RBAPI(void) spi_CloseSW(void) {
	if (spi_InUse() == false) return;
	if (SPISW_active == false) return;

    if (OLD_SPIGPIO3FLAG != 0xffffffffL)
        write_sb_reg(SB_MULTIFUNC_REG, (read_sb_reg(SB_MULTIFUNC_REG) & 0xfffffffeL) + OLD_SPIGPIO3FLAG);

	// restore GPIO3[3:0]
    io_outpb(GPIO3_DIR,  (io_inpb(GPIO3_DIR)  & 0xf0) + OLD_SPIGPIODIR);
    io_outpb(GPIO3_DATA, (io_inpb(GPIO3_DATA) & 0xf0) + OLD_SPIGPIODATA);

    io_Close(SPI_ioSection);
    SPI_ioSection = -1;

	OLD_SPIGPIO3FLAG = 0xffffffffL;
	SPISW_active = false;
}



// transmit or receive one bit for SPI software simulatived mode
_RB_INLINE void sw_dataout(unsigned char val) {
	io_outpb(GPIO3_DATA, (io_inpb(GPIO3_DATA) & 0xf1) + val);
}

_RB_INLINE unsigned sw_datain(void) {
	return (((unsigned)io_inpb(GPIO3_DATA) >> 3) & 0x01);
}

//use GPIO3[3:0] to simulate SPI full-duplex interface for SPI S/W mode
static unsigned spisw_Exchange(unsigned char val, int mode, unsigned long delay) {
	unsigned char clk, _clk, mosi;
	unsigned data;
	int i;

	data = 0;
	clk = _clk = 0x00;
	if((mode & SPIMODE_CPOL1) != 0) clk = 0x02; else _clk = 0x02;

	switch (mode)
	{
		case (SPIMODE_CPOL0 + SPIMODE_SMOD0 + SPIMODE_CPHA0):
		case (SPIMODE_CPOL1 + SPIMODE_SMOD0 + SPIMODE_CPHA0):

			for(i=0; i<8; i++)
			{
				if((val & (0x80 >> i)) > 0) mosi = 0x04; else mosi = 0x00;
				sw_dataout(clk + mosi);
				delay_us(delay);
				data = (data << 1) + sw_datain();
				sw_dataout(_clk + mosi);
				delay_us(delay);
			}
			break;

		case (SPIMODE_CPOL0 + SPIMODE_SMOD0 + SPIMODE_CPHA1):
		case (SPIMODE_CPOL1 + SPIMODE_SMOD0 + SPIMODE_CPHA1):

			for(i=0; i<8; i++)
			{
				delay_us(delay);
				if((val & (0x80 >> i)) > 0) mosi = 0x04; else mosi = 0x00;
				sw_dataout(_clk + mosi);
				delay_us(delay);
				data = (data << 1) + sw_datain();
				sw_dataout(clk + mosi);
			}
			break;

		case (SPIMODE_CPOL0 + SPIMODE_SMOD1 + SPIMODE_CPHA0):
		case (SPIMODE_CPOL1 + SPIMODE_SMOD1 + SPIMODE_CPHA0):

			for(i=0; i<8; i++)
			{
				delay_us(delay);
				data = (data << 1) + sw_datain();
				if((val & (0x80 >> i)) > 0) mosi = 0x04; else mosi = 0x00;
				sw_dataout(_clk + mosi);
				delay_us(delay);
				sw_dataout(clk + mosi);
			}
			break;

		case (SPIMODE_CPOL0 + SPIMODE_SMOD1 + SPIMODE_CPHA1):
		case (SPIMODE_CPOL1 + SPIMODE_SMOD1 + SPIMODE_CPHA1):

			for(i=0; i<8; i++)
			{
				if((val & (0x80 >> i)) > 0) mosi = 0x04; else mosi = 0x00;
				sw_dataout(clk + mosi);
				delay_us(delay);
				sw_dataout(_clk + mosi);
				delay_us(delay);
				data = (data << 1) + sw_datain();
			}
			break;

		default:
            err_SetMsg(ERROR_SPISW_INVALIDMODE, "invalid SPI clock mode");
			return 0xffff;
	}

	return data;
}
/*-----------------  end of SPI Software-Simulated Function  -----------------*/



/***************************  SPI Common Functions  ***************************/
void spi_DisableCS(void) {
	if(SPISW_active == true)
        io_outpb(GPIO3_DATA,(io_inpb(GPIO3_DATA) | 0x01));  // assume the direction of the GPIO pin has been set correctly
    else
        spidx_DisableCS();
}

void spi_EnableCS(void) {
	if(SPISW_active == true)
		io_outpb(GPIO3_DATA,(io_inpb(GPIO3_DATA) & 0xfe));  // assume the direction of the GPIO pin has been set correctly
	else
		spidx_EnableCS();
}


RBAPI(unsigned) spi_Exchange(unsigned char val) {
	if (SPISW_active == true)
        return spisw_Exchange(val, SPISW_mode, SPISW_delay);

	spidx_Write(val);
	return 0xff;
}

RBAPI(bool) spi_Write(unsigned char val) {
	if (SPISW_active == false) return spidx_Write(val);

	spisw_Exchange(val, SPISW_mode, SPISW_delay);
	return true;
}

RBAPI(bool) spi_WriteFlush(unsigned char val) {
	if(SPISW_active == false) return spidx_WriteFlush(val);
	
	spisw_Exchange(val, SPISW_mode, SPISW_delay);
	return true;
}

RBAPI(bool) spi_FIFOFlush(void) {
	if(SPISW_active == false) return spidx_FIFOFlush();

	return true;
}

RBAPI(unsigned) spi_Read(void) {
	if(SPISW_active == false) return spidx_Read();

	return spisw_Exchange(0x00, SPISW_mode, SPISW_delay);
}

RBAPI(unsigned) spi_Read2(void) {
	if(SPISW_active == false) return spidx_Read2();

	return spisw_Exchange(0x00, SPISW_mode, SPISW_delay);
}

RBAPI(unsigned) spi_ReadStrict(void) {
	if(SPISW_active == false) return spi_ReadStrict();

	return spisw_Exchange(0x00, SPISW_mode, SPISW_delay);
}
/*----------------------  end of SPI Common Functions  -----------------------*/



//use GPIO port 3 pin 7 to simulate SPI_SS (on RoBoard RB-100)
RBAPI(bool) spi_EnableSS(void) {
    if ((roboio_GetRBVer() != RB_100) && (roboio_GetRBVer() != RB_100b1) && (roboio_GetRBVer() != RB_100b2))
    {
        err_SetMsg(ERROR_SPI_UNSUPPORTED, "this version of RoBoard is unsupported");
        return false;
    }

    // avoid enable-after-write timing issue
    if (SPISW_active == false) //&&
    if (spi_OutputFIFOEmpty() == false) //&&
    if (spi_FIFOFlush() == false)
        return false;

	io_outpb(GPIO3_DIR, io_inpb(GPIO3_DIR) | 0x80);
    io_outpb(GPIO3_DATA, io_inpb(GPIO3_DATA) & 0x7f);
    return true;
}

RBAPI(bool) spi_DisableSS(void) {
    if ((roboio_GetRBVer() != RB_100) && (roboio_GetRBVer() != RB_100b1) && (roboio_GetRBVer() != RB_100b2))
    {
        err_SetMsg(ERROR_SPI_UNSUPPORTED, "this version of RoBoard is unsupported");
        return false;
    }

    // avoid disable-after-write timing issue
    if (SPISW_active == false) //&&
    if (spi_OutputFIFOEmpty() == false) //&&
    if (spi_FIFOFlush() == false)
        return false;

	io_outpb(GPIO3_DIR, io_inpb(GPIO3_DIR) | 0x80);
    io_outpb(GPIO3_DATA, io_inpb(GPIO3_DATA) | 0x80);
    return true;
}


/*
//set SPI CLK to spiclk MHZ; return false if spiclk given by the user is not supported
double ROBOARD_DRAMClock = 300.0; //300MHZ
bool spi_SetClock(double spiclk) {
    unsigned char clkdiv;

    clkdiv = (unsigned char) ceil(ROBOARD_DRAMClock / (2.0 * spiclk));
	if (clkdiv > 1) //&&
	if ((ROBOARD_DRAMClock / (2.0 * clkdiv) - spiclk) > (spiclk - ROBOARD_DRAMClock / (2.0 * (clkdiv - 1))))
		clkdiv = clkdiv - 1;

	return spi_SetClockDivisor(clkdiv);
}
*/

