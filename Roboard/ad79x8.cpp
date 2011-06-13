#define __AD79X8_LIB

#include "defines.h"

#define  USE_COMMON
#include "common.h"
#include "io.h"
#include "spi.h"
void spi_EnableCS(void);
void spi_DisableCS(void);
#include "ad79x8.h"


/***********************  PM (ADC operation mode)  **********************
 *    00:   invalid
 *    01:   auto shutdown; in this mode, the ADC automatically enters
 *          full shutdown mode at the end of each conversion (note that
 *          wake-up from full shutdown needs 1 £gs)
 *    10:   full shutdown; in this mode, the ADC is in full shutdown mode
 *          with all circuitry powering down
 *    11:   normal operation; in this mode, the ADC remain in full power
 *          mode which allows the fastest possible throughput rate
 ************************************************************************/
#define ad79x8_SETPM(ctrlreg, val)       ctrlreg = (ctrlreg & (~(0x0003 << 8))) | (val << 8)


/****************  SEQ, SHADOW (sequencer operation mode)  **************
 *    00:   the sequence function is not used
 *    01:   selects the SHADOW register for programming; the following
 *          CTRLREG write will decide the contents of the SHADOW register
 *          which determines the sequence of channels to be converted on
 *          continuously
 *    10:   update CTRLREG in sequencing mode
 *    11:   continuous conversions from Channel 0 to Channel ADD2...ADD0
 ************************************************************************/
#define ad79x8_SEQENABLE(ctrlreg)        ctrlreg |=  (0x0001 << 14)
#define ad79x8_SEQDISABLE(ctrlreg)       ctrlreg &= ~(0x0001 << 14)
#define ad79x8_SWENABLE(ctrlreg)         ctrlreg |=  (0x0001 << 7)
#define ad79x8_SWDISABLE(ctrlreg)        ctrlreg &= ~(0x0001 << 7)


//set ADD2...ADD0 = val
#define ad79x8_SETCHANNEL(ctrlreg, val)  ctrlreg = (ctrlreg & (~(0x0007 << 10))) | (val << 10)

#define ad79x8_SETRANGE_VREF(ctrlreg)    ctrlreg |=  (0x0001 << 5) //set AD range: 0~Vref
#define ad79x8_SETRANGE_2VREF(ctrlreg)   ctrlreg &= ~(0x0001 << 5) //set AD range: 0~2Vref

#define ad79x8_SETCODING_SYM(ctrlreg)    ctrlreg &= ~(0x0001 << 4) //set AD coding: -128/-512/-2048~127/511/2047
#define ad79x8_SETCODING_ASYM(ctrlreg)   ctrlreg |= (0x0001 << 4)  //set AD coding: 0~255/1023/4095

#define ad79x8_WRITEENABLE(ctrlreg)      ctrlreg |=  (0x0001 << 15)
#define ad79x8_WRITEDISABLE(ctrlreg)     ctrlreg &= ~(0x0001 << 15)



#define GPIO2_DATA   (0x7a)
#define GPIO2_DIR    (0x9a)
RBAPI(bool) ad79x8_RawWrite(unsigned val) {
    bool b1, b2;

    #ifdef ROBOIO
        if (roboio_GetRBVer() == RB_110)
        {
            io_outpb(GPIO2_DIR, io_inpb(GPIO2_DIR) | 0x20);
            io_outpb(GPIO2_DATA, io_inpb(GPIO2_DATA) & 0xdf);
        }
    #endif
    spi_EnableCS();

    b1 = spi_Write(val >> 8);
    b2 = spi_WriteFlush(val & 0x00ff);

    spi_DisableCS();
    #ifdef ROBOIO
        if (roboio_GetRBVer() == RB_110)
        {
            io_outpb(GPIO2_DIR, io_inpb(GPIO2_DIR) | 0x20);
            io_outpb(GPIO2_DATA, io_inpb(GPIO2_DATA) | 0x20);
        }
    #endif

    if ((b1 == false) || (b2 == false))
    {
        err_SetMsg(ERROR_ADCFAIL, "fail to write ADC because SPI fails");
        return false;
    }

    return true;
}

RBAPI(unsigned) ad79x8_RawRead(void) {
    unsigned val1, val2;

    #ifdef ROBOIO
        if (roboio_GetRBVer() == RB_110)
        {
            io_outpb(GPIO2_DIR, io_inpb(GPIO2_DIR) | 0x20);
            io_outpb(GPIO2_DATA, io_inpb(GPIO2_DATA) & 0xdf);
        }
    #endif
    spi_EnableCS();

    val1 = spi_Read2();
    val2 = spi_Read2();

    spi_DisableCS();
    #ifdef ROBOIO
        if (roboio_GetRBVer() == RB_110)
        {
            io_outpb(GPIO2_DIR, io_inpb(GPIO2_DIR) | 0x20);
            io_outpb(GPIO2_DATA, io_inpb(GPIO2_DATA) | 0x20);
        }
    #endif

    if ((val1 == 0xffff) || (val2 == 0xffff))
    {
        err_SetMsg(ERROR_ADCFAIL, "fail to read ADC because SPI fails");
        return AD79x8_READFAIL;
    }

    return (val1 << 8) + val2;
}

RBAPI(bool) ad79x8_WriteCTRLREG(unsigned ctrlreg) {
    if (ad79x8_RawWrite(ctrlreg) == true)
        return ad79x8_RawWrite(0x0000); //dummy write to avoid DX's SPI bug

    return false;
}



static bool AD79x8_inUse = false;
RBAPI(bool) ad79x8_InUse(void) {
    return AD79x8_inUse;
}

RBAPI(bool) ad79x8_Initialize(int channel, int range, int coding) {
	if (spi_InUse() == false)
	{
		err_SetMsg(ERROR_ADC_NOSPI, "detect no SPI lib");
		return false;
	}

	if (AD79x8_inUse == true)
	{
		err_SetMsg(ERROR_ADC_INUSE, "batch mode has been opened");
		return false;
	}

	if (ad79x8_ChangeChannel(channel, range, coding) == false) return false;

    AD79x8_inUse = true;
    return true;
}

RBAPI(bool) ad79x8_InitializeMCH(unsigned char usedchannels, int range, int coding) {
	if (spi_InUse() == false)
	{
		err_SetMsg(ERROR_ADC_NOSPI, "detect no SPI lib");
		return false;
	}

	if (AD79x8_inUse == true)
	{
		err_SetMsg(ERROR_ADC_INUSE, "batch mode has been opened");
		return false;
	}
	
	if (ad79x8_ChangeChannels(usedchannels, range, coding) == false) return false;

    AD79x8_inUse = true;
    return true;
}

RBAPI(bool) ad79x8_Close(void) { //will shutdown the A/D (force it to enter power-off mode)
	unsigned AD79x8_CTRLREG = 0x8300;

	if (AD79x8_inUse == false) return true;

    AD79x8_inUse = false;

	ad79x8_SETRANGE_VREF(AD79x8_CTRLREG);
	ad79x8_SETCODING_SYM(AD79x8_CTRLREG);
	ad79x8_SETPM(AD79x8_CTRLREG, 2);      //full-shutdown power mode
	return ad79x8_RawWrite(AD79x8_CTRLREG);
}


static int AD79x8_currentCoding = AD79x8MODE_CODING_UNSIGNED;
RBAPI(bool) ad79x8_ChangeChannel(int channel, int range, int coding) {
	unsigned AD79x8_CTRLREG = 0x8300;
    //ad79x8_WRITEENABLE(AD7908_CTRLREG);
    //ad79x8_SETPM(AD7908_CTRLREG, 3);      //normal power mode
    //ad79x8_SEQDISABLE(AD7908_CTRLREG);    //no sequence mode
    //ad79x8_SWDISABLE(AD7908_CTRLREG);

	if ((channel >= 0) || (channel <= 7))
    {
        ad79x8_SETCHANNEL(AD79x8_CTRLREG, channel);
        if (range  == AD79x8MODE_RANGE_VREF)    ad79x8_SETRANGE_VREF(AD79x8_CTRLREG); else ad79x8_SETRANGE_2VREF(AD79x8_CTRLREG);
        if (coding == AD79x8MODE_CODING_SIGNED) ad79x8_SETCODING_SYM(AD79x8_CTRLREG); else ad79x8_SETCODING_ASYM(AD79x8_CTRLREG);
    
        if (ad79x8_WriteCTRLREG(AD79x8_CTRLREG) == false) return false;
        AD79x8_currentCoding = coding;
        return true;
    }
    else
    {
        err_SetMsg(ERROR_ADC_WRONGCHANNEL, "invalid ADC channel");
        return false;
    }
}


static unsigned char AD79x8_usedChannels = 0;
static int           AD79x8_numChannels = 0;
RBAPI(bool) ad79x8_ChangeChannels(unsigned char usedchannels, int range, int coding) {
	unsigned AD79x8_CTRLREG = 0x8300; //SEQ = 0, SHADOW = 0
	int i;

    if (range  == AD79x8MODE_RANGE_VREF)    ad79x8_SETRANGE_VREF(AD79x8_CTRLREG); else ad79x8_SETRANGE_2VREF(AD79x8_CTRLREG);
    if (coding == AD79x8MODE_CODING_SIGNED) ad79x8_SETCODING_SYM(AD79x8_CTRLREG); else ad79x8_SETCODING_ASYM(AD79x8_CTRLREG);

	if ((AD79x8_inUse == true) && (usedchannels == AD79x8_usedChannels))
	{
		if (ad79x8_WriteCTRLREG(AD79x8_CTRLREG | 0x4000) == false) return false; //SEQ = 1, SHADOW = 0
	}
	else
	{
		if (ad79x8_RawWrite(AD79x8_CTRLREG | 0x0080) == false) return false; //SEQ = 0, SHADOW = 1 for setting SHADOW REG

		AD79x8_usedChannels = usedchannels;
        AD79x8_numChannels  = 0;
		for (i=0; i<8; i++) if ((AD79x8_usedChannels & (1<<i)) != 0) AD79x8_numChannels++;

		if (ad79x8_WriteCTRLREG(((unsigned)AD79x8_usedChannels) << 8) == false) return false; //write to SHADOW REG
	}
	
	AD79x8_currentCoding = coding;
	return true;
}



/*******************************  AD7908  *********************************/
RBAPI(int) ad7908_Read(void) {
    unsigned val;

    if ((val = ad79x8_RawRead()) != AD79x8_READFAIL)
	{
		val = (val >> 4) & 0x00ff;
		if (AD79x8_currentCoding == AD79x8MODE_CODING_UNSIGNED) return (int)val;
		
		return ((val & 0x80) != 0)? (int)val - 256 : (int)val;
	}

    return AD7908_READFAIL;
}

static int AD79x8_readBuffer[8];
RBAPI(int*) ad7908_ReadMCH(void) {
	int i, channel;
	unsigned val;

	for (i=0; i<8; i++) AD79x8_readBuffer[i] = AD7908_READFAIL;

	for (i=0; i<AD79x8_numChannels; i++)
	{
		if ((val = ad79x8_RawRead()) == AD79x8_READFAIL) continue;
		
        channel = (int)((val >> 12) & 0x07);
        val     = (val >> 4) & 0xff;
        
		if (AD79x8_currentCoding == AD79x8MODE_CODING_UNSIGNED)
            AD79x8_readBuffer[channel] = (int)val;
        else
            AD79x8_readBuffer[channel] = ((val & 0x80) != 0)? (int)val - 256 : (int)val;
	}

	return AD79x8_readBuffer;
}

RBAPI(void) ad7908_ReadMCH2(int* buf) {  // for usage in VB6 & Labview
    int* tmp = ad7908_ReadMCH();
    int  i;

    for (i=0; i<8; i++) buf[i] = tmp[i];
}

RBAPI(int) ad7908_ReadChannel(int channel, int range, int coding) {
	if (spi_InUse() == false)
	{
		err_SetMsg(ERROR_ADC_NOSPI, "detect no SPI lib");
		return AD7908_READFAIL;
	}

	if (ad79x8_ChangeChannel(channel, range, coding) == false) return AD7908_READFAIL;
	return ad7908_Read();
}
/*---------------------------  end of AD7908  ----------------------------*/



/*******************************  AD7918  *********************************/
RBAPI(int) ad7918_Read(void) {
    unsigned val;

    if ((val = ad79x8_RawRead()) != AD79x8_READFAIL)
	{
		val = (val >> 2) & 0x03ff;
		if (AD79x8_currentCoding == AD79x8MODE_CODING_UNSIGNED) return (int)val;
		
		return ((val & 0x200) != 0)? (int)val - 1024 : (int)val;
	}

    return AD7908_READFAIL;
}

RBAPI(int*) ad7918_ReadMCH(void) {
	int i, channel;
	unsigned val;

	for (i=0; i<8; i++) AD79x8_readBuffer[i] = AD7918_READFAIL;

	for (i=0; i<AD79x8_numChannels; i++)
	{
		if ((val = ad79x8_RawRead()) == AD79x8_READFAIL) continue;
		
        channel = (int)((val >> 12) & 0x07);
        val     = (val >> 2) & 0x3ff;
        
		if (AD79x8_currentCoding == AD79x8MODE_CODING_UNSIGNED)
            AD79x8_readBuffer[channel] = (int)val;
        else
            AD79x8_readBuffer[channel] = ((val & 0x200) != 0)? (int)val - 1024 : (int)val;
	}
	return AD79x8_readBuffer;
}

RBAPI(void) ad7918_ReadMCH2(int* buf) {  // for usage in VB6 & Labview
    int* tmp = ad7918_ReadMCH();
    int  i;

    for (i=0; i<8; i++) buf[i] = tmp[i];
}

RBAPI(int) ad7918_ReadChannel(int channel, int range, int coding) {
	if (spi_InUse() == false)
	{
		err_SetMsg(ERROR_ADC_NOSPI, "detect no SPI lib");
		return AD7918_READFAIL;
	}

	if (ad7918_ChangeChannel(channel, range, coding) == false) return AD7918_READFAIL;
	return ad7918_Read();
}
/*---------------------------  end of AD7918  ----------------------------*/



/*******************************  AD7928  *********************************/
RBAPI(int) ad7928_Read(void) {
    unsigned val;

    if ((val = ad79x8_RawRead()) != AD79x8_READFAIL)
	{
		val = val & 0xfff;
		if (AD79x8_currentCoding == AD79x8MODE_CODING_UNSIGNED) return (int)val;
		
		return ((val & 0x800) != 0)? (int)val - 4096 : (int)val;
	}

    return AD7908_READFAIL;
}

RBAPI(int*) ad7928_ReadMCH(void) {
	int i, channel;
	unsigned val;

	for (i=0; i<8; i++) AD79x8_readBuffer[i] = AD7928_READFAIL;

	for (i=0; i<AD79x8_numChannels; i++)
	{
		if ((val = ad79x8_RawRead()) == AD79x8_READFAIL) continue;
		
        channel = (int)((val >> 12) & 0x07);
        val     = val & 0xfff;
        
		if (AD79x8_currentCoding == AD79x8MODE_CODING_UNSIGNED)
            AD79x8_readBuffer[channel] = (int)val;
        else
            AD79x8_readBuffer[channel] = ((val & 0x800) != 0)? (int)val - 4096 : (int)val;
	}
	return AD79x8_readBuffer;
}

RBAPI(void) ad7928_ReadMCH2(int* buf) {  // for usage in VB6 & Labview
    int* tmp = ad7928_ReadMCH();
    int  i;

    for (i=0; i<8; i++) buf[i] = tmp[i];
}

RBAPI(int) ad7928_ReadChannel(int channel, int range, int coding) {
	if (spi_InUse() == false)
	{
		err_SetMsg(ERROR_ADC_NOSPI, "detect no SPI lib");
		return AD7928_READFAIL;
	}

	if (ad7928_ChangeChannel(channel, range, coding) == false) return AD7928_READFAIL;
	return ad7928_Read();
}
/*---------------------------  end of AD7928  ----------------------------*/



#ifdef ROBOIO
    RBAPI(int) adc_Read(void) {
        switch (roboio_GetRBVer())
        {
        case RB_100b1:
        case RB_100b2:
        case RB_100:
        case RB_110:
        case RB_050:
            return ad7918_Read();
        default:
            return ADC_READFAIL;
        }
    }
    
    RBAPI(int*) adc_ReadMCH(void) {
        int i;
    
        switch (roboio_GetRBVer())
        {
        case RB_100b1:
        case RB_100b2:
        case RB_100:
        case RB_110:
        case RB_050:
            return ad7918_ReadMCH();
        default:
	        for (i=0; i<8; i++) AD79x8_readBuffer[i] = ADC_READFAIL;
            return AD79x8_readBuffer;
        }
    }
    
    RBAPI(void) adc_ReadMCH2(int* buf) {  // for usage in VB6 & Labview
        int* tmp = adc_ReadMCH();
        int  i;
    
        for (i=0; i<8; i++) buf[i] = tmp[i];
    }
    
    RBAPI(int) adc_ReadChannel(int channel, int range, int coding) {
        switch (roboio_GetRBVer())
        {
        case RB_100b1:
        case RB_100b2:
        case RB_100:
        case RB_110:
        case RB_050:
            return ad7918_ReadChannel(channel, range, coding);
        default:
            return ADC_READFAIL;
        }
    }
#endif

