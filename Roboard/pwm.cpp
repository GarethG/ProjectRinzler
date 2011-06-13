#define __PWM_LIB

#include "defines.h"

#define  USE_COMMON
#include "common.h"
#include "io.h"
#include "pwm.h"

#define LONG_LSHIFT(a, b) (((b) >= 32)? 0L : (a) << (b))


#ifdef ROBOIO
    static int PWM_numChannels = 0;
#else
    static int PWM_numChannels = 32;
#endif
RBAPI(int) pwm_NumCh(void) {
    return PWM_numChannels;
}

static int PWM_ioSection = -1;
RBAPI(bool) pwm_InUse(void) {
    if (PWM_ioSection == -1) return false; else return true;
}

RBAPI(bool) pwm_Init2(unsigned baseaddr, int clkmode, int irqmode) {
    int i;
    
	if (PWM_ioSection != -1)
	{
        err_SetMsg(ERROR_PWM_INUSE, "PWM lib was already opened");
		return false;
	}

	if ((PWM_ioSection = io_Init()) == -1) return false;

    #ifdef ROBOIO
        switch (roboio_GetRBVer())
        {
        case RB_100b1:
            PWM_numChannels = 32; break;
        case RB_100b2:
        case RB_100:
            PWM_numChannels = 24; break;
        case RB_110:
        case RB_050:
            PWM_numChannels = 16; break;
        }
    #endif

	//NOTE: base address should be selected carefully to avoid conflicts with other devices!
	if (baseaddr != 0xffff)
	    pwm_SetBaseAddress(baseaddr);
	else
	{
        if (pwm_SetDefaultBaseAddress() == 0x0000) pwm_SetBaseAddress(0xfe00);
    }

	pwm_SetBaseClock(clkmode);
	pwm_SetIRQ(irqmode);

	pwm_DisableMultiPWM(0xffffffffL);
	pwm_DisableMultiINT(0xffffffffL);
	pwm_ClearMultiFLAG(0xffffffffL);

	for (i=0; i<32; i++)
	{
		pwm_SetCountingMode(i, PWM_COUNT_MODE);
		pwm_SetWaveform(i, PWM_WAVEFORM_NORMAL);
		pwm_SetPulse(i, 20000L, 1500L);
		pwm_SetPulseCount(i, 0L);
	}

	pwm_UnlockMulti(0xffffffffL);
	return true;
}

RBAPI(void) pwm_Close(void) {
	if (PWM_ioSection == -1) return;

	pwm_DisableMultiPWM(0xffffffffL);
	pwm_DisableMultiINT(0xffffffffL);
	pwm_ClearMultiFLAG(0xffffffffL);

    #ifdef ROBOIO
        PWM_numChannels = 0;
    #else
        PWM_numChannels = 32;
    #endif

	io_Close(PWM_ioSection);
	PWM_ioSection = -1;
}


static unsigned long us = 10L;
RBAPI(void) pwm_SetBaseClock(int clock) {
    if (clock == PWMCLOCK_10MHZ)
        us = 10L;
    else
        us = 50L;

    pwm_SelectClock(clock);
}


RBAPI(bool) pwm_SetPulse(int channel, unsigned long period, unsigned long duty) {
    if ((channel<0) || (channel>=PWM_numChannels))
    {
        err_SetMsg(ERROR_PWM_WRONGCHANNEL, "channel %d doesn't exist", channel);
        return false;
    }

    if ((duty==0L) || (period<=duty))
    {
        err_SetMsg(ERROR_PWM_INVALIDPULSE, "invalid pulse setting");
        return false;
    }

    pwm_SetHREG(channel, duty*us);
    pwm_SetLREG(channel, (period - duty)*us);
	return true;
}


RBAPI(bool) pwm_SetPulse_10MHZ(int channel, unsigned long period, unsigned long duty) {
    if ((channel<0) || (channel>=PWM_numChannels))
    {
        err_SetMsg(ERROR_PWM_WRONGCHANNEL, "channel %d doesn't exist", channel);
        return false;
    }

    if ((duty==0L) || (period<=duty))
    {
        err_SetMsg(ERROR_PWM_INVALIDPULSE, "invalid pulse setting");
        return false;
    }

    if (us == 50L)
    {
        duty   = duty * 5L;
        period = period * 5L;
    }

	pwm_SetHREG(channel, duty);
	pwm_SetLREG(channel, period - duty);
	return true;
}


RBAPI(bool) pwm_SetPulse_50MHZ(int channel, unsigned long period, unsigned long duty) {
    if ((channel<0) || (channel>=PWM_numChannels))
    {
        err_SetMsg(ERROR_PWM_WRONGCHANNEL, "channel %d doesn't exist", channel);
        return false;
    }

    if ((duty==0L) || (period<=duty))
    {
        err_SetMsg(ERROR_PWM_INVALIDPULSE, "invalid pulse setting");
        return false;
    }

    if (us == 10L)
    {
        err_SetMsg(ERROR_PWM_CLOCKMISMATCH, "PWM clock mode doesn't match (require 50MHZ)");
        return false;
    }

	pwm_SetHREG(channel, duty);
	pwm_SetLREG(channel, period - duty);
	return true;
}



/*
typedef struct {
    unsigned long hreg;
    unsigned long lreg;
    unsigned long creg;
} PWMSETTING;
static PWMSETTING pwm_settings[32];
static unsigned long syncreg;
static unsigned long intreg;

void pwm_BackupAllSettings(void) {
    int i;

    syncreg = pwm_ReadSyncREG();
    pwm_LockMulti(0xffffffffL);

    for (i=0; i<32; i++)
    {
        pwm_settings[i].hreg = pwm_ReadHREG(i);
        pwm_settings[i].lreg = pwm_ReadLREG(i);
        pwm_settings[i].creg = pwm_ReadCtrlREG(i);
    }
    
    intreg  = pwm_ReadINTREG();
}

void pwm_RestoreAllSettings(void) {
    int i;

    pwm_LockMulti(0xffffffffL);
    for (i=0; i<32; i++)
    {
        pwm_SetHREG(i, pwm_settings[i].hreg);
        pwm_SetLREG(i, pwm_settings[i].lreg);
        pwm_SetCtrlREG(i, pwm_settings[i].creg);
    }

    pwm_SetINTREG(intreg);
    pwm_SetSyncREG(syncreg);
}
*/



RBAPI(void) pwm_EnablePin(int channel) {
    if ((channel<0) || (channel>=PWM_numChannels)) return;
    pwmdx_EnablePin(channel);
}
RBAPI(void) pwm_DisablePin(int channel) {
    if ((channel<0) || (channel>=PWM_numChannels)) return;
    pwmdx_DisablePin(channel);
}
RBAPI(void) pwm_EnableMultiPin(unsigned long channels) {
    pwmdx_EnableMultiPin(channels & ~LONG_LSHIFT(0xffffffffL, PWM_numChannels));
}
RBAPI(void) pwm_DisableMultiPin(unsigned long channels) {
    pwmdx_DisableMultiPin(channels & ~LONG_LSHIFT(0xffffffffL, PWM_numChannels));
}

RBAPI(void) pwm_Lock(int channel) {
    if ((channel<0) || (channel>=PWM_numChannels)) return;
    pwmdx_Lock(channel);
}
RBAPI(void) pwm_Unlock(int channel) {
    if ((channel<0) || (channel>=PWM_numChannels)) return;
    pwmdx_Unlock(channel);
}
RBAPI(void) pwm_LockMulti(unsigned long channels) {
    pwmdx_LockMulti(channels & ~LONG_LSHIFT(0xffffffffL, PWM_numChannels));
}
RBAPI(void) pwm_UnlockMulti(unsigned long channels) {
    pwmdx_UnlockMulti(channels & ~LONG_LSHIFT(0xffffffffL, PWM_numChannels));
}

RBAPI(void) pwm_EnableINT(int channel) {
    if ((channel<0) || (channel>=PWM_numChannels)) return;
    pwmdx_EnableINT(channel);
}
RBAPI(void) pwm_DisableINT(int channel) {
    if ((channel<0) || (channel>=PWM_numChannels)) return;
    pwmdx_DisableINT(channel);
}
RBAPI(void) pwm_EnableMultiINT(unsigned long channels) {
    pwmdx_EnableMultiINT(channels & ~LONG_LSHIFT(0xffffffffL, PWM_numChannels));
}
RBAPI(void) pwm_DisableMultiINT(unsigned long channels) {
    pwmdx_DisableMultiINT(channels & ~LONG_LSHIFT(0xffffffffL, PWM_numChannels));
}

RBAPI(void) pwm_ClearMultiFLAG(unsigned long channels) {
    pwmdx_ClearMultiFLAG(channels & ~LONG_LSHIFT(0xffffffffL, PWM_numChannels));
}
RBAPI(unsigned long) pwm_ReadMultiFLAG(unsigned long channels) {
    return pwmdx_ReadMultiFLAG(channels & ~LONG_LSHIFT(0xffffffffL, PWM_numChannels));
}

RBAPI(void) pwm_EnablePWM(int channel) {
    if ((channel<0) || (channel>=PWM_numChannels)) return;
    pwmdx_EnablePWM(channel);
}
RBAPI(void) pwm_DisablePWM(int channel) {
    if ((channel<0) || (channel>=PWM_numChannels)) return;
    pwmdx_DisablePWM(channel);
}
RBAPI(void) pwm_EnableMultiPWM(unsigned long channels) {
    pwmdx_EnableMultiPWM(channels & ~LONG_LSHIFT(0xffffffffL, PWM_numChannels));
}
RBAPI(void) pwm_DisableMultiPWM(unsigned long channels) {
    pwmdx_DisableMultiPWM(channels & ~LONG_LSHIFT(0xffffffffL, PWM_numChannels));
}

RBAPI(void) pwm_SetCountingMode(int channel, int mode) {
    if ((channel<0) || (channel>=PWM_numChannels)) return;
    pwmdx_SetCountingMode(channel, mode);
}
RBAPI(int) pwm_ReadCountingMode(int channel) {
    if ((channel<0) || (channel>=PWM_numChannels))
    {
        err_SetMsg(ERROR_PWM_WRONGCHANNEL, "channel %d doesn't exist", channel);
        return -1;
    }
    return pwmdx_ReadCountingMode(channel);
}
RBAPI(void) pwm_SetWaveform(int channel, int mode) {
    if ((channel<0) || (channel>=PWM_numChannels)) return;
    pwmdx_SetWaveform(channel, mode);
}

RBAPI(void) pwm_SetPulseCount(int channel, unsigned long count) {
    if ((channel<0) || (channel>=PWM_numChannels)) return;
    pwmdx_SetPulseCount(channel, count);
}
RBAPI(void) pwm_SetCtrlREG(int channel, unsigned long creg) {
    if ((channel<0) || (channel>=PWM_numChannels)) return;
    pwmdx_SetCtrlREG(channel, creg);
}
RBAPI(void) pwm_SetHREG(int channel, unsigned long hreg) {
    if ((channel<0) || (channel>=PWM_numChannels)) return;
    pwmdx_SetHREG(channel, hreg);
}
RBAPI(void) pwm_SetLREG(int channel, unsigned long lreg) {
    if ((channel<0) || (channel>=PWM_numChannels)) return;
    pwmdx_SetLREG(channel, lreg);
}
RBAPI(unsigned long) pwm_ReadPulseCount(int channel) {
    if ((channel<0) || (channel>=PWM_numChannels))
    {
        err_SetMsg(ERROR_PWM_WRONGCHANNEL, "channel %d doesn't exist", channel);
        return 0xffffffffL;
    }
    return pwmdx_ReadPulseCount(channel);
}
RBAPI(unsigned long) pwm_ReadCtrlREG(int channel) {
    if ((channel<0) || (channel>=PWM_numChannels))
    {
        err_SetMsg(ERROR_PWM_WRONGCHANNEL, "channel %d doesn't exist", channel);
        return 0xffffffffL;
    }
    return pwmdx_ReadCtrlREG(channel);
}
RBAPI(unsigned long) pwm_ReadHREG(int channel) {
    if ((channel<0) || (channel>=PWM_numChannels))
    {
        err_SetMsg(ERROR_PWM_WRONGCHANNEL, "channel %d doesn't exist", channel);
        return 0xffffffffL;
    }
    return pwmdx_ReadHREG(channel);
}
RBAPI(unsigned long) pwm_ReadLREG(int channel) {
    if ((channel<0) || (channel>=PWM_numChannels))
    {
        err_SetMsg(ERROR_PWM_WRONGCHANNEL, "channel %d doesn't exist", channel);
        return 0xffffffffL;
    }
    return pwmdx_ReadLREG(channel);
}

