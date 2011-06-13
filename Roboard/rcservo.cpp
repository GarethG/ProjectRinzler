#define __RCSERVO_LIB

#include "defines.h"

#define  USE_COMMON
#include "common.h"
#include "io.h"
#include "pwm.h"
#include "rcservo.h"

#if defined(RB_MSVC_WIN32) || defined(RB_MSVC_WINCE)
    #include <windows.h>
#elif defined(RB_LINUX)
    #include <sys/resource.h>
#endif


#define LONG_LSHIFT(a, b) (((b) >= 32)? 0L : (a) << (b))


/*************************  Vortex86DX GPIO Address  ***************************
 *  I/O addr for GPIO port0--4: 0x78--0x7c
 *  
 *  I/O addr for direction of GPIO port0--4: 0x98--0x9c
 *      bit i = 1 => set i-th pin as output pin
 *      bit i = 0 => set i-th pin as input pin
 * 
 * Remarks:
 * 1. In fact, we must read SB reg 0x6a-6b to get the base address of GPIO 
 *    direction REG, and read SB reg 0x60-61, ..., 0x68-69 to get the base 
 *    address of GPIO0, ..., GPIO4 data REGs (Here, I am lazy, and directly use 
 *    Vortex86DX's default setting, which is also the default setting of 
 *    RoBoard's BIOS:p)
 * 2. When a GPIO pin is switch from input into output, it will always 
 *    output 1. 
 ******************************************************************************/
static unsigned GPIO_port[4]    = {0x78, 0x79, 0x7a, 0x7c};  // GPIO port 0, 1, 2, 4
static unsigned GPIO_dirPort[4] = {0x98, 0x99, 0x9a, 0x9c};


#define RCSERVO_SV_DISABLE (0xffff)
#define MAX_FBFILTERWIDTH  (100)
typedef struct
{
    unsigned servoType;
    
	unsigned long period;              //(unit: us) PWM period
	unsigned long minDuty;             //(unit: us) minimum PWM duty
	unsigned long maxDuty;             //(unit: us) maximum PWM duty
	unsigned long invalidDuty;         //(unit: us) PWM duty that the servo will ignore (useless for non-feedback servos)
	unsigned long minLowPhase;         //(unit: us) minimum value of (PWM period - PWM duty) that the servo allows

	unsigned fbReadMode;               //methods for reading servo's position feedback (see below)
	int      fbMaxNumFail;             //maximum number of feedback-reading failures
	int      fbFilterWidth;            //width of the noise filter used by DENOISE method; maximum 100

	unsigned long cmdDuty[8];          //(unit: us) PWM commands for reading position feedback or change servo's internal setting;
                                       //note that cmdDuty[0] must be the power-off command
	int           cmdInPlayMode;       //command used by rcservo_EnterPlayMode() to read position feedback

	unsigned long capInitDelay;        //(unit: us) time for waiting servo to switch its signal line from IN to OUT when capture begins
	unsigned long capLastDelay;        //(unit: us) time for waiting servo to switch its signal line from OUT to IN when capture completes
	unsigned long capMaxWidth;         //(unit: us) must > the maximum width of the position feedback pulse
	unsigned long capResolution;       //(unit: 0.1us) resolution for capturing the position feedback pulse; must > 0
	long          capWidthOffset;      //(unit: captureResolution) used to fix the captured result of the feedback pulse; determined
                                       //according to RoBoard's and servo's H/W design, as well as the used I/O lib's performance
} RCSERVO_SVPARAM_t;

static RCSERVO_SVPARAM_t RCSERVO_svParams[32] = 
	{
		{RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE},
		{RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE},
		{RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE},
		{RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE},
		{RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE},
		{RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE},
		{RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE},
		{RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE}, {RCSERVO_SV_DISABLE}
	};


static unsigned long RCSERVO_usedChannels  = 0L;
static unsigned long RCSERVO_validChannels = 0L;  //used in Play Mode

static int RCSERVO_operationMode;
           #define RCSERVO_CAPTUREMODE	(0)
           #define RCSERVO_PLAYMODE		(1)
           #define RCSERVO_PWMMODE		(2)



static bool RCSERVO_inUse = false;
RBAPI(bool) rcservo_InUse(void) {
    return RCSERVO_inUse;
}

static unsigned char RCSERVO_oldGPIODIR[4];
static unsigned char RCSERVO_oldGPIO[4];
static bool rcservo_Init2(unsigned long usedchannels, unsigned default_servono) {
/*********************************  Remarks  ***********************************
 * Before calling function, we suggested that all PWM pins have stayed in
 * logic 0. Without this, it may cause instant servo shakes due to long
 * indefinite pulses that may occur when switching GPIO/PWM pins. (In our
 * experiment, this is necessary for KONDO servos but not for HiTEC servos.)
 ******************************************************************************/
    unsigned long hack_t1 = 1L, hack_t2 = 1L;
	int i;

	if (pwm_InUse() == true)
	{
        err_SetMsg(ERROR_RCSERVO_PWMINUSE, "can't use RCSERVO & PWM lib at the same time");
		return false;
	}
	if (RCSERVO_inUse == true)
	{
        err_SetMsg(ERROR_RCSERVO_INUSE, "RCSERVO lib was already opened");
		return false;
	}

	if (pwm_Init2(0xffff, PWMCLOCK_50MHZ, PWMIRQ_DISABLE) == false) return false;
    usedchannels = usedchannels & ~LONG_LSHIFT(0xffffffffL, pwm_NumCh());

    //initialize all channels' servo parameter settings
	RCSERVO_usedChannels = usedchannels;
	for (i=0; i<32; i++)
	{
		if ((RCSERVO_usedChannels & (1L<<i)) == 0L) continue;
	
		if (RCSERVO_svParams[i].servoType == RCSERVO_SV_DISABLE)
            rcservo_SetServo(i, default_servono);

		if (RCSERVO_svParams[i].invalidDuty > hack_t1) hack_t1 = RCSERVO_svParams[i].invalidDuty;
		if (RCSERVO_svParams[i].minLowPhase > hack_t2) hack_t2 = RCSERVO_svParams[i].minLowPhase;
	}
	// experimental trick to avoid RC servos' shake actions (invalid for KONDO's KRS motors)
	delay_us(hack_t1);                           // wait to make an invalid pulse for those channels of staying at GPIO input or nonzero output
    pwmdx_DisableMultiPWM(RCSERVO_usedChannels);
    pwmdx_EnableMultiPin(RCSERVO_usedChannels);  // all channels now output 0
    delay_us(hack_t2);

    // initialize all RC servos to POWEROFF state
    for (i=0; i<32; i++)
    {
		if ((RCSERVO_usedChannels & (1L<<i)) == 0L) continue;

		pwmdx_SetCountingMode(i, PWM_COUNT_MODE);
		pwmdx_SetWaveform(i, PWM_WAVEFORM_NORMAL);

        if (RCSERVO_svParams[i].servoType == RCSERVO_SV_FEEDBACK)
        {
            pwm_SetPulse(i, 2L*RCSERVO_svParams[i].period, RCSERVO_svParams[i].cmdDuty[RCSERVO_CMD_POWEROFF]);
            pwmdx_SetPulseCount(i, 1L);  // send a power-off pulse
        }
		else
		{
            pwm_SetPulse(i, RCSERVO_svParams[i].period, RCSERVO_svParams[i].minDuty);
            pwmdx_SetPulseCount(i, 0L);
        }

        pwmdx_EnablePWM(i);
    }
	for (i=0; i<32; i++)
	{
		if ((RCSERVO_usedChannels & (1L<<i)) == 0L) continue;
        while (pwmdx_ReadPulseCount(i) != 0L);  //should check timeout here, but I'm lazy:p
    }
	pwmdx_ClearMultiFLAG(RCSERVO_usedChannels);

	//set the pins of GPIO ports 0,1,2,4, which correspond to the used channels, as input pins for position-feedback reading
	for (i=0; i<4; i++)
	{
		RCSERVO_oldGPIO[i] = io_inpb(GPIO_port[i]) & (unsigned char)(usedchannels & 0xffL);   //backup GPIO-out status for used channels

		RCSERVO_oldGPIODIR[i] = io_inpb(GPIO_dirPort[i]);
		io_outpb(GPIO_dirPort[i], RCSERVO_oldGPIODIR[i] & (unsigned char)((~usedchannels) & 0xffL));
		RCSERVO_oldGPIODIR[i] = RCSERVO_oldGPIODIR[i] & (unsigned char)(usedchannels & 0xffL); //backup GPIO-dir status for used channels

		usedchannels = usedchannels>>8;
	}

	RCSERVO_operationMode = RCSERVO_CAPTUREMODE;
	RCSERVO_inUse = true;
	return true;
}

RBAPI(bool) rcservo_Init(unsigned long usedchannels) {
    return rcservo_Init2(usedchannels, RCSERVO_SERVO_DEFAULT_NOFB);
}

RBAPI(bool) rcservo_Initialize(unsigned long usedchannels) {  // for compatibility to RoBoIO 1.61
    return rcservo_Init2(usedchannels, RCSERVO_SERVO_DEFAULT);
}

RBAPI(void) rcservo_Close(void) {
    unsigned long usedchannels;
	int i;

	if (RCSERVO_inUse == false) return;

    //set all RC servos with POWEROFF state
    rcservo_EnterPWMMode();
	for (i=0; i<32; i++)
	{
		if (((RCSERVO_usedChannels & (1L<<i)) == 0L) || (RCSERVO_svParams[i].servoType != RCSERVO_SV_FEEDBACK)) continue;
		rcservo_SendPWMPulses(i, 2L*RCSERVO_svParams[i].period, RCSERVO_svParams[i].cmdDuty[RCSERVO_CMD_POWEROFF], 1L);
	}

	for (i=0; i<32; i++)
	{
		if (((RCSERVO_usedChannels & (1L<<i)) == 0L) || (RCSERVO_svParams[i].servoType != RCSERVO_SV_FEEDBACK)) continue;
		while (rcservo_IsPWMCompleted(i) == false); //should check timeout here, but I'm lazy:p
	}

	//restore GPIO output and direction setting
    usedchannels = ~RCSERVO_usedChannels;
	for (i=0; i<4; i++)
	{
		io_outpb(GPIO_dirPort[i], (io_inpb(GPIO_dirPort[i]) & (unsigned char)(usedchannels & 0xffL)) | RCSERVO_oldGPIODIR[i]);
		io_outpb(GPIO_port[i], (io_inpb(GPIO_port[i]) & (unsigned char)(usedchannels & 0xffL)) | RCSERVO_oldGPIO[i]);
		usedchannels = usedchannels>>8;
	}

	pwmdx_DisableMultiPin(RCSERVO_usedChannels);
	pwm_Close();

    RCSERVO_usedChannels  = 0L;
    RCSERVO_validChannels = 0L;
	RCSERVO_inUse = false;
}



/************************  Position Reading Functions  ************************/
static unsigned long rcservo_ReadPosition_normal(int channel, int cmd) {
	unsigned long width;
	RCSERVO_SVPARAM_t* servo  = &RCSERVO_svParams[channel];
	unsigned long capTimeOut  = (servo->capInitDelay+servo->capMaxWidth+servo->capLastDelay) * 10L / servo->capResolution;
	unsigned long capMaxWidth = (servo->capMaxWidth * 10L / servo->capResolution) + 1L;
	unsigned long capRes      = servo->capResolution * 5L;

	unsigned	  dirport = GPIO_dirPort[channel/8];
	unsigned	  port = GPIO_port[channel/8];
	unsigned char pin  = 1 << (channel % 8);

    io_DisableIRQ();


	//^^^^^^^^^ send command to RC servo ^^^^^^^^^^^^^^^^^^^^^^^^^^^
	pwm_SetPulse(channel, servo->cmdDuty[cmd]+servo->capInitDelay, servo->cmdDuty[cmd]);
	pwmdx_SetPulseCount(channel, 1L);
    while (pwmdx_ReadPulseCount(channel) != 0);


	//^^^^^^^^^ read feedback of RC servo ^^^^^^^^^^^^^^^^^^^^^^^^^^
    //set the PWM channel as GPIO input
	pwmdx_DisablePin(channel);

	//detect the raising edge
	pwm_SetPulse_50MHZ(channel, capRes, 1L);
	pwmdx_SetPulseCount(channel, capTimeOut);
    while ((io_inpb(port) & pin) == 0)
		if ((width = pwmdx_ReadPulseCount(channel)) == 0L)
		{   //timeout
			width = 0xffffffffL;
			goto CAPTURE_END;
		}

	//capture the falling edge
	pwmdx_DisablePWM(channel);
	pwmdx_SetPulseCount(channel, capMaxWidth);
	pwmdx_EnablePWM(channel);
	while ((io_inpb(port) & pin) != 0)
		if ((width = pwmdx_ReadPulseCount(channel)) == 0L)
		{   //timeout
			width = 0xffffffffL;
			goto CAPTURE_END;
		}

	width = (unsigned long)((long)(capMaxWidth - width) + servo->capWidthOffset); //unit: servo->capResolution

	//trick to avoid RC servo's shake in multiprogramming OS
    while ((io_inpb(port) & pin) == 0);  //should check timeout here; but I'm lazy:p

	pwmdx_DisablePWM(channel); //remain the channel's pull-up state
	pwm_SetPulse(channel, servo->invalidDuty+2L, 1L);
	pwmdx_SetPulseCount(channel, 1L);
	pwmdx_EnablePWM(channel);
    while (pwmdx_ReadPulseCount(channel) != 0L);

	io_outpb(dirport, io_inpb(dirport) | pin); //force the channel to output 0
	io_outpb(port, io_inpb(port) & (~pin));
	pwm_SetPulse(channel, servo->minLowPhase+2L, 1L);
	pwmdx_SetPulseCount(channel, 1L);
    while (pwmdx_ReadPulseCount(channel) != 0L);

CAPTURE_END:
	pwmdx_EnablePin(channel);
	io_outpb(dirport, io_inpb(dirport) & (~pin));


    io_EnableIRQ();

	return width;
} //end rcservo_ReadPosition_normal(...)


//faster but possibly causing RC servo's shake in multiprogramming OS
static unsigned long rcservo_ReadPosition_fast(int channel, int cmd) {
	unsigned long width;
	RCSERVO_SVPARAM_t* servo  = &RCSERVO_svParams[channel];
	unsigned long capTimeOut  = (servo->capInitDelay+servo->capMaxWidth+servo->capLastDelay) * 10L / servo->capResolution;
	unsigned long capMaxWidth = (servo->capMaxWidth * 10L / servo->capResolution) + 1L;
	unsigned long capRes      = servo->capResolution * 5L;

	unsigned	  dirport = GPIO_dirPort[channel/8];
	unsigned	  port    = GPIO_port[channel/8];
	unsigned char pin     = 1 << (channel % 8);

    io_DisableIRQ();


	//^^^^^^^^^ send command to RC servo ^^^^^^^^^^^^^^^^^^^^^^
	pwm_SetPulse(channel, servo->cmdDuty[cmd]+servo->capInitDelay, servo->cmdDuty[cmd]);
	pwmdx_SetPulseCount(channel, 1L);
    while (pwmdx_ReadPulseCount(channel) != 0);


	//^^^^^^^^^ read feedback of RC servo ^^^^^^^^^^^^^^^^^^^^^
    //set the PWM channel as GPIO input
	pwmdx_DisablePin(channel);

	//detect the raising edge
	pwm_SetPulse_50MHZ(channel, capRes, 1L);
	pwmdx_SetPulseCount(channel, capTimeOut);
    while ((io_inpb(port) & pin) == 0)
		if ((width = pwmdx_ReadPulseCount(channel)) == 0L)
		{   //timeout
			width = 0xffffffffL;
			goto CAPTURE_END;
		}

	//capture falling edge
	pwmdx_DisablePWM(channel);
	pwmdx_SetPulseCount(channel, capMaxWidth);
	pwmdx_EnablePWM(channel);
	while ((io_inpb(port) & pin) != 0)
		if ((width = pwmdx_ReadPulseCount(channel)) == 0L)
		{   //timeout
			width = 0xffffffffL;
			goto CAPTURE_END;
		}

	width = (unsigned long)((long)(capMaxWidth - width) + servo->capWidthOffset); //unit: servo->capResolution

	pwmdx_DisablePWM(channel); //set the channel to output 0; may cause a false pulse if OS context-switches here
	pwmdx_EnablePin(channel);
	io_outpb(dirport, io_inpb(dirport) | pin);
	io_outpb(port, io_inpb(port) & (~pin));
	pwmdx_DisablePin(channel);

	//wait RC servo to switch from "feedback" state to "PWM" state
	pwm_SetPulse_10MHZ(channel, servo->capLastDelay * 10L, 1L);
	pwmdx_SetPulseCount(channel, 1L);
	pwmdx_EnablePWM(channel);
    while (pwmdx_ReadPulseCount(channel) != 0L);

CAPTURE_END:
	pwmdx_EnablePin(channel);
	io_outpb(dirport, io_inpb(dirport) & (~pin));


    io_EnableIRQ();

	return width;
} //end rcservo_ReadPosition_fast(...)



_RB_INLINE void InsertionSort(unsigned long* A, int size) {
	unsigned long tmp;
	int i, j;

	for (i=1; i<size; i++)
	{
		tmp = A[i];
		for (j=i-1; j>=0; j--)
			if (tmp<A[j]) A[j+1]=A[j]; else break;
		A[j+1]=tmp;
	}
}

_RB_INLINE unsigned long rcservo_ReadPOS_Denoise(int channel, int cmd) {
    int i, j = 0;
	unsigned long wbuf[MAX_FBFILTERWIDTH];

	for (i=0; i<RCSERVO_svParams[channel].fbFilterWidth; i++)
	{
		for (; j<RCSERVO_svParams[channel].fbMaxNumFail; j++)
	    {
	        if ((RCSERVO_svParams[channel].fbReadMode & RCSERVO_FB_FASTMODE) != 0)
		        wbuf[i] = rcservo_ReadPosition_fast(channel, cmd);
	        else
		        wbuf[i] = rcservo_ReadPosition_normal(channel, cmd);

			if (wbuf[i] != 0xffffffffL) break;
		}
		if (j == RCSERVO_svParams[channel].fbMaxNumFail) return 0xffffffffL; //fail to read position!

		if (wbuf[i] < RCSERVO_svParams[channel].minDuty) wbuf[i] = RCSERVO_svParams[channel].minDuty;
		if (wbuf[i] > RCSERVO_svParams[channel].maxDuty) wbuf[i] = RCSERVO_svParams[channel].maxDuty;
	}

	InsertionSort(wbuf, RCSERVO_svParams[channel].fbFilterWidth);
	return wbuf[RCSERVO_svParams[channel].fbFilterWidth/2];
}

_RB_INLINE unsigned long rcservo_ReadPOS_Robust(int channel, int cmd) {
    int i;
	unsigned long width;

	for (i=0; i<RCSERVO_svParams[channel].fbMaxNumFail; i++)
	{
	    if ((RCSERVO_svParams[channel].fbReadMode & RCSERVO_FB_FASTMODE) != 0)
		    width = rcservo_ReadPosition_fast(channel, cmd);
	    else
		    width = rcservo_ReadPosition_normal(channel, cmd);

	    if (width != 0xffffffffL)
        {
		    if (width < RCSERVO_svParams[channel].minDuty) width = RCSERVO_svParams[channel].minDuty;
		    if (width > RCSERVO_svParams[channel].maxDuty) width = RCSERVO_svParams[channel].maxDuty;
            return width;
        }
	}

    return 0xffffffffL;
}



static bool RCSERVO_ReadPOS_MPOS = false;
static bool MPOS;

#if defined(RB_MSVC_WIN32)
    static HANDLE hApp, hThread;
    static int iPriorityClass, iPriority;
#elif defined(RB_MSVC_WINCE)
    static HANDLE hThread;
    static int iPriority;
#elif defined(RB_LINUX)
	static int iPriority;
#endif

_RB_INLINE void MPOS_Start(void) {
	if (RCSERVO_ReadPOS_MPOS == false) return;

    #if defined(RB_MSVC_WIN32)
	    hApp = GetCurrentProcess();
        hThread = GetCurrentThread();

        iPriorityClass = GetPriorityClass(hApp);
        iPriority = GetThreadPriority(hThread);

        if (iPriorityClass != 0) SetPriorityClass(hApp, REALTIME_PRIORITY_CLASS);
        if (iPriority != THREAD_PRIORITY_ERROR_RETURN) SetThreadPriority(hThread, THREAD_PRIORITY_TIME_CRITICAL);
	#elif defined(RB_MSVC_WINCE)
        hThread = GetCurrentThread();
        iPriority = GetThreadPriority(hThread);
        if (iPriority != THREAD_PRIORITY_ERROR_RETURN) SetThreadPriority(hThread, THREAD_PRIORITY_TIME_CRITICAL);
    #elif defined(RB_LINUX)
	    iPriority = getpriority(PRIO_PROCESS, 0); //lazy to check error:p
        setpriority(PRIO_PROCESS, 0, -20);
    #endif
}

_RB_INLINE void MPOS_End(void) {
	if (RCSERVO_ReadPOS_MPOS == false) return;

    #if defined(RB_MSVC_WIN32)
        if (iPriorityClass != 0) SetPriorityClass(hApp, iPriorityClass);
		if (iPriority != THREAD_PRIORITY_ERROR_RETURN) SetThreadPriority(hThread, iPriority);
	#elif defined(RB_MSVC_WINCE)
		if (iPriority != THREAD_PRIORITY_ERROR_RETURN) SetThreadPriority(hThread, iPriority);
    #elif defined(RB_LINUX)
		setpriority(PRIO_PROCESS, 0, iPriority);
    #endif
}

_RB_INLINE void MPOS_Starts(void) {
	MPOS = RCSERVO_ReadPOS_MPOS;
	if (RCSERVO_ReadPOS_MPOS == false) return; else RCSERVO_ReadPOS_MPOS = false;

    #if defined(RB_MSVC_WIN32)
        hApp = GetCurrentProcess();
        hThread = GetCurrentThread();

        iPriorityClass = GetPriorityClass(hApp);
        iPriority = GetThreadPriority(hThread);

		if (iPriorityClass != 0) SetPriorityClass(hApp, REALTIME_PRIORITY_CLASS);
        if (iPriority != THREAD_PRIORITY_ERROR_RETURN) SetThreadPriority(hThread, THREAD_PRIORITY_TIME_CRITICAL);
	#elif defined(RB_MSVC_WINCE)
        hThread = GetCurrentThread();
        iPriority = GetThreadPriority(hThread);
        if (iPriority != THREAD_PRIORITY_ERROR_RETURN) SetThreadPriority(hThread, THREAD_PRIORITY_TIME_CRITICAL);
    #elif defined(RB_LINUX)
	    iPriority = getpriority(PRIO_PROCESS, 0); //lazy to check error:p
        setpriority(PRIO_PROCESS, 0, -20);
    #endif
}

_RB_INLINE void MPOS_Ends(void) {
	if (MPOS == false) return; else RCSERVO_ReadPOS_MPOS = MPOS;

    #if defined(RB_MSVC_WIN32)
		if (iPriorityClass != 0) SetPriorityClass(hApp, iPriorityClass);
		if (iPriority != THREAD_PRIORITY_ERROR_RETURN) SetThreadPriority(hThread, iPriority);
	#elif defined(RB_MSVC_WINCE)
		if (iPriority != THREAD_PRIORITY_ERROR_RETURN) SetThreadPriority(hThread, iPriority);
	#elif defined(RB_LINUX)
		setpriority(PRIO_PROCESS, 0, iPriority);
    #endif
}

RBAPI(void) rcservo_EnableMPOS(void) {
    RCSERVO_ReadPOS_MPOS = true;
}

RBAPI(void) rcservo_DisableMPOS(void) {
    RCSERVO_ReadPOS_MPOS = false;
}



RBAPI(unsigned long) rcservo_ReadPosition(int channel, int cmd) {
	unsigned long width;

	if ((RCSERVO_usedChannels & LONG_LSHIFT(1L, channel)) == 0L) return 0xffffffffL;
	if (RCSERVO_svParams[channel].servoType != RCSERVO_SV_FEEDBACK) return 0xffffffffL;

	if (RCSERVO_operationMode != RCSERVO_CAPTUREMODE)
	{
		errmsg("WARNING: %s() can't work in PLAY or PWM mode!\n", __FUNCTION__);
		return 0xffffffffL;
	}

	if ((cmd < 0) || (cmd > 7))
	{
		errmsg("WARNING: %s() receives an unknown command for reading feedback!\n", __FUNCTION__);
		return 0xffffffffL;
	}

	MPOS_Start();
	if ((RCSERVO_svParams[channel].fbReadMode & RCSERVO_FB_DENOISE) != 0)
        width = rcservo_ReadPOS_Denoise(channel, cmd);
    else
        width = rcservo_ReadPOS_Robust(channel, cmd);
	MPOS_End();

	return width;
}

RBAPI(void) rcservo_ReadPositions(unsigned long channels, int cmd, unsigned long* width) {
	unsigned long readchannels = RCSERVO_usedChannels & channels;
	int i;
	
	MPOS_Starts();
	for (i=0; i<32; i++)
	{
		if (((readchannels & (1L<<i)) == 0L) || (RCSERVO_svParams[i].servoType != RCSERVO_SV_FEEDBACK))
			width[i] = 0L;
		else
			width[i] = rcservo_ReadPosition(i, cmd);
	}
	MPOS_Ends();
}

RBAPI(unsigned long) rcservo_CapOne(int channel) {
    return rcservo_ReadPosition(channel, RCSERVO_CMD_POWEROFF);
}

RBAPI(bool) rcservo_CapAll(unsigned long* width) {
    int i;

    rcservo_ReadPositions(RCSERVO_usedChannels, RCSERVO_CMD_POWEROFF, width);
    for (i=0; i<32; i++)
        if (((RCSERVO_usedChannels & (1L<<i)) != 0L) && (width[i] == 0xffffffffL)) return false;

    return true;
}



//only for backward compatibility to RoBoIO library 1.1
RBAPI(unsigned long) rcservo_ReadPositionDN(int channel, int cmd) {
	unsigned long width;

	if ((RCSERVO_usedChannels & LONG_LSHIFT(1L, channel)) == 0L) return 0xffffffffL;
	if (RCSERVO_svParams[channel].servoType != RCSERVO_SV_FEEDBACK) return 0xffffffffL;

	if (RCSERVO_operationMode != RCSERVO_CAPTUREMODE)
	{
		errmsg("WARNING: %s() can't work in PLAY or PWM mode!\n", __FUNCTION__);
		return 0xffffffffL;
	}

	if ((cmd < 0) || (cmd > 7))
	{
		errmsg("WARNING: %s() receives an unknown command for reading feedback!\n", __FUNCTION__);
		return 0xffffffffL;
	}

	MPOS_Start();
    width = rcservo_ReadPOS_Denoise(channel, cmd);
	MPOS_End();
	
	return width;
}

//only for backward compatibility to RoBoIO library 1.1
RBAPI(void) rcservo_ReadPositionsDN(unsigned long channels, int cmd, unsigned long* width) {
	unsigned long readchannels = RCSERVO_usedChannels & channels;
	int i;
	
	MPOS_Starts();
	for (i=0; i<32; i++)
	{
		if (((readchannels & (1L<<i)) == 0L) || (RCSERVO_svParams[i].servoType != RCSERVO_SV_FEEDBACK))
			width[i] = 0L;
		else
			width[i] = rcservo_ReadPositionDN(i, cmd);
	}
	MPOS_Ends();
}



_RB_INLINE void stop_pwmmode_pulses(void) {
	int i;

	for (i=0; i<32; i++)
	{
		if ((RCSERVO_usedChannels & (1L<<i)) == 0L) continue;
		if (pwmdx_ReadCountingMode(i) == PWM_CONTINUE_MODE)
		{
            pwmdx_SetPulseCount(i, 1L);
            pwmdx_SetCountingMode(i, PWM_COUNT_MODE);
		}
        if (pwmdx_ReadPulseCount(i) > 1L) pwmdx_SetPulseCount(i, 1L);
	}

	for (i=0; i<32; i++)
	{
		if ((RCSERVO_usedChannels & (1L<<i)) == 0L) continue;
		while (pwmdx_ReadPulseCount(i) != 0L);
	}
}

_RB_INLINE void stop_playmode_pulses(void) {
	int i;

	pwmdx_ClearMultiFLAG(RCSERVO_validChannels);
	for (i=0; i<32; i++)
	{
		if ((RCSERVO_validChannels & (1L<<i)) == 0L) continue;
		pwmdx_SetPulseCount(i, 1L);
		pwmdx_SetCountingMode(i, PWM_COUNT_MODE);
	}
	while (pwmdx_ReadMultiFLAG(RCSERVO_validChannels) != RCSERVO_validChannels);
}

RBAPI(void) rcservo_EnterCaptureMode(void) {
	if (RCSERVO_operationMode == RCSERVO_CAPTUREMODE) return;

	if (RCSERVO_operationMode == RCSERVO_PLAYMODE)
		stop_playmode_pulses();
	else
	if (RCSERVO_operationMode == RCSERVO_PWMMODE)
		stop_pwmmode_pulses();

	RCSERVO_operationMode = RCSERVO_CAPTUREMODE;
}

RBAPI(void) rcservo_SendCMD(int channel, int cmd) {
    if (RCSERVO_operationMode != RCSERVO_CAPTUREMODE) return;
    if ((channel < 0) || (channel > 31) || ((RCSERVO_usedChannels & (1L<<channel)) == 0L)) return;

    pwm_SetPulse(channel, RCSERVO_svParams[channel].period, RCSERVO_svParams[channel].cmdDuty[cmd]);
    pwmdx_SetPulseCount(channel, 1L);
	
    while (pwmdx_ReadPulseCount(channel) != 0L);
}
/*--------------------  end of Position Reading Functions  -------------------*/



/********************  Ad Hoc Position Reading Functions  *********************/
//void rcservo_PipelinedReadPositions(unsigned long channels, int cmd, unsigned long* width) {
//}

//void rcservo_ParallelReadPositions(unsigned long channels, int cmd, unsigned long* width) {
//}
/*----------------  end of Ad Hoc Position Reading Functions  ----------------*/



/***********************  Action Playing Functions  ***************************/
static int RCSERVO_playState = RCSERVO_PLAYEND;
static unsigned long RCSERVO_playInterval = 25L; //25ms (40fps)
static unsigned long RCSERVO_curaction_starttime;
static unsigned long RCSERVO_curaction_nexttime;
static unsigned long RCSERVO_curaction_endtime;
static unsigned long RCSERVO_curaction[32];
static unsigned long RCSERVO_midaction[32];
static unsigned long RCSERVO_lastaction[32];

_RB_INLINE unsigned long interpolate(unsigned long start_val, unsigned long end_val, unsigned long total, unsigned long t) {
	if (total == 0L) return end_val;

	return ((total-t)*start_val + t*end_val)/total;
}

_RB_INLINE void rcservo_ReadPositions_PlayMode(unsigned long* width) { //used by rcservo_EnterPlayMode()
	int i;
	
	RCSERVO_validChannels = RCSERVO_usedChannels;

	MPOS_Starts();
	for (i=0; i<32; i++)
	{
		if (((RCSERVO_usedChannels & (1L<<i)) == 0L) || (RCSERVO_svParams[i].servoType != RCSERVO_SV_FEEDBACK))
			width[i] = 0L;
		else
			width[i] = rcservo_ReadPositionDN(i, RCSERVO_svParams[i].cmdInPlayMode);

	    if (width[i] == 0xffffffffL)
		{
			width[i] = 0L;
			RCSERVO_validChannels = RCSERVO_validChannels & (~(1L<<i));

			errmsg("WARNING: %s() fails to read channel %d!\n", __FUNCTION__, i);
		}
	}
	MPOS_Ends();
}

_RB_INLINE void set_playmode_pulses(unsigned long* width) {
	int i;

	if (RCSERVO_operationMode == RCSERVO_PLAYMODE) stop_playmode_pulses();

	for (i=0; i<32; i++)
	{
		if ((RCSERVO_validChannels & (1L<<i)) == 0L) continue;

		if ((width[i] < RCSERVO_svParams[i].minDuty) || (width[i] > RCSERVO_svParams[i].maxDuty)) //&&
        if ( (width[i] != RCSERVO_svParams[i].cmdDuty[RCSERVO_CMD_POWEROFF]) &&
             (width[i] != RCSERVO_svParams[i].cmdDuty[RCSERVO_CMD1]) &&
             (width[i] != RCSERVO_svParams[i].cmdDuty[RCSERVO_CMD2]) &&
             (width[i] != RCSERVO_svParams[i].cmdDuty[RCSERVO_CMD3]) &&
             (width[i] != RCSERVO_svParams[i].cmdDuty[RCSERVO_CMD4]) &&
             (width[i] != RCSERVO_svParams[i].cmdDuty[RCSERVO_CMD5]) &&
             (width[i] != RCSERVO_svParams[i].cmdDuty[RCSERVO_CMD6]) &&
             (width[i] != RCSERVO_svParams[i].cmdDuty[RCSERVO_CMD7]) )
        {
                errmsg("WARNING: %s() receives width %lu out of range for channel %d!\n", __FUNCTION__, width[i], i);
                continue;
		}

		pwm_SetPulse(i, RCSERVO_svParams[i].period, width[i]);
		pwmdx_SetCountingMode(i, PWM_CONTINUE_MODE);
	} //end for i...
}

_RB_INLINE void add_mixwidth(unsigned long* width, long* mixwidth) {
    int i;

    if ((mixwidth == NULL) || (width ==  NULL)) return;

    for (i=0; i<32; i++)
    {
        switch (mixwidth[i])
        {
            case RCSERVO_MIXWIDTH_POWEROFF:
                width[i] = RCSERVO_svParams[i].cmdDuty[RCSERVO_CMD_POWEROFF]; break;
            case RCSERVO_MIXWIDTH_CMD1:
                width[i] = RCSERVO_svParams[i].cmdDuty[RCSERVO_CMD1]; break;
            case RCSERVO_MIXWIDTH_CMD2:
                width[i] = RCSERVO_svParams[i].cmdDuty[RCSERVO_CMD2]; break;
            case RCSERVO_MIXWIDTH_CMD3:
                width[i] = RCSERVO_svParams[i].cmdDuty[RCSERVO_CMD3]; break;
            case RCSERVO_MIXWIDTH_CMD4:
                width[i] = RCSERVO_svParams[i].cmdDuty[RCSERVO_CMD4]; break;
            case RCSERVO_MIXWIDTH_CMD5:
                width[i] = RCSERVO_svParams[i].cmdDuty[RCSERVO_CMD5]; break;
            case RCSERVO_MIXWIDTH_CMD6:
                width[i] = RCSERVO_svParams[i].cmdDuty[RCSERVO_CMD6]; break;
            case RCSERVO_MIXWIDTH_CMD7:
                width[i] = RCSERVO_svParams[i].cmdDuty[RCSERVO_CMD7]; break;
            default:
                width[i] = (unsigned long)((long)width[i] + mixwidth[i]);
                break;
        }
    } //end for i...
}



RBAPI(void) rcservo_SetFPS(int fps) {
    if ((fps <= 0) || (fps > 1000))
    {
		errmsg("WARNING: %s() receives invalid FPS: %d!\n", fps);
        return;
    }

    RCSERVO_playInterval = 1000L/(unsigned long)fps;
}



RBAPI(void) rcservo_EnterPlayMode(void) {
	int i;

	rcservo_EnterCaptureMode();
	rcservo_ReadPositions_PlayMode(RCSERVO_curaction);  // will initialize RCSERVO_validChannels
	for (i=0; i<32; i++)
    {
        //all servos without feedback will be ignored
        if (RCSERVO_svParams[i].servoType != RCSERVO_SV_FEEDBACK)
            RCSERVO_validChannels = RCSERVO_validChannels & (~(1L<<i));

        RCSERVO_lastaction[i] = RCSERVO_curaction[i];
        RCSERVO_midaction[i]  = RCSERVO_curaction[i];
    }
	set_playmode_pulses(RCSERVO_curaction);

	RCSERVO_operationMode = RCSERVO_PLAYMODE;
	RCSERVO_playState     = RCSERVO_PLAYEND;
}

RBAPI(void) rcservo_EnterPlayMode_NOFB(unsigned long* width) {
	int i;

	rcservo_EnterCaptureMode();
	rcservo_ReadPositions_PlayMode(RCSERVO_curaction);  // will initialize RCSERVO_validChannels
	for (i=0; i<32; i++)
    {
        //initial positions for servos without feedback will be set to width[i]
		if (RCSERVO_svParams[i].servoType == RCSERVO_SV_NOFEEDBACK)
            RCSERVO_curaction[i] = width[i];
		else
        if (RCSERVO_svParams[i].servoType == RCSERVO_SV_DISABLE)
            RCSERVO_validChannels = RCSERVO_validChannels & (~(1L<<i));

        RCSERVO_lastaction[i] = RCSERVO_curaction[i];
        RCSERVO_midaction[i]  = RCSERVO_curaction[i];
    }
	set_playmode_pulses(RCSERVO_curaction);

	RCSERVO_operationMode = RCSERVO_PLAYMODE;
	RCSERVO_playState     = RCSERVO_PLAYEND;
}

RBAPI(void) rcservo_EnterPlayMode_HOME(unsigned long* width) {
	int i;

	rcservo_EnterCaptureMode();
	RCSERVO_validChannels = RCSERVO_usedChannels;
	for (i=0; i<32; i++)
    {
		//initial positions for all servos will be set to width[i]
        if (RCSERVO_svParams[i].servoType != RCSERVO_SV_DISABLE)
			RCSERVO_curaction[i]  = width[i];
		else
            RCSERVO_validChannels = RCSERVO_validChannels & (~(1L<<i));

		RCSERVO_lastaction[i] = RCSERVO_curaction[i];
        RCSERVO_midaction[i]  = RCSERVO_curaction[i];
    }
	set_playmode_pulses(RCSERVO_curaction);

	RCSERVO_operationMode = RCSERVO_PLAYMODE;
	RCSERVO_playState     = RCSERVO_PLAYEND;
}



RBAPI(void) rcservo_SetAction(unsigned long* width, unsigned long playtime) {
	int i;

    if (RCSERVO_playState != RCSERVO_PLAYEND) rcservo_StopAction();

	for (i=0; i<32; i++)
	{
		RCSERVO_lastaction[i] = RCSERVO_curaction[i];
		RCSERVO_curaction[i]  = (width[i] == 0L)? RCSERVO_curaction[i] : width[i];
	}

	RCSERVO_curaction_starttime = timer_nowtime();
	RCSERVO_curaction_nexttime  = RCSERVO_curaction_starttime + RCSERVO_playInterval;
	RCSERVO_curaction_endtime   = RCSERVO_curaction_starttime + playtime;

	RCSERVO_playState = RCSERVO_PLAYING;
}

RBAPI(int) rcservo_PlayAction(void) {
    unsigned long t, tt;
    int i;

    if (RCSERVO_playState != RCSERVO_PLAYING) return RCSERVO_playState;

    t  = timer_nowtime() - RCSERVO_curaction_starttime;
    tt = RCSERVO_curaction_endtime - RCSERVO_curaction_starttime;
    if (t >= tt)  // note: don't compare timer_nowtime() with RCSERVO_curaction_endtime directly to avoid timer overflow
    {
        for (i=0; i<32; i++) RCSERVO_midaction[i] = RCSERVO_curaction[i];
        set_playmode_pulses(RCSERVO_midaction);
        RCSERVO_playState = RCSERVO_PLAYEND;
    }
    else
    if (t >= (RCSERVO_curaction_nexttime - RCSERVO_curaction_starttime))
    {
        for (i=0; i<32; i++)
        {
            if ((RCSERVO_validChannels & (1L<<i)) == 0L) continue;
            RCSERVO_midaction[i] = interpolate(RCSERVO_lastaction[i], RCSERVO_curaction[i], tt, t);
        }
        set_playmode_pulses(RCSERVO_midaction);

        while ((RCSERVO_curaction_nexttime - RCSERVO_curaction_starttime) <= t)
            RCSERVO_curaction_nexttime = RCSERVO_curaction_nexttime + RCSERVO_playInterval;
	}

    return RCSERVO_playState;
}

RBAPI(int) rcservo_PlayActionMix(long* mixwidth) {
    static unsigned long lasttime;
    static bool lasttime_used = false;
    
    unsigned long t, tt, nowtime = timer_nowtime();
    int i;

    if (RCSERVO_playState == RCSERVO_PLAYING)
    {
        t  = nowtime - RCSERVO_curaction_starttime; 
        tt = RCSERVO_curaction_endtime - RCSERVO_curaction_starttime; 
        if (t >= tt)
        {
            RCSERVO_playState = RCSERVO_PLAYEND;
            lasttime = nowtime - RCSERVO_playInterval;
        }
        else
        if (t >= (RCSERVO_curaction_nexttime - RCSERVO_curaction_starttime))
        { 
            for (i=0; i<32; i++) 
            {
                if ((RCSERVO_validChannels & (1L<<i)) == 0L) continue;
		        RCSERVO_midaction[i] = interpolate(RCSERVO_lastaction[i], RCSERVO_curaction[i], tt, t);
            }
            add_mixwidth(RCSERVO_midaction, mixwidth);
            set_playmode_pulses(RCSERVO_midaction);

            while ((RCSERVO_curaction_nexttime - RCSERVO_curaction_starttime) <= t)
                RCSERVO_curaction_nexttime = RCSERVO_curaction_nexttime + RCSERVO_playInterval;
        }
    } //end if (RCSERVO_playState...

    if (RCSERVO_playState == RCSERVO_PLAYEND)
    {
        if (lasttime_used == false)
        {
            lasttime = nowtime - RCSERVO_playInterval;
            lasttime_used = true;
        }

        if ((nowtime - lasttime) >= RCSERVO_playInterval)
        {
            for (i=0; i<32; i++) RCSERVO_midaction[i] = RCSERVO_curaction[i];
            add_mixwidth(RCSERVO_midaction, mixwidth);
            set_playmode_pulses(RCSERVO_midaction);
            while ((nowtime - lasttime) >= RCSERVO_playInterval) lasttime = lasttime + RCSERVO_playInterval;
        }
    }

    return RCSERVO_playState;
}


static unsigned long RCSERVO_pausedtime;
RBAPI(void) rcservo_PauseAction(void) {
	if (RCSERVO_playState != RCSERVO_PLAYING) return;

    RCSERVO_pausedtime = timer_nowtime();
	RCSERVO_playState  = RCSERVO_PAUSED;
}

RBAPI(void) rcservo_ReleaseAction(void) {
	if (RCSERVO_playState != RCSERVO_PAUSED) return;

	RCSERVO_pausedtime = timer_nowtime() - RCSERVO_pausedtime;
    RCSERVO_curaction_starttime = RCSERVO_curaction_starttime + RCSERVO_pausedtime;
    RCSERVO_curaction_endtime   = RCSERVO_curaction_endtime   + RCSERVO_pausedtime;
    RCSERVO_curaction_nexttime  = RCSERVO_curaction_nexttime  + RCSERVO_pausedtime;

	RCSERVO_playState = RCSERVO_PLAYING;
}

RBAPI(void) rcservo_StopAction(void) {
    int i;

    if (rcservo_PlayAction() == RCSERVO_PLAYEND) return;
    if (RCSERVO_playState == RCSERVO_PAUSED) rcservo_ReleaseAction();

    for (i=0; i<32; i++) RCSERVO_curaction[i] = RCSERVO_midaction[i];

    RCSERVO_playState = RCSERVO_PLAYEND;
}

RBAPI(void) rcservo_GetAction(unsigned long* width) {
    int i;

    for (i=0; i<32; i++) width[i] = ((RCSERVO_validChannels & (1L<<i)) == 0L)? 0L : RCSERVO_midaction[i];
}


RBAPI(void) rcservo_MoveTo(unsigned long* width, unsigned long playtime) {
    rcservo_SetAction(width, playtime);
    while (rcservo_PlayAction() != RCSERVO_PLAYEND);
}

RBAPI(void) rcservo_MoveOne(int channel, unsigned long pos, unsigned long playtime) {
    unsigned long width[32] = {0L};

	if ((RCSERVO_usedChannels & LONG_LSHIFT(1L, channel)) == 0L) return;

    width[channel] = pos;
    rcservo_MoveTo(width, playtime);  // width[i] = 0L ==> remain the servo position on channel i
}
/*---------------------  end of Action Playing Functions  --------------------*/



/***************************  PWM Mode Functions  *****************************/
RBAPI(void) rcservo_EnterPWMMode(void) {
	rcservo_EnterCaptureMode();  // make all PWM channels switch into Counting Mode
	RCSERVO_operationMode = RCSERVO_PWMMODE;
}

RBAPI(bool) rcservo_SendPWM(int channel, unsigned long period, unsigned long duty, unsigned long count) {
	if ((period == 0L) || (duty == 0L) || (count == 0L))
	{
		err_SetMsg(ERROR_RCSERVO_PWMFAIL, "zero period or duty or count");
		return false;
	}

	if (count > 0x0fffffffL)
	{
		err_SetMsg(ERROR_RCSERVO_PWMFAIL, "too large pulse count");
		return false;
	}

	if ((RCSERVO_usedChannels & LONG_LSHIFT(1L, channel)) == 0L) return true;

	rcservo_StopPWM(channel);
	pwm_SetPulse(channel, period, duty);
	pwmdx_SetPulseCount(channel, count);
	
	return true;
}

RBAPI(bool) rcservo_SendCPWM(int channel, unsigned long period, unsigned long duty) {
	if ((period == 0L) || (duty == 0L))
	{
		err_SetMsg(ERROR_RCSERVO_PWMFAIL, "zero period or duty");
		return false;
	}

	if ((RCSERVO_usedChannels & LONG_LSHIFT(1L, channel)) == 0L) return true;
	
	rcservo_StopPWM(channel);
	pwm_SetPulse(channel, period, duty);
    pwmdx_SetCountingMode(channel, PWM_CONTINUE_MODE);
	
	return true;
}

RBAPI(void) rcservo_StopPWM(int channel) {
	if ((RCSERVO_usedChannels & LONG_LSHIFT(1L, channel)) == 0L) return;

	if (pwmdx_ReadCountingMode(channel) == PWM_CONTINUE_MODE)
	{
        pwmdx_SetPulseCount(channel, 1L);
        pwmdx_SetCountingMode(channel, PWM_COUNT_MODE);
	}
    if (pwmdx_ReadPulseCount(channel) > 1L) pwmdx_SetPulseCount(channel, 1L);
    
    while (pwmdx_ReadPulseCount(channel) != 0L);
}

/*
RBAPI(bool) rcservo_IsPWMCompleted(int channel) {
	if ((RCSERVO_usedChannels & LONG_LSHIFT(1L, channel)) == 0L) return true;
	
	if (pwmdx_ReadCountingMode(channel) == PWM_CONTINUE_MODE) return false;
	if (pwmdx_ReadPulseCount(channel) == 0L) return true; else return false;
}
*/
RBAPI(unsigned long) rcservo_CheckPWM(int channel) {
	if ((RCSERVO_usedChannels & LONG_LSHIFT(1L, channel)) == 0L) return 0L;
	
	if (pwmdx_ReadCountingMode(channel) == PWM_CONTINUE_MODE) return 0xffffffffL;

	return pwmdx_ReadPulseCount(channel);
}
/*------------------------  end of PWM Mode Functions  -----------------------*/



/*****************************  GPIO Functions  *******************************/
RBAPI(void) rcservo_OutPin(int channel, int value) {
	unsigned	  port, dirport;
	unsigned char pin;

    if ((channel < 0) || (channel >= pwm_NumCh()) || ((RCSERVO_usedChannels & LONG_LSHIFT(1L, channel)) != 0L)) return;

    port    = GPIO_port[channel/8];
    dirport = GPIO_dirPort[channel/8];
    pin     = 1 << (channel % 8);

	io_outpb(dirport, io_inpb(dirport) | pin);  //must change port direction first; GPIO out is invalid if its direction is "IN"
	if (value == 0)
	    io_outpb(port, io_inpb(port) & (~pin));
	else
	    io_outpb(port, io_inpb(port) | pin);
}

RBAPI(int) rcservo_InPin(int channel) {
	unsigned	  port, dirport;
	unsigned char pin;

    if ((channel < 0) || (channel >= pwm_NumCh()) || ((RCSERVO_usedChannels & LONG_LSHIFT(1L, channel)) != 0L)) return 0;

    port    = GPIO_port[channel/8];
    dirport = GPIO_dirPort[channel/8];
    pin     = 1 << (channel % 8);
    
	io_outpb(dirport, io_inpb(dirport) & (~pin));
    if ((io_inpb(port) & pin) != 0) return 1; else return 0;
}


RBAPI(void) rcservo_OutPort(unsigned long channels, unsigned long values) {
    unsigned long gpio012ch;
    unsigned char gpio4ch;

    channels  = channels & ~LONG_LSHIFT(0xffffffffL, pwm_NumCh());
    gpio012ch = ((~RCSERVO_usedChannels) & channels) & 0x00ffffffL;
    gpio4ch   = (unsigned char)(0xffL & (((~RCSERVO_usedChannels) & channels) >> 24));

    //output to GPIO ports 0, 1, 2 (assume sequential base address for performance)
    io_outpdw(GPIO_dirPort[0], io_inpdw(GPIO_dirPort[0]) | gpio012ch);
    io_outpdw(GPIO_port[0], (io_inpdw(GPIO_port[0]) & (~gpio012ch)) | (values & gpio012ch));

    //output to GPIO port 4
    io_outpb(GPIO_dirPort[3], io_inpb(GPIO_dirPort[3]) | gpio4ch);
    io_outpb(GPIO_port[3], (io_inpb(GPIO_port[3]) & (~gpio4ch)) | ((unsigned char)(values >> 24) & gpio4ch));
}

RBAPI(unsigned long) rcservo_InPort(unsigned long channels) {
    unsigned long gpio012ch;
    unsigned char gpio4ch;
    unsigned long values;

    channels  = channels & ~LONG_LSHIFT(0xffffffffL, pwm_NumCh());
    gpio012ch = ((~RCSERVO_usedChannels) & channels) & 0x00ffffffL;
    gpio4ch   = (unsigned char)(0xffL & (((~RCSERVO_usedChannels) & channels) >> 24));

    //input from GPIO ports 0, 1, 2 (assume sequential base address for performance)
    io_outpdw(GPIO_dirPort[0], io_inpdw(GPIO_dirPort[0]) & (~gpio012ch));
    values = io_inpdw(GPIO_port[0]) & gpio012ch;
    
    //input from GPIO port 4
    io_outpb(GPIO_dirPort[3], io_inpb(GPIO_dirPort[3]) & (~gpio4ch));
    values = values | ((unsigned long)(io_inpb(GPIO_port[3]) & gpio4ch) << 24);
    
    return values;
}
/*-------------------------- end of GPIO Functions  --------------------------*/



RBAPI(bool) rcservo_SetServoType(int channel, unsigned svtype, unsigned fbmethod) {
	if ((channel < 0) || (channel > 31))
	{
		err_SetMsg(ERROR_RCSERVO_WRONGCHANNEL, "unsupported channel");
		return false;
	}

	if (RCSERVO_svParams[channel].servoType == RCSERVO_SV_DISABLE)
		rcservo_SetServo(channel, RCSERVO_SERVO_DEFAULT);

	RCSERVO_svParams[channel].servoType = svtype;
	RCSERVO_svParams[channel].fbReadMode = fbmethod;
	return true;
}

RBAPI(bool) rcservo_SetServoParams1(int channel, unsigned long period, unsigned long minduty, unsigned long maxduty) {
	if ((channel < 0) || (channel > 31))
	{
		err_SetMsg(ERROR_RCSERVO_WRONGCHANNEL, "unsupported channel");
		return false;
	}

    if ((period == 0L)  || (period > 1000000L) ||
		(minduty == 0L) || (minduty >= period) ||
		(maxduty == 0L) || (maxduty >= period))
    {
		err_SetMsg(ERROR_RCSERVO_INVALIDPARAMS, "invalid parameters");
		return false;
    }
	
	if (RCSERVO_svParams[channel].servoType == RCSERVO_SV_DISABLE)
		rcservo_SetServo(channel, RCSERVO_SERVO_DEFAULT);

	RCSERVO_svParams[channel].period  = period;
	RCSERVO_svParams[channel].minDuty = minduty;
	RCSERVO_svParams[channel].maxDuty = maxduty;
	return true;
}

RBAPI(bool) rcservo_SetServoParams2(int channel, unsigned long invalidduty, unsigned long minlowphase) {
	if (RCSERVO_inUse == true)
	{
		err_SetMsg(ERROR_RCSERVO_INUSE, "can only set the parameters before initializing RCSERVO lib");
		return false;
	}

	if ((channel < 0) || (channel > 31))
	{
		err_SetMsg(ERROR_RCSERVO_WRONGCHANNEL, "unsupported channel");
		return false;
	}

    if ((invalidduty == 0L) || (invalidduty > 1000000L) ||
		(minlowphase == 0L) || (minlowphase > 1000000L))
    {
		err_SetMsg(ERROR_RCSERVO_INVALIDPARAMS, "invalid parameters");
		return false;
    }

	if (RCSERVO_svParams[channel].servoType == RCSERVO_SV_DISABLE)
		rcservo_SetServo(channel, RCSERVO_SERVO_DEFAULT);

	RCSERVO_svParams[channel].invalidDuty = invalidduty;
	RCSERVO_svParams[channel].minLowPhase = minlowphase;
	return true;
}

RBAPI(bool) rcservo_SetReadFBParams1(int channel, unsigned long initdelay, unsigned long lastdelay, unsigned long maxwidth) {
	if (RCSERVO_inUse == true)
	{
		err_SetMsg(ERROR_RCSERVO_INUSE, "can only set the parameters before initializing RCSERVO lib");
		return false;
	}

	if ((channel < 0) || (channel > 31))
	{
		err_SetMsg(ERROR_RCSERVO_WRONGCHANNEL, "unsupported channel");
		return false;
	}

    if ((initdelay == 0L) || (initdelay > 100000L) ||
		(lastdelay == 0L) || (lastdelay > 100000L) ||
		(maxwidth == 0L)  || (maxwidth > 1000000L))
    {
		err_SetMsg(ERROR_RCSERVO_INVALIDPARAMS, "invalid parameters");
		return false;
    }

	if (RCSERVO_svParams[channel].servoType == RCSERVO_SV_DISABLE)
		rcservo_SetServo(channel, RCSERVO_SERVO_DEFAULT);

	RCSERVO_svParams[channel].capInitDelay = initdelay;
	RCSERVO_svParams[channel].capLastDelay = lastdelay;
	RCSERVO_svParams[channel].capMaxWidth  = maxwidth;
	return true;
}

RBAPI(bool) rcservo_SetReadFBParams2(int channel, unsigned long resolution, long offset) {
	if (RCSERVO_inUse == true)
	{
		err_SetMsg(ERROR_RCSERVO_INUSE, "can only set the parameters before initializing RCSERVO lib");
		return false;
	}

	if ((channel < 0) || (channel > 31))
	{
		err_SetMsg(ERROR_RCSERVO_WRONGCHANNEL, "unsupported channel");
		return false;
	}

    if ((resolution == 0L) || (resolution > 1000L) || (offset > 100000L))
    {
		err_SetMsg(ERROR_RCSERVO_INVALIDPARAMS, "invalid parameters");
		return false;
    }

	if (RCSERVO_svParams[channel].servoType == RCSERVO_SV_DISABLE)
		rcservo_SetServo(channel, RCSERVO_SERVO_DEFAULT);

	RCSERVO_svParams[channel].capResolution  = resolution;
	RCSERVO_svParams[channel].capWidthOffset = offset;
	return true;
}

RBAPI(bool) rcservo_SetReadFBParams3(int channel, int maxfails, int filterwidth) {
	if ((channel < 0) || (channel > 31))
	{
		err_SetMsg(ERROR_RCSERVO_WRONGCHANNEL, "unsupported channel");
		return false;
	}

    if ((maxfails < 1)    || (maxfails > 1000) ||
		(filterwidth < 1) || (filterwidth > MAX_FBFILTERWIDTH))
    {
		err_SetMsg(ERROR_RCSERVO_INVALIDPARAMS, "invalid parameters");
		return false;
    }

	if (RCSERVO_svParams[channel].servoType == RCSERVO_SV_DISABLE)
		rcservo_SetServo(channel, RCSERVO_SERVO_DEFAULT);

	RCSERVO_svParams[channel].fbFilterWidth = filterwidth;
	RCSERVO_svParams[channel].fbMaxNumFail  = maxfails;
	return true;
}

RBAPI(bool) rcservo_SetCmdPulse(int channel, int cmd, unsigned long duty) {
	if ((channel < 0) || (channel > 31))
	{
		err_SetMsg(ERROR_RCSERVO_WRONGCHANNEL, "unsupported channel");
		return false;
	}

    if ((cmd < 0) || (cmd > 7) || (duty == 0L) || (duty > 1000000L))
    {
		err_SetMsg(ERROR_RCSERVO_INVALIDPARAMS, "invalid parameters");
		return false;
    }

	if (RCSERVO_svParams[channel].servoType == RCSERVO_SV_DISABLE)
		rcservo_SetServo(channel, RCSERVO_SERVO_DEFAULT);

	RCSERVO_svParams[channel].cmdDuty[cmd] = duty;
	return true;
}

RBAPI(bool) rcservo_SetPlayModeCMD(int channel, int cmd) {
	if ((channel < 0) || (channel > 31))
	{
		err_SetMsg(ERROR_RCSERVO_WRONGCHANNEL, "unsupported channel");
		return false;
	}

    if ((cmd < 0) || (cmd > 7))
    {
		err_SetMsg(ERROR_RCSERVO_INVALIDPARAMS, "invalid parameters");
		return false;
    }

	if (RCSERVO_svParams[channel].servoType == RCSERVO_SV_DISABLE)
		rcservo_SetServo(channel, RCSERVO_SERVO_DEFAULT);

	RCSERVO_svParams[channel].cmdInPlayMode = cmd;
	return true;
}



RBAPI(bool) rcservo_SetServos(unsigned long channels, unsigned servono) {
    int i;
    
	for (i=0; i<32; i++)
	{
        if ((channels & (1L<<i)) == 0L) continue;
        if (rcservo_SetServo(i, servono) == false) return false;
    }
    
    return true;
}

#if defined     USE_WINIO
	#define ROBOARD_WOFFSET		(29L)     //=124L without external pull-high for KONDO KRS-78x
#elif defined   USE_PCIDEBUG
	#define ROBOARD_WOFFSET		(124L)    //PCIDEBUG seems to be slower than WINIO
#elif defined   USE_PHYMEM
	#define ROBOARD_WOFFSET		(145L)    //PHYMEM seems to be slower than WINIO
#elif defined   RB_MSVC_WINCE
	#define ROBOARD_WOFFSET		(27L)
#elif defined   RB_LINUX
	#define ROBOARD_WOFFSET		(27L)
#else //DOS
	#define ROBOARD_WOFFSET		(25L)
#endif

#define RCSERVO_KONDO_COMMON    (0x10)
#define RCSERVO_HITEC_COMMON    (0x20)

RBAPI(bool) rcservo_SetServo(int channel, unsigned servono) {
	if (RCSERVO_inUse == true)
	{
		err_SetMsg(ERROR_RCSERVO_INUSE, "can only set the servo type before initializing RCSERVO lib");
		return false;
	}

	if ((channel < 0) || (channel > 31))
	{
		err_SetMsg(ERROR_RCSERVO_WRONGCHANNEL, "unsupported channel");
		return false;
	}

	switch (servono)
	{
		case RCSERVO_SERVO_DEFAULT:

			RCSERVO_svParams[channel].servoType  = RCSERVO_SV_FEEDBACK;
			RCSERVO_svParams[channel].fbReadMode = RCSERVO_FB_SAFEMODE + RCSERVO_FB_DENOISE;

            rcservo_SetServoParams1(channel, 20000L, 350L, 4000L);   //period = 20ms, min duty = 350us, max duty = 4000us
            rcservo_SetServoParams2(channel, 5000L, 1000L);          //invalid duty = 5ms, min low-phase width  = 1000us

            rcservo_SetReadFBParams1(channel, 50L, 500L, 4500L);     //cap init delay = 50us (can't < 30us for KONDO's KRS motors), last delay = 500us, max width = 4500us
            rcservo_SetReadFBParams2(channel, 10L, ROBOARD_WOFFSET); //cap resolution = 1us, cap width offset = ROBOARD_WOFFSET
            rcservo_SetReadFBParams3(channel, 3, 3);                 //cap max fails = 3, noise filter width = 3

            rcservo_SetCmdPulse(channel, RCSERVO_CMD_POWEROFF, 50L);
            rcservo_SetCmdPulse(channel, RCSERVO_CMD1, 100L);
            rcservo_SetCmdPulse(channel, RCSERVO_CMD2, 150L);
            rcservo_SetCmdPulse(channel, RCSERVO_CMD3, 200L);
            rcservo_SetCmdPulse(channel, RCSERVO_CMD4, 250L);
            rcservo_SetCmdPulse(channel, RCSERVO_CMD5, 300L);
            rcservo_SetCmdPulse(channel, RCSERVO_CMD6, 50L);
            rcservo_SetCmdPulse(channel, RCSERVO_CMD7, 50L);
            
            rcservo_SetPlayModeCMD(channel, RCSERVO_CMD_POWEROFF);
			break;

		case RCSERVO_SERVO_DEFAULT_NOFB:

            rcservo_SetServo(channel, RCSERVO_SERVO_DEFAULT);
			RCSERVO_svParams[channel].servoType = RCSERVO_SV_NOFEEDBACK;
			break;

		case RCSERVO_KONDO_COMMON:  //common setting for KONDO's servos

			RCSERVO_svParams[channel].servoType  = RCSERVO_SV_FEEDBACK;
			RCSERVO_svParams[channel].fbReadMode = RCSERVO_FB_SAFEMODE;

            rcservo_SetServoParams1(channel, 10000L, 600L, 2400L);
            rcservo_SetServoParams2(channel, 5000L, 500L);

            rcservo_SetReadFBParams1(channel, 35L, 200L, 2800L);
            rcservo_SetReadFBParams2(channel, 10L, ROBOARD_WOFFSET);
            rcservo_SetReadFBParams3(channel, 3, 3);

            rcservo_SetCmdPulse(channel, RCSERVO_CMD_POWEROFF, 50L);
            rcservo_SetCmdPulse(channel, RCSERVO_CMD1, 100L);
            rcservo_SetCmdPulse(channel, RCSERVO_CMD2, 150L);
            rcservo_SetCmdPulse(channel, RCSERVO_CMD3, 200L);
            rcservo_SetCmdPulse(channel, RCSERVO_CMD4, 50L);
            rcservo_SetCmdPulse(channel, RCSERVO_CMD5, 50L);
            rcservo_SetCmdPulse(channel, RCSERVO_CMD6, 50L);
            rcservo_SetCmdPulse(channel, RCSERVO_CMD7, 50L);
            
            rcservo_SetPlayModeCMD(channel, RCSERVO_CMD1);
			break;

		case RCSERVO_KONDO_KRS786:
		case RCSERVO_KONDO_KRS788:
		case RCSERVO_KONDO_KRS78X:

            rcservo_SetServo(channel, RCSERVO_KONDO_COMMON);
            //rcservo_SetServoParams1(channel, 8000L, 700L, 2300L);
            rcservo_SetServoParams1(channel, 8000L, 500L, 2500L);
            rcservo_SetServoParams2(channel, 3000L, 200L);

            rcservo_SetReadFBParams1(channel, 35L, 150L, 2800L);
			break;

		case RCSERVO_KONDO_KRS4024:

            rcservo_SetServo(channel, RCSERVO_KONDO_COMMON);
            rcservo_SetServoParams1(channel, 8000L, 630L, 2380L);
            rcservo_SetServoParams2(channel, 3000L, 200L);

            rcservo_SetReadFBParams1(channel, 35L, 150L, 2800L);
			break;

		case RCSERVO_KONDO_KRS4014:

            rcservo_SetServo(channel, RCSERVO_KONDO_COMMON);
            //rcservo_SetServoParams1(channel, 8000L, 700L, 2300L);
            rcservo_SetServoParams1(channel, 8000L, 450L, 2500L);
            rcservo_SetServoParams2(channel, 3500L, 200L + 2800L);  // note that using the safe-mode feedback method, KRS-4014 will return a feedback pulse again 
                                                                    // after the dummy invalid pulse; so we extend the length of the minimum low phase 
                                                                    // to bypass the second feedback pulse
            //rcservo_SetServoParams2(channel, 3500L, 200L);

            rcservo_SetReadFBParams1(channel, 50L, 250L, 2800L);
			break;

		case RCSERVO_HITEC_COMMON:  //conservative setting for HITEC's servos

			RCSERVO_svParams[channel].servoType  = RCSERVO_SV_FEEDBACK;
			RCSERVO_svParams[channel].fbReadMode = RCSERVO_FB_SAFEMODE;

            rcservo_SetServoParams1(channel, 10000L, 500L, 2600L);
            rcservo_SetServoParams2(channel, 5000L, 500L);

            rcservo_SetReadFBParams1(channel, 25L, 250L, 2800L);
            rcservo_SetReadFBParams2(channel, 10L, ROBOARD_WOFFSET);
            rcservo_SetReadFBParams3(channel, 3, 3);

            rcservo_SetCmdPulse(channel, RCSERVO_CMD_POWEROFF, 50L);
            rcservo_SetCmdPulse(channel, RCSERVO_CMD1, 100L);
            rcservo_SetCmdPulse(channel, RCSERVO_CMD2, 150L);
            rcservo_SetCmdPulse(channel, RCSERVO_CMD3, 200L);
            rcservo_SetCmdPulse(channel, RCSERVO_CMD4, 50L);
            rcservo_SetCmdPulse(channel, RCSERVO_CMD5, 50L);
            rcservo_SetCmdPulse(channel, RCSERVO_CMD6, 50L);
            rcservo_SetCmdPulse(channel, RCSERVO_CMD7, 50L);

            rcservo_SetPlayModeCMD(channel, RCSERVO_CMD_POWEROFF);
			break;

		case RCSERVO_HITEC_HSR8498:

            rcservo_SetServo(channel, RCSERVO_HITEC_COMMON);
            rcservo_SetServoParams1(channel, 10000L, 550L, 2450L);
            rcservo_SetServoParams2(channel, 3500L, 200L);
			break;

		case RCSERVO_FUTABA_S3003:

            rcservo_SetServo(channel, RCSERVO_SERVO_DEFAULT_NOFB);
            rcservo_SetServoParams1(channel, 10000L, 450L, 2350L);
			break;

		case RCSERVO_SHAYYE_SYS214050:

            rcservo_SetServo(channel, RCSERVO_SERVO_DEFAULT_NOFB);
            rcservo_SetServoParams1(channel, 4000L, 600L, 2350L);
			break;

		case RCSERVO_TOWERPRO_MG995:
		case RCSERVO_TOWERPRO_MG996:

            rcservo_SetServo(channel, RCSERVO_SERVO_DEFAULT_NOFB);
            rcservo_SetServoParams1(channel, 3500L, 800L, 2200L);
			break;

		case RCSERVO_DMP_RS0263:

            rcservo_SetServo(channel, RCSERVO_SERVO_DEFAULT_NOFB);
            rcservo_SetServoParams1(channel, 4000L, 700L, 2280L);
			break;

		default:
			err_SetMsg(ERROR_RCSERVO_UNKNOWNSERVO, "can't recognize the servo type");
			return false;
	}//end switch (servono)
	
	return true;
}

