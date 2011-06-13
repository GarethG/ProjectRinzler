#ifndef __RCSERVO_H
#define __RCSERVO_H

#include "defines.h"


#ifdef __cplusplus
extern "C" {
#endif

RBAPI(bool) rcservo_InUse(void);

RBAPI(bool) rcservo_Init(unsigned long usedchannels);
//-- values for the "usedchannels" argument (note that different values can be added to use multiple channels)
     #define RCSERVO_USENOPIN                (0L)
     #define RCSERVO_USEPINS1                (1L)
     #define RCSERVO_USEPINS2                (1L<<1)
     #define RCSERVO_USEPINS3                (1L<<2)
     #define RCSERVO_USEPINS4                (1L<<3)
     #define RCSERVO_USEPINS5                (1L<<4)
     #define RCSERVO_USEPINS6                (1L<<5)
     #define RCSERVO_USEPINS7                (1L<<6)
     #define RCSERVO_USEPINS8                (1L<<7)
     #define RCSERVO_USEPINS9                (1L<<8)
     #define RCSERVO_USEPINS10               (1L<<9)
     #define RCSERVO_USEPINS11               (1L<<10)
     #define RCSERVO_USEPINS12               (1L<<11)
     #define RCSERVO_USEPINS13               (1L<<12)
     #define RCSERVO_USEPINS14               (1L<<13)
     #define RCSERVO_USEPINS15               (1L<<14)
     #define RCSERVO_USEPINS16               (1L<<15)
     #define RCSERVO_USEPINS17               (1L<<16)  // only for RoBoard RB-100/RB-100b1/RB-100b2
     #define RCSERVO_USEPINS18               (1L<<17)  // only for RoBoard RB-100/RB-100b1/RB-100b2
     #define RCSERVO_USEPINS19               (1L<<18)  // only for RoBoard RB-100/RB-100b1/RB-100b2
     #define RCSERVO_USEPINS20               (1L<<19)  // only for RoBoard RB-100/RB-100b1/RB-100b2
     #define RCSERVO_USEPINS21               (1L<<20)  // only for RoBoard RB-100/RB-100b1/RB-100b2
     #define RCSERVO_USEPINS22               (1L<<21)  // only for RoBoard RB-100/RB-100b1/RB-100b2
     #define RCSERVO_USEPINS23               (1L<<22)  // only for RoBoard RB-100/RB-100b1/RB-100b2
     #define RCSERVO_USEPINS24               (1L<<23)  // only for RoBoard RB-100/RB-100b1/RB-100b2
     #define RCSERVO_USEPINS25               (1L<<24)  // only for RoBoard RB-100b1
     #define RCSERVO_USEPINS26               (1L<<25)  // only for RoBoard RB-100b1
     #define RCSERVO_USEPINS27               (1L<<26)  // only for RoBoard RB-100b1
     #define RCSERVO_USEPINS28               (1L<<27)  // only for RoBoard RB-100b1
     #define RCSERVO_USEPINS29               (1L<<28)  // only for RoBoard RB-100b1
     #define RCSERVO_USEPINS30               (1L<<29)  // only for RoBoard RB-100b1
     #define RCSERVO_USEPINS31               (1L<<30)  // only for RoBoard RB-100b1
     #define RCSERVO_USEPINS32               (1L<<31)  // only for RoBoard RB-100b1
//-- if the above function returns false, roboio_GetErrCode() may return:
//   #define ERROR_RBVER_UNKNOWN             (ERR_NOERROR + 801) //need include <common.h>
//   #define ERROR_RBVER_UNMATCH             (ERR_NOERROR + 800) //need include <common.h>
//   #define ERROR_IOINITFAIL                (ERR_NOERROR + 100) //need include <io.h>
//   #define ERROR_IOSECTIONFULL             (ERR_NOERROR + 101) //need include <io.h>
//   #define ERROR_CPUUNSUPPORTED            (ERR_NOERROR + 102) //need include <io.h>
//   #define ERROR_PWM_INUSE                 (ERR_NOERROR + 300) //need include <pwm.h>
     #define ERROR_RCSERVO_INUSE             (ERR_NOERROR + 400)
     #define ERROR_RCSERVO_PWMINUSE          (ERR_NOERROR + 401)

RBAPI(void) rcservo_Close(void);


RBAPI(bool) rcservo_SetServo(int channel, unsigned servono);
//-- values for the "channel" argument
     #define RCSERVO_PINS1                   (0)
     #define RCSERVO_PINS2                   (1)
     #define RCSERVO_PINS3                   (2)
     #define RCSERVO_PINS4                   (3)
     #define RCSERVO_PINS5                   (4)
     #define RCSERVO_PINS6                   (5)
     #define RCSERVO_PINS7                   (6)
     #define RCSERVO_PINS8                   (7)
     #define RCSERVO_PINS9                   (8)
     #define RCSERVO_PINS10                  (9)
     #define RCSERVO_PINS11                  (10)
     #define RCSERVO_PINS12                  (11)
     #define RCSERVO_PINS13                  (12)
     #define RCSERVO_PINS14                  (13)
     #define RCSERVO_PINS15                  (14)
     #define RCSERVO_PINS16                  (15)
     #define RCSERVO_PINS17                  (16)  // only for RoBoard RB-100/RB-100b1/RB-100b2
     #define RCSERVO_PINS18                  (17)  // only for RoBoard RB-100/RB-100b1/RB-100b2
     #define RCSERVO_PINS19                  (18)  // only for RoBoard RB-100/RB-100b1/RB-100b2
     #define RCSERVO_PINS20                  (19)  // only for RoBoard RB-100/RB-100b1/RB-100b2
     #define RCSERVO_PINS21                  (20)  // only for RoBoard RB-100/RB-100b1/RB-100b2
     #define RCSERVO_PINS22                  (21)  // only for RoBoard RB-100/RB-100b1/RB-100b2
     #define RCSERVO_PINS23                  (22)  // only for RoBoard RB-100/RB-100b1/RB-100b2
     #define RCSERVO_PINS24                  (23)  // only for RoBoard RB-100/RB-100b1/RB-100b2
     #define RCSERVO_PINS25                  (24)  // only for RoBoard RB-100b1
     #define RCSERVO_PINS26                  (25)  // only for RoBoard RB-100b1
     #define RCSERVO_PINS27                  (26)  // only for RoBoard RB-100b1
     #define RCSERVO_PINS28                  (27)  // only for RoBoard RB-100b1
     #define RCSERVO_PINS29                  (28)  // only for RoBoard RB-100b1
     #define RCSERVO_PINS30                  (29)  // only for RoBoard RB-100b1
     #define RCSERVO_PINS31                  (30)  // only for RoBoard RB-100b1
     #define RCSERVO_PINS32                  (31)  // only for RoBoard RB-100b1
RBAPI(bool) rcservo_SetServos(unsigned long channels, unsigned servono);
//-- values for the "channels" argument (note that different values can be added to use multiple channels)
//   #define RCSERVO_USEPINS1                (1L)
//   #define RCSERVO_USEPINS2                (1L<<1)
//   ...... (refer to the "usedchannels" argument of rcservo_Init())
//-- values for the "servono" argument
     #define RCSERVO_SERVO_DEFAULT           (0x00)  //conservative setting for most servos with pulse feedback
     #define RCSERVO_SERVO_DEFAULT_NOFB      (0x01)  //conservative setting for most servos without pulse feedback
     #define RCSERVO_KONDO_KRS786            (0x11)
     #define RCSERVO_KONDO_KRS788            (0x12)
     #define RCSERVO_KONDO_KRS78X            (0x13)
     #define RCSERVO_KONDO_KRS4014           (0x14)  //not directly work on RB-100/RB-110 due to the unmatched PWM pull-up/-down resistors
     #define RCSERVO_KONDO_KRS4024           (0x15)
     #define RCSERVO_HITEC_HSR8498           (0x22)
     #define RCSERVO_FUTABA_S3003            (0x31)
     #define RCSERVO_SHAYYE_SYS214050        (0x41)
     #define RCSERVO_TOWERPRO_MG995          (0x51)
     #define RCSERVO_TOWERPRO_MG996          (0x52)
     #define RCSERVO_DMP_RS0263              (0x61)
//-- if the above two functions return false, roboio_GetErrCode() may return:
//   #define ERROR_RCSERVO_INUSE             (ERR_NOERROR + 400)
     #define ERROR_RCSERVO_UNKNOWNSERVO      (ERR_NOERROR + 461)
     #define ERROR_RCSERVO_WRONGCHANNEL      (ERR_NOERROR + 462)

RBAPI(bool) rcservo_SetServoType(int channel, unsigned svtype, unsigned fbmethod);
//-- values for the "channel" argument
//   #define RCSERVO_PINS1                   (0)
//   #define RCSERVO_PINS2                   (1)
//   ...... (refer to the "channel" argument of rcservo_SetServo())
//-- values for the "type" argument
     #define RCSERVO_SV_FEEDBACK             (1)     //servo with pulse feedback
     #define RCSERVO_SV_NOFEEDBACK           (2)     //servo without pulse feedback
//-- values for the "fbmethod" argument (note that different values can be added to use multiple settings)
     #define RCSERVO_FB_SAFEMODE             (0)
     #define RCSERVO_FB_FASTMODE             (1)     //much faster than safe mode but could cause servo's shake
     #define RCSERVO_FB_DENOISE              (1<<1)  //use denoise filter; more accurate but very slow
//-- if the above function returns false, roboio_GetErrCode() may return:
//   #define ERROR_RCSERVO_WRONGCHANNEL      (ERR_NOERROR + 462)

RBAPI(bool) rcservo_SetServoParams1(int channel, unsigned long period, unsigned long minduty, unsigned long maxduty);
RBAPI(bool) rcservo_SetServoParams2(int channel, unsigned long invalidduty, unsigned long minlowphase);
//-- values for the "channel" argument
//   #define RCSERVO_PINS1                   (0)
//   #define RCSERVO_PINS2                   (1)
//   ...... (refer to the "channel" argument of rcservo_SetServo())
//-- if the above functions return false, roboio_GetErrCode() may return:
//   #define ERROR_RCSERVO_INUSE             (ERR_NOERROR + 400)
//   #define ERROR_RCSERVO_WRONGCHANNEL      (ERR_NOERROR + 462)
     #define ERROR_RCSERVO_INVALIDPARAMS     (ERR_NOERROR + 463)

RBAPI(bool) rcservo_SetReadFBParams1(int channel, unsigned long initdelay, unsigned long lastdelay, unsigned long maxwidth);
RBAPI(bool) rcservo_SetReadFBParams2(int channel, unsigned long resolution, long offset);
RBAPI(bool) rcservo_SetReadFBParams3(int channel, int maxfails, int filterwidth);
//-- values for the "channel" argument
//   #define RCSERVO_PINS1                   (0)
//   #define RCSERVO_PINS2                   (1)
//   ...... (refer to the "channel" argument of rcservo_SetServo())
//-- if the above functions return false, roboio_GetErrCode() may return:
//   #define ERROR_RCSERVO_INUSE             (ERR_NOERROR + 400)
//   #define ERROR_RCSERVO_WRONGCHANNEL      (ERR_NOERROR + 462)
//   #define ERROR_RCSERVO_INVALIDPARAMS     (ERR_NOERROR + 463)

RBAPI(bool) rcservo_SetCmdPulse(int channel, int cmd, unsigned long duty);
RBAPI(bool) rcservo_SetPlayModeCMD(int channel, int cmd);
//-- values for the "channel" argument
//   #define RCSERVO_PINS1                   (0)
//   #define RCSERVO_PINS2                   (1)
//   ...... (refer to the "channel" argument of rcservo_SetServo())
//-- values for the "cmd" argument
     #define RCSERVO_CMD_POWEROFF            (0)
     #define RCSERVO_CMD1                    (1)
     #define RCSERVO_CMD2                    (2)
     #define RCSERVO_CMD3                    (3)
     #define RCSERVO_CMD4                    (4)
     #define RCSERVO_CMD5                    (5)
     #define RCSERVO_CMD6                    (6)
     #define RCSERVO_CMD7                    (7)
//-- if the above function return false, roboio_GetErrCode() may return:
//   #define ERROR_RCSERVO_WRONGCHANNEL      (ERR_NOERROR + 462)
//   #define ERROR_RCSERVO_INVALIDPARAMS     (ERR_NOERROR + 463)



//******************  Functions in Capture Mode  *********************
RBAPI(void) rcservo_EnterCaptureMode(void);

RBAPI(unsigned long) rcservo_ReadPosition(int channel, int cmd);
RBAPI(unsigned long) rcservo_ReadPositionDN(int channel, int cmd);  // for backward compatibility to RoBoIO 1.1
//-- values for the "channel" argument
//   #define RCSERVO_PINS1                   (0)
//   #define RCSERVO_PINS2                   (1)
//   ...... (refer to the "channel" argument of rcservo_SetServo())
RBAPI(void) rcservo_ReadPositions(unsigned long channels, int cmd, unsigned long* width);
RBAPI(void) rcservo_ReadPositionsDN(unsigned long channels, int cmd, unsigned long* width);  // for backward compatibility to RoBoIO 1.1
//-- values for the "channels" argument (note that different values can be added to use multiple channels)
//   #define RCSERVO_USEPINS1                (1L)
//   #define RCSERVO_USEPINS2                (1L<<1)
//   ...... (refer to the "usedchannels" argument of rcservo_Init())
//-- values for the "cmd" argument
//   #define RCSERVO_CMD_POWEROFF            (0)
//   #define RCSERVO_CMD1                    (1)
//   ...... (refer to the "cmd" argument of rcservo_SetCmdPulse())
//-- each return value (= 0xffffffffL if fails) of the above functions indicates the length of the feedback pulse, 
//   which is equal to [return value] * [resolution] * 0.1us, where [resolution] is set by rcservo_SetReadFBParams2()

RBAPI(unsigned long) rcservo_CapOne(int channel);  // simplified version of rcservo_ReadPosition()
RBAPI(bool) rcservo_CapAll(unsigned long* width);  // simplified version of rcservo_ReadPositions()

//enable/disable multiprogramming OS heuristic
RBAPI(void) rcservo_EnableMPOS(void);
RBAPI(void) rcservo_DisableMPOS(void);

RBAPI(void) rcservo_SendCMD(int channel, int cmd);
//-- values for the "channels" argument (note that different values can be added to use multiple channels)
//   #define RCSERVO_USEPINS1                (1L)
//   #define RCSERVO_USEPINS2                (1L<<1)
//   ...... (refer to the "usedchannels" argument of rcservo_Init())
//-- values for the "cmd" argument
//   #define RCSERVO_CMD_POWEROFF            (0)
//   #define RCSERVO_CMD1                    (1)
//   ...... (refer to the "cmd" argument of rcservo_SetCmdPulse())



//******************  Functions in Play Mode  ************************
RBAPI(void) rcservo_EnterPlayMode(void);
RBAPI(void) rcservo_EnterPlayMode_NOFB(unsigned long* width);
RBAPI(void) rcservo_EnterPlayMode_HOME(unsigned long* width);

RBAPI(void) rcservo_SetFPS(int fps);
RBAPI(void) rcservo_SetAction(unsigned long* width, unsigned long playtime);

RBAPI(int)  rcservo_PlayAction(void);
RBAPI(int)  rcservo_PlayActionMix(long* mixwidth);
//-- return values of rcservo_PlayAction()
     #define RCSERVO_PLAYEND                 (0)
     #define RCSERVO_PLAYING                 (1)
     #define RCSERVO_PAUSED                  (2)
//-- special width values for mixwidth[i]
     #define RCSERVO_MIXWIDTH_POWEROFF       (0x7fffff00L)
     #define RCSERVO_MIXWIDTH_CMD1           (0x7fffff01L)
     #define RCSERVO_MIXWIDTH_CMD2           (0x7fffff02L)
     #define RCSERVO_MIXWIDTH_CMD3           (0x7fffff03L)
     #define RCSERVO_MIXWIDTH_CMD4           (0x7fffff04L)
     #define RCSERVO_MIXWIDTH_CMD5           (0x7fffff05L)
     #define RCSERVO_MIXWIDTH_CMD6           (0x7fffff06L)
     #define RCSERVO_MIXWIDTH_CMD7           (0x7fffff07L)

RBAPI(void) rcservo_PauseAction(void);
RBAPI(void) rcservo_ReleaseAction(void);
RBAPI(void) rcservo_StopAction(void);
RBAPI(void) rcservo_GetAction(unsigned long* width);

RBAPI(void) rcservo_MoveTo(unsigned long* width, unsigned long playtime);
RBAPI(void) rcservo_MoveOne(int channel, unsigned long pos, unsigned long playtime);



//******************  Functions in PWM Mode  *************************
RBAPI(void) rcservo_EnterPWMMode(void);

RBAPI(bool) rcservo_SendPWM(int channel, unsigned long period, unsigned long duty, unsigned long count);
RBAPI(bool) rcservo_SendCPWM(int channel, unsigned long period, unsigned long duty);
//-- values for the "channel" argument
//   #define RCSERVO_PINS1                   (0)
//   #define RCSERVO_PINS2                   (1)
//   ...... (refer to the "channel" argument of rcservo_SetServo())
//-- if the above function returns false, roboio_GetErrCode() may return:
     #define ERROR_RCSERVO_PWMFAIL           (ERR_NOERROR + 430)

RBAPI(void) rcservo_StopPWM(int channel);
//-- values for the "channel" argument
//   #define RCSERVO_PINS1                   (0)
//   #define RCSERVO_PINS2                   (1)
//   ...... (refer to the "channel" argument of rcservo_SetServo())

RBAPI(unsigned long) rcservo_CheckPWM(int channel);
//-- values for the "channel" argument
//   #define RCSERVO_PINS1                   (0)
//   #define RCSERVO_PINS2                   (1)
//   ...... (refer to the "channel" argument of rcservo_SetServo())
//-- returns the remaining number of pulses to send (returns 0xffffffffL for continuous pulses)



//*******************  Functions for GPIO Channels  ******************
RBAPI(void) rcservo_OutPin(int channel, int value);
RBAPI(int)  rcservo_InPin(int channel);
//-- values for the "channel" argument
//   #define RCSERVO_PINS1                   (0)
//   #define RCSERVO_PINS2                   (1)
//   ...... (refer to the "channel" argument of rcservo_SetServo())

RBAPI(void) rcservo_OutPort(unsigned long channels, unsigned long values);
RBAPI(unsigned long) rcservo_InPort(unsigned long channels);
//-- values for the "channels" argument (note that different values can be added to use multiple channels)
//   #define RCSERVO_USEPINS1                (1L)
//   #define RCSERVO_USEPINS2                (1L<<1)
//   ...... (refer to the "usedchannels" argument of rcservo_Init())



//***********  Constants for Compatibility to RoBoIO 1.61  ***********
#define RCSERVO_USENOCHANNEL	    (0L)
#define RCSERVO_USECHANNEL0			(1L)
#define RCSERVO_USECHANNEL1			(1L<<1)
#define RCSERVO_USECHANNEL2			(1L<<2)
#define RCSERVO_USECHANNEL3			(1L<<3)
#define RCSERVO_USECHANNEL4			(1L<<4)
#define RCSERVO_USECHANNEL5			(1L<<5)
#define RCSERVO_USECHANNEL6			(1L<<6)
#define RCSERVO_USECHANNEL7			(1L<<7)
#define RCSERVO_USECHANNEL8			(1L<<8)
#define RCSERVO_USECHANNEL9			(1L<<9)
#define RCSERVO_USECHANNEL10		(1L<<10)
#define RCSERVO_USECHANNEL11		(1L<<11)
#define RCSERVO_USECHANNEL12		(1L<<12)
#define RCSERVO_USECHANNEL13		(1L<<13)
#define RCSERVO_USECHANNEL14		(1L<<14)
#define RCSERVO_USECHANNEL15		(1L<<15)
#define RCSERVO_USECHANNEL16		(1L<<16)  // only for RoBoard RB-100/RB-100b1/RB-100b2
#define RCSERVO_USECHANNEL17		(1L<<17)  // only for RoBoard RB-100/RB-100b1/RB-100b2
#define RCSERVO_USECHANNEL18		(1L<<18)  // only for RoBoard RB-100/RB-100b1/RB-100b2
#define RCSERVO_USECHANNEL19		(1L<<19)  // only for RoBoard RB-100/RB-100b1/RB-100b2
#define RCSERVO_USECHANNEL20		(1L<<20)  // only for RoBoard RB-100/RB-100b1/RB-100b2
#define RCSERVO_USECHANNEL21		(1L<<21)  // only for RoBoard RB-100/RB-100b1/RB-100b2
#define RCSERVO_USECHANNEL22		(1L<<22)  // only for RoBoard RB-100/RB-100b1/RB-100b2
#define RCSERVO_USECHANNEL23		(1L<<23)  // only for RoBoard RB-100/RB-100b1/RB-100b2
#define RCSERVO_USECHANNEL24		(1L<<24)  // only for RoBoard RB-100b1
#define RCSERVO_USECHANNEL25		(1L<<25)  // only for RoBoard RB-100b1
#define RCSERVO_USECHANNEL26		(1L<<26)  // only for RoBoard RB-100b1
#define RCSERVO_USECHANNEL27		(1L<<27)  // only for RoBoard RB-100b1
#define RCSERVO_USECHANNEL28		(1L<<28)  // only for RoBoard RB-100b1
#define RCSERVO_USECHANNEL29		(1L<<29)  // only for RoBoard RB-100b1
#define RCSERVO_USECHANNEL30		(1L<<30)  // only for RoBoard RB-100b1
#define RCSERVO_USECHANNEL31		(1L<<31)  // only for RoBoard RB-100b1

RBAPI(bool) rcservo_Initialize(unsigned long usedchannels);  // for compatibility to RoBoIO 1.61


#ifdef __cplusplus
}
#endif



/****************************  Inline Functions  **************************/
#ifdef ROBOIO_DLL //use no inline functions for DLL
#ifdef __cplusplus
extern "C" {
#endif
    // for compatibility to RoBoIO 1.61
    RBAPI(void) rcservo_MoveToMix(unsigned long* width, unsigned long playtime, long* mixwidth);
    RBAPI(bool) rcservo_SendPWMPulses(int channel, unsigned long period, unsigned long duty, unsigned long count);
    RBAPI(bool) rcservo_IsPWMCompleted(int channel);
    RBAPI(void) rcservo_Outp(int channel, int value);
    RBAPI(int) rcservo_Inp(int channel);
    RBAPI(void) rcservo_Outps(unsigned long channels, unsigned long value);
    RBAPI(unsigned long) rcservo_Inps(unsigned long channels);
#ifdef __cplusplus
}
#endif
#endif

#if !defined(ROBOIO_DLL) || defined(__RCSERVO_LIB)
    // for compatibility to RoBoIO 1.61
    RB_INLINE RBAPI(void) rcservo_MoveToMix(unsigned long* width, unsigned long playtime, long* mixwidth) {
        rcservo_SetAction(width, playtime);
        while (rcservo_PlayActionMix(mixwidth) != RCSERVO_PLAYEND);
    }
    RB_INLINE RBAPI(bool) rcservo_SendPWMPulses(int channel, unsigned long period, unsigned long duty, unsigned long count) {
        return rcservo_SendPWM(channel, period, duty, count);
    }
    RB_INLINE RBAPI(bool) rcservo_IsPWMCompleted(int channel) {
        if (rcservo_CheckPWM(channel) == 0L) return true; else return false;
    }
    RB_INLINE RBAPI(void) rcservo_Outp(int channel, int value) {
        rcservo_OutPin(channel, value);
    }
    RB_INLINE RBAPI(int) rcservo_Inp(int channel) {
        return rcservo_InPin(channel);
    }
    RB_INLINE RBAPI(void) rcservo_Outps(unsigned long channels, unsigned long value) {
        rcservo_OutPort(channels, value);
    }
    RB_INLINE RBAPI(unsigned long) rcservo_Inps(unsigned long channels) {
        return rcservo_InPort(channels);
    }
#endif
/*-----------------------  end of Inline Functions  ----------------------*/

#endif

