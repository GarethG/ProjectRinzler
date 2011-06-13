#define __PWMDX_LIB

#define  USE_COMMON
#include "io.h"
#include "pwmdx.h"


static unsigned BaseAddress = 0xfe00;

#define SB_PWMSETTING_REG   (0xd0)
#define SB_GPIOFUNC_REG     (0xc8)
#define SB_MULTIFUNC_REG    (0xc0)

/*************** South Bridge PWM Setting REG: 0xD0~0xD3 *************
 *    bit 31-25: =0
 *    bit 24:    =0 10MHz; =1 50MHz
 *    bit 23:    =1 enable I/O address decoding
 *    bit 20-22: =0
 *    bit 16-19: IRQ routing 
 *               B19 B18 B17 B16   Routing Table
 *                 0   0   0   0   Disable.
 *                 0   0   0   1   IRQ[9]
 *                 0   0   1   0   IRQ[3] 
 *                 0   0   1   1   IRQ[10]
 *                 0   1   0   0   IRQ[4]
 *                 0   1   0   1   IRQ[5]
 *                 0   1   1   0   IRQ[7]
 *                 0   1   1   1   IRQ[6]
 *                 1   0   0   0   IRQ[1]
 *                 1   0   0   1   IRQ[11]
 *                 1   0   1   0   Reserved
 *                 1   0   1   1   IRQ[12]
 *                 1   1   0   0   Reserved
 *                 1   1   0   1   IRQ[14]
 *                 1   1   1   0   Reserved
 *                 1   1   1   1   IRQ[15]
 *    bit 15-8:  base address bits 15~8
 *    bit 7-0:   =0
 *********************************************************************/
 
RBAPI(void) pwm_SetBaseAddress(unsigned baseaddr) {
    write_sb_reg(SB_PWMSETTING_REG,  //set base address with enabling I/O address decoding
                 ((read_sb_reg(SB_PWMSETTING_REG) & 0xff7f0000L) | 0x00800000L) + (unsigned long)(baseaddr & 0xfe00) );
    BaseAddress = baseaddr;
}

RBAPI(unsigned) pwm_SetDefaultBaseAddress(void) {
    unsigned long reg = read_sb_reg(SB_PWMSETTING_REG);
    
    if ((reg & 0x00800000L) == 0L) return 0x0000;

    BaseAddress = (unsigned)(reg & 0xfe00L);
    return BaseAddress;
}

RBAPI(void) pwm_SelectClock(int clock) {
    write_sb_reg(SB_PWMSETTING_REG,
                 (read_sb_reg(SB_PWMSETTING_REG) & 0xfeffffffL) + ((unsigned long)(clock & 1) << 24) );
}

RBAPI(void) pwm_SetIRQ(int pwmirq) {
    write_sb_reg(SB_PWMSETTING_REG,
                 (read_sb_reg(SB_PWMSETTING_REG) & 0xfff0ffffL) + ((unsigned long)(pwmirq & 0x000f) << 16) );
}

RBAPI(void) pwm_SetPWMSettingREG(unsigned long psreg) {
    write_sb_reg(SB_PWMSETTING_REG, psreg);
    BaseAddress = (unsigned)(psreg & 0x0000fe00L);
}

RBAPI(unsigned long) pwm_ReadPWMSettingREG(void) {
    return read_sb_reg(SB_PWMSETTING_REG);
}


//ugly code below is just for fun:p
static int GPIO4_FUNC = 0;
static unsigned long old_GPIO4PWM;

static void enable_GPIO4PWM(void) {
    if (GPIO4_FUNC != 0) return;

    old_GPIO4PWM = read_sb_reg(SB_MULTIFUNC_REG);
    write_sb_reg(SB_MULTIFUNC_REG, old_GPIO4PWM | 2L); // disable GPIO port 4's COM function
    old_GPIO4PWM = old_GPIO4PWM & 2L;
    GPIO4_FUNC = 1;
}
static void restore_GPIO4PWM(void) {
    if (GPIO4_FUNC != 1) return;

    write_sb_reg(SB_MULTIFUNC_REG, (read_sb_reg(SB_MULTIFUNC_REG) & (~2L)) + old_GPIO4PWM);
    GPIO4_FUNC = 0;
}

RBAPI(void) pwmdx_EnablePin(int channel) {
    if (channel >= 24) enable_GPIO4PWM();
    write_sb_reg(SB_GPIOFUNC_REG, read_sb_reg(SB_GPIOFUNC_REG) | (1L << channel) );
}

RBAPI(void) pwmdx_DisablePin(int channel) {
    unsigned long a;

    a = read_sb_reg(SB_GPIOFUNC_REG) & (~(1L << channel));
    write_sb_reg(SB_GPIOFUNC_REG, a);
    if ((channel >= 24) && ((a & 0xff000000L) == 0L)) restore_GPIO4PWM();
}

//bit i of channels = 1 ==> enable channel i PWM pin
RBAPI(void) pwmdx_EnableMultiPin(unsigned long channels) {
    if ((channels & 0xff000000L) != 0L) enable_GPIO4PWM();
    write_sb_reg(SB_GPIOFUNC_REG, read_sb_reg(SB_GPIOFUNC_REG) | channels);
}

//bit i of channels = 1 ==> disable channel i PWM pin
RBAPI(void) pwmdx_DisableMultiPin(unsigned long channels) {
    unsigned long a;

    a = read_sb_reg(SB_GPIOFUNC_REG) & (~channels);
    write_sb_reg(SB_GPIOFUNC_REG, a);

    if (((channels & 0xff000000L) != 0L) && ((a & 0xff000000L) == 0L)) restore_GPIO4PWM();
}



#define PWM_SYNCREG         (BaseAddress + 0x08)

RBAPI(void) pwmdx_Lock(int channel) {
    io_outpdw(PWM_SYNCREG, io_inpdw(PWM_SYNCREG) | (1L << channel) );
}

RBAPI(void) pwmdx_Unlock(int channel) {
    io_outpdw(PWM_SYNCREG, io_inpdw(PWM_SYNCREG) & (~(1L << channel)) );
}

//bit i of channels = 1 ==> lock PWM channel i
RBAPI(void) pwmdx_LockMulti(unsigned long channels) {
    io_outpdw(PWM_SYNCREG, io_inpdw(PWM_SYNCREG) | channels );
}

//bit i of channels = 1 ==> unlock PWM channel i
RBAPI(void) pwmdx_UnlockMulti(unsigned long channels) {
    io_outpdw(PWM_SYNCREG, io_inpdw(PWM_SYNCREG) & (~channels) );
}

RBAPI(void) pwm_SetSyncREG(unsigned long sreg) {
    io_outpdw(PWM_SYNCREG, sreg);
}

RBAPI(unsigned long) pwm_ReadSyncREG(void) {
    return io_inpdw(PWM_SYNCREG);
}


#define PWM_INTMASKREG      (BaseAddress + 0x00)

RBAPI(void) pwmdx_EnableINT(int channel) {
    io_outpdw(PWM_INTMASKREG, io_inpdw(PWM_INTMASKREG) | (1L << channel) );
}

RBAPI(void) pwmdx_DisableINT(int channel) {
    io_outpdw(PWM_INTMASKREG, io_inpdw(PWM_INTMASKREG) & (~(1L << channel)) );
}

//bit i of channels = 1 ==> enable PWM channel i interrupt
RBAPI(void) pwmdx_EnableMultiINT(unsigned long channels) {
    io_outpdw(PWM_INTMASKREG, io_inpdw(PWM_INTMASKREG) | channels );
}

//bit i of channels = 1 ==> disable PWM channel i interrupt
RBAPI(void) pwmdx_DisableMultiINT(unsigned long channels) {
    io_outpdw(PWM_INTMASKREG, io_inpdw(PWM_INTMASKREG) & (~channels) );
}

RBAPI(void) pwm_SetINTREG(unsigned long ireg) {
    io_outpdw(PWM_INTMASKREG, ireg);
}

RBAPI(unsigned long) pwm_ReadINTREG(void) {
    return io_inpdw(PWM_INTMASKREG);
}


#define PWM_INTFLAGREG      (BaseAddress + 0x04)

RBAPI(void) pwmdx_ClearMultiFLAG(unsigned long channels) {
    io_outpdw(PWM_INTFLAGREG, (~io_inpdw(PWM_INTFLAGREG)) | channels );
}

RBAPI(unsigned long) pwmdx_ReadMultiFLAG(unsigned long channels) {
    return (io_inpdw(PWM_INTFLAGREG) & channels);
}


#define PWM_LREG_BASEADDR   (BaseAddress + 0x0c)
#define PWM_HREG_BASEADDR   (BaseAddress + 0x10)
#define PWM_CTRL_BASEADDR   (BaseAddress + 0x14)

/************************ PWM Control REG *****************************
 *    bit 31:   =1 enable PWM
 *    bit 30:   =0 pulse counting mode; =1 continue mode
 *    bit 29:   =0 waveform 1->0; =1 waveform 0->1
 *    bit 28:   reserved
 *    bit 27-0: pulse count
 *********************************************************************/

RBAPI(void) pwmdx_EnablePWM(int channel) {
    unsigned ctrl_addr;
    
    ctrl_addr = PWM_CTRL_BASEADDR + channel*12;
    io_outpdw(ctrl_addr, io_inpdw(ctrl_addr) | 0x80000000L);
}

RBAPI(void) pwmdx_DisablePWM(int channel) {
    unsigned ctrl_addr;
    
    ctrl_addr = PWM_CTRL_BASEADDR + channel*12;
    io_outpdw(ctrl_addr, io_inpdw(ctrl_addr) & 0x7fffffffL);
}

//bit i of channels = 1 ==> enable PWM channel i
RBAPI(void) pwmdx_EnableMultiPWM(unsigned long channels) {
    int i;
    
    for (i=0; i<32; i++)
        if ((channels & (1L << i)) != 0) pwmdx_EnablePWM(i);
}

//bit i of channels = 1 ==> disable PWM channel i
RBAPI(void) pwmdx_DisableMultiPWM(unsigned long channels) {
    int i;

    for (i=0; i<32; i++)
        if ((channels & (1L << i)) != 0) pwmdx_DisablePWM(i);
}

RBAPI(void) pwmdx_SetCountingMode(int channel, int mode) {
    unsigned ctrl_addr;
    
    ctrl_addr = PWM_CTRL_BASEADDR + channel*12;
    io_outpdw(ctrl_addr, (io_inpdw(ctrl_addr) & 0xbfffffffL) + ((unsigned long)mode << 30) );
}

RBAPI(int) pwmdx_ReadCountingMode(int channel) {
    if ((io_inpdw(PWM_CTRL_BASEADDR + channel*12) & 0x40000000L) == 0L) return PWM_COUNT_MODE;
    
    return PWM_CONTINUE_MODE;
}

RBAPI(void) pwmdx_SetWaveform(int channel, int mode) {
    unsigned ctrl_addr;
    
    ctrl_addr = PWM_CTRL_BASEADDR + channel*12;
    io_outpdw(ctrl_addr, (io_inpdw(ctrl_addr) & 0xdfffffffL) + ((unsigned long)mode << 29) );
}

RBAPI(void) pwmdx_SetPulseCount(int channel, unsigned long count) {
    unsigned ctrl_addr;
    
    ctrl_addr = PWM_CTRL_BASEADDR + channel*12;
    io_outpdw(ctrl_addr, (io_inpdw(ctrl_addr) & 0xf0000000L) + (count & 0x0fffffffL) );
}

RBAPI(void) pwmdx_SetCtrlREG(int channel, unsigned long creg) {
    io_outpdw(PWM_CTRL_BASEADDR + channel*12, creg);
}

RBAPI(void) pwmdx_SetHREG(int channel, unsigned long hreg) {
    io_outpdw(PWM_HREG_BASEADDR + channel*12, hreg);
}

RBAPI(void) pwmdx_SetLREG(int channel, unsigned long lreg) {
    io_outpdw(PWM_LREG_BASEADDR + channel*12, lreg);
}

RBAPI(unsigned long) pwmdx_ReadPulseCount(int channel) {
    return (io_inpdw(PWM_CTRL_BASEADDR + channel*12) & 0x0fffffffL);
}

RBAPI(unsigned long) pwmdx_ReadCtrlREG(int channel) {
    return io_inpdw(PWM_CTRL_BASEADDR + channel*12);
}

RBAPI(unsigned long) pwmdx_ReadHREG(int channel) {
    return io_inpdw(PWM_HREG_BASEADDR + channel*12);
}

RBAPI(unsigned long) pwmdx_ReadLREG(int channel) {
    return io_inpdw(PWM_LREG_BASEADDR + channel*12);
}

