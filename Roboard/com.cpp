#define __COM_LIB

#include "defines.h"

#define  USE_COMMON
#include "common.h"
#include "io.h"
#include "com.h"


static int COM_ioSection = -1;
RBAPI(bool) com_InUse(void) {
    if (COM_ioSection == -1) return false; else return true;
}



/*************************  Temporary COM lib Functions  **********************/
static unsigned char COM_ctrlREG[4] = {0x53, 0xa0, 0xa4, 0xa8};

RBAPI(void) com_EnableTurboMode(int idx) {  // not work when COM lib is in use
    if ((com_InUse() == true) || (idx < 0) || (idx > 3) || (io_CpuID() != CPU_VORTEX86DX_3)) return;

    if (idx == 0)  // COM1
        write_sb_regb(COM_ctrlREG[idx], read_sb_regb(COM_ctrlREG[idx]) | ((unsigned char)1<<6));
    else           // COM2 ~ COM4
        write_sb_reg(COM_ctrlREG[idx], read_sb_reg(COM_ctrlREG[idx]) | (1L<<22));
}

RBAPI(void) com_DisableTurboMode(int idx) {  // not work when COM lib is in use
    if ((com_InUse() == true) || (idx < 0) || (idx > 3) || (io_CpuID() != CPU_VORTEX86DX_3)) return;

    if (idx == 0)  // COM1
        write_sb_regb(COM_ctrlREG[idx], read_sb_regb(COM_ctrlREG[idx]) & ~((unsigned char)1<<6));
    else           // COM2 ~ COM4
        write_sb_reg(COM_ctrlREG[idx], read_sb_reg(COM_ctrlREG[idx]) & ~(1L<<22));
}
/*--------------------  end of Temporary COM lib Functions  ------------------*/


