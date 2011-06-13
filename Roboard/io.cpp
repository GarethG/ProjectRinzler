#define __IO_LIB

#include "defines.h"

#define  USE_COMMON
#include "common.h"
#include "io.h"


#if defined     USE_WINIO
    #include <winio.h>
#elif defined   USE_PCIDEBUG
	#include <windows.h>
    #include <pcifunc.h>
#elif defined   USE_PHYMEM
	#include <windows.h>
    #include <pmdll.h>
#elif defined   RB_LINUX
    #include <sys/io.h>
#elif defined   RB_DJGPP
    #include <pc.h>
    #include <dos.h>
#elif defined   RB_WATCOM
    #include <conio.h>
    #include <i86.h>
#elif defined   RB_BC_DOS
    #include <conio.h>
    #include <dos.h>

    //386 instructions
    #define EMIT_DWORD(x) \
        __emit__((unsigned char)x);__emit__((unsigned char)(x>>8));\
        __emit__((unsigned char)(x>>16));__emit__((unsigned char)(x>>24))

    #define MOV_EAX(x) __emit__(0x66, 0xb8); EMIT_DWORD(x)

    #define MOV_VAR_EAX(x) __emit__(0x66); _asm mov WORD PTR x, ax
    #define MOV_VAR_EDX(x) __emit__(0x66); _asm mov WORD PTR x, dx

    #define MOV_EAX_VAR(x) __emit__(0x66); _asm mov ax, WORD PTR x

    #define OUT_EDX_EAX __emit__(0x66); _asm out dx, ax
    #define IN_EAX_EDX __emit__(0x66); _asm in ax, dx

    //#define RDTSC(x,y) __emit__(0x0f, 0x31); MOV_VAR_EAX(x); MOV_VAR_EDX(y)
#endif


static int IO_edIrqCnt = 0;
RBAPI(void) io_EnableIRQ(void) {
    if (IO_edIrqCnt > 0) IO_edIrqCnt--;
    if (IO_edIrqCnt == 0)
    {
        #if defined(RB_BC_DOS) || defined(RB_DJGPP)
            enable();
        #elif defined(RB_WATCOM)
            _enable();
        #elif defined(RB_LINUX)
            asm("sti");  // only work on Vortex86SX/DX/...
        #endif
    }
}

RBAPI(void) io_DisableIRQ(void) {
    if (IO_edIrqCnt == 0)
    {
        #if defined(RB_BC_DOS) || defined(RB_DJGPP)
            disable();
        #elif defined(RB_WATCOM)
            _disable();
        #elif defined(RB_LINUX)
            asm("cli");  // only work on Vortex86SX/DX/...
        #endif
    }
    IO_edIrqCnt++;
}


static bool IO_inUse = false;
RBAPI(bool) io_InUse(void) {
    return IO_inUse;
}

#define IO_MAXSECTIONS  (128)
static bool IO_sections[IO_MAXSECTIONS] = {false};
RBAPI(int) io_Init(void) {
    int i;
    
    if (IO_inUse == true)
    {
        for (i=0; i<IO_MAXSECTIONS; i++)
            if (IO_sections[i] == false)
            {
                IO_sections[i] = true;
                return i;
            }
        err_SetMsg(ERROR_IOSECTIONFULL, "no free I/O section is available");
        return -1;
    }

	#if defined		USE_WINIO
        if (!InitializeWinIo())
        {
			err_SetMsg(ERROR_IOINITFAIL, "I/O library fails to initialize");
            return -1;
        }
    #elif defined	USE_PCIDEBUG
        if (getdllstatus() != DLLSTATUS_NOERROR)
        {
            err_SetMsg(ERROR_IOINITFAIL, "I/O library fails to initialize");
            return -1;
        }
    #elif defined   USE_PHYMEM
        if (LoadPhyMemDriver() == FALSE)
        {
            err_SetMsg(ERROR_IOINITFAIL, "I/O library fails to initialize");
            return -1;
        }
    #elif defined	RB_LINUX
        if (iopl(3) != 0)
		{
            err_SetMsg(ERROR_IOINITFAIL, "I/O library fails to initialize");
            return -1;
        }
    #endif

	IO_sections[0] = true;
	IO_inUse = true;

	if ((io_CpuID() != CPU_VORTEX86DX_1) && (io_CpuID() != CPU_VORTEX86DX_2) && (io_CpuID() != CPU_VORTEX86DX_3))
	{
		io_Close(0);
		err_SetMsg(ERROR_CPUUNSUPPORTED, "unrecognized CPU");
        return -1;
	}

    #ifdef ROBOIO
        if (roboio_CheckRBVer() == false)
        {
            io_Close(0);
            return -1;
        }
    #endif

	return 0;
}

RBAPI(void) io_Close(int section) {
    int i;
    
    if ((IO_inUse == false) || (section < 0) || (section >= IO_MAXSECTIONS)) return;

    IO_sections[section] = false;
    for (i=0; i<IO_MAXSECTIONS; i++) if (IO_sections[i] == true) return;

	#if defined		USE_WINIO
        ShutdownWinIo();
    #elif defined   USE_PHYMEM
        UnloadPhyMemDriver();
    #endif

    IO_inUse = false;
}


//the following functions are for backward compatibility to RoBoIO library 1.0
static int IO_ioSection = -1;
RBAPI(bool) io_init(void) {
    if (IO_ioSection != -1) return true;

    if ((IO_ioSection = io_Init()) != -1) return true;
    
    return false;
}

RBAPI(void) io_close(void) {
    if (IO_ioSection == -1) return;

    io_Close(IO_ioSection);
    IO_ioSection = -1;
}


//outport: double word
RBAPI(void) io_outpdw(unsigned short addr, unsigned long val) {
    #if   defined   USE_WINIO
	    SetPortVal(addr, val, 4);
    #elif defined	USE_PCIDEBUG
		_IoWriteLong((ULONG)addr, val);
    #elif defined   USE_PHYMEM
        WritePortLong(addr, val);
    #elif defined	RB_MSVC_WINCE
        _asm {
            mov dx, WORD PTR addr
            mov eax, val
            out dx, eax
        }
    #elif defined	RB_LINUX
        outl(val, addr);
    #elif defined   RB_DJGPP
        outportl(addr, val);
    #elif defined   RB_WATCOM
        outpd(addr, val);
    #elif defined   RB_BC_DOS
        _asm mov dx, WORD PTR addr
        MOV_EAX_VAR(val);
        OUT_EDX_EAX;
    #endif
}

//inport: double word
RBAPI(unsigned long) io_inpdw(unsigned short addr) {
    #if   defined   USE_WINIO
        unsigned long val;

	    GetPortVal(addr, &val, 4);
	    return val;
    #elif defined	USE_PCIDEBUG
		return _IoReadLong((ULONG)addr);
    #elif defined   USE_PHYMEM
        return ReadPortLong(addr);
    #elif defined	RB_MSVC_WINCE
        unsigned long val = 0L;

        _asm {
            mov dx, WORD PTR addr
            in eax, dx
            mov val, eax
        }
        return val;
    #elif defined	RB_LINUX
        return inl(addr);
	#elif defined   RB_DJGPP
        return inportl(addr);
    #elif defined   RB_WATCOM
        return inpd(addr);
    #elif defined   RB_BC_DOS
        unsigned long retval;

        _asm mov dx, WORD PTR addr
        IN_EAX_EDX;
        MOV_VAR_EAX(retval);

        return retval;
    #endif
}


//outport: word
RBAPI(void) io_outpw(unsigned short addr, unsigned short val) {
    #if   defined   USE_WINIO
	    SetPortVal(addr, val, 2);
    #elif defined	USE_PCIDEBUG
		_IoWriteShort((ULONG)addr, val);
    #elif defined   USE_PHYMEM
        WritePortWord(addr, val);
    #elif defined	RB_MSVC_WINCE
        _asm {
            mov dx, WORD PTR addr
            mov ax, WORD PTR val
            out dx, ax
        }
    #elif defined	RB_LINUX
        outw(val, addr);
    #elif defined   RB_DJGPP
        outportw(addr, val);
    #elif defined   RB_WATCOM
        outpw(addr, val);
    #elif defined   RB_BC_DOS
        outpw(addr, val);
    #endif
}

//inport: word
RBAPI(unsigned short) io_inpw(unsigned short addr) {
    #if   defined   USE_WINIO
        unsigned long val;

	    GetPortVal(addr, &val, 2);
	    return (unsigned short)val;
    #elif defined	USE_PCIDEBUG
		return _IoReadShort((ULONG)addr);
    #elif defined   USE_PHYMEM
        return ReadPortWord(addr);
    #elif defined	RB_MSVC_WINCE
        unsigned val = 0;

        _asm {
            mov dx, WORD PTR addr
            in ax, dx
            mov WORD PTR val, ax
        }
        return val;
    #elif defined	RB_LINUX
        return inw(addr);
    #elif defined   RB_DJGPP
        return inportw(addr);
    #elif defined   RB_WATCOM
        return (unsigned short)inpw(addr);
    #elif defined   RB_BC_DOS
        return inpw(addr);
    #endif
}


//outport: byte
RBAPI(void) io_outpb(unsigned short addr, unsigned char val) {
    #if   defined   USE_WINIO
	    SetPortVal(addr, val, 1);
    #elif defined	USE_PCIDEBUG
		_IoWriteChar((ULONG)addr, val);
    #elif defined   USE_PHYMEM
        WritePortByte(addr, val);
    #elif defined	RB_MSVC_WINCE
        _asm {
            mov dx, WORD PTR addr
            mov al, val
            out dx, al
        }
    #elif defined	RB_LINUX
        outb(val, addr);
    #elif defined   RB_DJGPP
        outportb(addr, val);
    #elif defined   RB_WATCOM
        outp(addr, val);
    #elif defined   RB_BC_DOS
        outp(addr, val);
    #endif
}

//inport: byte
RBAPI(unsigned char) io_inpb(unsigned short addr) {
    #if   defined   USE_WINIO
        unsigned long val;

	    GetPortVal(addr, &val, 1);
	    return (unsigned char)val;
    #elif defined	USE_PCIDEBUG
		return _IoReadChar((ULONG)addr);
    #elif defined   USE_PHYMEM
        return ReadPortByte(addr);
    #elif defined	RB_MSVC_WINCE
        unsigned char val = 0;

        _asm {
            mov dx, WORD PTR addr
            in al, dx
            mov val, al
        }
        return val;
    #elif defined	RB_LINUX
        return inb(addr);
    #elif defined   RB_DJGPP
        return inportb(addr);
    #elif defined   RB_WATCOM
        return (unsigned char)inp(addr);
    #elif defined   RB_BC_DOS
        return inp(addr);
    #endif
}



#define SB_MULTIFUNC_REG      (0xc0)
/************** South Bridge Multifunction Pin Control REG: 0xC0~0xC3 ************
 *    To change the ISA clock source, the user should set ISACLK (bit 17 of 
 *    SB_MULTIFUNC_REG). (ISACLK = 1: PCI clock/4; PINS0 = 1: PCI clock/2)
 *********************************************************************************/
RBAPI(void) io_SetISASpeed(int mode) {
    if (mode == ISAMODE_NORMAL)
        write_sb_reg(SB_MULTIFUNC_REG, read_sb_reg(SB_MULTIFUNC_REG) & 0xfffdffffL);
    else
        write_sb_reg(SB_MULTIFUNC_REG, read_sb_reg(SB_MULTIFUNC_REG) | 0x00020000L);
}



_RB_INLINE unsigned long read_ide_reg(unsigned char idx) {
    return io_inpcidw(0x80006000L+(unsigned long)idx);
}

RBAPI(int) io_CpuID(void) {
	unsigned long id   = read_nb_reg(0x90);
	unsigned char nbrv, sbrv;
	unsigned long ide;

	switch (id)
	{
		case 0x31504D44L: //"DMP1"
			return CPU_VORTEX86SX;
		case 0x32504D44L: //"DMP2"
	        nbrv = read_nb_regb(0x08);
	        sbrv = read_sb_regb(0x08);
	        ide  = read_ide_reg(0x00);

		    if ((nbrv == 1) && (sbrv == 1) && (ide == 0x101017f3L)) return CPU_VORTEX86DX_1;  // Vortex86DX ver. A
		    if ((nbrv == 1) && (sbrv == 2) && (ide == 0x101117f3L)) return CPU_VORTEX86DX_2;  // Vortex86DX ver. C (PBA/PBB)
		    if ((nbrv == 1) && (sbrv == 2) && (ide == 0x24db8086L)) return CPU_VORTEX86DX_2;  // Vortex86DX ver. C (PBA/PBB) with standard IDE enabled
		    if ((nbrv == 2) && (sbrv == 2) && (ide == 0x101117f3L)) return CPU_VORTEX86DX_3;  // Vortex86DX ver. D
		    if ((nbrv == 2) && (sbrv == 2) && (ide == 0x24db8086L)) return CPU_VORTEX86DX_3;  // Vortex86DX ver. D with standard IDE enabled

            return CPU_VORTEX86DX_UNKNOWN;
		case 0x33504D44L: //"DMP3"
			return CPU_VORTEX86MX;
		case 0x35504D44L: //"DMP5"
			return CPU_VORTEX86MX_PLUS;
	}

	return CPU_UNSUPPORTED;
}



/******************  PCI Configuration Space Access Functions  ****************/
RBAPI(void) io_outpcidw(unsigned long addr, unsigned long val) {
    #if   defined   USE_PCIDEBUG
        _pciConfigWriteLong((addr & 0x0fffffffL) >> 8, addr & 0xfcL, val);
    #elif defined   USE_PHYMEM
        WritePCI((addr>>16) & 0xffL,  // bus no.
                 (addr>>11) & 0x1fL,  // dev no.
                 (addr>>8)  & 0x07L,  // fun no.
                 addr & 0xfcL,        // reg offset
                 4, &val);
    #else
        io_DisableIRQ();
	    io_outpdw(0x0cf8, addr & 0xfffffffcL);
	    io_outpdw(0x0cfc, val);
        io_EnableIRQ();
    #endif
}

RBAPI(void) io_outpciw(unsigned long addr, unsigned short val) {
    #if   defined   USE_PCIDEBUG
        _pciConfigWriteShort((addr & 0x0fffffffL) >> 8, addr & 0xfcL, val);
    #elif defined   USE_PHYMEM
	    int i = (int)(addr & 0x03) * 8;
	    io_outpcidw(addr, (io_inpcidw(addr) & ~(0x0000ffffL << i)) | ((unsigned long)val << i));
    #else
	    int i = (int)(addr & 0x03) * 8;

        io_DisableIRQ();
	    io_outpdw(0x0cf8, addr & 0xfffffffcL);
	    io_outpdw(0x0cfc, (io_inpdw(0x0cfc) & ~(0x0000ffffL << i)) | ((unsigned long)val << i));
        io_EnableIRQ();
    #endif
}

RBAPI(void) io_outpcib(unsigned long addr, unsigned char val) {
    #if   defined   USE_PCIDEBUG
        _pciConfigWriteChar((addr & 0x0fffffffL) >> 8, addr & 0xfcL, val);
    #elif defined   USE_PHYMEM
	    int i = (int)(addr & 0x03) * 8;
	    io_outpcidw(addr, (io_inpcidw(addr) & ~(0x000000ffL << i)) | ((unsigned long)val << i));
    #else
	    int i = (int)(addr & 0x03) * 8;

        io_DisableIRQ();
	    io_outpdw(0x0cf8, addr & 0xfffffffcL);
	    io_outpdw(0x0cfc, (io_inpdw(0x0cfc) & ~(0x000000ffL << i)) | ((unsigned long)val << i));
        io_EnableIRQ();
    #endif
}

RBAPI(unsigned long) io_inpcidw(unsigned long addr) {
    #if   defined   USE_PCIDEBUG
        return _pciConfigReadLong((addr & 0x0fffffffL) >> 8, addr & 0xfcL);
    #elif defined   USE_PHYMEM
        unsigned long tmp;

        ReadPCI((addr>>16) & 0xffL,  // bus no.
                (addr>>11) & 0x1fL,  // dev no.
                (addr>>8)  & 0x07L,  // fun no.
                addr & 0xfcL,        // reg offset
                4, &tmp);

        return tmp;
    #else
        unsigned long tmp;

        io_DisableIRQ();
	    io_outpdw(0x0cf8, addr & 0xfffffffcL);
        tmp = io_inpdw(0x0cfc);
        io_EnableIRQ();

        return tmp;
    #endif
}

RBAPI(unsigned short) io_inpciw(unsigned long addr) {
    #if   defined   USE_PCIDEBUG
        return _pciConfigReadShort((addr & 0x0fffffffL) >> 8, addr & 0xfcL);
    #elif defined   USE_PHYMEM
	    int i = (int)(addr & 0x03) * 8;
        return (unsigned short)((io_inpcidw(addr) >> i) & 0xffffL);
    #else
        unsigned short tmp;
	    int i = (int)(addr & 0x03) * 8;

        io_DisableIRQ();
	    io_outpdw(0x0cf8, addr & 0xfffffffcL);
        tmp = (unsigned short)((io_inpdw(0x0cfc) >> i) & 0xffffL);
        io_EnableIRQ();

        return tmp;
    #endif
}

RBAPI(unsigned char) io_inpcib(unsigned long addr) {
    #if   defined   USE_PCIDEBUG
        return _pciConfigReadChar((addr & 0x0fffffffL) >> 8, addr & 0xfcL);
    #elif defined   USE_PHYMEM
	    int i = (int)(addr & 0x03) * 8;
        return (unsigned char)((io_inpcidw(addr) >> i) & 0xffL);
    #else
        unsigned char tmp;
	    int i = (int)(addr & 0x03) * 8;

        io_DisableIRQ();
	    io_outpdw(0x0cf8, addr & 0xfffffffcL);
        tmp = (unsigned char)((io_inpdw(0x0cfc) >> i) & 0xffL);
        io_EnableIRQ();

        return tmp;
    #endif
}
/*-------------  end of PCI Configuration Space Access Functions  ------------*/


