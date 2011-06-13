#ifndef __COM_H
#define __COM_H

#include "defines.h"
//#include "comdx.h"


#ifdef __cplusplus
extern "C" {
#endif

RBAPI(bool) com_InUse(void);



RBAPI(void) com_EnableTurboMode(int idx);
RBAPI(void) com_DisableTurboMode(int idx);

#ifdef __cplusplus
}
#endif

#endif

