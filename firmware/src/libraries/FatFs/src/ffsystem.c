#ifndef _FFSYSTEM_H_
#define _FFSYSTEM_H_

#if defined(USE_FREERTOS)
#include "../ffsystem/ffsystem_cmsis_os.c"
#else
#include "../ffsystem/ffsystem_baremetal.c"
#endif

#endif /* _FFSYSTEM_H_ */