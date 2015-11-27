/*
 *  linux/arch/arm/kernel/setjmp.S
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Based on setjmp from ecos/redboot context.S
 */
#include <linux/linkage.h>
#include <asm/assembler.h>

// ----------------------------------------------------------------------------
//  function declaration macro (start body in ARM mode)
                
#ifdef __thumb__
#define FUNC_START_ARM(_name_, _r_)              \
        .code   16                              ;\
        .thumb_func                             ;\
        .globl _name_                           ;\
		ALIGN                                   ;\
_name_:                                         ;\
        ldr     _r_,=_name_ ## _ARM             ;\
        bx      _r_                             ;\
        .code   32                              ;\
_name_ ## _ARM:

#else

#define FUNC_START_ARM(_name_, _r_) \
        .globl _name_; \
_name_: 
        
#endif
        
	.text
// ----------------------------------------------------------------------------
//  HAL longjmp, setjmp implementations
//  setjmp saves only to callee save registers 4-14
//  and lr into buffer supplied in r0[arg0]

FUNC_START_ARM(setjmp, r2)
		smp_dmb
        stmea   r0,{r4-r14}
        mov     r0,#0
		smp_dmb
#ifdef __thumb__
        bx      lr
#else
        mov     pc,lr;          # return
#endif

//  hal_longjmp loads state from r0[arg0] and returns
        
FUNC_START_ARM(hal_longjmp, r2)
		smp_dmb
        ldmfd   r0,{r4-r14}
        mov     r0,r1;          # return [arg1]
		smp_dmb
#ifdef __thumb__
        bx      lr
#else
        mov     pc,lr
#endif

// ----------------------------------------------------------------------------
//  end of setjmp.S
