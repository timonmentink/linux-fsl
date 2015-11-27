
#include <asm/setjmp.h>                // Header for setjmp/longjmp
extern void hal_longjmp(jmp_buf env, int val);

#define HAL_REORDER_BARRIER() asm volatile ( "" : : : "memory" )

void longjmp( jmp_buf buf, int val)
{
    // ANSI says that if we are passed cyg_val==0, then we change it to 1
    if (val == 0)
        ++val;

    // we let the HAL do the work

    HAL_REORDER_BARRIER(); // prevent any chance of optimisation re-ordering
    hal_longjmp( buf, val );
    HAL_REORDER_BARRIER(); // prevent any chance of optimisation re-ordering

} // longjmp()
