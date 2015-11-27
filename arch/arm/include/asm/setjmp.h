
#ifndef __SETJMP__
#define __SETJMP__

//--------------------------------------------------------------------------
// setjmp

#define JMP_BUF_SIZE 16  // Actually 11, but some room left over

typedef unsigned int jmp_buf[JMP_BUF_SIZE];

extern int setjmp(jmp_buf env);
extern void longjmp(jmp_buf env, int val);

#endif
