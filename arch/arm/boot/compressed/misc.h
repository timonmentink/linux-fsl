#ifndef _MISC_H
#define _MISC_H

void putstr(const char *ptr);
void putaddr( unsigned long addr);

#ifdef DEBUG
	#define putstrdbg(a) putstr(a)
	#define putaddrdbg(a) putaddr(a)
#else
	#define putstrdbg(a) while(0)
	#define putaddrdbg(a) while(0)
#endif

#endif
