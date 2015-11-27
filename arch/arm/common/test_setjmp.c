
#include <asm/setjmp.h>
static jmp_buf jmpbuf;

static void dump_jmpbuf(void)
{
	int i;
	printk("Dump jmpbuf\n");
	for(i=0;i< JMP_BUF_SIZE; i ++)
		printk("jmpbuf[%d] = 0x%0X\n", i, jmpbuf[i]);
}
int test_2( int a, int * b)
{
	int i;
	printk("Test 2\n");
	for(i = 0; i< 100; i++)
		b[i] = i*a;
	longjmp(jmpbuf, 2);
}

int test_1(void)
{
	int a = 3;
	int b = 5;
	int c[100];
	int i;

	printk("Test 1\n");
	for(i = 0; i< 100; i++)
		c[i] = i*b+a;
	//dump_jmpbuf();
	//longjmp(jmpbuf, 2);
	b = test_2(a, &c);
	return b;
}

static void test_setjmp(void)
{
	int ret, i;

	for(i=0;i< JMP_BUF_SIZE; i ++)
	{
		jmpbuf[i] = 0;
	}

	printk("Test setjmp\n");

	printk("Call setjmp\n");
	ret = setjmp(jmpbuf);
	dump_jmpbuf();
	if( 0 == ret )
	{	
		printk("setjmp returned 0\n");
		printk("call longjmp\n");
		longjmp(jmpbuf, 1);
	}
	else
	{
		printk("setjmp returned %d\n", ret);
	}
		
	printk("Call setjmp 2\n");
	ret = setjmp(jmpbuf);
	dump_jmpbuf();
	if( 0 == ret )
	{	
		printk("setjmp returned 0\n");
	}
	else
	{
		printk("setjmp returned %d\n", ret);
		goto Finished;
	}
	// bigger stack
	test_1();
Finished:;
	
}
