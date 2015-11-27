
//#define DEBUG

#include <asm/setup.h>
#include <libfdt.h>
#include <misc.h>
#include <linux/io.h>
#include <linux/elf.h>


#if defined(CONFIG_ARM_ATAG_DTB_COMPAT_CMDLINE_EXTEND)
#define do_extend_cmdline 1
#else
#define do_extend_cmdline 0
#endif

static int node_offset(void *fdt, const char *node_path)
{
	int offset = fdt_path_offset(fdt, node_path);
	if (offset == -FDT_ERR_NOTFOUND)
		offset = fdt_add_subnode(fdt, 0, node_path);
	return offset;
}

static int setprop(void *fdt, const char *node_path, const char *property,
		   uint32_t *val_array, int size)
{
	int offset = node_offset(fdt, node_path);
	if (offset < 0)
		return offset;
	return fdt_setprop(fdt, offset, property, val_array, size);
}

static int setprop_string(void *fdt, const char *node_path,
			  const char *property, const char *string)
{
	int offset = node_offset(fdt, node_path);
	if (offset < 0)
		return offset;
	return fdt_setprop_string(fdt, offset, property, string);
}

static int setprop_cell(void *fdt, const char *node_path,
			const char *property, uint32_t val)
{
	int offset = node_offset(fdt, node_path);
	if (offset < 0)
		return offset;
	return fdt_setprop_cell(fdt, offset, property, val);
}

static const void *getprop(const void *fdt, const char *node_path,
			   const char *property, int *len)
{
	int offset = fdt_path_offset(fdt, node_path);

	if (offset == -FDT_ERR_NOTFOUND)
		return NULL;

	return fdt_getprop(fdt, offset, property, len);
}

static uint32_t get_cell_size(const void *fdt)
{
	int len;
	uint32_t cell_size = 1;
	const uint32_t *size_len =  getprop(fdt, "/", "#size-cells", &len);

	if (size_len)
		cell_size = fdt32_to_cpu(*size_len);
	return cell_size;
}

static void merge_fdt_bootargs(void *fdt, const char *fdt_cmdline)
{
	char cmdline[COMMAND_LINE_SIZE];
	const char *fdt_bootargs;
	char *ptr = cmdline;
	int len = 0;

	/* copy the fdt command line into the buffer */
	fdt_bootargs = getprop(fdt, "/chosen", "bootargs", &len);
	if (fdt_bootargs)
		if (len < COMMAND_LINE_SIZE) {
			memcpy(ptr, fdt_bootargs, len);
			/* len is the length of the string
			 * including the NULL terminator */
			ptr += len - 1;
		}

	/* and append the ATAG_CMDLINE */
	if (fdt_cmdline) {
		len = strlen(fdt_cmdline);
		if (ptr - cmdline + len + 2 < COMMAND_LINE_SIZE) {
			*ptr++ = ' ';
			memcpy(ptr, fdt_cmdline, len);
			ptr += len;
		}
	}
	*ptr = '\0';

	setprop_string(fdt, "/chosen", "bootargs", cmdline);
}

/*
 * Convert and fold provided ATAGs into the provided FDT.
 *
 * REturn values:
 *    = 0 -> pretend success
 *    = 1 -> bad ATAG (may retry with another possible ATAG pointer)
 *    < 0 -> error from libfdt
 */
int atags_to_fdt(void *atag_list, void *fdt, int total_space)
{
	struct tag *atag = atag_list;
	/* In the case of 64 bits memory size, need to reserve 2 cells for
	 * address and size for each bank */
	uint32_t mem_reg_property[2 * 2 * NR_BANKS];
	int memcount = 0;
	int ret, memsize;

	putstr(__func__);
	putstr(" atags: 0x");
	putaddr((unsigned int) atag_list);
	putstr(" fdt: 0x");
	putaddr((unsigned int) fdt);

	/* make sure we've got an aligned pointer */
	if ((u32)atag_list & 0x3)
	{
		putstr(" Atags not aligned\n");
		return 1;
	}

	/* if we get a DTB here we're done already */
	if (*(u32 *)atag_list == fdt32_to_cpu(FDT_MAGIC))
	{
			putstr(" Got DTS as ATAGS pointer\n");

			putstr("Bootargs: \"");
			putstr( getprop(atag_list, "/chosen", "bootargs", 0));
			putstr("\"\n");

	       return 0;
	}

	/* validate the ATAG */
	if (atag->hdr.tag != ATAG_CORE ||
	    (atag->hdr.size != tag_size(tag_core) &&
	     atag->hdr.size != 2))
	{
		putstr(" ATAGS invalid\n");
		return 1;
	}

	/* let's give it all the room it could need */
	ret = fdt_open_into(fdt, fdt, total_space);	
	if (ret < 0)
	{
		putstr(" Failed to open fdt\n");
		return ret;
	}

	for_each_tag(atag, atag_list) {
		putstrdbg("ATAG Tag: ");
		putaddrdbg(atag->hdr.tag);
		putstrdbg("\n");
		if (atag->hdr.tag == ATAG_CMDLINE) {
			/* Append the ATAGS command line to the device tree
			 * command line.
			 * NB: This means that if the same parameter is set in
			 * the device tree and in the tags, the one from the
			 * tags will be chosen.
			 */
			if (do_extend_cmdline)
				merge_fdt_bootargs(fdt,
						   atag->u.cmdline.cmdline);
			else
				setprop_string(fdt, "/chosen", "bootargs",
					       atag->u.cmdline.cmdline);
		} else if (atag->hdr.tag == ATAG_MEM) {
			if (memcount >= sizeof(mem_reg_property)/4)
				continue;
			if (!atag->u.mem.size)
				continue;
			memsize = get_cell_size(fdt);

			if (memsize == 2) {
				/* if memsize is 2, that means that
				 * each data needs 2 cells of 32 bits,
				 * so the data are 64 bits */
				uint64_t *mem_reg_prop64 =
					(uint64_t *)mem_reg_property;
				mem_reg_prop64[memcount++] =
					cpu_to_fdt64(atag->u.mem.start);
				mem_reg_prop64[memcount++] =
					cpu_to_fdt64(atag->u.mem.size);
			} else {
				mem_reg_property[memcount++] =
					cpu_to_fdt32(atag->u.mem.start);
				mem_reg_property[memcount++] =
					cpu_to_fdt32(atag->u.mem.size);
			}

		} else if (atag->hdr.tag == ATAG_INITRD2) {
			uint32_t initrd_start, initrd_size;
			initrd_start = atag->u.initrd.start;
			initrd_size = atag->u.initrd.size;
			setprop_cell(fdt, "/chosen", "linux,initrd-start",
					initrd_start);
			setprop_cell(fdt, "/chosen", "linux,initrd-end",
					initrd_start + initrd_size);
		} else if (atag->hdr.tag == ATAG_REVISION) {
			putstrdbg("Found ATAG REVISION: ");
			putaddrdbg(atag->u.revision.rev);
			putstrdbg("\n");
			setprop_cell(fdt, "/", "revision", atag->u.revision.rev);


		}
	}

	if (memcount) {
		setprop(fdt, "/memory", "reg", mem_reg_property,
			4 * memcount * memsize);
	}
	ret = fdt_pack(fdt);

	if( ret == 0)
		putstr(" done\n");
	else
		putstr(" packing failed\n");
	return ret;
}

#ifdef CONFIG_ARM_DTB_TAG_IN_ATAGS
static void dump_one_atag( const struct tag* atag)
{
	putaddrdbg( (long unsigned int)atag);
	putstrdbg(" -- ");
	putaddrdbg( atag->hdr.size);
	putstrdbg(" -- ");	
	putaddrdbg(atag->hdr.tag);
	putstrdbg("\n");
}

static char * atags_find_cmdline(void *atag_list)
{
	void *atags = atag_list;
	struct tag * atag;

	atag = (struct tag *) atags;

	// Check if it is a valid atag list
	if(atag->hdr.tag != ATAG_CORE)
		return 0;

	while( atag->hdr.tag != ATAG_NONE)
	{
		dump_one_atag(atag);

		if( atag->hdr.tag == ATAG_CMDLINE)
			return atag->u.cmdline.cmdline;

		atags += atag->hdr.size * 4;
		atag = (struct tag *) atags;
	}
	return 0;
}

static char * atags_next( char * last)
{
	int i = 0;
	while(last[i] != 0 && last[i] != ' ') i++;
	while(last[i] != 0 && last[i] == ' ') i++;
	if( last[i] != 0)
		return &last[i];
	return 0;
}

static char * atags_check_keyword( char * cmdline, char *keyword)
{
	int i = 0;
	while( cmdline[i] && keyword[i])
	{
		if( cmdline[i] != keyword[i])
			return 0;
		i++;
	}
	if(!keyword[i])
		return &cmdline[i];
	return 0;
}

static char * atags_find_parameter( char * cmdline, char * keyword)
{
	char * param;
    char * cmd;
	cmd = cmdline;

	param = atags_check_keyword( cmd, keyword);
	while(param==0 && cmd!=0)
	{
		cmd = atags_next(cmd);
		if( !cmd ) break;
		param = atags_check_keyword( cmd, keyword);
	}
	if( param)
	{
		putstrdbg("Found: ");
		putstrdbg(param);
		putstrdbg("\n");
	}
	return param;
}
static char * atags_decode_number( char * param, int * value)
{
	int val=0, v, i;
	int mode = 0;
	char * p = &param[0];

	putstrdbg("atags_decode_number:");
	putstrdbg(param);
	putstrdbg("\n");

	// Check for prefix
	if(p[0] == '0' && (p[1] == 'x' || p[1] == 'X'))
		mode =  0;
	else
	{
		putstr("atags_decode_address: Only hexadecimal format supported\n");
		return 0;
	}

	p = &p[2];

	for( i = 0; i < 8; i++)
	{
		if( p[i] >= '0' && p[i] <= '9')
			v = p[i] - '0';
		else if( p[i] >= 'A' && p[i] <= 'F')
			v = p[i] - 'A' + 10;
		else if( p[i] >= 'a' && p[i] <= 'f')
			v = p[i] - 'a' + 10;
		else
			break;

		val = val << 4 | v;
		putaddrdbg(val);
		putstrdbg("\n");
	}

	*value = val;
	return &p[i];
}
static char * atags_decode_address( char * param, int * dtaddress, int * socId)
{
	int val=0;
	char * p = param;

	
	p = atags_decode_number( p, &val);
	if(p[0] == ':')
	{
		// ':' is the devider between socid and address
		if(socId) *socId = val;
		p = atags_decode_number(&p[1], dtaddress);
	}
	else
	{
		*dtaddress = val;
		if(socId) *socId = 0xFFFFFFFF; // not marked
	}

	if(socId){ 
		putstrdbg("Found socid: ");
		putaddrdbg(*socId);
		putstrdbg("\n");
	}
	putaddrdbg(*dtaddress);
	putstrdbg("\n");

	return p;
}

static void get_imx6_id(int *cpu_type, int *cpu_rev)
{
	/* Read Silicon information from Anatop register */
	/* (not documented in Reference Manual */
	/* The register layout:
	 * bit 16-23: Chip Silicon ID
	 * 0x60: i.MX6 SoloLite
	 * 0x61: i.MX6 Solo/DualLite
	 * 0x63: i.MX6 Dual/Quad
	 *
	 * bit 0-7: Chip Revision ID
	 * 0x00: TO1.0
	 * 0x01: TO1.1
	 * 0x02: TO1.2
	 *
	 * exp:
	 * Chip             Major    Minor
	 * i.MX6Q1.0:       6300     00
	 * i.MX6Q1.1:       6300     01
	 * i.MX6Solo1.0:    6100     00
	 */
	/*u32 val = REG(ANATOP_BASE_ADDR + 0x260);*/

    u32 val = __raw_readl((void*)(0x020c8000 + 0x260));

	if (cpu_type)
		*cpu_type = (val & 0x00ff0000) >> 16;
	if (cpu_rev)
		*cpu_rev = (val & 0x000000ff);
}


static void dump_mem( const void * start, u32 len)
{
	int j;
	const u32 * data = (const u32 *) start;
	const u32 * end = (const u32 *) (start + len);
	
	while(data < end)
	{
		putaddr((const u32)data);putstr(": ");
		for( j = 0; j < 8; j++)
		{
			putaddr( *data  );
			putstr(" ");
			data++;
		}
		putstr("\n");
	}
}

static int dump_core_plain( u32 elfcorehdr_addr)
{
	// New approach, everything in place, dump real addresses
	const Elf32_Ehdr * ehdr = (const Elf32_Ehdr *) elfcorehdr_addr;
	const Elf32_Phdr * phdr = (const Elf32_Phdr *) (elfcorehdr_addr + sizeof(Elf32_Ehdr));
	int phdr_count = ehdr->e_phnum;
	u32 total_length, header_length;
	int i;

	/* Do some basic Verification. */
	if (memcmp(ehdr->e_ident, ELFMAG, SELFMAG) != 0 ||
		(ehdr->e_type != ET_CORE) ||
		//!elf_check_arch(&ehdr) ||
		ehdr->e_ident[EI_CLASS] != ELFCLASS32||
		ehdr->e_ident[EI_VERSION] != EV_CURRENT ||
		ehdr->e_version != EV_CURRENT ||
		ehdr->e_ehsize != sizeof(Elf32_Ehdr) ||
		ehdr->e_phentsize != sizeof(Elf32_Phdr) ||
		ehdr->e_phnum == 0) {
		putstr("Warning: Core image elf header is not sane\n");
		return -1;
	}
	
	header_length = sizeof(Elf32_Ehdr) + sizeof(Elf32_Phdr) * phdr_count;
	total_length = header_length;
	for( i = 0; i < phdr_count; i++)
	{
		total_length += phdr[i].p_filesz;
	}
	putstr("Core dump from "); putaddr(elfcorehdr_addr); 
	putstr(" length: ");putaddr(total_length);putstr("\n");
	putstr("Sections:\n");
	for( i = 0; i < phdr_count; i++)
	{
		putaddr(i);putstr(":  type 0x"); putaddr(phdr[i].p_type); putstr(" start 0x"); 
		putaddr(phdr[i].p_offset);putstr(" len 0x"); putaddr(phdr[i].p_filesz); 
		putstr("\n");
	}

	dump_mem( ehdr, header_length);

	for( i = 0; i < phdr_count; i++)
	{
		void * start = (void*) phdr[i].p_offset;
		u32 length = phdr[i].p_filesz;
		dump_mem( start, length);
	}
	

	while(1);
	return 0;
}

/* Check for crashkernel stuff
 *
 * For now we only care or the address elfcorehdr=0x2342345
 */
static int parse_elfcorehdr( char * cmdline)
{
	char * p;
	char * keyword = "elfcorehdr=";
	int elf=0;
	p = cmdline;
	while(1){
		p = atags_find_parameter( p,  keyword);
		if(!p) break;

		p = atags_decode_address( p, &elf, NULL);
		putstr("Found elfcorehdr=");
		putaddr(elf);
		putstr("\n");
		break;
	}
	if(elf)
		dump_core_plain(elf);

	return 0;
}

int atags_find_dt_tag(void *atag_list)
{
	char * cmdline;
	char * p;
	char * keyword = "devicetree=";
	int dtaddress=0, socId=0xFFFFFFFF;
	int result = 0;
	int cpu, rev;


	get_imx6_id(&cpu, &rev);
	putstrdbg("CPU 0x");
	putaddrdbg( cpu);
	putstrdbg(" REV: 0x");
	putaddrdbg( rev);
	putstrdbg("\n");

	/* if we get a DTB here we're done already */
	if (*(u32 *)atag_list == fdt32_to_cpu(FDT_MAGIC))
	{
		putstrdbg(" got DTS as ATAGS pointer\n");
		cmdline = (char*) getprop(atag_list, "/chosen", "bootargs", 0);
		parse_elfcorehdr( cmdline);
	    return (int)atag_list;
	}

	cmdline = atags_find_cmdline(atag_list);

	if( 0 == cmdline)
		return -1;

	putstrdbg( cmdline);
	putstrdbg("\n");

	parse_elfcorehdr( cmdline);
	p = cmdline;

	while(1){
		p = atags_find_parameter( p,  keyword);
		if(!p) break;

		p = atags_decode_address( p, &dtaddress, &socId);

		if( dtaddress)
		{	
			if( 0 == result || socId == cpu)	// Use any given devicetree but prefer the one with the correct soc id
				result = dtaddress;
			putstrdbg("Device tree for socId 0x");
			putaddrdbg(socId);
			putstrdbg(" should be found at 0x");
			putaddrdbg(dtaddress);
			putstrdbg("\n");
		}
	}
	return result;
}
#endif // CONFIG_ARM_DTB_TAG_IN_ATAGS
