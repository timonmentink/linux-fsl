
#ifndef _UAPI__LINUX_GFEEPROM_H
#define _UAPI__LINUX_GFEEPROM_H

#include <linux/ioctl.h>

#ifndef __user
#define __user 
#endif

#ifndef __KERNEL__
typedef int8_t s8;
typedef uint8_t u8;

typedef int16_t s16;
typedef uint16_t u16;

typedef int32_t s32;
typedef uint32_t u32;

typedef int64_t s64;
typedef uint64_t u64;

typedef _Bool bool;

enum {
    false   = 0,
	true    = 1
};

#endif

#define GFEEPROM_NAME "gfeeprom"
//max number of supported devices
#define MAX_GFEEPROM_DEV	1
#define MAX_GFEEPROM_CHIPS	8
#define MAX_GFEEPROM_OPEN	(MAX_GFEEPROM_CHIPS * MAX_GFEEPROM_DEV)

#define ERROR_SUCCESS                    0
#define ERROR_PATH_NOT_FOUND             3
#define ERROR_NOT_ENOUGH_MEMORY          8
#define ERROR_INVALID_DATA               13
#define ERROR_CRC                        23
#define ERROR_BAD_ARGUMENTS              160
#define ERROR_INSUFFICIENT_BUFFER        122
#define ERROR_NO_VOLUME_LABEL            125
#define ERROR_BAD_ARGUMENTS              160
#define ERROR_I2C_FAILURE				 244
#define ERROR_LIST_INCONSISTENT          255
#define ERROR_MOD_NOT_FOUND              126
#define ERROR_EEPROM_HEAD_READ_FAILURE	 275
#define ERROR_OLD_VERSION            	 1150

/** predefined lengths for the different elements in the structures
*   !! Should be removed if a full GFVersion port to linux is ready
*/
#define GFV_UNIQUE_ID_LENGTH			33
#define GFV_COMMENT_LENGTH				128
#define GFV_COMPONENT_LENGTH			32
#define GFV_BUILD_DATE_LENGTH			64
#define GFV_SVN_BUILD_NUMBER_LENGTH		512
#define GFV_MAX_INFO_DATA_LENGTH		2024
#define GFV_NUMBER_LENGTH				8
#define GFV_ARTICLE_NUMBER_LENGTH		128
#define GFV_SERIAL_NUMBER_LENGTH		128
#define GFV_KEY_LENGTH					21

/** The layout of the G&F hardware information struture.
*	!! Should be removed if a full GFVersion port to linux is ready
*/
struct gfv_hw_version_data
{
	u32			size_of_struct;									/*!< The size of the structure for API version verification. */
	char		major[GFV_NUMBER_LENGTH];						/*!< The major version of the hardware component. */
	char		minor[GFV_NUMBER_LENGTH];						/*!< The minor version of the hardware component. */
	char		article_number[GFV_ARTICLE_NUMBER_LENGTH];		/*!< The G&F article number of the hardware component. */
	char		serial_numbers[GFV_SERIAL_NUMBER_LENGTH];		/*!< The serial numbers of the hardware component(s). */
	char		component[GFV_COMPONENT_LENGTH];				/*!< The component to which this version information belongs. */
	char		comment[GFV_COMMENT_LENGTH];					/*!< A user specific comment. */
	u32			address_of_component;							/*!< If addressable, the physical address of the hardware component, else 0. */
	u32			phys_mem_size;									/*!< If addressable, the physical memory size of the hardware component, else 0 */
};

#define GFEEPROM_MIN_AREA_TYPE_NUMBER             50

/** The command (ioctl) defines
*
*/
#define GFEEPROM_IOC_MAGIC							'k'
#define GFEEPROM_IOC_ACCESS_NUMBER					101
#define GFEEPROM_IOC_PROGRAM_HW_NUMBER				102
#define GFEEPROM_IOC_GET_CUSTOM_AREA_COUNT_NUMBER	103
#define GFEEPROM_IOC_GET_USED_CUSTOM_AREA_NUMBER	104

#define GFEEPROM_IOCTL_ACCESS_EEPROM_AREA			_IOWR(GFEEPROM_IOC_MAGIC, GFEEPROM_IOC_ACCESS_NUMBER, struct gfeeprom_access)
#define GFEEPROM_IOCTL_PROGRAM_HW_INFO				_IOW(GFEEPROM_IOC_MAGIC, GFEEPROM_IOC_PROGRAM_HW_NUMBER, struct gfeeprom_program_area)
#define GFEEPROM_IOCTL_GET_CUSTOM_AREA_COUNT		_IOR(GFEEPROM_IOC_MAGIC, GFEEPROM_IOC_GET_CUSTOM_AREA_COUNT_NUMBER, u16)
#define GFEEPROM_IOCTL_GET_USED_CUSTOM_AREAS		_IOWR(GFEEPROM_IOC_MAGIC, GFEEPROM_IOC_GET_USED_CUSTOM_AREA_NUMBER, struct gfeeprom_area_array)

//------------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------------
#define GFEEPROM_VALIDAT_ID(controller, address) (((controller & 0xFF) << 8) | (address << 1))
#define GFEEPROM_HW_SMALL_ARTICLE_NUMBER_LENGTH		13

enum gfeeprom_type
{
	EEPROM_ST_M24C = 0,
	EEPROM_ATMEL_AT24C,
	EEPROM_LAST_TYPE_ENTRY,
	EEPROM_UNKNOWN_TYPE	= 0xFFFF
};

enum gfeeprom_area_type
{
	EEPROM_TYPE_EHW = 0x0001,
	EEPROM_TYPE_OPERATING_HOURS_COUNTER_0,
	EEPROM_TYPE_OPERATING_HOURS_COUNTER_1,
	EEPROM_TYPE_S0_1_COUNTER_0,
	EEPROM_TYPE_S0_1_COUNTER_1,
	EEPROM_TYPE_S0_2_COUNTER_0,
	EEPROM_TYPE_S0_2_COUNTER_1,
	EEPROM_TYPE_S0_3_COUNTER_0,
	EEPROM_TYPE_S0_3_COUNTER_1,
	EEPROM_TYPE_S0_4_COUNTER_0,
	EEPROM_TYPE_S0_4_COUNTER_1,
	EEPROM_TYPE_CUSTOMER_AREA,
	EEPROM_UNUSED_AREA	= 0xFFFF
};

enum gfeeprom_access_type
{
	EEPROM_ACCESS_TYPE_READ,
	EEPROM_ACCESS_TYPE_WRITE,
	EEPROM_ACCESS_TYPE_DELETE_ALL,
	EEPROM_ACCESS_TYPE_CLEAN
};

enum gfeeprom_address_width
{
	EEPROM_ADDRSS_WIDTH_8BIT = 0,
	EEPROM_ADDRSS_WIDTH_16BIT,
};

struct gfeeprom_info_type
{
	enum gfeeprom_address_width 	adress_width;
	u32								page_size;
	u32								eeprom_size;
};

struct gfeeprom_hw_small_version_data
{
	u16		size_of_struct;										/*!< The size of the structure for API version verification. */
	char	major[GFV_NUMBER_LENGTH];							/*!< The major version of the hardware component. */
	char	minor[GFV_NUMBER_LENGTH];							/*!< The minor version of the hardware component. */
	char	article_number[GFEEPROM_HW_SMALL_ARTICLE_NUMBER_LENGTH];	/*!< Smaller than the G&F Version information. */
	char	component[GFV_COMPONENT_LENGTH];					/*!< The component to which this version information belongs. */
	char	comment[GFV_COMPONENT_LENGTH];						/*!< A user specific comment. Slightly smaller than in GFVersionApi.h.*/
	u32		address_of_component;								/*!< If addressable, the physical address of the hardware component, else 0. */
	u32		phys_mem_size;										/*!< If addressable, the physical memory size of the hardware component, else 0 */
};

struct gfeeprom_hw_normal_version_data
{
	u16		size_of_struct;									/*!< The size of the structure for API version verification. */
	char	major[GFV_NUMBER_LENGTH];						/*!< The major version of the hardware component. */
	char	minor[GFV_NUMBER_LENGTH];						/*!< The minor version of the hardware component. */
	char	article_number[GFV_ARTICLE_NUMBER_LENGTH];		/*!< The G&F article number of the hardware component. */
	char	component[GFV_COMPONENT_LENGTH];				/*!< The component to which this version information belongs. */
	char	comment[GFV_COMMENT_LENGTH];					/*!< Compatible to the GFVersionApi.h.*/
	u32		address_of_component;							/*!< If addressable, the physical address of the hardware component, else 0. */
	u32		phys_mem_size;									/*!< If addressable, the physical memory size of the hardware component, else 0 */
	char	serial_numbers[GFV_SERIAL_NUMBER_LENGTH];		/*!< The serial numbers of the hardware component(s). */
};

struct gfeeprom_head
{
	char	controller_index;
	char	address;
};

struct gfeeprom_area_head
{
	u16	type;
	u16	length;
	u32	crc32;
};

struct gfeeprom_program_area
{
	u32 						size_of_struct;
	bool 						verify;
	bool						force_write;
	struct gfv_hw_version_data	hw_version;
};

struct gfeeprom_access
{
	u32							size_of_struct;
	enum gfeeprom_area_type 	area_type;
	enum gfeeprom_access_type 	access_type;
	bool						force_write;
	char* 						data;
	u16							data_length;
	char*						verify_buffer;
	u16 						verify_buffer_length;
};

struct gfeeprom_area_array
{
	u32							size_of_struct;
	u16							elements;
	u16* __user					areas;
};

#endif
