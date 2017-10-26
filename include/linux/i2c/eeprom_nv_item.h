#ifndef _LINUX_EEPROM_NV_ITEM_H
#define _LINUX_EEPROM_NV_ITEM_H

#include <linux/types.h>
#include <linux/memory.h>

/*
 * As seen through Linux I2C, differences between the most common types of I2C
 * memory include:
 * - How much memory is available (usually specified in bit)?
 * - What write page size does it support?
 * - Special flags (16 bit addresses, read_only, world readable...)?
 *
 * If you set up a custom eeprom type, please double-check the parameters.
 * Especially page_size needs extra care, as you risk data loss if your value
 * is bigger than what the chip actually supports!
 */

struct eeprom_nv_item {
	enum eeprom_nv_item_enum	id;
	u8		name[EEPROM_NV_ITEM_MAX_NAME_LEN+1];
	u16		start_byte;
	u16		num_byte;
	struct bin_attribute bin_attr;
};

struct eeprom_nv_item_platform_data{
	u16 num_item;
	struct eeprom_nv_item items[EEPROM_NV_ITEM_MAX_NUM];
};

ssize_t eeprom_read_nv_item(enum eeprom_nv_item_enum, char *buf, u16 bufsize);
ssize_t eeprom_write_nv_item(enum eeprom_nv_item_enum, char *buf, u16 bufsize);
u16 eeprom_get_nv_item_size(enum eeprom_nv_item_enum);

#endif /* _LINUX_EEPROM_NV_ITEM_H */
