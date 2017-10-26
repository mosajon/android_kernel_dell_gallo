/*
    Copyright (C) 1998, 1999  Frodo Looijaard <frodol@dds.nl> and
			       Philip Edelbrock <phil@netroedge.com>
    Copyright (C) 2003 Greg Kroah-Hartman <greg@kroah.com>
    Copyright (C) 2003 IBM Corp.
    Copyright (C) 2004 Jean Delvare <khali@linux-fr.org>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/i2c/eeprom_nv_item_common.h>
#include <linux/i2c/eeprom_nv_item.h>
#include <linux/rtc.h>


static u8 use_smbus;
static unsigned write_timeout = 50;
static struct i2c_client *eeprom_nv_item_i2c_client;

#define attr_to_nv_item(c) container_of(c, struct eeprom_nv_item, bin_attr)

#define EEPROM_START_USAGE_MAGIC_NUMBER "EDCBA"

#define EEPROM_NV_ITEM_LOCK_NAME "lock"
#define EEPROM_NV_ITEM_UNLOCK_NAME "unlock"
#define EEPROM_NV_ITEM_PASSWD "33076"
#define EEPROM_NV_ITEM_PASSWD_SIZE 5

static struct miscdevice eeprom_nv_item_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "eeprom_nv_item",
};

/* Each client has this additional data */
struct eeprom_data {
	struct mutex update_lock;
	u8 nv_item_num;
	struct eeprom_nv_item  nv_item[EEPROM_NV_ITEM_MAX_NUM];
	struct delayed_work set_start_usage_work;
	struct bin_attribute attr_lock;
	struct bin_attribute attr_unlock;
};

static ssize_t eeprom_nv_item_read_byte(struct i2c_client *client, u8 addr, char *buf)
{
	unsigned long timeout, read_time;
	struct i2c_msg msg[2];
	u8 msgbuf[2];
	ssize_t status;
	int i;

	if (!use_smbus) {
		i = 0;
		msgbuf[i++] = addr;
		msg[0].addr = client->addr;
		msg[0].buf = msgbuf;
		msg[0].len = i;
		msg[0].flags = I2C_M_NOSTART;

		msg[1].addr = client->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].buf = buf;
		msg[1].len = 1;
	}

	/*
	 * Reads fail if the previous write didn't complete yet. We may
	 * loop a few times until this one succeeds, waiting at least
	 * long enough for one entire page write to work.
	 */
	timeout = jiffies + msecs_to_jiffies(write_timeout);
	do {
		read_time = jiffies;
		if (use_smbus) {
			status = i2c_smbus_read_i2c_block_data(client, addr, 1, buf);
		} else {
			status = i2c_transfer(client->adapter, msg, 2);
			if (status == 2)
				status = 1;
		}

		if (status == 1) {
			break;
		}

		/* REVISIT: at HZ=100, this is sloooow */
		msleep(1);
		status = -ETIMEDOUT;
	} while (time_before(read_time, timeout));

	return status;
}


static ssize_t eeprom_nv_item_read(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr,
			   char *buf, loff_t off, size_t count)
{
	struct i2c_client *client = to_i2c_client(container_of(kobj, struct device, kobj));
	struct eeprom_data *data = i2c_get_clientdata(client);
       struct eeprom_nv_item *nv_item = attr_to_nv_item(bin_attr);
	ssize_t status;
	ssize_t retval = 0;
	int i;

	if (unlikely(!count))
		return count;

	if (off < 0 || off > nv_item->num_byte)
		return 0;
	if (count == 0 || off > nv_item->num_byte)
		return 0;
	if (off + count > nv_item->num_byte)
		count = nv_item->num_byte - off;


	printk(KERN_DEBUG "eeprom_nv_item_read(), bin_attr->size=%d \n", bin_attr->size);
	printk(KERN_DEBUG "eeprom_nv_item_read(), off=%d, count=%d, \n", (int)off, count);
	printk(KERN_DEBUG "nv_item(), start_byte=%d, num_byte=%d, \n", nv_item->start_byte, nv_item->num_byte);

	mutex_lock(&data->update_lock);
	for (i = 0; i < count; i++) {
		status = eeprom_nv_item_read_byte(client, nv_item->start_byte+off+i, &buf[i]);
		if (status <= 0) {
			if (retval == 0)
				retval = status;
			break;
		}
		retval += status;
	}
	mutex_unlock(&data->update_lock);

	{
	int i = 0;
		for (i = 0; i < count; i++) {
			printk(KERN_DEBUG "buf[%d]=%c \n", i, buf[i]);
		}
	}

	return retval;
}

static ssize_t eeprom_nv_item_write_byte(struct i2c_client *client, u8 addr, char *buf)
{
	struct i2c_msg msg;
	ssize_t status;
	unsigned long timeout, write_time;
	u8 i2c_buf[2];
	int i;

	if (!use_smbus) {
		i = 0;
		msg.addr = client->addr;
		msg.flags = 0;
		msg.buf = i2c_buf;
		msg.buf[i++] = addr;
		msg.buf[i] = buf[0];
		msg.len = i + 1;
	}

	/*
	 * Writes fail if the previous one didn't complete yet. We may
	 * loop a few times until this one succeeds, waiting at least
	 * long enough for one entire page write to work.
	 */
	timeout = jiffies + msecs_to_jiffies(write_timeout);
	do {
		write_time = jiffies;
		if (use_smbus) {
			status = i2c_smbus_write_i2c_block_data(client, addr, 1, buf);
			if (status == 0)
				status = 1;
		} else {
			status = i2c_transfer(client->adapter, &msg, 1);
			if (status == 1)
				status = 1;
		}

		if (status == 1) {
			break;
		}

		/* REVISIT: at HZ=100, this is sloooow */
		msleep(1);
		status = -ETIMEDOUT;
	} while (time_before(write_time, timeout));

	return status;
}

static ssize_t eeprom_nv_item_write(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr,
			   char *buf, loff_t off, size_t count)
{
	struct i2c_client *client = to_i2c_client(container_of(kobj, struct device, kobj));
	struct eeprom_data *data = i2c_get_clientdata(client);
       struct eeprom_nv_item *nv_item = attr_to_nv_item(bin_attr);
	ssize_t status;
	ssize_t retval = 0;
	int i;

	if (unlikely(!count))
		return count;

	if (off < 0 || off > nv_item->num_byte)
		return 0;
	if (count == 0 || off > nv_item->num_byte)
		return 0;
	if (off + count > nv_item->num_byte)
		count = nv_item->num_byte - off;

	printk(KERN_DEBUG "eeprom_nv_item_write(), bin_attr->size=%d \n", bin_attr->size);
	printk(KERN_DEBUG "eeprom_nv_item_write(), off=%d, count=%d, \n", (int)off, count);
	printk(KERN_DEBUG "nv_item(), start_byte=%d, num_byte=%d, \n", nv_item->start_byte, nv_item->num_byte);

	{
		int i = 0;
		for (i = 0; i < count; i++) {
			printk(KERN_DEBUG "buf[%d]=%c \n", i, buf[i]);
		}
	}

	mutex_lock(&data->update_lock);

	for (i = 0; i < count; i++) {
		status = eeprom_nv_item_write_byte(client, nv_item->start_byte+off+i, &buf[i]);
		if (status <= 0) {
			if (retval == 0)
				retval = status;
			break;
		}
		retval += status;
	}

	mutex_unlock(&data->update_lock);
	return retval;
}

static void eeprom_check_and_update_start_usage()
{
	char buf[EEPROM_SIZE_START_USAGE+1];
	unsigned long *seconds_long = buf;
	struct timeval		stv;

	printk(KERN_INFO "%s: ++ \n", __func__);
	memset(buf, sizeof(buf), 0x00);
	if (eeprom_read_nv_item(EEPROM_NV_ITEM_START_USAGE, buf, EEPROM_SIZE_START_USAGE)
		== EEPROM_SIZE_START_USAGE) {
		if (!strncmp(buf, EEPROM_START_USAGE_MAGIC_NUMBER, EEPROM_SIZE_START_USAGE)) {
			do_gettimeofday(&stv);

			memset(buf, sizeof(buf), 0x00);
			seconds_long = buf;
			*seconds_long = stv.tv_sec;
			printk(KERN_INFO "%s: before+++, update start_usage to %d \n", __func__, *seconds_long);
			eeprom_write_nv_item(EEPROM_NV_ITEM_START_USAGE, buf, EEPROM_SIZE_START_USAGE);

			memset(buf, sizeof(buf), 0x00);
			eeprom_read_nv_item(EEPROM_NV_ITEM_START_USAGE, buf, EEPROM_SIZE_START_USAGE);
			printk(KERN_INFO "%s: end+++, update start_usage to %d \n", __func__, *seconds_long);
		} else {
			seconds_long = buf;
			printk(KERN_INFO "%s: start_usage is %d seconds after 1970/1/1. \n", __func__, *seconds_long);
		}
	}
	printk(KERN_INFO "%s: -- \n", __func__);
}


static void set_start_usage_work(struct work_struct *work)
{
	eeprom_check_and_update_start_usage();
}

static ssize_t eeprom_nv_item_lock_attr_write(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr,
			   char *buf, loff_t off, size_t count)
{
	struct i2c_client *client = to_i2c_client(container_of(kobj, struct device, kobj));
	struct eeprom_data *data = i2c_get_clientdata(client);
	int err = 0, i;

	if (unlikely(!count))
		return count;


	printk(KERN_DEBUG "%s: bin_attr->size=%d \n", __func__, bin_attr->size);
	printk(KERN_DEBUG "%s: off=%d, count=%d, \n", __func__, (int)off, count);

	if (((int)off) != 0 || count != EEPROM_NV_ITEM_PASSWD_SIZE)
		return 0;


	if (count != EEPROM_NV_ITEM_PASSWD_SIZE ||
		memcmp(buf, EEPROM_NV_ITEM_PASSWD, EEPROM_NV_ITEM_PASSWD_SIZE)) {
		printk(KERN_ERR "%s(%d): wrong passwd. \n", __func__, __LINE__);
		return count;
	}

	if (!strcmp(bin_attr->attr.name, EEPROM_NV_ITEM_LOCK_NAME)) {
		sysfs_remove_bin_file(&client->dev.kobj,  &data->nv_item[EEPROM_NV_ITEM_OTA_VERSION].bin_attr);
	}

	if (!strcmp(bin_attr->attr.name, EEPROM_NV_ITEM_UNLOCK_NAME)) {
		err = sysfs_create_bin_file(&client->dev.kobj, &data->nv_item[EEPROM_NV_ITEM_OTA_VERSION].bin_attr);
	}
	return count;
}

static void init_lock_attr(struct i2c_client *client, struct bin_attribute *lock_attr, char* name)
{
	int err = 0;
	sysfs_bin_attr_init(lock_attr);
	lock_attr->attr.name = name;
	lock_attr->attr.mode = S_IRUGO|S_IWUSR;
	lock_attr->size =  EEPROM_NV_ITEM_PASSWD_SIZE;
	lock_attr->read = NULL;
	lock_attr->write = eeprom_nv_item_lock_attr_write;
	err = sysfs_create_bin_file(&client->dev.kobj, lock_attr);
	if (err) {
		printk(KERN_ERR "%s(%d): create attribue (%s) fail \n", __func__, __LINE__, name);
	}
}

static int eeprom_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct eeprom_data *data;
	struct eeprom_nv_item_platform_data *eeprom_nv_item_pdata = client->dev.platform_data;
	int err, i;

	use_smbus = 0;
	eeprom_nv_item_i2c_client = NULL;
	data = kzalloc(sizeof(struct eeprom_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, data);
	mutex_init(&data->update_lock);

	eeprom_nv_item_device.parent = &client->dev;
	err = misc_register(&eeprom_nv_item_device);
	if (err) {
		pr_err("%s: misc_register() register failed\n", __func__);
		goto exit_kfree;
	}

	/* create the sysfs eeprom file */
	data->nv_item_num = eeprom_nv_item_pdata->num_item;
	for (i = 0; i < data->nv_item_num; i++) {
		eeprom_nv_item_pdata->items[i].id = i;
		memcpy(&data->nv_item[i], &eeprom_nv_item_pdata->items[i], sizeof(struct eeprom_nv_item));

		printk(KERN_INFO "EEPROM[%d][%s] start:%d, size=%d\n", i, eeprom_nv_item_pdata->items[i].name,
			eeprom_nv_item_pdata->items[i].start_byte, eeprom_nv_item_pdata->items[i].num_byte);

		sysfs_bin_attr_init(&data->nv_item[i].bin_attr);
		data->nv_item[i].bin_attr.attr.name = data->nv_item[i].name;
		data->nv_item[i].bin_attr.attr.mode = S_IRUGO|S_IWUGO;
		data->nv_item[i].bin_attr.size =  data->nv_item[i].num_byte;
		data->nv_item[i].bin_attr.read = eeprom_nv_item_read;
		data->nv_item[i].bin_attr.write = eeprom_nv_item_write;

		#if 0
		err = sysfs_create_bin_file(&client->dev.kobj, &data->nv_item[i].bin_attr);
		if (err) {
			while (--i >= 0)
				sysfs_remove_bin_file(&client->dev.kobj,  &data->nv_item[i].bin_attr);
			goto exit_kfree;
		}
		#endif
	}

	init_lock_attr(client, &data->attr_lock, EEPROM_NV_ITEM_LOCK_NAME);
	init_lock_attr(client, &data->attr_unlock, EEPROM_NV_ITEM_UNLOCK_NAME);


	eeprom_nv_item_i2c_client = client;
	printk(KERN_DEBUG "%s: eeprom_nv_item_i2c_client=0x%x \n",  __FUNCTION__, (u32)eeprom_nv_item_i2c_client);

	INIT_DELAYED_WORK(&data->set_start_usage_work, set_start_usage_work);
	schedule_delayed_work(&data->set_start_usage_work, msecs_to_jiffies(30*1000));
	return 0;

exit_kfree:
	kfree(data);

exit:
	return err;
}

static int eeprom_remove(struct i2c_client *client)
{
	struct eeprom_data *data = i2c_get_clientdata(client);
	int i;

	misc_deregister(&eeprom_nv_item_device);
	for (i = 0; i < data->nv_item_num; i++) {
		sysfs_remove_bin_file(&client->dev.kobj, &data->nv_item[i].bin_attr);
	}

	sysfs_remove_bin_file(&client->dev.kobj, &data->attr_lock);
	sysfs_remove_bin_file(&client->dev.kobj, &data->attr_unlock);

	cancel_delayed_work(&data->set_start_usage_work);

	kfree(i2c_get_clientdata(client));
	eeprom_nv_item_i2c_client = NULL;

	return 0;
}

static const struct i2c_device_id eeprom_id[] = {
	{ "eeprom_nv_item", 0 },
	{ }
};

static struct i2c_driver eeprom_driver = {
	.driver = {
		.name	= "eeprom_nv_item",
		.owner = THIS_MODULE,
	},
	.probe		= eeprom_probe,
	.remove		= eeprom_remove,
	.id_table	= eeprom_id,
};

static int __init eeprom_init(void)
{
	return i2c_add_driver(&eeprom_driver);
}

static void __exit eeprom_exit(void)
{
	i2c_del_driver(&eeprom_driver);
}

ssize_t eeprom_read_nv_item(enum eeprom_nv_item_enum item, char *buf, u16 bufsize)
{
	struct i2c_client *client = eeprom_nv_item_i2c_client;
	struct eeprom_data *data;

	if (client == NULL) {
		printk(KERN_ERR "ERROR, eeprom_read_nv_item(): eeprom_nv_item_i2c_client is NULL \n");
		return -ENODEV;
	}
	data = i2c_get_clientdata(client);

	if (bufsize < data->nv_item[item].num_byte) {
		printk(KERN_ERR "ERROR, %s: buffer size(%d) should be equal or more than %d. \n", __func__,
				bufsize, data->nv_item[item].num_byte);
		return -ENODEV;
	}
	return eeprom_nv_item_read(NULL, &client->dev.kobj, &data->nv_item[item].bin_attr, buf, 0, data->nv_item[item].num_byte);
}

ssize_t eeprom_write_nv_item(enum eeprom_nv_item_enum item, char *buf, u16 bufsize)
{
	struct i2c_client *client = eeprom_nv_item_i2c_client;
	struct eeprom_data *data;

	if (client == NULL) {
		printk(KERN_ERR "ERROR, eeprom_write_nv_item(): eeprom_nv_item_i2c_client is NULL \n");
		return -ENODEV;
	}
	data = i2c_get_clientdata(client);

	if (bufsize < data->nv_item[item].num_byte) {
		printk(KERN_ERR "ERROR, %s: buffer size(%d) should be equal or more than %d. \n", __func__,
				bufsize, data->nv_item[item].num_byte);
		return -ENODEV;
	}
	return eeprom_nv_item_write(NULL, &client->dev.kobj, &data->nv_item[item].bin_attr, buf, 0, data->nv_item[item].num_byte);
}

u16 eeprom_get_nv_item_size(enum eeprom_nv_item_enum item)
{
	struct i2c_client *client = eeprom_nv_item_i2c_client;
	struct eeprom_data *data;

	if (client == NULL) {
		printk(KERN_ERR "ERROR, %s(): eeprom_nv_item_i2c_client is NULL \n", __func__);
		return -ENODEV;
	}
	if (item >=  EEPROM_NV_ITEM_TOTAL) {
		printk(KERN_ERR "ERROR, %s(): enum: %d is invalid. \n", __func__, item);
		return -ENODEV;
	}
	data = i2c_get_clientdata(client);
	return data->nv_item[item].num_byte;
}

MODULE_AUTHOR("Frodo Looijaard <frodol@dds.nl> and "
		"Philip Edelbrock <phil@netroedge.com> and "
		"Greg Kroah-Hartman <greg@kroah.com>");
MODULE_DESCRIPTION("I2C EEPROM driver");
MODULE_LICENSE("GPL");

module_init(eeprom_init);
module_exit(eeprom_exit);
