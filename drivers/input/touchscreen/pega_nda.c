/*
 * Cryptographic API.
 *
 * MD5 Message Digest Algorithm (RFC1321).
 *
 * Derived from cryptoapi implementation, originally based on the
 * public domain implementation written by Colin Plumb in 1993.
 *
 * Copyright (c) Cryptoapi developers.
 * Copyright (c) 2002 James Morris <jmorris@intercode.com.au>
 * Copyright (c) 2011 Lon Lu <Lon_Lu@pegatroncorp.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 */

#include <crypto/md5.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/types.h>
#include <asm/byteorder.h>

#define F1(x, y, z)	(z ^ (x & (y ^ z)))
#define F2(x, y, z)	F1(z, x, y)
#define F3(x, y, z)	(x ^ y ^ z)
#define F4(x, y, z)	(y ^ (x | ~z))

#define MD5STEP(f, w, x, y, z, in, s) \
	(w += f(x, y, z) + in, w = (w<<s | w>>(32-s)) + x)

static void md5_transform(u32 *hash, u32 const *in)
{
	u32 a, b, c, d;

	a = hash[0];
	b = hash[1];
	c = hash[2];
	d = hash[3];

	MD5STEP(F1, a, b, c, d, in[0] + 0xd76aa478, 7);
	MD5STEP(F1, d, a, b, c, in[1] + 0xe8c7b756, 12);
	MD5STEP(F1, c, d, a, b, in[2] + 0x242070db, 17);
	MD5STEP(F1, b, c, d, a, in[3] + 0xc1bdceee, 22);
	MD5STEP(F1, a, b, c, d, in[4] + 0xf57c0faf, 7);
	MD5STEP(F1, d, a, b, c, in[5] + 0x4787c62a, 12);
	MD5STEP(F1, c, d, a, b, in[6] + 0xa8304613, 17);
	MD5STEP(F1, b, c, d, a, in[7] + 0xfd469501, 22);
	MD5STEP(F1, a, b, c, d, in[8] + 0x698098d8, 7);
	MD5STEP(F1, d, a, b, c, in[9] + 0x8b44f7af, 12);
	MD5STEP(F1, c, d, a, b, in[10] + 0xffff5bb1, 17);
	MD5STEP(F1, b, c, d, a, in[11] + 0x895cd7be, 22);
	MD5STEP(F1, a, b, c, d, in[12] + 0x6b901122, 7);
	MD5STEP(F1, d, a, b, c, in[13] + 0xfd987193, 12);
	MD5STEP(F1, c, d, a, b, in[14] + 0xa679438e, 17);
	MD5STEP(F1, b, c, d, a, in[15] + 0x49b40821, 22);

	MD5STEP(F2, a, b, c, d, in[1] + 0xf61e2562, 5);
	MD5STEP(F2, d, a, b, c, in[6] + 0xc040b340, 9);
	MD5STEP(F2, c, d, a, b, in[11] + 0x265e5a51, 14);
	MD5STEP(F2, b, c, d, a, in[0] + 0xe9b6c7aa, 20);
	MD5STEP(F2, a, b, c, d, in[5] + 0xd62f105d, 5);
	MD5STEP(F2, d, a, b, c, in[10] + 0x02441453, 9);
	MD5STEP(F2, c, d, a, b, in[15] + 0xd8a1e681, 14);
	MD5STEP(F2, b, c, d, a, in[4] + 0xe7d3fbc8, 20);
	MD5STEP(F2, a, b, c, d, in[9] + 0x21e1cde6, 5);
	MD5STEP(F2, d, a, b, c, in[14] + 0xc33707d6, 9);
	MD5STEP(F2, c, d, a, b, in[3] + 0xf4d50d87, 14);
	MD5STEP(F2, b, c, d, a, in[8] + 0x455a14ed, 20);
	MD5STEP(F2, a, b, c, d, in[13] + 0xa9e3e905, 5);
	MD5STEP(F2, d, a, b, c, in[2] + 0xfcefa3f8, 9);
	MD5STEP(F2, c, d, a, b, in[7] + 0x676f02d9, 14);
	MD5STEP(F2, b, c, d, a, in[12] + 0x8d2a4c8a, 20);

	MD5STEP(F3, a, b, c, d, in[5] + 0xfffa3942, 4);
	MD5STEP(F3, d, a, b, c, in[8] + 0x8771f681, 11);
	MD5STEP(F3, c, d, a, b, in[11] + 0x6d9d6122, 16);
	MD5STEP(F3, b, c, d, a, in[14] + 0xfde5380c, 23);
	MD5STEP(F3, a, b, c, d, in[1] + 0xa4beea44, 4);
	MD5STEP(F3, d, a, b, c, in[4] + 0x4bdecfa9, 11);
	MD5STEP(F3, c, d, a, b, in[7] + 0xf6bb4b60, 16);
	MD5STEP(F3, b, c, d, a, in[10] + 0xbebfbc70, 23);
	MD5STEP(F3, a, b, c, d, in[13] + 0x289b7ec6, 4);
	MD5STEP(F3, d, a, b, c, in[0] + 0xeaa127fa, 11);
	MD5STEP(F3, c, d, a, b, in[3] + 0xd4ef3085, 16);
	MD5STEP(F3, b, c, d, a, in[6] + 0x04881d05, 23);
	MD5STEP(F3, a, b, c, d, in[9] + 0xd9d4d039, 4);
	MD5STEP(F3, d, a, b, c, in[12] + 0xe6db99e5, 11);
	MD5STEP(F3, c, d, a, b, in[15] + 0x1fa27cf8, 16);
	MD5STEP(F3, b, c, d, a, in[2] + 0xc4ac5665, 23);

	MD5STEP(F4, a, b, c, d, in[0] + 0xf4292244, 6);
	MD5STEP(F4, d, a, b, c, in[7] + 0x432aff97, 10);
	MD5STEP(F4, c, d, a, b, in[14] + 0xab9423a7, 15);
	MD5STEP(F4, b, c, d, a, in[5] + 0xfc93a039, 21);
	MD5STEP(F4, a, b, c, d, in[12] + 0x655b59c3, 6);
	MD5STEP(F4, d, a, b, c, in[3] + 0x8f0ccc92, 10);
	MD5STEP(F4, c, d, a, b, in[10] + 0xffeff47d, 15);
	MD5STEP(F4, b, c, d, a, in[1] + 0x85845dd1, 21);
	MD5STEP(F4, a, b, c, d, in[8] + 0x6fa87e4f, 6);
	MD5STEP(F4, d, a, b, c, in[15] + 0xfe2ce6e0, 10);
	MD5STEP(F4, c, d, a, b, in[6] + 0xa3014314, 15);
	MD5STEP(F4, b, c, d, a, in[13] + 0x4e0811a1, 21);
	MD5STEP(F4, a, b, c, d, in[4] + 0xf7537e82, 6);
	MD5STEP(F4, d, a, b, c, in[11] + 0xbd3af235, 10);
	MD5STEP(F4, c, d, a, b, in[2] + 0x2ad7d2bb, 15);
	MD5STEP(F4, b, c, d, a, in[9] + 0xeb86d391, 21);

	hash[0] += a;
	hash[1] += b;
	hash[2] += c;
	hash[3] += d;
}

/* XXX: this stuff can be optimized */
static inline void le32_to_cpu_array(u32 *buf, unsigned int words)
{
	while (words--) {
		__le32_to_cpus(buf);
		buf++;
	}
}

static inline void cpu_to_le32_array(u32 *buf, unsigned int words)
{
	while (words--) {
		__cpu_to_le32s(buf);
		buf++;
	}
}

static inline void md5_transform_helper(struct md5_state *ctx)
{
	le32_to_cpu_array(ctx->block, sizeof(ctx->block) / sizeof(u32));
	md5_transform(ctx->hash, ctx->block);
}

static int md5_init(struct md5_state *mctx)
{
	mctx->hash[0] = 0x67452301;
	mctx->hash[1] = 0xefcdab89;
	mctx->hash[2] = 0x98badcfe;
	mctx->hash[3] = 0x10325476;
	mctx->byte_count = 0;

	return 0;
}

static int md5_update(struct md5_state *mctx, const u8 *data, unsigned int len)
{
	const u32 avail = sizeof(mctx->block) - (mctx->byte_count & 0x3f);

	mctx->byte_count += len;

	if (avail > len) {
		memcpy((char *)mctx->block + (sizeof(mctx->block) - avail),
		       data, len);
		return 0;
	}

	memcpy((char *)mctx->block + (sizeof(mctx->block) - avail),
	       data, avail);

	md5_transform_helper(mctx);
	data += avail;
	len -= avail;

	while (len >= sizeof(mctx->block)) {
		memcpy(mctx->block, data, sizeof(mctx->block));
		md5_transform_helper(mctx);
		data += sizeof(mctx->block);
		len -= sizeof(mctx->block);
	}

	memcpy(mctx->block, data, len);

	return 0;
}

static int md5_final(struct md5_state *mctx, u8 *out)
{
	const unsigned int offset = mctx->byte_count & 0x3f;
	char *p = (char *)mctx->block + offset;
	int padding = 56 - (offset + 1);

	*p++ = 0x80;
	if (padding < 0) {
		memset(p, 0x00, padding + sizeof (u64));
		md5_transform_helper(mctx);
		p = (char *)mctx->block;
		padding = 56;
	}

	memset(p, 0, padding);
	mctx->block[14] = mctx->byte_count << 3;
	mctx->block[15] = mctx->byte_count >> 29;
	le32_to_cpu_array(mctx->block, (sizeof(mctx->block) -
	                  sizeof(u64)) / sizeof(u32));
	md5_transform(mctx->hash, mctx->block);
	cpu_to_le32_array(mctx->hash, sizeof(mctx->hash) / sizeof(u32));
	memcpy(out, mctx->hash, sizeof(mctx->hash));
	memset(mctx, 0, sizeof(*mctx));

	return 0;
}


#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/firmware.h>

#define INCLUDE_MXT_STATIC_FUNCTION
#include <linux/i2c/atmel_maxtouch.h>
#include <linux/i2c/atmel_maxtouch_nda.h>

unsigned char mxt1386_fw[] = {0};

int g_object, g_offset, g_length;
int is_cfg_format_enabled = true;

static bool mxt_ota_rw_block(struct i2c_client *client, u16 flag, u16 length, u8 *value)
{
	struct i2c_msg msg;
	int i, i2c_status;
	bool is_OK = false;

	msg.addr = client->addr - 0x26;
	msg.flags = flag;
	msg.len = length;
	msg.buf = value;

	for (i = 0; i < I2C_RETRY_COUNT; i++) {
		i2c_status = i2c_transfer(client->adapter, &msg, 1);

		if (i2c_status == 1) {
			is_OK = true;
			break;
		}

		mdelay(20);
		printk(KERN_INFO "[Touch] - ERROR - Cannot access Touch by I2C (%d) (%d) ---> %s (%d)\n", i, i2c_status, __FUNCTION__, __LINE__);
	}

	return is_OK;
}

static int mxt_ota(const u8 *firmware_data, int fw_size)
{
	unsigned char	ota_status[1];
	unsigned int	offset = 0;
	unsigned int	frame_size, frame_count = 0;
	unsigned int	crc_error_count = 0;
	unsigned int	size1, size2;
	unsigned char	unlock_data[2] = {0xDC, 0xAA};

	bool is_new_frame = false;
	bool is_result = false;

	printk(KERN_INFO "[Touch] OTA start, please wait for 55 sec..\n");
	disable_irq(mxt->irq);

	/* set bootloader mode */
	mxt_write_byte(mxt->client, MXT_REG_T6_RESET, 0xA5);

	mdelay(200);

	while (1) {
		if (! mxt_ota_rw_block(mxt->client, I2C_M_RD, 1, &ota_status[0])) {
			printk(KERN_INFO "[Touch] - ERROR - Cannot read ota_status\n");
			if (frame_count >= 234) {
				is_result = true;
				printk(KERN_CRIT "[Lon] frame_count: %d ---> %s (%d)\n", frame_count, __FUNCTION__, __LINE__);
			}
			else
				printk(KERN_CRIT "[Lon] frame_count: %d ---> %s (%d)\n", frame_count, __FUNCTION__, __LINE__);
			goto out;
		}

		mdelay(60);

		if ((ota_status[0] & 0xC0) == FW_WAITING_BOOTLOAD_COMMAND) {
			/* set unlock mode */
			if (mxt_ota_rw_block(mxt->client, I2C_M_WRITE, sizeof(unlock_data), unlock_data))
				printk(KERN_INFO "[Touch] Unlock OK: 0x%2X\n", ota_status[0]);
			else
				printk(KERN_INFO "[Touch] - ERROR - Unlock fail\n");

			mdelay(10);
		}
		else if ((ota_status[0] & 0xC0) == FW_WAITING_FRAME_DATA) {
			 /* Add 2 to frame size, as the CRC bytes are not included */
			size1 = *(firmware_data + offset);
			size2 = *(firmware_data + offset + 1);

			frame_size = (size1 << 8) + size2 + 2;

/* 			printk(KERN_INFO "[Touch] Frame size:%d,\toffset: %d\n", frame_size, offset); */
			if (offset + frame_size > fw_size) {
				printk(KERN_INFO "[Touch] - ERROR - Out of boundary\n");
				is_result = false;

				goto out;
			}

			is_new_frame = true;
			mxt_ota_rw_block(mxt->client, I2C_M_WRITE, frame_size, (u8 *) firmware_data + offset);
			mdelay(10);
		}
		else if (ota_status[0] == FW_FRAME_CRC_CHECK) {
/* 			printk(KERN_INFO "[Touch] CRC checking..\n"); */
		}
		else if (ota_status[0] == FW_FRAME_CRC_PASS) {
			if (is_new_frame) {
/* 				printk(KERN_INFO "[Touch] CRC Ok\n"); */

				offset += frame_size;
				is_new_frame = false;
				frame_count++;
			}
			else
				printk(KERN_INFO "[Touch] - ERROR - Not a new frame\n");

			if (frame_count >= 234)
				printk(KERN_CRIT "[Lon] frame_count: %d, offset: %d ---> %s (%d)\n", frame_count, offset, __FUNCTION__, __LINE__);

			if (frame_count == 234 && offset == 67064) {
				is_result = true;
				goto out;
			}
		}
		else if (ota_status[0] == FW_FRAME_CRC_FAIL) {
			printk(KERN_INFO "[Touch] - ERROR - CRC fail\n");
			crc_error_count++;
		}
		else
			printk(KERN_CRIT "[Lon] - ERROR - Unexpected ota status: 0x%2X ---> %s (%d)\n", ota_status[0], __FUNCTION__, __LINE__);

		if (crc_error_count > 10) {
			printk(KERN_INFO "[Touch] - ERROR - FW_FRAME_CRC_FAIL ---> %s (%d)\n", __FUNCTION__, __LINE__);
			is_result = false;
			goto out;
		}
	}

out:

	gpio_set_value(mxt->gpio_reset, 0);
	enable_irq(mxt->irq);
	msleep(1);
	gpio_set_value(mxt->gpio_reset, 1);
	msleep(100);

	if (is_result) {
		printk(KERN_INFO "[Touch] OTA success\n");
		return 0;
	}
	else {
		printk(KERN_INFO "[Touch] OTA fail\n");
		return -1;
	}
}

ssize_t ota_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	char *buf_iter = buf;
	static bool is_finished = false;

	if (is_finished) {
		goto finished;
	}

	if (mxt_ota(mxt1386_fw, sizeof(mxt1386_fw)))
		buf_iter += sprintf(buf_iter, "[Touch] FW upgrade success\n");
	else
		buf_iter += sprintf(buf_iter, "[Touch] FW upgrade fail\n");

	is_finished = true;

	return buf_iter - buf;

finished:
	is_finished = false;
	return 0;
}

static void ota_worker(struct work_struct *work)
{
	const struct firmware *fw;
	char event[32] = {0};
	char *envp[] = {event, NULL};
	int status = -1;

	status = request_firmware(&fw, "mxt1386", &mxt->client->dev);

	if (status == 0) {
		pr_info("[Touch] FW size:%d\n", fw->size);
#if 0
		int i;
		for (i = 0; i < fw->size; i++) {
			if (i % 10 == 0)
				printk("\n");
			printk("0x%2X, ");
		}
#else
		status = mxt_ota(fw->data, fw->size);
#endif

		release_firmware(fw);

		if (status == 0) {
			snprintf(event, sizeof(event), "EVENT=FIRMWARE_UPDATE_SUCCESS");
			kobject_uevent_env(&mxt->client->dev.kobj, KOBJ_CHANGE, envp);
		} else {
			snprintf(event, sizeof(event), "EVENT=FIRMWARE_UPDATE_FAIL");
			kobject_uevent_env(&mxt->client->dev.kobj, KOBJ_CHANGE, envp);
		}
	}
	else {
		pr_err("[Touch] - ERROR - Request_firmware status = %d!\n", status);
		snprintf(event, sizeof(event), "EVENT=FIRMWARE_UPDATE_FAIL");
		kobject_uevent_env(&mxt->client->dev.kobj, KOBJ_CHANGE, envp);
	}
}

#define BUF_I2C_SIZE	64

ssize_t cfg_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	char *buf_iter = buf, buf_i2c[BUF_I2C_SIZE];
	static bool is_finished = false;
	static bool is_hex = false;
	int status, i, j;
	struct mxt_platform_data *pdata = mxt->client->dev.platform_data;
	struct mxt_cfg_object *mxt_object_list = pdata->cfg_data[0].cfg_objects;
	int object_length = pdata->cfg_data[0].cfg_objects_length;

	if (is_finished) {
		goto finished;
	}

	if (is_cfg_format_enabled)
		is_hex = !is_hex;
	else
		is_cfg_format_enabled = true;

	for (i = 0; i < object_length; i++) {
		buf_iter += sprintf(buf_iter, "- T%d -", mxt_object_list[i].type);

		if (mxt_object_list[i].instance != 0)
			buf_iter += sprintf(buf_iter, "\t[%d]", mxt_object_list[i].instance);

		if (mxt_object_list[i].length <= BUF_I2C_SIZE) {
			status = mxt_read_block( mxt->client,
					MXT_BASE_ADDR_EX(mxt_object_list[i].type, mxt_object_list[i].instance, mxt),
					mxt_object_list[i].length, &buf_i2c[0]);
			if (status == -EIO) {
				buf_iter = buf;
				goto err_exit;
			}
		}

		for (j = 0; j < mxt_object_list[i].length; j++) {
			if (j % 5 == 0)
				buf_iter += sprintf(buf_iter, "\n");
			if (is_hex)
				buf_iter += sprintf(buf_iter, "[%02d]: 0x%02X  ", j, buf_i2c[j]);
			else
				buf_iter += sprintf(buf_iter, "[%02d]: %-4d  ", j, buf_i2c[j]);
		}

		buf_iter += sprintf(buf_iter, "\n");
	}

	buf_iter += sprintf(buf_iter, "- T38 -");

	mxt_read_block( mxt->client,
			MXT_BASE_ADDR_EX(MXT_T38_USER_INFO, 0, mxt),
			5, &buf_i2c[0]);

	for (j = 0; j < 5; j++) {
		if (j % 5 == 0)
			buf_iter += sprintf(buf_iter, "\n");

			if (is_hex)
				buf_iter += sprintf(buf_iter, "[%02d]: 0x%02X  ", j, buf_i2c[j]);
			else
				buf_iter += sprintf(buf_iter, "[%02d]: %-4d  ", j, buf_i2c[j]);
	}

	buf_iter += sprintf(buf_iter, "\n\n");
	buf_iter += sprintf(buf_iter, "Total string size: %4d < %4d\n", buf_iter - buf + 32, count);

	is_finished = true;

	return buf_iter - buf;

finished:
	is_finished = false;
	return 0;
err_exit:
	buf_iter += sprintf(buf_iter, "[Touch] - ERROR - Cannot read Touch by I2C..\n");
	is_finished = true;
	return buf_iter - buf;
}

ssize_t cfg_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	static int i, count_i, object, offset, buf_i[10];

	count_i = sscanf(buf, "t%d %d "
			"0x%x 0x%x 0x%x 0x%x 0x%x "
			"0x%x 0x%x 0x%x 0x%x 0x%x ",
			&object, &offset,
			&buf_i[0], &buf_i[1], &buf_i[2], &buf_i[3], &buf_i[4],
			&buf_i[5], &buf_i[6], &buf_i[7], &buf_i[8], &buf_i[9]);

	if (count_i == 2) {
		count_i = sscanf(buf, "t%d %d "
				"%d %d %d %d %d "
				"%d %d %d %d %d ",
				&object, &offset,
				&buf_i[0], &buf_i[1], &buf_i[2], &buf_i[3], &buf_i[4],
				&buf_i[5], &buf_i[6], &buf_i[7], &buf_i[8], &buf_i[9]);
	}

	printk(KERN_INFO "[Touch] count_i: %d, buf: %s", count_i, buf);

	for (i = 0; i < count_i - 2; i++) {
		printk(KERN_INFO "[Touch] T%d[%d] = 0x%02x\n", object, offset + i, (char) buf_i[i]);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(object, mxt) + offset + i, buf_i[i] & 0xFF);
	}

	Write_Byte(MXT_T6_GEN_COMMANDPROCESSOR, MXT_ADR_T6_BACKUPNV, 0x55);

	is_cfg_format_enabled = false;

	return count;
}

struct tegra_i2c_bus {
	struct tegra_i2c_dev *dev;
	const struct tegra_pingroup_config *mux;
	int mux_len;
	unsigned long bus_clk_rate;
	struct i2c_adapter adapter;
};

struct tegra_i2c_dev {
	struct device *dev;
	struct clk *clk;
	struct clk *i2c_clk;
	struct resource *iomem;
	struct rt_mutex dev_lock;
	void __iomem *base;
	int cont_id;
	int irq;
	bool irq_disabled;
	int is_dvc;
	struct completion msg_complete;
	int msg_err;
	u8 *msg_buf;
	u32 packet_header;
	u32 payload_size;
	u32 io_header;
	size_t msg_buf_remaining;
	int msg_read;
	int msg_transfer_complete;
	struct i2c_msg *msgs;
	int msgs_num;
	bool is_suspended;
	int bus_count;
	const struct tegra_pingroup_config *last_mux;
	int last_mux_len;
	unsigned long last_bus_clk;
	struct tegra_i2c_bus busses[1];
};


ssize_t clk_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	struct i2c_adapter *adap = mxt->client->adapter;
	struct tegra_i2c_bus *i2c_bus = i2c_get_adapdata(adap);
	char *buf_iter = buf;
	static bool is_finished = false;

	if (is_finished) {
		goto finished;
	}

	buf_iter += sprintf(buf_iter, "[Touch] Bus clock rate: %d kHz\n", (int) i2c_bus->bus_clk_rate / 1000);

	is_finished = true;

	return buf_iter - buf;

finished:
	is_finished = false;
	return 0;
}

ssize_t clk_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	struct i2c_adapter *adap = mxt->client->adapter;
	struct tegra_i2c_bus *i2c_bus = i2c_get_adapdata(adap);
	static int count_i;
	int rate = 0;

	count_i = sscanf(buf, "%d", &rate);

	if (0 < rate && rate <= 4000)
		i2c_bus->bus_clk_rate = rate * 1000;
	else if (4000 < rate)
		i2c_bus->bus_clk_rate = 4000 * 1000;

	printk(KERN_INFO "[Touch] Bus clock rate: %d kHz\n", (int) i2c_bus->bus_clk_rate);

	return count;
}

ssize_t reg_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	char *buf_iter = buf, buf_i2c[BUF_I2C_SIZE];
	static bool is_finished = false;
	int j;

	if (is_finished) {
		goto finished;
	}

	buf_iter += sprintf(buf_iter, "- T%d -", g_object);

	if (g_length <= BUF_I2C_SIZE) {
		mxt_read_block( mxt->client,
				MXT_BASE_ADDR_EX(g_object, 0, mxt) + g_offset,
				g_length, &buf_i2c[0]);
	}

	for (j = 0; j < g_length; j++) {
		if (j % 5 == 0)
			buf_iter += sprintf(buf_iter, "\n");

		buf_iter += sprintf(buf_iter, "[%02d]: 0x%02X  ", g_offset + j, buf_i2c[j]);
	}

	buf_iter += sprintf(buf_iter, "\n\n");
	buf_iter += sprintf(buf_iter, "Total string size: %4d < %4d\n", buf_iter - buf + 32, count);

	is_finished = true;

	return buf_iter - buf;

finished:
	is_finished = false;
	return 0;
}

ssize_t reg_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	static int count_i;

	count_i = sscanf(buf, "t%d %d %d", &g_object, &g_offset, &g_length);

	printk(KERN_INFO "[Touch] T%d offset: %d length: %d (%d)\n", g_object, g_offset, g_length, MXT_BASE_ADDR(g_object, mxt) + g_offset);

	return count;
}

ssize_t init_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	char *buf_iter = buf;
	static bool is_finished = false;
	static int counter = 0, status = 0;
	u8 id_data[MXT_ID_BLOCK_SIZE] = {0};

	if (is_finished) {
		goto finished;
	}

	buf_iter += sprintf(buf_iter, "%d\n", ++counter);

	status = mxt_identify(mxt->client, mxt, id_data);
	if (status == 0)
		is_finished = true;

	return buf_iter - buf;

finished:
	is_finished = false;
	counter = 0;

	return 0;
}

ssize_t self_test_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	char *buf_iter = buf;
	static int i, high_test, test_value;
	static bool is_finished = false, is_free_run = false, is_test_low;

	if (is_finished) {
		goto finished;
	}

	if (! is_free_run) {
		mxt_write_block(mxt->client,
				MXT_BASE_ADDR_EX(MXT_T7_GEN_POWERCONFIG, 0, mxt),
				2, &t7_cfg_free_run[0]);

		test_value = 0;
		high_test = 65535;

		Write_Byte(MXT_T25_SPT_SELFTEST, MXT_ADDR_T25_CTRL, 0x03);
		Write_Byte(MXT_T25_SPT_SELFTEST, MXT_ADDR_T25_HISIGLIM_L, 0xFF & (high_test >> 0));
		Write_Byte(MXT_T25_SPT_SELFTEST, MXT_ADDR_T25_HISIGLIM_H, 0xFF & (high_test >> 8));

		is_test_low = true;
		is_free_run = true;

		buf_iter += sprintf(buf_iter, "Self test:");
	}

	self_test_result = e_self_test_none;
	test_value += 1000;

	if (is_test_low)
	{
		Write_Byte(MXT_T25_SPT_SELFTEST, MXT_ADDR_T25_LOSIGLIM_L, 0xFF & test_value);
		Write_Byte(MXT_T25_SPT_SELFTEST, MXT_ADDR_T25_LOSIGLIM_H, 0xFF & (test_value >> 8));

		Write_Byte(MXT_T25_SPT_SELFTEST, MXT_ADDR_T25_CMD, MXT_CMD_T25_ALL_TEST);

		for (i = 0; self_test_result == e_self_test_none && i < 20; i++) {
			msleep(100);	/* require 650 ~ 700 ms */
		}

		if (self_test_result == e_self_test_pass)
			buf_iter += sprintf(buf_iter, "\nTest low  limit: %5d --> Pass", test_value);
		else {
			buf_iter += sprintf(buf_iter, " <--\n");
			buf_iter += sprintf(buf_iter, "Test low  limit: %5d --> Fail\n", test_value);

			is_test_low = false;

			test_value -= 1000;
			Write_Byte(MXT_T25_SPT_SELFTEST, MXT_ADDR_T25_LOSIGLIM_L, 0xFF & test_value);
			Write_Byte(MXT_T25_SPT_SELFTEST, MXT_ADDR_T25_LOSIGLIM_H, 0xFF & (test_value >> 8));
			test_value += 1000;
		}
	}
	else { /* test_high */
		Write_Byte(MXT_T25_SPT_SELFTEST, MXT_ADDR_T25_HISIGLIM_L, 0xFF & test_value);
		Write_Byte(MXT_T25_SPT_SELFTEST, MXT_ADDR_T25_HISIGLIM_H, 0xFF & (test_value >> 8));

		Write_Byte(MXT_T25_SPT_SELFTEST, MXT_ADDR_T25_CMD, MXT_CMD_T25_ALL_TEST);

		for (i = 0; self_test_result == e_self_test_none && i < 20; i++) {
			msleep(100);	/* require 650 ~ 700 ms */
		}

		buf_iter += sprintf(buf_iter, "Test high limit: %5d --> ", test_value);

		if (self_test_result == e_self_test_fail)
			buf_iter += sprintf(buf_iter, "Fail\n");
		else {
			buf_iter += sprintf(buf_iter, "Pass <--");
			is_finished = true;
		}
	}

	return buf_iter - buf;

finished:
	mxt_write_block(mxt->client,
			MXT_BASE_ADDR_EX(MXT_T7_GEN_POWERCONFIG, 0, mxt),
			2, &t7_cfg[0]);
	is_free_run = false;
	is_finished = false;

	return 0;
}

/*
 * Copies messages from buffer to user space.
 *
 * NOTE: if less than (mxt->message_size * 5 + 1) bytes requested,
 * this will return 0!
 *
 */
ssize_t mxt_message_read(struct file *file, char *buf, size_t count,
			 loff_t *ppos)
{
	int i;
/* 	struct mxt_data *mxt; */
	char *buf_start;

/* 	mxt = file->private_data; */
	if (mxt == NULL)
		return -EIO;
	buf_start = buf;

	mutex_lock(&mxt->msg_mutex);
	/* Copy messages until buffer empty, or 'count' bytes written */
	while ((mxt->msg_buffer_startp != mxt->msg_buffer_endp) &&
	       ((buf - buf_start) < (count - 5 * mxt->message_size - 1))) {
		if (mxt->msg_buffer_endp < MXT_MESSAGE_BUFFER_SIZE - 1)
			mxt->msg_buffer_endp++;
		else
			mxt->msg_buffer_endp = 0;

		for (i = 0; i < mxt->message_size; i++) {
			buf += sprintf(buf, "%02X ",
				       *(mxt->messages + mxt->msg_buffer_endp *
					 mxt->message_size + i));
		}
		buf += sprintf(buf, "\n");
	}
	mutex_unlock(&mxt->msg_mutex);
	return buf - buf_start;
}

size_t debug_data_read(struct mxt_data *mxt, char *buf, size_t count,
			loff_t *ppos, u8 debug_command)
{
	int i;
	u16 *data;
	u16 diagnostics_reg;
	int offset = 0;
	int size;
	int read_size;
	int error;
	char *buf_start;
	u16 debug_data_addr;
	u16 page_address;
	u8 page;
	u8 debug_command_reg;

	data = mxt->debug_data;
	if (data == NULL)
		return -EIO;

	/* If first read after open, read all data to buffer. */
	if (mxt->current_debug_datap == 0) {

		diagnostics_reg = MXT_BASE_ADDR(MXT_T6_GEN_COMMANDPROCESSOR,
						mxt) + MXT_ADR_T6_DIAGNOSTIC;
		if (count > (mxt->device_info.num_nodes * 2))
			count = mxt->device_info.num_nodes;

		debug_data_addr = MXT_BASE_ADDR(MXT_T37_DEBUG_DIAGNOSTIC, mxt) +
		    MXT_ADR_T37_DATA;
		page_address = MXT_BASE_ADDR(MXT_T37_DEBUG_DIAGNOSTIC, mxt) +
		    MXT_ADR_T37_PAGE;
		error = mxt_read_block(mxt->client, page_address, 1, &page);
		if (error < 0)
			return error;
		mxt_debug(DEBUG_TRACE, "debug data page = %d\n", page);
		while (page != 0) {
			error = mxt_write_byte(mxt->client,
					       diagnostics_reg,
					       MXT_CMD_T6_PAGE_DOWN);
			if (error < 0)
				return error;
			/* Wait for command to be handled; when it has, the
			   register will be cleared. */
			debug_command_reg = 1;
			while (debug_command_reg != 0) {
				error = mxt_read_block(mxt->client,
						       diagnostics_reg, 1,
						       &debug_command_reg);
				if (error < 0)
					return error;
				mxt_debug(DEBUG_TRACE,
					  "Waiting for debug diag command "
					  "to propagate...\n");

			}
			error = mxt_read_block(mxt->client, page_address, 1,
					       &page);
			if (error < 0)
				return error;
			mxt_debug(DEBUG_TRACE, "debug data page = %d\n", page);
		}

		/*
		 * Lock mutex to prevent writing some unwanted data to debug
		 * command register. User can still write through the char
		 * device interface though. TODO: fix?
		 */

		mutex_lock(&mxt->debug_mutex);
		/* Configure Debug Diagnostics object to show deltas/refs */
		error = mxt_write_byte(mxt->client, diagnostics_reg,
				       debug_command);

		/* Wait for command to be handled; when it has, the
		 * register will be cleared. */
		debug_command_reg = 1;
		while (debug_command_reg != 0) {
			error = mxt_read_block(mxt->client,
					       diagnostics_reg, 1,
					       &debug_command_reg);
			if (error < 0)
				return error;
			mxt_debug(DEBUG_TRACE, "Waiting for debug diag command "
				  "to propagate...\n");

		}

		if (error < 0) {
			printk(KERN_WARNING
			       "Error writing to maXTouch device!\n");
			return error;
		}

		size = mxt->device_info.num_nodes * sizeof(u16);

		while (size > 0) {
			read_size = size > 128 ? 128 : size;
			mxt_debug(DEBUG_TRACE,
				  "Debug data read loop, reading %d bytes...\n",
				  read_size);
			error = mxt_read_block(mxt->client,
					       debug_data_addr,
					       read_size,
					       (u8 *) &data[offset]);
			if (error < 0) {
				printk(KERN_WARNING
				       "Error reading debug data\n");
				goto error;
			}
			offset += read_size / 2;
			size -= read_size;

			/* Select next page */
			error = mxt_write_byte(mxt->client, diagnostics_reg,
					       MXT_CMD_T6_PAGE_UP);
			if (error < 0) {
				printk(KERN_WARNING
				       "Error writing to maXTouch device!\n");
				goto error;
			}
		}
		mutex_unlock(&mxt->debug_mutex);
	}

	buf_start = buf;
	i = mxt->current_debug_datap;

	while (((buf - buf_start) < (count - 15)) &&
	       (i < mxt->device_info.num_nodes)) {

		mxt->current_debug_datap++;
		if (debug_command == MXT_CMD_T6_REFERENCES_MODE)
			buf += sprintf(buf, "[%d]\t%d\n", i,
				       (u16) le16_to_cpu(data[i]));
		else if (debug_command == MXT_CMD_T6_DELTAS_MODE)
			buf += sprintf(buf, "[%d]\t%d\n", i,
				       (s16) le16_to_cpu(data[i]));
		i++;
	}

	return buf - buf_start;
 error:
	mutex_unlock(&mxt->debug_mutex);
	return error;
}

ssize_t deltas_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	return debug_data_read(file->private_data, buf, count, ppos,
			       MXT_CMD_T6_DELTAS_MODE);
}

ssize_t refs_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	return debug_data_read(file->private_data, buf, count, ppos,
			       MXT_CMD_T6_REFERENCES_MODE);
}

int debug_data_open(struct inode *inode, struct file *file)
{
	struct mxt_data *mxt;
	int i;
	mxt = inode->i_private;
	if (mxt == NULL)
		return -EIO;
	mxt->current_debug_datap = 0;
	mxt->debug_data = kmalloc(mxt->device_info.num_nodes * sizeof(u16),
				  GFP_KERNEL);
	if (mxt->debug_data == NULL)
		return -ENOMEM;

	for (i = 0; i < mxt->device_info.num_nodes; i++)
		mxt->debug_data[i] = 7777;

	file->private_data = mxt;

	mxt_write_block(mxt->client,
			MXT_BASE_ADDR_EX(MXT_T7_GEN_POWERCONFIG, 0, mxt),
			2, &t7_cfg_free_run[0]);
	msleep(25);

	return 0;
}

int debug_data_release(struct inode *inode, struct file *file)
{
	struct mxt_data *mxt;
	mxt = file->private_data;
	kfree(mxt->debug_data);

	mxt_write_block(mxt->client,
			MXT_BASE_ADDR_EX(MXT_T7_GEN_POWERCONFIG, 0, mxt),
			2, &t7_cfg[0]);

	return 0;
}

const struct file_operations delta_fops = {
	.owner = THIS_MODULE,
	.open = debug_data_open,
	.release = debug_data_release,
	.read = deltas_read,
};

const struct file_operations refs_fops = {
	.owner = THIS_MODULE,
	.open = debug_data_open,
	.release = debug_data_release,
	.read = refs_read,
};

const struct file_operations cfg_fops = {
	.owner = THIS_MODULE,
	.read = cfg_read,
	.write = cfg_write,
};

const struct file_operations clk_fops = {
	.owner = THIS_MODULE,
	.read = clk_read,
	.write = clk_write,
};

const struct file_operations ota_fops = {
	.owner = THIS_MODULE,
	.read = ota_read,
};

const struct file_operations reg_fops = {
	.owner = THIS_MODULE,
	.read = reg_read,
	.write = reg_write,
};

const struct file_operations init_fops = {
	.owner = THIS_MODULE,
	.read = init_read,
};

const struct file_operations msg_fops = {
	.owner = THIS_MODULE,
	.read = mxt_message_read,
};

const struct file_operations self_test_fops = {
	.owner = THIS_MODULE,
	.read = self_test_read,
};

ssize_t pass_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	char *buf_iter = buf;
	static bool is_finished = false;

	if (is_finished) {
		goto finished;
	}

	buf_iter += sprintf(buf_iter, "Please call Lon for detail.\n");

	is_finished = true;

	return buf_iter - buf;

finished:
	is_finished = false;
	return 0;

}

ssize_t pass_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	static bool is_ready = false;
	bool	is_pass = true, is_ota = true;
	int i;
	struct md5_state mctx;
	u8 out[MD5_DIGEST_SIZE];
	const u8 md5_pass[] = {	0x33, 0x7C, 0xD2, 0xCA, 0x14, 0x46, 0xCF, 0xD4, 0x6A, 0xF5,
				0x5B, 0x5C, 0xEA, 0x64, 0xD6, 0xC4};
	const u8 md5_ota[] = {	0x87, 0x3A, 0x12, 0x36, 0x42, 0x17, 0xC3, 0x87, 0xE2, 0x55,
				0x0D, 0x98, 0xE0, 0x15, 0x94, 0x1F};

	md5_init(&mctx);
	md5_update(&mctx, (const u8*) buf, count);
	md5_final(&mctx, (u8*) &out);

#if 0
	printk(KERN_CRIT "[Lon] 0x%.2X, 0x%.2X, 0x%.2X, 0x%.2X, 0x%.2X, 0x%.2X, 0x%.2X, 0x%.2X, 0x%.2X, 0x%.2X, "
				"0x%.2X, 0x%.2X, 0x%.2X, 0x%.2X, 0x%.2X, 0x%.2X ---> %s (%d)\n", 
				out[0], out[1], out[2], out[3], out[4], out[5], out[6], out[7], out[8], out[9],
				out[10], out[11], out[12], out[13], out[14], out[15], __FUNCTION__, __LINE__);
#endif

	for (i = 0; i < MD5_DIGEST_SIZE; i++) {
		if (out[i] != md5_pass[i]) {
			is_pass = false;
			break;
		}
	}

	for (i = 0; i < MD5_DIGEST_SIZE; i++) {
		if (out[i] != md5_ota[i]) {
			is_ota = false;
			break;
		}
	}

	if (is_pass && ! is_ready) {
		debugfs_create_file("deltas", S_IRUSR, mxt->debug_dir, mxt,
				    &delta_fops);
		debugfs_create_file("refs", S_IRUSR, mxt->debug_dir, mxt,
				    &refs_fops);
		debugfs_create_file("cfg", S_IRUSR, mxt->debug_dir, mxt,
				    &cfg_fops);
		debugfs_create_file("reg", S_IRUSR, mxt->debug_dir, mxt,
				    &reg_fops);
		debugfs_create_file("clk", S_IRUSR, mxt->debug_dir, mxt,
				    &clk_fops);
		debugfs_create_file("ota", S_IRUSR, mxt->debug_dir, mxt,
				    &ota_fops);
		debugfs_create_file("self_test", S_IRUSR, mxt->debug_dir, mxt,
				    &self_test_fops);
		debugfs_create_file("init", S_IRUSR, mxt->debug_dir, mxt,
				    &init_fops);
		debugfs_create_file("msg", S_IRUSR, mxt->debug_dir, mxt,
				    &msg_fops);

		printk(KERN_INFO "[Touch] Create debug interface\n");
		is_ready = true;
	}
	else if (is_ota)
		ota_worker(NULL);

	return count;
}

const struct file_operations pass_fops = {
	.owner = THIS_MODULE,
	.read = pass_read,
	.write = pass_write,
};

