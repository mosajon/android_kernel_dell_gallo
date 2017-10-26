/*
 *  LDS62XX capacitive sensor driver
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/delay.h>

#define LDS6202_STATUS		0x43
#define LDS6202_PROXIMITY	0x46
#define LDS6202_CAP_VALUE	0x83

static struct lds6202_data{
	struct i2c_client	*client;
	struct task_struct	*task;
	int					irq;
};

static int write_reg(struct i2c_client *pI2C,
					 unsigned short reg_addr, unsigned short data)
{
	unsigned char buffer[] = {0x00, 0x00, 0x00, 0x00};

	buffer[0] = (reg_addr>>8) & 0xFF;
	buffer[1] = reg_addr & 0xFF;
	buffer[2] = (data>>8) & 0xFF;
	buffer[3] = data & 0xFF;

	int ret = 0;
	ret = i2c_master_send(pI2C, buffer, sizeof(buffer));
	if (ret < 0) {
		printk(KERN_ERR "LDS6202: i2c write error %d\n", ret);
		return ret;
	}

	return 0;
}

static int read_reg(struct i2c_client *pI2C,
					unsigned short reg_addr, unsigned short *pData)
{
	unsigned char buffer[] = {0x00, 0x00};

	buffer[0] = (reg_addr>>8) & 0xFF;
	buffer[1] = reg_addr & 0xFF;

	int ret = 0;
	ret = i2c_master_send(pI2C, buffer, sizeof(buffer));
	if (ret < 0) {
		printk(KERN_ERR "LDS6202: i2c read error at send %d\n", ret);
		return -1;
	}
	ret = i2c_master_recv(pI2C, buffer, sizeof(buffer));
	if (ret < 0) {
		printk(KERN_ERR "LDS6202: i2c read error at recv %d\n", ret);
		return -1;
	} else {
		unsigned char* temp = (unsigned char *)pData;
		temp[0] = buffer[1];
		temp[1] = buffer[0];
	}

	return 0;
}

static void get_key_status(struct i2c_client *pI2C)
{
	unsigned short value = 0;
	char event[32] = {0};
	char *envp[] = {event, NULL};

	read_reg(pI2C, LDS6202_PROXIMITY, &value);

	if ((value>>3) & 0x01) {
		printk(KERN_INFO "%s() LDS6202 approaching\n", __func__);
		snprintf(event, sizeof(event), "EVENT=CAP_SENSOR_PRESS");
		kobject_uevent_env(&pI2C->dev.kobj, KOBJ_CHANGE, envp);
	} else {
		printk(KERN_INFO "%s() LDS6202 leaving\n", __func__);
		snprintf(event, sizeof(event), "EVENT=CAP_SENSOR_RELEASE");
		kobject_uevent_env(&pI2C->dev.kobj, KOBJ_CHANGE, envp);
	}
}

static ssize_t get_cap_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *pI2C = to_i2c_client(dev);
	unsigned short value = 0;

	read_reg(pI2C, LDS6202_CAP_VALUE, &value);

	return sprintf(buf, "%4d\n", value);
}
static DEVICE_ATTR(lds6202_cap, 0444, get_cap_value, 0);

static irqreturn_t lds6202_irq(int irq, void *_cap)
{
	struct lds6202_data *pCap = _cap;

	get_key_status(pCap->client);

	return IRQ_HANDLED;
}

static int lds6202_thread(void *pData)
{
	struct lds6202_data *pCap = (struct lds6202_data *)pData;
	unsigned int timeout = 0;

	for (;;) {
		if (kthread_should_stop())
			break;
		get_key_status(pCap->client);

		do {
			set_current_state(TASK_INTERRUPTIBLE);
			timeout = schedule_timeout(5 * HZ);
		} while (timeout);
	}
	return 0;
}

static int lds6202_initialize(struct lds6202_data *pCap)
{
	unsigned short index = 0;
	unsigned short buffer[][2] = {
		{0x0000, 0x0000}, /* cold reset */
		{0x0040, 0x0030}, /* touch disabled */
		{0x0041, 0x0008}, /* define sensor channel */
		{0x0042, 0x0008}, /* define interrupt channel */
		{0x004E, 0x0000}, /* SELC config */
		{0x0051, 0x0A1F}, /* ambient config */
		{0x0052, 0x07FF}, /* recalibration delay set to max */
		{0x0053, 0x07FF}, /* long touch */
		{0x0039, 0x3C08}, /* c3 proximity channel */
		{0x005F, 0x0001}, /* select page 1 for threshold */
		{0x0063, 0x000F}, /* c3 threshold 15 */
		{0x0023, 0x0100}, /* c3 debounce */
		{0x0040, 0xB000}, /* touch enabled, bit[5:4] shield pin state 00 floating */
		{0x0001, 0x0000}, /* soft reset */
	};
	for (index = 0; index < ARRAY_SIZE(buffer); index++)
		if(0 != write_reg(pCap->client, buffer[index][0], buffer[index][1]))
			return -1;
	return 0;
}

static int __init lds6202_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	printk(KERN_INFO "%s()", __func__);

	struct lds6202_data *pCap;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "%s() i2c check error\n", __func__);
		return -ENODEV;
	}

	pCap = kzalloc(sizeof(struct lds6202_data), GFP_KERNEL);
	if (!pCap)
		return -ENOMEM;

	pCap->irq = client->irq;
	pCap->client = client;
	i2c_set_clientdata(client, pCap);

	int ret = -1;
	ret = lds6202_initialize(pCap);
	if(ret != 0) {
		printk(KERN_ERR "LDS6202 initialize fail");
		return ret;
	}
	ret = request_threaded_irq(pCap->irq, NULL, lds6202_irq,
			IRQF_TRIGGER_LOW, "lds6202", pCap);
	if (ret < 0) {
		printk(KERN_ERR "LDS6202 request IRQ fail: %d\n", ret);
		kfree(pCap);
		return ret;
	}
	device_create_file(&pCap->client->dev, &dev_attr_lds6202_cap);

	return 0;
}


static int __devexit lds6202_remove(struct i2c_client *client)
{
	struct lds6202_data *pCap = i2c_get_clientdata(client);

	free_irq(pCap->irq, pCap);
	kfree(pCap);

	return 0;
}

static const struct i2c_device_id lds6202_id[] = {
	{ "lds6202", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lds6202_id);

static struct i2c_driver lds6202_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "lds6202",
	},
	.id_table	= lds6202_id,
	.probe		= lds6202_probe,
	.remove		= __devexit_p(lds6202_remove),
};

static int __init lds6202_init(void)
{
	return i2c_add_driver(&lds6202_driver);
}

static void __exit lds6202_exit(void)
{
	i2c_del_driver(&lds6202_driver);
}

MODULE_DESCRIPTION("LDS6202 driver");

module_init(lds6202_init);
module_exit(lds6202_exit);

