// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Rockchip Electronics Co. Ltd.
 *
 * Author: Zorro Liu <zorro.liu@rock-chips.com>
 */

#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/major.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <linux/suspend.h>
#include <linux/soc/rockchip/rk_vendor_storage.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include "ebc_pmic.h"

//#define V3P3_CTRL_IN_POWER_ONOFF_OPS

#define SY7673A_MAX_REG_NUMBER		9

//register name and address
#define SY7673A_OPERATION_MODE_CTRL				0x00
#define SY7673A_VCOM_ADJUSTMENT_CTRL01			0x01
#define SY7673A_VCOM_ADJUSTMENT_CTRL02			0x02
#define SY7673A_VLDO_VOLTAGE_ADJUSTMENT_CTRL	0x03
#define SY7673A_POWER_ON_DELAY_TIME				0x06
#define SY7673A_FAULT_FLAG						0x07
#define SY7673A_THERMISTOR_READ_OUT				0x08


#define SY7673A_GET_STATE_INTERVAL_MS  8000

#define mv_to_vcom1_reg(mv)	(((mv) / 10) & 0xff)
#define mv_to_vcom2_reg(mv)	(((((mv) / 10) & 0x100) >> 8)<< 7)

//chencx add
int startup_vcom = 0 ;

struct sy7673a_sess {
	struct device *dev;
	struct i2c_client *client;
	uint8_t vcom1;
	uint8_t vcom2;
	struct gpio_desc *chip_en_pin;
	struct gpio_desc *vcom_ctrl_pin;
	struct gpio_desc *pwr_good_pin;
	struct gpio_desc *p_vdd_en_pin;
	struct mutex power_lock;
	struct workqueue_struct *tmp_monitor_wq;
	struct delayed_work tmp_delay_work;
};

static int sy7673a_pwr_up_flag = 0;
static int pre_temp = 25;

static int sy7673a_hw_setreg(struct sy7673a_sess *sess, uint8_t regaddr, uint8_t val)
{
	int stat;
	uint8_t txbuf[2] = { regaddr, val };
	struct i2c_msg msgs[] = 
	{
		{
			.addr = sess->client->addr,
			.flags = 0,
			.len = 2,
			.buf = txbuf,
		}
	};

	stat = i2c_transfer(sess->client->adapter, msgs, ARRAY_SIZE(msgs));

	if (stat < 0) 
	{
		printk("davidpeng,%s,Line:%d,i2c send error: %d\n",__func__,__LINE__,stat);
	} 
	else if (stat != ARRAY_SIZE(msgs)) 
	{
		printk("davidpeng,%s,Line:%d,i2c send N mismatch: %d\n",__func__,__LINE__,stat);
		stat = -EIO;
	} 
	else 
	{
		stat = 0;
	}

	return stat;
}

static int sy7673a_hw_getreg(struct sy7673a_sess *sess, uint8_t regaddr, uint8_t *val)
{
	int stat;
	struct i2c_msg msgs[] = 
	{
		{
			.addr = sess->client->addr,
			.flags = 0,
			.len = 1,
			.buf = &regaddr,
		},
		{
			.addr = sess->client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = val,
		}
	};

	stat = i2c_transfer(sess->client->adapter, msgs, ARRAY_SIZE(msgs));
	if (stat < 0) 
	{
		printk("davidpeng,%s,Line:%d,i2c read error: %d\n",__func__,__LINE__,stat);
	} 
	else if (stat != ARRAY_SIZE(msgs)) 
	{
		printk("davidpeng,%s,Line:%d,i2c read N mismatch: %d\n",__func__,__LINE__,stat);
		stat = -EIO;
	} 
	else 
	{
		stat = 0;
	}

	return stat;
}

#if 0
static bool sy7673a_hw_power_ack(struct sy7673a_sess *sss, int up)
{
	u8 pg_status;
	int st, ret, retries_left = 10;

	do 
	{
		ret = sy7673a_hw_getreg(sss, SY7673A_FAULT_FLAG, &pg_status);
		if (ret < 0)
			printk("davidpeng,%s,Line:%d,read SY7673A_FAULT_FLAG failed: %d\n", __func__,__LINE__,ret);

		printk("davidpeng,%s,Line:%d,SY7673A_FAULT_FLAG: 0x%02X\n", __func__,__LINE__,pg_status);

		pg_status &= 0x01;
		if ((pg_status == 0x01) && (up == 1)) 
		{
			st = 1;
			printk("davidpeng,%s,Line:%d, power up OK\n", __func__,__LINE__);
		} 
		else if ((pg_status == 0x00) && (up == 0)) 
		{
			st = 0;
			printk("davidpeng,%s,Line:%d, power down OK\n", __func__,__LINE__);
		} 
		else 
		{
			st = -1;	/* not settled yet */
			usleep_range(SY7673A_GET_STATE_INTERVAL_MS, (SY7673A_GET_STATE_INTERVAL_MS + 3000));
		}
		retries_left--;
	} while ((st == -1) && retries_left);

	if ((st == -1) && (retries_left == 0))
		printk("davidpeng,%s,Line:%d,power %s settle error (PG = 0x%02X)\n", __func__,__LINE__, up ? "up" : "down", pg_status);

	return (st == up);
}
#else
static bool sy7673a_hw_check_power_good(struct sy7673a_sess *sss, int up)
{
	int reg_val=0;
	int st, retries_left = 12;

	gpiod_direction_input(sss->pwr_good_pin);

	do 
	{
		reg_val = gpiod_get_value(sss->pwr_good_pin);

		//printk("davidpeng,%s,Line:%d,power good signal: %d\n", __func__,__LINE__,reg_val);

		if ((reg_val == 0x01) && (up == 1)) 
		{
			st = 1;
			//printk("davidpeng,%s,Line:%d, power up OK\n", __func__,__LINE__);
		} 
		else if ((reg_val == 0x00) && (up == 0)) 
		{
			st = 0;
			//printk("davidpeng,%s,Line:%d, power down OK\n", __func__,__LINE__);
		} 
		else 
		{
			st = -1;	/* not settled yet */
			usleep_range(SY7673A_GET_STATE_INTERVAL_MS, (SY7673A_GET_STATE_INTERVAL_MS + 3000));
		}
		retries_left--;
	} while ((st == -1) && retries_left);

	if ((st == -1) && (retries_left == 0))
	{
		printk("davidpeng,%s,Line:%d,power %s settle error\n", __func__,__LINE__, up ? "up" : "down");
	}
	else
	{
		//printk("davidpeng,%s,Line:%d,power %s settle OK\n", __func__,__LINE__, up ? "up" : "down");
	}

	return (st == up);
}

#endif

static void sy7673a_hw_send_powerup(struct sy7673a_sess *sess)
{
	int stat = 0;
	u8 reg_val;
	bool result;

	//printk("davidpeng,%s,Line:%d, entering\n",__func__,__LINE__);

	#ifdef V3P3_CTRL_IN_POWER_ONOFF_OPS
	gpiod_direction_output(sess->p_vdd_en_pin, 1);
	usleep_range(4 * 1000, 5 * 1000);
	#endif

	stat = sy7673a_hw_getreg(sess, SY7673A_OPERATION_MODE_CTRL, &reg_val);
	if (stat < 0)
	{
		printk("davidpeng,%s,Line:%d, read reg[0x00] failed\n",__func__,__LINE__);
	}
	
	reg_val |= 0x80;
	stat |= sy7673a_hw_setreg(sess, SY7673A_OPERATION_MODE_CTRL, reg_val);
	if (stat < 0)
	{
		printk("davidpeng,%s,Line:%d, POWER ON all power rails failed\n",__func__,__LINE__);
	}

	#if 0
	result = sy7673a_hw_power_ack(sess, 1);
	#else
	result = sy7673a_hw_check_power_good(sess, 1);
	#endif
	
	if (!result)
		printk("davidpeng,%s,Line:%d, power good signal checking is failed\n",__func__,__LINE__);

	stat = sy7673a_hw_getreg(sess, SY7673A_THERMISTOR_READ_OUT, &reg_val);
	if (stat < 0)
	{
		printk("davidpeng,%s,Line:%d, read temperature failed\n",__func__,__LINE__);
	}
	else
	{
		pre_temp = (int)reg_val;
		//printk("davidpeng,%s,Line:%d, read temperature ok, temperature: %d\n",__func__,__LINE__,pre_temp);
	}

	sy7673a_pwr_up_flag = 1;

	return;
}

static void sy7673a_hw_send_powerdown(struct sy7673a_sess *sess)
{
	int stat = 0;
	u8 reg_val;
	bool result;

	//printk("davidpeng,%s,Line:%d, entering\n",__func__,__LINE__);

	//return ;//added by pengwei 20210427 temp

	sy7673a_pwr_up_flag = 0;

	stat = sy7673a_hw_getreg(sess, SY7673A_OPERATION_MODE_CTRL, &reg_val);
	if (stat < 0)
	{
		printk("davidpeng,%s,Line:%d, read reg[0x00] failed\n",__func__,__LINE__);
	}

	reg_val &= 0x7F;
	stat |= sy7673a_hw_setreg(sess, SY7673A_OPERATION_MODE_CTRL, reg_val);
	if (stat < 0)
	{
		printk("davidpeng,%s,Line:%d, POWER OFF all power rails failed\n",__func__,__LINE__);
	}

	#if 0
	result = sy7673a_hw_power_ack(sess, 0);
	#else
	result = sy7673a_hw_check_power_good(sess, 0);
	#endif
	if (!result)
		printk("davidpeng,%s,Line:%d, power good signal checking is failed\n",__func__,__LINE__);

	#ifdef V3P3_CTRL_IN_POWER_ONOFF_OPS
	usleep_range(4 * 1000, 5 * 1000);
	gpiod_direction_output(sess->p_vdd_en_pin, 0);
	#endif

	return;
}

static void sy7673a_hw_arg_init(struct sy7673a_sess *sess)
{
	sess->vcom1 = mv_to_vcom1_reg(startup_vcom);
	sess->vcom2 = mv_to_vcom2_reg(startup_vcom);

	printk("====sess->vcom1=%d  sess->vcom2=%d startup_vcom=%d====\n", sess->vcom1, sess->vcom2, startup_vcom) ;
}

static int sy7673a_hw_init(struct sy7673a_sess *sess)
{
	printk("davidpeng,%s,Line:%d, entering\n",__func__,__LINE__);
	
	//chip enable
	gpiod_direction_output(sess->vcom_ctrl_pin, 0);
	gpiod_direction_output(sess->chip_en_pin, 1);
	msleep(20);

	#ifndef V3P3_CTRL_IN_POWER_ONOFF_OPS
	gpiod_direction_output(sess->p_vdd_en_pin, 1);
	msleep(10);
	#endif

	sy7673a_pwr_up_flag = 0;

	return 0;
}

static void sy7673a_set_vcom_voltage(struct sy7673a_sess *sess, int vcom_mv)
{
	sess->vcom1 = mv_to_vcom1_reg(vcom_mv);
	sess->vcom2 = mv_to_vcom2_reg(vcom_mv);
}

static int sy7673a_hw_read_temperature(struct ebc_pmic *pmic, int *t)
{
	struct sy7673a_sess *sess = (struct sy7673a_sess *)pmic->drvpar;
	int stat=0;
	uint8_t tb;

	//printk("davidpeng,%s,Line:%d, entering\n",__func__,__LINE__);

	if (sy7673a_pwr_up_flag == 0)
	{
		*t = pre_temp;
		printk("davidpeng,%s,Line:%d,all power rails are powered off, use last temperature\n",__func__,__LINE__);
		return stat;
	}
	
	stat = sy7673a_hw_getreg(sess, SY7673A_THERMISTOR_READ_OUT, &tb);
	if (stat < 0)
	{
		printk("davidpeng,%s,Line:%d,read temperature failed, use last temperature\n",__func__,__LINE__);
		*t = pre_temp;
	}
	else
	{
		*t = (int)tb;
		pre_temp = (int)tb;
	}

	//printk("davidpeng,%s,Line:%d,tb temp: %d, return temp: %d\n",__func__,__LINE__,tb, *t);

	return stat;
}

static void sy7673a_hw_power_req(struct ebc_pmic *pmic, bool up)
{
	struct sy7673a_sess *sess = (struct sy7673a_sess *)pmic->drvpar;

	if (up)
		mutex_lock(&sess->power_lock);

	if (up) 
	{
		sy7673a_hw_send_powerup(sess);
	} 
	else 
	{
		sy7673a_hw_send_powerdown(sess);
	}
	
	if (!up)
		mutex_unlock(&sess->power_lock);
	
	return;
}

static int sy7673a_hw_vcom_get(struct ebc_pmic *pmic)
{
	struct sy7673a_sess *sess = (struct sy7673a_sess *)pmic->drvpar;
	uint8_t rev_val = 0;
	int stat = 0;
	int read_vcom_mv = 0;

	printk("======chencx,will read vcom from sy7636a======\n") ;
	mutex_lock(&sess->power_lock);

	read_vcom_mv = 0;
	stat = sy7673a_hw_getreg(sess, SY7673A_VCOM_ADJUSTMENT_CTRL01, &rev_val);
	read_vcom_mv += rev_val;
	stat |= sy7673a_hw_getreg(sess, SY7673A_VCOM_ADJUSTMENT_CTRL02, &rev_val);
	read_vcom_mv += ((rev_val & 0x0080) << 1);

	if (stat)
	{
		printk("davidpeng,%s,Line:%d,sy7673a: I2C error: %d\n",__func__,__LINE__,stat);
	}
	
	mutex_unlock(&sess->power_lock);

	return read_vcom_mv * 10;
}

static int sy7673a_hw_vcom_set(struct ebc_pmic *pmic, int vcom_mv)
{
	struct sy7673a_sess *sess = (struct sy7673a_sess *)pmic->drvpar;
	int stat = 0;

	mutex_lock(&sess->power_lock);

	printk("======chencx,sy7636a vcom will set to %d======\n\n\n\n\n\n\n", vcom_mv) ;
	startup_vcom = vcom_mv ; //修改为新的值
	// Set vcom voltage
	sy7673a_set_vcom_voltage(sess, startup_vcom);
	//sy7673a_set_vcom_voltage(sess, vcom_mv);
	stat = sy7673a_hw_setreg(sess, SY7673A_VCOM_ADJUSTMENT_CTRL01, sess->vcom1);
	stat |= sy7673a_hw_setreg(sess, SY7673A_VCOM_ADJUSTMENT_CTRL02, sess->vcom2);

	if (stat)
	{
		printk("davidpeng,%s,Line:%d,sy7673a: I2C error: %d\n",__func__,__LINE__,stat);
	}
	
	mutex_unlock(&sess->power_lock);

	return 0;
}

static void sy7673a_pm_sleep(struct ebc_pmic *pmic)
{
	struct sy7673a_sess *s = (struct sy7673a_sess *)pmic->drvpar;

	cancel_delayed_work_sync(&s->tmp_delay_work);

	mutex_lock(&s->power_lock);
	
	gpiod_direction_output(s->chip_en_pin, 0);

	#ifndef V3P3_CTRL_IN_POWER_ONOFF_OPS
	gpiod_direction_output(s->p_vdd_en_pin, 0);
	#endif
	
	mutex_unlock(&s->power_lock);
}

static void sy7673a_pm_resume(struct ebc_pmic *pmic)
{
	struct sy7673a_sess *s = (struct sy7673a_sess *)pmic->drvpar;
	int stat = 0;

	mutex_lock(&s->power_lock);
	
	gpiod_direction_output(s->vcom_ctrl_pin, 0);
	
	#ifndef V3P3_CTRL_IN_POWER_ONOFF_OPS
	gpiod_direction_output(s->p_vdd_en_pin, 1);
	usleep_range(2 * 1000, 3 * 1000);
	#endif

	gpiod_direction_output(s->chip_en_pin, 1);
	usleep_range(3 * 1000, 5 * 1000);
	//chencx
	usleep_range(30 * 1000, 50 * 1000);
printk("----sy7673a_pm_resume  s->vcom1=%d-----\n", s->vcom1) ;
	//----
	stat = sy7673a_hw_setreg(s, SY7673A_VCOM_ADJUSTMENT_CTRL01, s->vcom1);
	stat |= sy7673a_hw_setreg(s, SY7673A_VCOM_ADJUSTMENT_CTRL02, s->vcom2);

	if (stat)
	{
		printk("davidpeng,%s,Line:%d,sy7673a: I2C error: %d\n",__func__,__LINE__,stat);
	}
	
	mutex_unlock(&s->power_lock);

	queue_delayed_work(s->tmp_monitor_wq, &s->tmp_delay_work,
			   msecs_to_jiffies(10000));
}

static void sy7673a_tmp_work(struct work_struct *work)
{
	struct sy7673a_sess *s =
		container_of(work, struct sy7673a_sess, tmp_delay_work.work);

	queue_delayed_work(s->tmp_monitor_wq, &s->tmp_delay_work,
			   msecs_to_jiffies(10000));
}

static int sy7673a_hw_probe(struct ebc_pmic *pmic, struct i2c_client *client)
{
	struct sy7673a_sess *sess;
	int stat;

	printk("--sy7673a_hw_probe--\n") ;

	sess = devm_kzalloc(&client->dev, sizeof(*sess), GFP_KERNEL);
	if (!sess) 
	{
		printk("davidpeng,%s,Line:%d, kzalloc failed\n",__func__,__LINE__);
		return -ENOMEM;
	}
	sess->client = client;
	mutex_init(&sess->power_lock);
//	sy7673a_hw_arg_init(sess);

	sess->chip_en_pin = devm_gpiod_get_optional(&client->dev, "chipen", GPIOD_OUT_HIGH);
	if (IS_ERR_OR_NULL(sess->chip_en_pin)) 
	{
		printk("davidpeng,%s,Line:%d, failed to find chipen pin, no defined\n",__func__,__LINE__);
		return -ENOMEM;
	}

	sess->vcom_ctrl_pin= devm_gpiod_get_optional(&client->dev, "vcomctl", GPIOD_OUT_HIGH);
	if (IS_ERR_OR_NULL(sess->vcom_ctrl_pin)) 
	{
		printk("davidpeng,%s,Line:%d, failed to find vcomctl pin, no defined\n",__func__,__LINE__);
		return -ENOMEM;
	}

	sess->pwr_good_pin= devm_gpiod_get_optional(&client->dev, "pwrgood", GPIOD_OUT_HIGH);
	if (IS_ERR_OR_NULL(sess->pwr_good_pin)) 
	{
		printk("davidpeng,%s,Line:%d, failed to find pwrgood pin, no defined\n",__func__,__LINE__);
		return -ENOMEM;
	}

	sess->p_vdd_en_pin= devm_gpiod_get_optional(&client->dev, "pvddctrl", GPIOD_OUT_HIGH);
	if (IS_ERR_OR_NULL(sess->p_vdd_en_pin))
	{
		printk("davidpeng,%s,Line:%d, failed to find pvddctrl pin, no defined\n",__func__,__LINE__);
		return -ENOMEM;
	}

	sess->tmp_monitor_wq = alloc_ordered_workqueue("%s", WQ_MEM_RECLAIM | WQ_FREEZABLE, "sy-tmp-monitor-wq");
	INIT_DELAYED_WORK(&sess->tmp_delay_work, sy7673a_tmp_work);
	queue_delayed_work(sess->tmp_monitor_wq, &sess->tmp_delay_work, msecs_to_jiffies(10000));

	stat = sy7673a_hw_init(sess);
	if (stat)
	{
		printk("davidpeng,%s,Line:%d, sy7673a_hw_init is failed, return now\n",__func__,__LINE__);
		return stat;
	}

	pmic->drvpar = sess;

	pmic->pmic_get_vcom = sy7673a_hw_vcom_get;
	pmic->pmic_set_vcom = sy7673a_hw_vcom_set;
	pmic->pmic_pm_resume = sy7673a_pm_resume;
	pmic->pmic_pm_suspend = sy7673a_pm_sleep;
	pmic->pmic_power_req = sy7673a_hw_power_req;
	pmic->pmic_read_temperature = sy7673a_hw_read_temperature;

	startup_vcom  = sy7673a_hw_vcom_get(pmic) ;
	sy7673a_hw_arg_init(sess) ;
	sy7673a_hw_vcom_set(pmic, startup_vcom) ;

	return 0;
}

static int sy7673a_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ebc_pmic *pmic;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		printk("davidpeng,%s,Line:%d, i2c check functionality failed.\n",__func__,__LINE__);
		return -ENODEV;
	}

	pmic = devm_kzalloc(&client->dev, sizeof(*pmic), GFP_KERNEL);
	if (!pmic) 
	{
		printk("davidpeng,%s,Line:%d, kzalloc failed\n",__func__,__LINE__);
		return -ENOMEM;
	}

	if (0 != sy7673a_hw_probe(pmic, client)) 
	{
		printk("davidpeng,%s,Line:%d, hw init failed.\n",__func__,__LINE__);
		return -ENODEV;
	}

	pmic->dev = &client->dev;
	sprintf(pmic->pmic_name, "sy7673a");
	i2c_set_clientdata(client, pmic);

	printk("davidpeng,%s,Line:%d, sy7673a probe ok.\n",__func__,__LINE__);

	return 0;
}

static int sy7673a_remove(struct i2c_client *client)
{
	sy7673a_pwr_up_flag = 0;
	
	return 0;
}

static const struct i2c_device_id sy7673a_id[] = {
	{ "sy7673a", 0 },
	{ }
};

static const struct of_device_id sy7673a_dt_ids[] = {
	{ .compatible = "sy,sy7673a", },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, sy7673a_dt_ids);
static struct i2c_driver sy7673a_driver = 
{
	.probe	= sy7673a_probe,
	.remove 	= sy7673a_remove,
	.id_table	= sy7673a_id,
	.driver = {
		.of_match_table = sy7673a_dt_ids,
		.name	  = "sy7673a",
		.owner	  = THIS_MODULE,
	},
};

module_i2c_driver(sy7673a_driver);

MODULE_DESCRIPTION("sy sy7673a pmic");
MODULE_LICENSE("GPL");
