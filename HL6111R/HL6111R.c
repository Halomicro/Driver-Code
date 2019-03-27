/*
 * drivers/power/hl6111r_charger.c
 *
 * hl6111r charging driver
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * 2019/03/27
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/wakelock.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
//#include <linux/power/hl6111r_charger.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/notifier.h>
#include <linux/mutex.h>


#include <linux/charger_core.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>

#define MIN_INT                (1 << 31)
#define MAX_INT                (~(1 << 31))
#define TEMP_CTL_POINTS_NUM    (12)
#define TEMP_CTL_TYPE_NUM      (4)
#define TEMP_AREA_NUM          (13)



/*IRECT*/
#define HL6111R_IRECT_MEASURED_AVG   (0x02)
#define HL6111R_IRECT_LIMIT (0X29)
/*IRECT*/

/*VRECT*/
#define HL6111R_VRECT_MEASURED_AVG  (0x01)
#define HL6111R_VRECT_HEADROOM (0X27)
/*VRECT*/

/*IOUT*/
#define HL6111R_IOUT_LIMIT (0X28)
#define HL6111R_IOUT_AVG (0X8F)
/*IOUT*/

/*VOUT*/
#define HL6111R_VOUT_TARGET_H (0X24)
#define HL6111R_VOUT_RANGE (0X30) /*Vout under 0x26*/
#define HL6111R_VOUT_RANGE_MODE (0x26)
#define HL6111R_VOUT_TARGET_Q_MOEDE (0x0E)
#define HL6111R_VOUT_AVG (0X8E)
/*VOUT*/




/*TEMPRATURE MEASURED AVERAGE*/
#define HL6111R_DIE_TEMP_MEASURED_AVG (0x03)
/*TEMPRATURE MEASURED AVERAGE*/

/*OUT POWER LIMIT*/
#define HL6111R_POWER_MAX_LIMIT (0x0C)
/*OUT POWER LIMIT*/


/*PART_ID*/
#define HL6111R_PART_ID (0x0A)
/*PART_ID*/

/*BYPASS*/
#define HL6111R_VOUT_BYPASS (0X20)
/*BYPASS*/


/*STATUS */
#define HL6111R_STARTUS2 (0X76)

#define HL6111R_STATUS3 (0X77)

#define HL6111R_BUCK_STATUS_AND_CONTROL (0X78)

#define HL6111R_STATUS4 (0X7B) /*0bit Iout limit Flag*/
/*STATUS */



/*0x27 Vrect_headroom [7:2]*/
#define VRECT_HEAD_ROOM_5 (7)
#define VRECT_HEAD_ROOM_4 (6)
#define VRECT_HEAD_ROOM_3 (5)
#define VRECT_HEAD_ROOM_2 (4)
#define VRECT_HEAD_ROOM_1 (3)
#define VRECT_HEAD_ROOM_0 (2)
#define VRECT_HEAD_SIGN    (1)
/*0x27 Vrect_headroom*/

/*0x01*/
/* VOUT=code x 24/256*/
#define VRCET_9 7
#define VRCET_8 6
#define VRCET_7 5
#define VRCET_6 4
#define VRCET_5 3
#define VRCET_4 2
#define VRCET_3 1
#define VRCET_2 0
//////////////////////////////////////////////////////////////////


/*0x28 IOUT_LIMIT_DAC [7:3]*/
#define IOUTLIM_DAC_4 (7)
#define IOUTLIM_DAC_3 (6)
#define IOUTLIM_DAC_2 (5)
#define IOUTLIM_DAC_1 (4)
#define IOUTLIM_DAC_0 (3)
/*0x28 IOUT_LIMIT_DAC*/

/*0x8F*/
/*Iout=code x 9.171mA IOUT ADC*/
#define IOUT_7 7
#define IOUT_6 6
#define IOUT_5 5
#define IOUT_4 4
#define IOUT_3 3
#define IOUT_2 2
#define IOUT_1 1
#define IOUT_0 0
/////////////////////////////////////////////////////////////////
/*0x30 VOUT_RANGE_SELCTION*/
#define VOUT_RANGE_Q_1 (7)
#define VOUT_RANGE_Q_0 (6)
/*0x30 VOUT_RANGE_SELCTION*/

/*0x8E*/
/*Vout=code x 93.75mV VOUT ADC*/
#define VOUT_7 7
#define VOUT_6 6
#define VOUT_5 5
#define VOUT_4 4
#define VOUT_3 3
#define VOUT_2 2
#define VOUT_1 1
#define VOUT_0 0
////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////
/*0x02 IRECT MEASURED AVERGE*/
#define IRCET_7 7
#define IRCET_6 6
#define IRCET_5 5
#define IRCET_4 4
#define IRCET_3 3
#define IRCET_2 2
#define IRCET_1 1
#define IRCET_0 0
/*0x02 IRECT MEASURED AVERGE*/
/*IRECT=code x 13maA IRECT ADC*/

/*0x03 DIE_TEMP_MEASURE AVERAGE*/
#define TEMP_7 7
#define TEMP_6 6
#define TEMP_5 5
#define TEMP_4 4
#define TEMP_3 3
#define TEMP_2 2
#define TEMP_1 1
#define TEMP_0 0
/*0x03 DIE_TEMP_MEASURE AVERAGE*/
/*(220.09-code)/0.6316)*/
////////////////////////////////////////////////////////////////////

/* 0x00 STAUS REG */
#define I_LDO_L 7
#define OCA_L 5
#define OTA_L 4
#define OVA_L 1
#define OUT_EN_L 0
/* 0x00 STAUS REG */

/* 0x7B STAUS4 REG */
#define IOUT_LIMIT_FLAG 0 /*IOUT LIMIT FLAG*/
/* 0x7B STAUS4 REG */


struct hl6111r_device_info
{
    struct device        *dev;
    struct i2c_client    *client;
    struct delayed_work   hl6111r_charger_work;
    struct power_supply    charger;
    struct mutex    hot_limit_lock;
    struct mutex    current_change_lock;
    struct hl6111r_temp_control_info *temp_ctrl;

    unsigned int      wakelock_enabled;

    unsigned short    charge_voption_reg30;
    unsigned long    vout_read_val;
    unsigned int        voltagemV;
	
    unsigned short    charge_ioption_reg28;
    unsigned short    charge_iout_limit_reg8F;
    unsigned int       Iout_Limit_MA;

    unsigned short   vrect_ioption_reg01;
    unsigned short   charge_vrect_reg27; 
    unsigned int       Vrect;

    unsigned short  Iout_adc_reg8F;
    unsigned short  Vrect_adc_reg01;
    unsigned short  Irect_adc_reg02;
    unsigned short  Die_temp_reg03;
    unsigned short  Dev_status_show_reg00;
    unsigned short  Iout_limit_flay_reg7B;
	unsigned int    VOUT_TARGET   ;
	unsigned short  VOUT_RNG_SEL;

    unsigned int    cin_dpmmV;
    unsigned int    cin_limit;
    unsigned int    chrg_config;
    unsigned int    sys_minmV;
    unsigned int    currentmA;

    unsigned int    prechrg_currentmA;
    unsigned int    term_currentmA;
    unsigned int    watchdog_timer;
    unsigned int    chrg_timer;
    unsigned int    bat_compohm;
    unsigned int    comp_vclampmV;a
    unsigned int    bhot;
    bool    hz_mode;
    bool    boost_lim;
    bool    bcold_threshold;
    bool    enable_low_chg;
    bool    cfg_params;
    bool    enable_iterm;
    bool    enable_timer;
    bool    enable_batfet;
    bool    cd_active;
    bool    factory_flag;
    bool    calling_limit;
    bool    battery_present;
    bool    enable_dpdm;
    bool    rt_discharge_flag;

    int     charger_source;
    int     timer_fault;
    unsigned int    battery_temp_status;
    unsigned short          event;
	
    int     irq_int;
    int     battery_voltage;
    int     temperature_cold;
    int     temperature_cool;
    int     temperature_warm;
    int     temperature_hot;
    bool    not_limit_chrg_flag;
    bool    not_stop_chrg_flag;
    bool    battery_full;
    bool   soc_resume_charging;
    int     temperature_5;
    int     temperature_10;


    int  bat_temp_ctl_type;
    int  charge_status;
    int  charger_present;
    int  charge_current_limit;
    int  hot_limit_current;
    int  capacity;
};


static struct wake_lock chrg_lock;
static struct wake_lock stop_chrg_lock;
u32 wakeup_timer_seconds;

static unsigned int irq_int_active;
static unsigned int input_current_iin;
static unsigned int input_current_ichg;

static int HL6111R_write_block(struct hl6111r_device_info *di, u8 *value, u8 reg, unsigned num_bytes)
{
    struct i2c_msg msg[1];
    int ret = 0;

    *value = reg;

    msg[0].addr = di->client->addr;
    msg[0].flags = 0;
    msg[0].buf = value;
    msg[0].len = num_bytes + 1;

    ret = i2c_transfer(di->client->adapter, msg, 1);
    if (ret != 1)
    {
        pr_err("i2c_write failed to transfer all messages\n");
        if (ret < 0)
            return ret;
        else
            return -EIO;
    }
    else
    {
        return 0;
    }
}

static int HL6111R_read_block(struct hl6111r_device_info *di, u8 *value,u8 reg, unsigned num_bytes)
{
    struct i2c_msg msg[2];
    u8 buf = 0;
    int ret = 0;

    buf = reg;

    msg[0].addr = di->client->addr;
    msg[0].flags = 0;
    msg[0].buf = &buf;
    msg[0].len = 1;

    msg[1].addr = di->client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].buf = value;
    msg[1].len = num_bytes;

    ret = i2c_transfer(di->client->adapter, msg, 2);
    if (ret != 2)
    {
        pr_err("i2c_write failed to transfer all messages\n");
		
        if (ret < 0)
            return ret;
        else
            return -EIO;
    }
    else
    {
        return 0;
    }
}


static int HL6111R_write_byte(struct hl6111r_device_info *di, u8 value, u8 reg)
{

    u8 temp_buffer[2] = { 0 };

    temp_buffer[1] = value;
    return HL6111R_write_block(di, temp_buffer, reg, 1);
}

static int HL6111R_read_byte(struct hl6111r_device_info *di, u8 *value, u8 reg)
{
    return HL6111R_read_block(di, value, reg, 1);
}


/*VOUT_RNG_SEL REG CONFIG*/
static void HL6111R_config_voption_reg(struct hl6111r_device_info *di)
{

	HL6111R_write_byte(di, di->charge_voltage_reg30, HL6111R_VOUT_RANGE);
	
	return;

}
/*VOUT_TARGET CONFIG*/
static void HL6111R_config_vout_reg(struct hl6111r_device_info *di)
{

	HL6111R_write_byte(di, di->charge_voltage_reg0E, HL6111R_VOUT_RANGE);
	
	return;

}
/*GET_VOUT_VALUE*/
static int hl6111r_get_vout_regulation(struct hl6111r_device_info *di)
{
    unsigned long voutval;
    unsigned short val;
    unsigned int ret;
    ret=HL6111R_read_byte(di,val,HL6111R_VOUT_AVG,1);//0x8e
    if(ret)
         return 0;
	voutval = val * 93750; //uV
	di->vout_read_val = voutval;
    return voutval; 
}
/*SET_VOUT_REGULATION*/
static int hl6111r_set_vout_regulation(struct hl6111r_device_info *di)
{
    unsigned int voltagemV = 0; 
    u8 Vreg = 0;
	unsigned short val;
	unsigned short option;
    unsigned int ret;
    ret=HL6111R_read_byte(di,val,HL6111R_VOUT_RANGE,1);//0x30
	if(ret)
		 return 0;
	option = di->VOUT_RNG_SEL;
	HL6111R_VOUT_RANGE = (option<<6)&val;//option = 0,1,2,3;
	di->charge_voltage_reg30 = HL6111R_VOUT_RANGE;
	HL6111R_config_voption_reg(di);
	
	voltagemV = di->VOUT_TARGET;  //mV
	switch(di->VOUT_RNG_SEL)
	{
	case 0:
	    if((voltagemV>=4940) && (voltagemV<=10040))
          voltagemV = (voltagemV-4940)/20;
		else
              ;			
        break;		
	case 1:
	     if ((voltagemV>=7410) && (voltagemV<=15060))
        voltagemV = (voltagemV-7410)/30;
	     else 
		   ;
		break;
	case 2:
       	if ((voltagemV>=9880) && (voltagemV<=20080))
        voltagemV = (voltagemV-9880)/40;
	     else 
		   ;
		break;
	case 3:
        if ((voltagemV>=3952) && (voltagemV<=8032))
        voltagemV = (voltagemV-3952)/16;
	     else 
		   ;
       break;
    default;
	   break;	
	}
	di->charge_voption_reg0E = voltagemV;
	HL6111R_config_vout_reg(di);
    return;

}


static DEVICE_ATTR(vout_regulation, S_IWUSR | S_IRUGO,
                   hl6111r_get_vout_regulation,
                   hl6111r_set_vout_regulation);
//static DEVICE_ATTR(iout_limit_regulation, S_IWUSR | S_IRUGO,
//                  hl6111r_get_iout_limit_regulation,
//                   hl6111r_set_iout_limit_regulation);
//static DEVICE_ATTR(iout_adc, S_IWUSR | S_IRUGO,
//                  hl6111r_get_iout_adc,
//                  hl6111r_set_iout_adc);
//static DEVICE_ATTR(vrect_regulation, S_IWUSR | S_IRUGO,
//                   hl6111r_get_vrect_regulation,
//                   hl6111r_set_vrect_regulation);
//static DEVICE_ATTR(vrect_adc, S_IWUSR | S_IRUGO,
//                   hl6111r_get_vrect_adc,
//                   hl6111r_set_vrect_adc);
//static DEVICE_ATTR(irect_adc, S_IWUSR | S_IRUGO,
//                   hl6111r_get_irect_adc,
//                   hl6111r_set_irect_adc);
//static DEVICE_ATTR(die_temp, S_IWUSR | S_IRUGO,
//                   hl6111r_get_die_temp,
//                   hl6111r_set_die_temp);
//static DEVICE_ATTR(dev_status, S_IWUSR | S_IRUGO,
//                   hl6111r_get_dev_status,
//                   hl6111r_set_dev_status);
//static DEVICE_ATTR(iout_limit_flag, S_IWUSR | S_IRUGO,
//                   hl6111r_get_iout_limit_flag,
//                   hl6111r_set_iout_limit_flag);


static struct attribute *hl6111r_attributes[] =
{
     &dev_attr_vout_regulation.attr,
     //&dev_attr_iout_limit_regulation.attr,
     //&dev_attr_iout_adc.attr,
     //&dev_attr_vrect_regulation.attr,
     //&dev_attr_vrect_adc.attr,
     //&dev_attr_irect_adc.attr,
     //&dev_attr_die_temp.attr,
     //&dev_attr_dev_status.attr,
     //&dev_attr_iout_limit_flag.attr,
     NULL
}


static const struct attribute_group hl6111r_attr_group =
{
    .attrs = hl6111r_attributes,
};


static int hl6111r_enable_charge(int val)
{
/*
	if(!bq_device){
        pr_info("bq_device is null, do nothing\n");
        return -1;
    }
    hl6111r_set_enable_charger_for_factory_diag(bq_device, val);
*/
    return 0;
}


struct charge_device_ops hl6111r_ops = {
    .set_enable_charger = hl6111r_enable_charge,

};



static char *hl6111r_supplied_to[] =
{
    "bms",
/* Remove unused code */
};

static int hl6111r_power_supply_init(struct hl6111r_device_info *di)
{
    int ret;
    //di->charger.name = "battery";
    //di->charger.type = POWER_SUPPLY_TYPE_BATTERY;
    di->charger.name = "halo_rx";
    di->charger.type = POWER_SUPPLY_TYPE_HALO;
  //  di->charger.properties = hl6111r_power_supply_props;
  //  di->charger.num_properties = ARRAY_SIZE(hl6111r_power_supply_props);
  // di->charger.get_property = hl6111r_power_supply_get_property;
  // di->charger.set_property = hl6111r_power_supply_set_property;
  // di->charger.property_is_writeable = hl6111r_property_is_writeable;
  //  di->charger.external_power_changed = hl6111r_external_power_changed;
    di->charger.supplied_to = hl6111r_supplied_to;
    di->charger.num_supplicants =ARRAY_SIZE(hl6111r_supplied_to);

    ret = power_supply_register(di->dev, &di->charger);
    if (ret)
    {
        return ret;
    }

    return 0;
}

static int  hl6111r_charger_probe(struct i2c_client *client,
                                   const struct i2c_device_id *id)
{
    struct hl6111r_device_info *di;
    struct charge_device_ops *ops = NULL;
    int ret = 0;
    u8 read_reg = 0;

    di = kzalloc(sizeof(*di), GFP_KERNEL);
    if (!di)
    {
        pr_err("hl6111r_device_info is NULL!\n");
        return -ENOMEM;
    }

    di->dev = &client->dev;
    di->client = client;
    i2c_set_clientdata(client, di);
    di->usb_psy = usb_psy;
    ops = &hl6111r_ops;
    ret = charge_ops_register(ops);
    if(ret)
    {
        pr_err("register charge ops failed!\n");
        goto err_kfree;
    }
    wake_lock_init(&chrg_lock, WAKE_LOCK_SUSPEND, "hl6111r_chrg_wakelock");
    wake_lock_init(&stop_chrg_lock, WAKE_LOCK_SUSPEND, "hl6111r_stop_chrg_wakelock");
    mutex_init(&di->hot_limit_lock);
    mutex_init(&di->current_change_lock);
    spin_lock_init(&di->psy_lock);

    ret = sysfs_create_group(&client->dev.kobj, &hl6111r_attr_group);
    if (ret)
    {
        pr_debug("could not create sysfs files\n");
        goto err_sysfs;
    }

    if(!di->charger_present){
        di->event = CHARGER_REMOVED;
        schedule_work(&di->usb_work);
    }else{
        di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
    }

    bq_device = di;
    pr_info("hl6111r probe ok!\n");
    return 0;

err_sysfs:
    power_supply_unregister(&di->charger);
    mutex_destroy(&di->current_change_lock);
    mutex_destroy(&di->hot_limit_lock);
    wake_lock_destroy(&chrg_lock);
    wake_lock_destroy(&stop_chrg_lock);
    free_irq(di->irq_int,di);
err_irq_int:
err_int_map_irq:
    gpio_free(di->gpio_int);
err_io:
err_kfree:
    kfree(di);
    di = NULL;

    return ret;
}

static int  hl6111r_charger_remove(struct i2c_client *client)
{
    struct hl6111r_device_info *di = i2c_get_clientdata(client);

    sysfs_remove_group(&client->dev.kobj, &hl6111r_attr_group);
    wake_lock_destroy(&chrg_lock);
    wake_lock_destroy(&stop_chrg_lock);
    mutex_destroy(&di->current_change_lock);
    mutex_destroy(&di->hot_limit_lock);
    cancel_delayed_work_sync(&di->hl6111r_charger_work);
    free_irq(di->irq_int,di);
    gpio_free(di->gpio_int);
    kfree(di);
    return 0;
}

static void hl6111r_charger_shutdown(struct i2c_client *client)
{
    struct hl6111r_device_info *di = i2c_get_clientdata(client);
    return;
}

static const struct i2c_device_id hl6111r_id[] =
{
    { "hl6111r_charger", 0 },
    {},
};

#ifdef CONFIG_PM
static int hl6111r_charger_suspend(struct i2c_client *client,
                                    pm_message_t state)
{
    struct hl6111r_device_info *di = i2c_get_clientdata(client);

    return 0;
}

static int hl6111r_charger_resume(struct i2c_client *client)
{
    struct hl6111r_device_info *di = i2c_get_clientdata(client);

    return 0;
}
#else
#define hl6111r_charger_suspend       NULL
#define hl6111r_charger_resume        NULL
#endif 

MODULE_DEVICE_TABLE(i2c, hl6111r);
static struct of_device_id hl6111r_charger_match_table[] =
{
    {
        .compatible = "zte,hl6111r_charger",
        .data = NULL,
    },
    {
    },
};
static struct i2c_driver hl6111r_charger_driver =
{
    .probe = hl6111r_charger_probe,
    .remove = hl6111r_charger_remove,
    .suspend = hl6111r_charger_suspend,
    .resume = hl6111r_charger_resume,
    .shutdown = hl6111r_charger_shutdown,
    .id_table = hl6111r_id,
    .driver = {
        .owner = THIS_MODULE,
        .name = "hl6111r_charger",
        .of_match_table = of_match_ptr(hl6111r_charger_match_table),
    },
};

static int __init HL6111R_charger_init(void)
{
    return i2c_add_driver(&hl6111r_charger_driver);
}
rootfs_initcall(HL6111R_charger_init);

static void __exit HL6111R_charger_exit(void)
{
    i2c_del_driver(&hl6111r_charger_driver);
}
module_exit(HL6111R_charger_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("HALO Inc");
