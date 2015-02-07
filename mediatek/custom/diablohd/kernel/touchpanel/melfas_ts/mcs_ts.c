#include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/reboot.h>
#include "tpd_custom_mcs.h"

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#include "cust_gpio_usage.h"
#include <linux/input/mt.h>

#define KPD_BUTTON_KEY	1

#if KPD_BUTTON_KEY
#include <mach/mt_pwm.h>
#include <mach/upmu_hw.h>
#include <mach/pmic_mt6329_sw.h>
#include <mach/pmic_mt6329_hw.h>
#include <mach/pmic_mt6329_sw_bank1.h>
#include <mach/pmic_mt6329_hw_bank1.h>
#endif

#define MCS_ON 1
 



#if KPD_BUTTON_KEY
void backkey_handler(void);
void homekey_handler(void);
void menukey_handler(void);



static DECLARE_TASKLET(homekey_tasklet, homekey_handler, 0);
static DECLARE_TASKLET(menukey_tasklet, menukey_handler, 0);

static DECLARE_TASKLET(backkey_tasklet, backkey_handler, 0);

#endif




extern struct tpd_device *tpd;
 
struct i2c_client *i2c_client = NULL;
struct task_struct *thread = NULL;
 
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DEFINE_MUTEX(i2c_access);
 
 
static void tpd_eint_interrupt_handler(void);
void release_all_fingers(void);
 
#ifdef MT6575 
 extern void mt65xx_eint_unmask(unsigned int line);
 extern void mt65xx_eint_mask(unsigned int line);
 extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
 extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
 extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
 extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
									  kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
									  kal_bool auto_umask);
#endif
#ifdef MT6577
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif

 
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
static int __mms114_read_reg(unsigned int reg,unsigned int len, u8 *val); 
static int mms114_set_palm_func(void);
static int firmware_check(void);

/*add for firmware update*/
#define MELFAS_FIRMWARE_UPDATE 0

#if MELFAS_FIRMWARE_UPDATE
#include "diabloHD_firmware_0x35.h"

extern int isp_fw_download(const UINT8 *pBianry, const UINT16 unLength);
//extern int melfas_fw_download(const u8 *data, size_t len);

#define TS_READ_HW_VER_ADDR 0xC1 //Model Dependent
#define TS_READ_SW_VER_ADDR 0xC3 //Model Dependent
#define LASTEST_VERSION 0x35
#endif

#if MCS_ON

struct mms114_platform_data {
	unsigned int x_size;
	unsigned int y_size;
	unsigned int contact_threshold;
	unsigned int moving_threshold;
	bool x_invert;
	bool y_invert;

	void (*cfg_pin)(bool);
};

static u8 current_hw_palm = 0x54;
static u8 current_chip_palm = 0x32;
static u8 current_hw = 0x45;
static u8 current_chip = 0x19;

static int palm_enable = 0;	/*0:disable palm 1: enable palm*/

static int palm_counter = 0;
static int palm_flag = 0; /* palm event flag 1:true*/

static u8 tsr_ver = 0;
static u8 hw_fw_ver = 0;
static u8 chip_fw_ver = 0;
static u8 err_num = 0;

int touch_mark[10] = {0};		/*add for debug*/

struct fw_point_state
{
	int  updated[10];
	unsigned int pressed[10];

	u8 max_id_num;
} ;

struct fw_point_state *fw_point_state = NULL;

static ssize_t tp_value_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	char *s = buf;
	int i =0;
	//int result;

	s += sprintf(s, "CTP Vendor: %s\n", "melfas");
	s += sprintf(s, "TSP ev: %x\n", tsr_ver);
	s += sprintf(s, "HW Rev: %x\n", hw_fw_ver);
	s += sprintf(s, "Firmware Ver: %x\n", chip_fw_ver);
      s += sprintf(s,"CTP Type: %s\n", "Interrupt trigger");
	s += sprintf(s, "%s\n", palm_enable ? "palm enabled" : "palm disabled");
	s += sprintf(s, "err_num: %d\n", err_num);

	for (i = 0; i < 10; i++)
	{
		s += sprintf(s, "touch_mark[%d]:%d\n",i,touch_mark[i]);
	}

	for (i = 0; i < 10; i++)
	{
		s += sprintf(s, "update[%d]:%d\n",i,fw_point_state->updated[i]);
	}
	for (i = 0; i < 10; i++)
		s += sprintf(s, "pressed[%d]:%d\n",i,fw_point_state->pressed[i]);
	
	return (s - buf);
}

static ssize_t tp_value_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	int save = 0;
	int i = 0;

	if (sscanf(buf, "%d", &save)==0) {
		printk(KERN_ERR "%s -- invalid save string '%s'...\n", __func__, buf);
		return -EINVAL;
	}


	return n;
}

struct kobject *tpswitch_ctrl_kobj;

#define tpswitch_ctrl_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0644,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}


tpswitch_ctrl_attr(tp_value);

static struct attribute *g_attr[] = {
	&tp_value_attr.attr,	
	NULL,
};

static struct attribute_group tpswitch_attr_group = {
	.attrs = g_attr,
};

static int tpswitch_sysfs_init(void)
{ 	
	fw_point_state = kzalloc(sizeof(struct fw_point_state), GFP_KERNEL);
	
	tpswitch_ctrl_kobj = kobject_create_and_add("tp_compatible", NULL);
	if (!tpswitch_ctrl_kobj)
		return -ENOMEM;
	
	return sysfs_create_group(tpswitch_ctrl_kobj, &tpswitch_attr_group);
}

static void tpswitch_sysfs_exit(void)
{
	sysfs_remove_group(tpswitch_ctrl_kobj, &tpswitch_attr_group);

	kobject_put(tpswitch_ctrl_kobj);
}




/* Write only registers */
#define MMS114_MODE_CONTROL		0x01
#define MMS114_OPERATION_MODE_MASK	0xE
#define MMS114_ACTIVE			(1 << 1)

#define MMS114_XY_RESOLUTION_H		0x02
#define MMS114_X_RESOLUTION		0x03
#define MMS114_Y_RESOLUTION		0x04
#define MMS114_CONTACT_THRESHOLD	0x05
#define MMS114_MOVING_THRESHOLD		0x06

/* Read only registers */
#define MMS114_PACKET_SIZE		0x0F

#define MMS114_INFOMATION		0x10
#define MMS114_ACT_OFFSET		7
#define MMS114_ACT_MASK			0x1
#define MMS114_TYPE_OFFSET		5
#define MMS114_TYPE_MASK		0x3
#define MMS114_ID_MASK			0xF

#define MMS114_TSP_REV			0xE1

/* Minimum delay time is 50us between stop and start signal of i2c */
#define MMS114_I2C_DELAY		50

/* 200ms needs after power on */
#define MMS114_POWERON_DELAY		200

/* Touchscreen absolute values */
#define MMS114_MAX_AREA			0xff

#define MMS114_MAX_TOUCH		10
#define MMS114_PACKET_NUM		6
#define MMS114_MAX_PACKET		((MMS114_MAX_TOUCH+1) * MMS114_PACKET_NUM)

/* Touch type */
#define MMS114_TYPE_NONE		0
#define MMS114_TYPE_TOUCHSCREEN		1
#define MMS114_TYPE_TOUCHKEY		2


struct mms114_touchdata {
	unsigned int x;
	unsigned int y;
	unsigned int width;
	unsigned int strength;
	unsigned int pressed;
	unsigned int palm_flag;
	bool updated;
};

struct mms114_data {
	struct i2c_client	*client;
//	struct input_dev	*input_dev;
//	struct mutex		mutex;
	struct mms114_touchdata	touchdata[MMS114_MAX_TOUCH];
//	struct regulator	*io_reg;
//	const struct mms114_platform_data	*pdata;

	/* Use cache data for mode control register(write only) */
	u8	cache_mode_control;
};


#define MELFAS_I2C_RETRY_TIMES 5

struct fw_info 
{
	u8 core_ver;
	u8 build_ver;
};

struct fw_info *fw_info = NULL;
/*add sysfs here*/
/*****************************************************************************/
static ssize_t mms114_fw_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int ret = 0;
	
	return snprintf(buf, PAGE_SIZE, "TP_FW:%x_%02x\n",
					fw_info->core_ver,fw_info->build_ver);
}
static DEVICE_ATTR(fw_version, S_IRUGO, mms114_fw_version_show, NULL);
static struct attribute *mms114_attrs[] = {
	&dev_attr_fw_version.attr,
	NULL
};
static const struct attribute_group mms114_attr_group = {
	.attrs = mms114_attrs,
//	.name = "mxt_tp",	
};
static int ctp_sysfs_init()
{
	int error = 0;
	int ret = 0;
	u8 buf[6] = {0};
	struct kobject *ctp_kobj = NULL;
	
	fw_info = kzalloc(sizeof(struct fw_info), GFP_KERNEL);

	ctp_kobj = kobject_create_and_add("ctp", NULL);
	if(ctp_kobj == NULL)
	{
		printk("kobject_create_and_add error\n");
	}
	else
	{
		printk("****path:%s*****\n",ctp_kobj->name);
	}

	error = sysfs_create_group(ctp_kobj,&mms114_attr_group);
	if (error != NULL) {
		printk( "Failure %d creating sysfs group\n",error);
		return  -EINVAL;
	}
	
	ret = __mms114_read_reg(MMS114_TSP_REV, 4, buf);
	if (ret < 0)
		return ret;
	fw_info->core_ver = buf[1];
	fw_info->build_ver = buf[2];

}

/*****************************************************************************/


//add for debug
/******************************************************/

static int get_fw_point_state(struct mms114_data *data)
{
	int i = 0;
	
	struct mms114_touchdata *touchdata = data->touchdata;

	for (i = 0; i <MMS114_MAX_TOUCH; i++ )
	{
		fw_point_state->pressed[i] = touchdata[i].pressed;
		fw_point_state->updated[i] = touchdata[i].updated;
	}

	return 0;
}

static int max_id_num_check(int id)
{
	int i = 0;
	if(id > fw_point_state->max_id_num )
	{
		fw_point_state->max_id_num = id;
		for(i = 0; i < id; i++)
		{
			if (!touch_mark[i])
			{	
				printk(KERN_ERR "***max_id_num:%d,id: %d,error_id:%d***\n",id,i,touch_mark[i]);
				return -1;
			}
		}
	}

	return 0;
}

/******************************************************/


static int __mms114_read_reg(unsigned int reg,
			     unsigned int len, u8 *val)
{
	u8 buf = reg & 0xff;
	int retry;

	int ret = 0; //dhlee

	if (reg == MMS114_MODE_CONTROL) {
		return -EINVAL;
	}
	i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
	struct i2c_msg msg[] = {
		{
			.addr = i2c_client->addr,
			.flags = 0,
			.len = 1,
			.buf = &buf,
		},
		{
			.addr = i2c_client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = val,
		}
	};
	for (retry = 0; retry < MELFAS_I2C_RETRY_TIMES; retry++) {
		//dhlee
		ret = i2c_transfer(i2c_client->adapter, msg, 2);
		printk(KERN_ERR "i2c_transfer %d\n",ret);
		if(ret == 2)
			break;
#if 0
		if (i2c_transfer(i2c_client->adapter, msg, 2) == 2)
			break;
#endif
		mdelay(5);
	}
	if (retry == MELFAS_I2C_RETRY_TIMES) {
		printk(KERN_ERR "i2c_read_block retry over %d\n",
			MELFAS_I2C_RETRY_TIMES);
		return -EIO;
	}
	return 0;

}


static int mms114_read_reg(unsigned int reg)
{
	u8 val;
	int ret;

	if (reg == MMS114_MODE_CONTROL)
		return -1;

	ret = __mms114_read_reg(reg, 1, &val);
	if (!ret)
		ret = val;

	return ret;
}


static int mms114_write_reg(struct mms114_data *data, unsigned int reg,
			    unsigned int val)
{
	struct i2c_client *client = data->client;
	u8 buf[2];
	int ret;

	buf[0] = reg & 0xff;
	buf[1] = val & 0xff;

	ret = i2c_master_send(client, buf, 2);
	udelay(MMS114_I2C_DELAY);
	if (ret != 2) {
		return -EIO;
	}

	if (reg == MMS114_MODE_CONTROL)
		data->cache_mode_control = val;

	return 0;
}
static unsigned char palm_area_flag = 0;
//change from 40 to 100
#define TRIGGER_AREA 80
static void mms114_input_report(struct mms114_data *data)
{
	int id;	
	struct mms114_touchdata *touchdata = data->touchdata;
	
	for (id = 0; id < MMS114_MAX_TOUCH; id++) {

		if (!touchdata[id].updated)
			continue;
		touchdata[id].updated = false;

		printk("=======> touch data id: %d palm flag: %d ,pressed:%d\n", id, touchdata[id].palm_flag,touchdata[id].pressed);

		if (touchdata[id].palm_flag) {		//strength change from 40 to 100
//			printk("xx palm down strength %d xxxx\n", touchdata[id].strength);	
			if (touchdata[id].pressed == 1) {
				printk("palm down strength: %d count:%d xxxx\n", touchdata[id].strength, palm_counter);
				palm_area_flag = 1;
				input_report_key(tpd->dev, KEY_PALMEVENT, 1);
				input_sync(tpd->dev);
				palm_counter++;
			} else if (touchdata[id].pressed == 0) {
				printk("palm up strength: %d count:%d xxxx\n", touchdata[id].strength, palm_counter);
				if(palm_area_flag == 1)
				{
					input_report_key(tpd->dev, KEY_PALMEVENT, 0);
					input_sync(tpd->dev);
				}
				palm_area_flag = 0;
				palm_counter++;
			}
		} else if (touchdata[id].palm_flag == 0) {
			if (touchdata[id].pressed) {
				input_mt_slot(tpd->dev, id);
				input_report_key(tpd->dev, BTN_TOUCH, 1);
				input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id+2);

				input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR,
						touchdata[id].width);
				input_report_abs(tpd->dev, ABS_MT_POSITION_X,
						touchdata[id].x);
				input_report_abs(tpd->dev, ABS_MT_POSITION_Y,
						touchdata[id].y);
			}
			else if(!touchdata[id].pressed)
			{
				input_mt_slot(tpd->dev, id);
				input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, -1);
			}
		}
	}
	input_sync(tpd->dev);
}

static void mms114_input_report_dhlee(struct mms114_data *data)
{
	int id, i;	
	struct mms114_touchdata *touchdata = data->touchdata;


	for (id = 0; id < MMS114_MAX_TOUCH; id++) {
		if (!touchdata[id].updated)
			continue;
		touchdata[id].updated = false;

		printk("=======> touch data id: %d palm flag: %d ,pressed:%d\n", id, touchdata[id].palm_flag,touchdata[id].pressed);
		
		if(touchdata[id].pressed )
		{
			printk("=======> touch data id: %d palm flag: %d pressed\n", id, touchdata[id].palm_flag);
			if(touchdata[id].palm_flag)
			{
//				for(i=0; i<MMS114_MAX_TOUCH; i++)
//					input_mt_slot(tpd->dev, i);
//				}	
//				input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, -1);

				release_all_fingers();		

				input_report_key(tpd->dev, KEY_PALMEVENT, 1);
				input_sync(tpd->dev);
				palm_counter++;
				break;
			}
			else
			{
				input_mt_slot(tpd->dev, id);
				input_report_key(tpd->dev, BTN_TOUCH, 1);
				input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id+2);

				input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR,
						touchdata[id].width);
				input_report_abs(tpd->dev, ABS_MT_POSITION_X,
						touchdata[id].x);
				input_report_abs(tpd->dev, ABS_MT_POSITION_Y,
						touchdata[id].y);
			}
		}
		else if(touchdata[id].pressed == 0)
		{
			printk("=======> touch data id: %d palm flag: %d released\n", id, touchdata[id].palm_flag);
			if(touchdata[id].palm_flag)
			{
				input_mt_slot(tpd->dev, id);
				input_report_key(tpd->dev, KEY_PALMEVENT, 0);
				input_sync(tpd->dev);
				palm_counter++;
			}
			else
			{
				input_mt_slot(tpd->dev, id);
				input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, -1);
			}
		}
	}
	input_sync(tpd->dev);
}

static void mms114_input_report_dhlee2(struct mms114_data *data)
{
	int id, i;	
	
	struct mms114_touchdata *touchdata = data->touchdata;

//      printk("mms114_input_report_dhlee2\n");

	for (id = 0; id < MMS114_MAX_TOUCH; id++) {
		if (!touchdata[id].updated)
			continue;
		touchdata[id].updated = false;
		
		max_id_num_check(id);

		if(touchdata[id].pressed )
		{
			input_mt_slot(tpd->dev, id);
			input_report_key(tpd->dev, BTN_TOUCH, 1);
			input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id+2);

			input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR,
					touchdata[id].width);
			input_report_abs(tpd->dev, ABS_MT_POSITION_X,
					touchdata[id].x);
			input_report_abs(tpd->dev, ABS_MT_POSITION_Y,
					touchdata[id].y);

			touch_mark[id]++;
			if (touch_mark[id] >= 1)
			{
				touch_mark[id] = 1;
			}
		}
		else if(touchdata[id].pressed == 0)
		{
			input_mt_slot(tpd->dev, id);
			input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, -1);
			touch_mark[id]--;
		}
			
	}
	input_sync(tpd->dev);
//	printk("*******touch repo end********\n");	
}

static void mms114_input_report_jshuai(struct mms114_data *data, u8* buf)
{
	int id;	
	struct mms114_touchdata *touchdata = data->touchdata;

      printk("mms114_input_report_jshuai");

	if((buf[0] == 0x0B) && (palm_counter == 1))
	{
		printk("mms114_input_report_jshuai, melfas mms144 Palm Touch 0x0B\n");

		//mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM); 
		
		input_report_key(tpd->dev, KEY_PALMEVENT, 1);
		input_report_key(tpd->dev, KEY_PALMEVENT, 0);
		input_sync(tpd->dev);	

		palm_counter =0;
		return;
		//ret = hwPowerDown(MT65XX_POWER_LDO_VMCH, "TP");
		//printk("tpd_XXXXXX%dXXXXXX\n",ret);
		
		//mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
		//printk("tpd enter sleep done\n");
		//release_all_fingers();		
	}
		
	for (id = 0; id < MMS114_MAX_TOUCH; id++) {

		if (!touchdata[id].updated)
			continue;
		touchdata[id].updated = false;

		printk("=======> touch data id: %d palm flag: %d ,pressed:%d\n", id, touchdata[id].palm_flag,touchdata[id].pressed);

		if (touchdata[id].palm_flag) {		//strength change from 40 to 100
//			printk("xx palm down strength %d xxxx\n", touchdata[id].strength);	
			if (touchdata[id].pressed == 1) {
				printk("palm down strength: %d count:%d xxxx\n", touchdata[id].strength, palm_counter);
				palm_area_flag = 1;
				input_report_key(tpd->dev, KEY_PALMEVENT, 1);
				input_sync(tpd->dev);
				palm_counter++;
			} else if (touchdata[id].pressed == 0) {
				printk("palm up strength: %d count:%d xxxx\n", touchdata[id].strength, palm_counter);
				if(palm_area_flag == 1)
				{
					input_report_key(tpd->dev, KEY_PALMEVENT, 0);
					input_sync(tpd->dev);
				}
				palm_area_flag = 0;
				palm_counter++;
			}
		} else if (touchdata[id].palm_flag == 0) {
			if (touchdata[id].pressed) {
				input_mt_slot(tpd->dev, id);
				input_report_key(tpd->dev, BTN_TOUCH, 1);
				input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id+2);

				input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR,
						touchdata[id].width);
				input_report_abs(tpd->dev, ABS_MT_POSITION_X,
						touchdata[id].x);
				input_report_abs(tpd->dev, ABS_MT_POSITION_Y,
						touchdata[id].y);
			}
			else if(!touchdata[id].pressed)
			{
				input_mt_slot(tpd->dev, id);
				input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, -1);
			}
		}
	}
	input_sync(tpd->dev);
}
static void mms114_proc_touchdata(struct mms114_data *data, u8 *buf)
{
	struct i2c_client *client = data->client;
	struct mms114_touchdata *touchdata;
	unsigned int id = (buf[0] & MMS114_ID_MASK) - 1;
	unsigned int type = (buf[0] >> MMS114_TYPE_OFFSET) & MMS114_TYPE_MASK;
	unsigned int pressed = ((buf[0] & 0x80) == 0x80); //(buf[0] >> MMS114_ACT_OFFSET) & MMS114_ACT_MASK;
	unsigned int x = buf[2] | (buf[1] & 0xf) << 8;
	unsigned int y = buf[3] | ((buf[1] >> 4) & 0xf) << 8;
	unsigned int palm_flag = (buf[0] >> 4) &1;
//	ret1 = (buf[0] >> 7) &1;
//		printk("buf[0] 0x%2x, palm %d, size %d xxxx\n",buf[0], ret, packet_size);

//	printk("xxxx  palm_flag: %d, id is %d, pressed is %d \n", palm_flag, id, pressed);
	if (id >= MMS114_MAX_TOUCH) {
		printk("xxxx Wrong touch id (%d)\n", id);
		return;
	}

	if (type != MMS114_TYPE_TOUCHSCREEN) {
		printk("xxxx Wrong touch type (%d)\n", type);
		return;
	}

	touchdata = &data->touchdata[id];

	if (!pressed && !touchdata->pressed) {
		dev_dbg(&client->dev, "Wrong touch release (id: %d)\n", id);
		return;
	}

	if (x > 720 || y > 1280) {
		printk(&client->dev, "Wrong touch coordinates (%d, %d)\n",
				x, y);
		return;
	}

	touchdata->x = x;
	touchdata->y = y;
	touchdata->width = buf[4];
	touchdata->strength = buf[5];
	touchdata->pressed = pressed;
	touchdata->updated = true;
	touchdata->palm_flag = palm_flag;
//	printk("XXXXXXXXXXXXXpalm_flag: %d, pressed is %dXXXXXXXXXXXXXX \n", palm_flag, pressed);

	dev_dbg(&client->dev, "id: %d, type: %d, pressed: %d\n",
			id, type, pressed);
	dev_dbg(&client->dev, "x: %d, y: %d, width: %d, strength: %d\n",
			touchdata->x, touchdata->y,
			touchdata->width, touchdata->strength);
}


#endif



static int tpd_flag = 0;
static int tpd_halt=0;

//#define TPD_CLOSE_POWER_IN_SLEEP

#define TPD_OK 0

#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

 
 static const struct i2c_device_id mms144_tpd_id[] = {{"mms144",0},{}};
 static struct i2c_board_info __initdata mms144_i2c_tpd={ I2C_BOARD_INFO("mms144", (0x48))};
 
 
 static struct i2c_driver tpd_i2c_driver = {
  .driver = {
	 .name = "mms144",//.name = TPD_DEVICE,
//	 .owner = THIS_MODULE,
  },
  .probe = tpd_probe,
  .remove = __devexit_p(tpd_remove),
  .id_table = mms144_tpd_id,
  .detect = tpd_detect,
//  .address_data = &addr_data,
 };
 static void restart_tp(void)
 {
	release_all_fingers();
	hwPowerDown(MT65XX_POWER_LDO_VMCH, "TP");
	msleep(5);
	hwPowerOn(MT65XX_POWER_LDO_VMCH, VOL_3300, "TP");
	msleep(200);
 }

static  void tpd_palm_down(void) {
	// input_report_abs(tpd->dev, ABS_PRESSURE, p);
	 input_report_key(tpd->dev, KEY_PALMEVENT, 1);	 

	 /* track id Start 0 */       
//	 input_mt_sync(tpd->dev);
	 input_sync(tpd->dev);
 	 
}

static  void tpd_palm_up(void) {
	// input_report_abs(tpd->dev, ABS_PRESSURE, p);
	 input_report_key(tpd->dev, KEY_PALMEVENT, 0);	 

	 /* track id Start 0 */       
//	 input_mt_sync(tpd->dev);
	 input_sync(tpd->dev);
     
}

 static int touch_event_handler(void *unused)
 {
  
	struct mms114_data *data = i2c_get_clientdata(i2c_client);
	u8 buf[MMS114_MAX_PACKET] = {0};
	int packet_size;
	int touch_size;
	int index;
	int ret;

	static u32 i = 0, j = 0;
//	u8 flag = 0;
	unsigned char mcs_version;
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	sched_setscheduler(current, SCHED_RR, &param);

	do
	{
		mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		set_current_state(TASK_INTERRUPTIBLE); 
		wait_event_interruptible(waiter,tpd_flag!=0);

		tpd_flag = 0; 

		set_current_state(TASK_RUNNING);
		for (i=0; i<5; i++) {
			packet_size = mms114_read_reg(MMS114_PACKET_SIZE);
			if ((packet_size > 0)&&(packet_size <= MMS114_MAX_PACKET))
				break;
			if (i == 5)
				printk("xx retry 5 times , error!\n");
			msleep(5);
		}

		touch_size = packet_size / MMS114_PACKET_NUM;
		
		ret = __mms114_read_reg(MMS114_INFOMATION, packet_size, buf);

//		printk("********buf[0]:0x%x********\n",buf[0]);

		if (buf[0] == 0x0F) {
			printk("melfas mms144 ESD Packet 0x0F\n");
			restart_tp();
			continue;
		}

		if (buf[0] == 0x0B) {
			printk("melfas mms144 Palm Touch 0x0B\n");
			palm_flag = 1;
			//palm_counter ++;
			//input_report_key(tpd->dev, KEY_PALMEVENT, 1);
			//input_sync(tpd->dev);
			//palm_counter++;
			//continue;
		} else {
			palm_flag = 0;
		}		
		
//		for(index=0; index<packet_size; index= index+MMS114_PACKET_NUM)
//			mms114_proc_touchdata(data, buf + index);


		//if the firmware didn't support palm,disable palm function
		if(palm_enable == 0)
		{
			palm_flag = 0;
		}
		
		if (palm_flag == 1) 
		{
//			tpd_palm_down();
	        	 palm_counter++;
	 		printk("palm event down %d\n",palm_counter);
			release_all_fingers();		
			tpd_palm_down();
			tpd_palm_up();
		}
		else 
		{
			for(index=0; index<packet_size; index= index+MMS114_PACKET_NUM)
				mms114_proc_touchdata(data, buf + index);

			mms114_input_report_dhlee2(data);
		}			

		//get the point state from the firmware
//		get_fw_point_state(data);
		
//		mms114_input_report_dhlee(data);
//		mms114_input_report_dhlee2(data);
//		mms114_input_report(data);
//		mms114_input_report_jshuai(data, buf);
		
	}while(!kthread_should_stop());
 
	 return 0;
 }
 
 static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info) 
 {
	 strcpy(info->type, TPD_DEVICE);	
	  return 0;
 }
 
 static void tpd_eint_interrupt_handler(void)
 {
	 //TPD_DEBUG("TPD interrupt has been triggered\n");
	 TPD_DEBUG_PRINT_INT;
	 tpd_flag = 1;
	 wake_up_interruptible(&waiter);
	 
 }

 static int mms114_get_version(struct mms114_data *data)
{
	u8 buf[6];
	int ret; 

	ret = __mms114_read_reg(MMS114_TSP_REV, 4, buf);
	if (ret < 0)
		return ret;
	tsr_ver = buf[0];
	hw_fw_ver = buf[1];
	chip_fw_ver = buf[2];	
	printk("TSP ev: 0x%x, HW Rev: 0x%x, Firmware Ver: 0x%x\n",
			buf[0], buf[1], buf[2]);

	return 0;
}

#if KPD_BUTTON_KEY
void backkey_handler(void)
{
	int i;
	i = mt_get_gpio_in(GPIO106);
#if 1
	if (!i) {
		input_report_key(tpd->dev, KEY_BACK, 1);
		input_sync(tpd->dev);

		mt_set_gpio_mode(GPIO106, 2);
		mt_set_gpio_dir(GPIO106, GPIO_DIR_IN);
		mt65xx_eint_set_sens(11, 0);
		mt65xx_eint_set_polarity(11, 1);
	}
	else if ( i ) {
		input_report_key(tpd->dev, KEY_BACK, 0);
		input_sync(tpd->dev);

		mt_set_gpio_mode(GPIO106, 2);
		mt_set_gpio_dir(GPIO106, GPIO_DIR_IN);
		mt65xx_eint_set_sens(11, 0);
		mt65xx_eint_set_polarity(11, 0);

	}
#endif
}

void homekey_handler(void)
{
	int i;
	i = mt_get_gpio_in(GPIO76);
#if 1
	if (!i) {
		input_report_key(tpd->dev, KEY_HOMEPAGE, 1);
		input_sync(tpd->dev);

		mt_set_gpio_mode(GPIO76, 1);
		mt_set_gpio_dir(GPIO76, GPIO_DIR_IN);
		mt65xx_eint_set_sens(1, 0);
		mt65xx_eint_set_polarity(1, 1);
	}
	else if (i) {
		input_report_key(tpd->dev, KEY_HOMEPAGE, 0);
		input_sync(tpd->dev);

		mt_set_gpio_mode(GPIO76, 1);
		mt_set_gpio_dir(GPIO76, GPIO_DIR_IN);
		mt65xx_eint_set_sens(1, 0);
		mt65xx_eint_set_polarity(1, 0);
	}
#endif

}
void menukey_handler(void)
{
	int i;
	i = mt_get_gpio_in(GPIO14);
//	printk("xxxxxxxxx value is %d xxxxxxxxx\n", i);
#if 1
	if ( !i ) {
//	printk("pressed menu!xxxxxxxxx\n");
		input_report_key(tpd->dev, KEY_MENU, 1);
		input_sync(tpd->dev);

		mt_set_gpio_mode(GPIO14, 2);
		mt_set_gpio_dir(GPIO14, GPIO_DIR_IN);
		mt65xx_eint_set_sens(12, 0);
		mt65xx_eint_set_polarity(12, 1);

	}
	else if ( i ) {
		input_report_key(tpd->dev, KEY_MENU, 0);
		input_sync(tpd->dev);

		mt_set_gpio_mode(GPIO14, 2);
		mt_set_gpio_dir(GPIO14, GPIO_DIR_IN);
		mt65xx_eint_set_sens(12, 0);
		mt65xx_eint_set_polarity(12, 0);

	}
#endif

}

static void back_key_handler(void)
{
	tasklet_schedule(&backkey_tasklet);
}
static void home_key_handler(void)
{
	tasklet_schedule(&homekey_tasklet);

}
static void menu_key_handler(void)
{
	tasklet_schedule(&menukey_tasklet);
	

}
#endif

static int mms114_set_palm_func(void)
{
	int ret; 
	
	//check the firmware version and set the palm function
	if ( (chip_fw_ver >= current_chip_palm))
	{
		set_bit(KEY_PALMEVENT, tpd->dev->keybit);
		palm_enable = 1;
	}
	else if  ((hw_fw_ver == current_hw) && (chip_fw_ver == current_chip))
	{
		palm_enable = 0;
	}
	else
	{
		palm_enable = 0;
		printk("CTP other version firmware\n");
	}
	
	printk("HW Rev: 0x%x, Firmware Ver: 0x%x  palm_enable: %d\n",
			 hw_fw_ver, chip_fw_ver,palm_enable);

	return 0;
}

/*check is hardware there */
static int mms114_check_hw()
{
	int i = 0;
	int ret = 0;
	unsigned char buf[4];
	
	for (i = 0; i < MELFAS_I2C_RETRY_TIMES - 2; i++)
	{
		ret = __mms114_read_reg(MMS114_TSP_REV, 4, buf);
		if (ret >= 0)
		{
			printk("tpd: TP hardware is OK!\n");
			return ret;
		}
		else
		{
			restart_tp();
		}
	}
	
	printk("tpd: TP hardware is not OK!\n");
	return ret;

}

#if MELFAS_FIRMWARE_UPDATE

static int firmware_check(void)
{
	unsigned char buf[4];
	int ret;
	ret = __mms114_read_reg(MMS114_TSP_REV, 4, buf);
	if (ret < 0) {
		printk("get version error!\n");
		return ret;
	}
	printk("now firmware version is %x\n", buf[2]);
	return buf[2];
}

static int mms114_update_fail_reboot(void)
{

	printk(KERN_EMERG "TP update failed,restarting system now...\n");

	hwPowerDown(MT65XX_POWER_LDO_VMCH, "TP");
	machine_restart(NULL);
//	kernel_restart(NULL);
//	sys_reboot(int magic1, int magic2, unsigned int cmd, void __user * arg)

}
static int firmware_update(struct mms114_data *ts)
{
	int ret = 0;
	int i =0;
	struct i2c_client *client = i2c_client;
	struct i2c_adapter *adap = client->adapter;
	
	for (i = 0; i < MELFAS_I2C_RETRY_TIMES - 2; i++)
	{
		ret = firmware_check();	
		if (ret >= 0)
		{
			printk("tpd: get the version success!\n");
			break;
		}
	}
	if (ret < 0) {
		err_num = -2;
		printk("tpd: get the version fail!\n");
		return -2;	//modify by ydyu
	}
	printk(KERN_ERR "***firmware_update***\n");
	if (ret < LASTEST_VERSION) 
	{
		i2c_lock_adapter(adap);
		ret = isp_fw_download(MELFAS_binary, MELFAS_binary_nLength);
//		ret = melfas_fw_download(MELFAS_binary, MELFAS_binary_nLength);
		if (ret != 0) {
			err_num = -3;
			ret = -1;
		}
		i2c_unlock_adapter(adap);
	}

    return ret;
}
#endif

 static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
 {	 
	int retval = TPD_OK;
	int err=0;
	int ret;
	int try_update = 3;
	int i = 0;
	
reset_proc:   
	i2c_client = client;

	struct mms114_data *data;
	data = kzalloc(sizeof(struct mms114_data), GFP_KERNEL);
	data->client = client;
	i2c_set_clientdata(client, data);

#if KPD_BUTTON_KEY

	printk("xxxxxxxxxxxxx %s xxxxxxxxxxxxxxxxxxxxx\n", __FUNCTION__);
	hwPowerOn(MT65XX_POWER_LDO_VSIM2,VOL_1800,"touch_key");

	mt_set_gpio_mode(GPIO114, 0);
	mt_set_gpio_dir(GPIO114, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO114, GPIO_OUT_ONE);


//back key
	mt_set_gpio_mode(GPIO106, 2);
	mt_set_gpio_dir(GPIO106, GPIO_DIR_IN);

	mt65xx_eint_set_sens(11, 0);
	mt65xx_eint_set_hw_debounce(11, 0);
	mt65xx_eint_registration(11, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, 0, back_key_handler, 1); 
	mt65xx_eint_unmask(11);


//home key
	mt_set_gpio_mode(GPIO76, 1);
	mt_set_gpio_dir(GPIO76, GPIO_DIR_IN);

	mt65xx_eint_set_sens(1, 0);
	mt65xx_eint_set_hw_debounce(1, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	mt65xx_eint_registration(1, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, 0, home_key_handler, 1); 
	mt65xx_eint_unmask(1);

//homepage key	
	mt_set_gpio_mode(GPIO14, 2);
	mt_set_gpio_dir(GPIO14, GPIO_DIR_IN);

	mt65xx_eint_set_sens(12, 0);
	mt65xx_eint_set_hw_debounce(12, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	mt65xx_eint_registration(12, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, 0, menu_key_handler, 1); 
	mt65xx_eint_unmask(12);
#endif

	hwPowerOn(MT65XX_POWER_LDO_VMCH, VOL_3300, "TP");
	msleep(300);
//	msleep(30);

	ret = mms114_check_hw();
	if(ret < 0)
	{
		err_num = -1;
		return -1;
	}

#if MELFAS_FIRMWARE_UPDATE

	for (i = 0; i < try_update; i++)
	{
		printk(KERN_ERR "***firmware_update_times:%d***\n",i);
		ret = firmware_update(data);

		if (ret >= 0)
		{
			break;
		}
	}

	if (ret < 0)
	{		
		printk(KERN_ERR "***firmware_update_fail***\n");
		mms114_update_fail_reboot();
		return -2;
	}
	
#endif

	restart_tp();

	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

	mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1); 
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
 

	tpd_load_status = 1;

	ctp_sysfs_init();	
	err = mms114_get_version(data);
	mms114_set_palm_func();

	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	 if (IS_ERR(thread))
	{
		retval = PTR_ERR(thread);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);

	}

	TPD_DMESG("mms144 Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");
	return 0;
   
 }

 static int __devexit tpd_remove(struct i2c_client *client)
 {
   
	 TPD_DEBUG("TPD removed\n");
 
	return 0;
 }
 
 
 static int tpd_local_init(void)
 {

 
  TPD_DMESG(" mms144 I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
 
 
   if(i2c_add_driver(&tpd_i2c_driver)!=0)
   {
  		TPD_DMESG("mms144 unable to add i2c driver.\n");
      	return -1;
    }
    if(tpd_load_status == 0) 
    {
    	TPD_DMESG("mms144 add error touch panel driver.\n");
    	i2c_del_driver(&tpd_i2c_driver);
    	return -1;
    }
	
	tpswitch_sysfs_init();
	
#ifdef TPD_HAVE_BUTTON     
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif   


#if KPD_BUTTON_KEY
	set_bit(KEY_BACK, tpd->dev->keybit);
	set_bit(KEY_HOMEPAGE, tpd->dev->keybit);
	set_bit(KEY_MENU, tpd->dev->keybit);
//	set_bit(KEY_PALMEVENT, tpd->dev->keybit);

#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif
		input_mt_init_slots(tpd->dev, MMS114_MAX_TOUCH);
		TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
		tpd_type_cap = 1;
    return 0; 
 }
void release_all_fingers(void)
{
	unsigned char i;

	for(i=0; i<MMS114_MAX_TOUCH; i++){
		input_mt_slot(tpd->dev, i);
		input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, -1);
	}
	input_sync(tpd->dev);
	printk(KERN_ERR "***release_all_fingers***\n");
}

 static void tpd_resume( struct early_suspend *h )
{
	u8 buf[MMS114_MAX_PACKET];
	u8 i = 0;
	int index;
	int ret;

	printk("xxxx melfas TP resume !\n");
	release_all_fingers();
	hwPowerOn(MT65XX_POWER_LDO_VMCH, VOL_3300, "TP");
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	msleep(200);

	while(!mt_get_gpio_in(GPIO_CTP_EINT_PIN))
	{
		i++;

//		ret = __mms114_read_reg(MMS114_INFOMATION, MMS114_MAX_PACKET, buf);
//		printk("xxxxtpd_buf[0] = %d\n", buf[0]);
//		if (ret < 0)
//		{
//			printk("xxxx tpd read error !\n");
//			if(!mt_get_gpio_in(GPIO_CTP_EINT_PIN))
//			{
//			      printk("xxxx tpd read error !  restart_tp\n");
//				restart_tp();
//			}
//			break;
//		}

#if 0 // dhlee 2013.02.04
		if(i == 3){
			printk("re-testing for three times, break now!");
			restart_tp(); //dhlee
			break;
		}
#endif
	}

	tpd_halt = 0;


}

static void tpd_suspend( struct early_suspend *h )
{
	int ret;
	
	tpd_halt = 1;
	printk("tpd enter sleep\n");
	//power down the SD card
//	hwPowerDown(MT65XX_POWER_LDO_VMCH, "msdc");
	
	ret = hwPowerDown(MT65XX_POWER_LDO_VMCH, "TP");
	printk("tpd_XXXXXX%dXXXXXX\n",ret);
	
	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	fw_point_state->max_id_num = 0;
	
	printk("tpd enter sleep done\n");
} 


 static struct tpd_driver_t tpd_device_driver = {
		 .tpd_device_name = "mms144",
		 .tpd_local_init = tpd_local_init,
		 .suspend = tpd_suspend,
		 .resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
		 .tpd_have_button = 1,
#else
		 .tpd_have_button = 0,
#endif		
 };
 /* called when loaded into kernel */
 static int __init tpd_driver_init(void) {
	 printk("MediaTek mms144 touch panel driver init\n");
	   i2c_register_board_info(0, &mms144_i2c_tpd, 1);
		 if(tpd_driver_add(&tpd_device_driver) < 0)
			 TPD_DMESG("add mms144 driver failed\n");
	 return 0;
 }
 
 /* should never be called */
 static void __exit tpd_driver_exit(void) {
	 TPD_DMESG("MediaTek mms144 touch panel driver exit\n");
	 //input_unregister_device(tpd->dev);
	 tpd_driver_remove(&tpd_device_driver);
 }
 
 module_init(tpd_driver_init);
 module_exit(tpd_driver_exit);


