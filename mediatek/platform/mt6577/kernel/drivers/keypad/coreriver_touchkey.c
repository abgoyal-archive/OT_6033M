#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/slab.h>
#include <asm/unistd.h>
#include <linux/kthread.h>

#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/mutex.h>


#include <asm/atomic.h>
#include <asm/uaccess.h>

#include <mach/mt_reg_base.h>
#include <mach/mt_boot.h>
#include <mtk_kpd.h>            
#include <mach/irqs.h>
#include <mach/eint.h>
#include <mach/mt_gpio.h>

#ifdef MT6577
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#endif

#include <linux/aee.h>
static int qt_flag = 0;

 
void coreriver_handler();

static DECLARE_WAIT_QUEUE_HEAD(waiter);
struct task_struct *coreriver_thread = NULL;
static struct coreriver_data *data;
static const struct i2c_device_id coreriver_i2c_id[] = {{"coreriver",0},{}};
static struct i2c_board_info __initdata i2c_coreriver={ I2C_BOARD_INFO("coreriver", (0x1A))};
static void coreriver_late_resume(struct early_suspend *h);
static void coreriver_early_suspend(struct early_suspend *h);
static void __exit coreriver_driver_exit (void);



struct coreriver_data {
        struct i2c_client *client;
        struct input_dev *input_dev;
	struct early_suspend    early_drv;
        atomic_t                early_suspend;
};


static int write_reg(u8 addr, u8 cmd)
{
        char buf[2];
        int ret = -1;

        buf[0] = addr;
        buf[1] = cmd;
	printk("coreriver write reg------------------------------------\n");
        ret = i2c_master_send(data->client, buf, 2);
        if (ret < 0){
                printk("coreriver: write reg failed! \n");
                return -1;
        }
        return 0;
}

static kal_uint32 read_reg(u8 addr_read)
{
        int ret;
	char buf[1];
	buf[0] = addr_read;
	printk("coreriver read reg-----------------------------------\n");
        i2c_master_send(data->client, buf, 1);
	udelay(31);
	char buf_rec[1];
        ret = i2c_master_recv(data->client, buf_rec, 1);
        if (ret < 0)
	{
                printk("coreriver: i2c read error\n");
		return -1;
	}	
        return buf_rec[0];
}

kal_uint32 coreriver_read_interface (kal_uint8 RegNum)
{
	kal_uint32 i = 0;
	i = read_reg(RegNum);
	return i;
}

kal_uint32 coreriver_config_interface (kal_uint8 RegNum, kal_uint8 val)
{
	write_reg(RegNum,val);
}


void coreriver_handler()
{

	printk("coreriver handler---------------------------------------------\n");

	mt65xx_eint_mask(4);

	kal_uint32 reg1 = 0;

	reg1 = coreriver_read_interface(0x0);

	printk("----------------------------------\n");
	if(reg1&0x4)
	{
		if(reg1&0x8)
		{
			input_report_key(data->input_dev, KEY_BACK, 0);
			input_sync(data->input_dev);
			printk("release back\n");
		}
		else
		{
			input_report_key(data->input_dev, KEY_BACK, 1);
			input_sync(data->input_dev);
			printk("press back\n");
		}
	}
	else if(reg1&0x1)
	{
		if(reg1&0x8)
		{
			input_report_key(data->input_dev, KEY_MENU, 0);
			input_sync(data->input_dev);
			printk("release menu\n");
		}
		else
		{
			input_report_key(data->input_dev, KEY_MENU, 1);
			input_sync(data->input_dev);
			printk("press menu\n");			
		}
	}
	else if(reg1&0x2)
	{

		if(reg1&0x8)
		{
			input_report_key(data->input_dev, KEY_HOMEPAGE, 0);
			input_sync(data->input_dev);
			printk("release homepage\n");
		}
		else
		{
			input_report_key(data->input_dev, KEY_HOMEPAGE, 1);
			input_sync(data->input_dev);
			printk("press homepage\n");
		}
	}

	mt65xx_eint_unmask(4);

}


static void coreriver_thread_handler(void)
{
        do
        {
                mt65xx_eint_unmask(4);
                set_current_state(TASK_INTERRUPTIBLE);
                wait_event_interruptible(waiter,qt_flag!=0);

                qt_flag = 0;

                set_current_state(TASK_RUNNING);
                mt65xx_eint_mask(4);

		coreriver_handler();
        }while(!kthread_should_stop());

        return 0;

}

void coreriver_eint_handler(void)
{
         qt_flag = 1;
         wake_up_interruptible(&waiter);
}


static int coreriver_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i,k;
	printk("coreriver_driver_probe\n");
	int retval = 1;
	
	hwPowerOn(MT65XX_POWER_LDO_VGP2,VOL_1800,"touchkey");

        data = kzalloc(sizeof(struct coreriver_data), GFP_KERNEL);
        if(!data)
	{
        	printk("coreriver: kzalloc coreriver_data fail!\n");
		return -1;
	}
	memset(data, 0, sizeof(*data));
	data->client = client;

	kal_uint32 reg0 = 0;

	msleep(300);
	reg0 = coreriver_read_interface(0x1);
        printk("----------------------------------------------coreriver_reg = %x\n",reg0);
	if(reg0 != 0x0)
	{
		printk("NOT CoreRiver !!!!\n");
		kfree(data);
		return -1;
	}


        data->input_dev = input_allocate_device();
        if (!(data->input_dev))
                return -ENOMEM;
        set_bit(EV_KEY, data->input_dev->evbit);
	set_bit(KEY_BACK, data->input_dev->keybit);
	set_bit(KEY_HOMEPAGE, data->input_dev->keybit);
	set_bit(KEY_MENU, data->input_dev->keybit);
	i = input_register_device(data->input_dev);
        if (i) {
                printk("register input device failed (%d)\n", i);
                input_free_device(data->input_dev);
                return i;
        }


	mt_set_gpio_mode(GPIO72, 1);
	mt_set_gpio_dir(GPIO72, GPIO_DIR_IN);

        mt_set_gpio_pull_select(GPIO72, GPIO_PULL_UP);
        mt_set_gpio_pull_enable(GPIO72, GPIO_PULL_ENABLE);


	mt65xx_eint_set_sens(4, 1);
	mt65xx_eint_set_polarity(4,0);
	mt65xx_eint_set_hw_debounce(4, 0);
	mt65xx_eint_registration(4, 0, 0, coreriver_eint_handler, 0); 
	mt65xx_eint_unmask(4);


        coreriver_thread = kthread_run(coreriver_thread_handler, 0, "coreriver");
        if (IS_ERR(coreriver_thread))
        {
                 retval = PTR_ERR(coreriver_thread);
		 printk("coreriver kthread_run fail \n");
        }

	atomic_set(&(data->early_suspend), 0);
        data->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
        data->early_drv.suspend  = coreriver_early_suspend,
        data->early_drv.resume   = coreriver_late_resume,
        register_early_suspend(&data->early_drv);

	return 0;
}

static void coreriver_late_resume(struct early_suspend *h)
{

	mt_set_gpio_mode(GPIO72, 0);
	mt_set_gpio_dir(GPIO72, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO72, GPIO_OUT_ONE);
	mdelay(10);
	mt_set_gpio_out(GPIO72, GPIO_OUT_ZERO);
	mdelay(10);
	mt_set_gpio_out(GPIO72, GPIO_OUT_ONE);
	printk("coreriver_late_resume        80\n");

	mt_set_gpio_mode(GPIO72, 1);
	mt_set_gpio_dir(GPIO72, GPIO_DIR_IN);
	mt_set_gpio_pull_select(GPIO72, GPIO_PULL_UP);
       mt_set_gpio_pull_enable(GPIO72, GPIO_PULL_ENABLE);
	atomic_set(&(data->early_suspend), 0);

        printk("coreriver resume------------------------------------------------\n");
}

static void coreriver_early_suspend(struct early_suspend *h)
{

	coreriver_config_interface(0x00,0x80);
	atomic_set(&(data->early_suspend), 1);

        printk("coreriver suspend------------------------------------------------\n");
}


static struct i2c_driver touchkey_coreriver_driver = {
    .probe              = coreriver_driver_probe,
    .id_table   	= coreriver_i2c_id,
    .driver     = {
        .name = "coreriver",
    },
};


static int __init coreriver_driver_init(void)
{

        i2c_register_board_info(3, &i2c_coreriver, 1);
	if(i2c_add_driver(&touchkey_coreriver_driver)!=0)
        {
		printk("failed to register coreriver i2c driver\n");
        }
	else
	{
		printk("success to register coreriver i2c driver\n");
	}
        return 0;	
}


static void __exit coreriver_driver_exit (void)
{
	i2c_del_driver(&touchkey_coreriver_driver);
	kfree(data);
}

module_init(coreriver_driver_init);
module_exit(coreriver_driver_exit);

MODULE_AUTHOR("zhang k");
MODULE_DESCRIPTION("Touchkey coreriver Driver");
MODULE_LICENSE("GPL");

