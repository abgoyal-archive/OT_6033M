
//--------------------------------------------------------
//
//
//	Melfas MMS100 Series Download base v1.0
//
//
//--------------------------------------------------------

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
#include "tpd_custom_mcs.h"

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#include "cust_gpio_usage.h"
#include <linux/input/mt.h>

//============================================================
//
//	Type define
//
//============================================================

//typedef char				INT8;
typedef unsigned char		UINT8;
typedef short				INT16;
typedef unsigned short		UINT16;
typedef int					INT32;
typedef unsigned int		UINT32;
typedef unsigned char		BOOLEAN;


//============================================================
//
//	Porting Download Options
//
//============================================================
#define MCSDL_USE_CE_CONTROL 0
#define MCSDL_USE_VDD_CONTROL 1
#define MCSDL_USE_RESETB_CONTROL 1

#define MELFAS_ENABLE_DBG_PRINT 1
#define MELFAS_ENABLE_DBG_PROGRESS_PRINT 1



//----------------
// VDD
//----------------
#if MCSDL_USE_VDD_CONTROL
#define mcsdl_vdd_on()                  
#define mcsdl_vdd_off()                 

#define MCSDL_VDD_SET_HIGH()            hwPowerOn(MT65XX_POWER_LDO_VMCH, VOL_3300, "TP")
#define MCSDL_VDD_SET_LOW()             hwPowerDown(MT65XX_POWER_LDO_VMCH, "TP")
#else
#define MCSDL_VDD_SET_HIGH()            // Nothing
#define MCSDL_VDD_SET_LOW()             // Nothing
#endif

//----------------
// CE
//----------------
#if MCSDL_USE_CE_CONTROL
#define MCSDL_CE_SET_HIGH()
#define MCSDL_CE_SET_LOW()
#define MCSDL_CE_SET_OUTPUT()
#else
#define MCSDL_CE_SET_HIGH()				// Nothing
#define MCSDL_CE_SET_LOW()				// Nothing
#define MCSDL_CE_SET_OUTPUT()			// Nothing
#endif


//----------------
// RESETB
//----------------
#if MCSDL_USE_RESETB_CONTROL
#define MCSDL_RESETB_SET_HIGH()		mt_set_gpio_out(GPIO75, GPIO_OUT_ONE)
#define MCSDL_RESETB_SET_LOW()		mt_set_gpio_out(GPIO75, GPIO_OUT_ZERO)
#define MCSDL_RESETB_SET_OUTPUT()		mt_set_gpio_dir(GPIO75, GPIO_DIR_OUT)
#define MCSDL_RESETB_SET_INPUT()		mt_set_gpio_dir(GPIO75, GPIO_DIR_IN)
#define MCSDL_RESETB_SET_ALT()			mt_set_gpio_mode(GPIO75, 1)
#define MCSDL_RESETB_SET_GPIO()		mt_set_gpio_mode(GPIO75, 0)
#else
#define MCSDL_RESETB_SET_HIGH()
#define MCSDL_RESETB_SET_LOW()
#define MCSDL_RESETB_SET_OUTPUT()
#define MCSDL_RESETB_SET_INPUT()
#endif


//------------------
// I2C SCL & SDA
//------------------
#define MCSDL_GPIO_SCL_SET_HIGH()		mt_set_gpio_out(GPIO87, GPIO_OUT_ONE)  
#define MCSDL_GPIO_SCL_SET_LOW()		mt_set_gpio_out(GPIO87, GPIO_OUT_ZERO)

#define MCSDL_GPIO_SDA_SET_HIGH()		mt_set_gpio_out(GPIO88, GPIO_OUT_ONE) 
#define MCSDL_GPIO_SDA_SET_LOW()		mt_set_gpio_out(GPIO88, GPIO_OUT_ZERO) 

#define MCSDL_GPIO_SCL_SET_OUTPUT()	      	mt_set_gpio_dir(GPIO87, GPIO_DIR_OUT)
#define MCSDL_GPIO_SCL_SET_INPUT()	    	mt_set_gpio_dir(GPIO87, GPIO_DIR_IN)
#define MCSDL_GPIO_SCL_SET_ALT()			mt_set_gpio_mode(GPIO87, 1)
#define MCSDL_GPIO_SDA_SET_OUTPUT()	   	mt_set_gpio_dir(GPIO88, GPIO_DIR_OUT)
#define MCSDL_GPIO_SDA_SET_INPUT()	    	mt_set_gpio_dir(GPIO88, GPIO_DIR_IN)
#define MCSDL_GPIO_SDA_SET_ALT()			mt_set_gpio_mode(GPIO88, 1)

#define MCSDL_GPIO_SCL_SET_GPIO()		mt_set_gpio_mode(GPIO87, 0) 
#define MCSDL_GPIO_SDA_SET_GPIO()		mt_set_gpio_mode(GPIO88, 0) 


#define MCSDL_GPIO_SDA_IS_HIGH()		((mt_get_gpio_in(GPIO88) > 0) ? 1 : 0)


