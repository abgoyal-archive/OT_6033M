
#ifndef _CUST_KPD_H_
#define _CUST_KPD_H_
#include <linux/input.h>
#include <cust_eint.h>

#define KPD_YES		1
#define KPD_NO		0

/* available keys (Linux keycodes) */
#define KEY_CALL	KEY_SEND
#define KEY_ENDCALL	KEY_END
#undef KEY_OK
#define KEY_OK		KEY_REPLY	/* DPAD_CENTER */
#define KEY_FOCUS	KEY_HP
#define KEY_AT		KEY_EMAIL
#define KEY_POUND	228	//KEY_KBDILLUMTOGGLE
#define KEY_STAR	227	//KEY_SWITCHVIDEOMODE
#define KEY_DEL 	KEY_BACKSPACE
#define KEY_SYM		KEY_COMPOSE
/* KEY_HOME */
/* KEY_BACK */
/* KEY_VOLUMEDOWN */
/* KEY_VOLUMEUP */
/* KEY_MUTE */
/* KEY_MENU */
/* KEY_UP */
/* KEY_DOWN */
/* KEY_LEFT */
/* KEY_RIGHT */
/* KEY_CAMERA */
/* KEY_POWER */
/* KEY_TAB */
/* KEY_ENTER */
/* KEY_LEFTSHIFT */
/* KEY_COMMA */
/* KEY_DOT */		/* PERIOD */
/* KEY_SLASH */
/* KEY_LEFTALT */
/* KEY_RIGHTALT */
/* KEY_SPACE */
/* KEY_SEARCH */
/* KEY_0 ~ KEY_9 */
/* KEY_A ~ KEY_Z */



#define KPD_KEY_DEBOUNCE  1024      /* (val / 32) ms */
#define KPD_PWRKEY_MAP    KEY_POWER

/* HW keycode [0 ~ 71] -> Linux keycode */
#define KPD_INIT_KEYMAP()	\
{	\
	[0] = KEY_VOLUMEUP,		\
	[2] = KEY_VOLUMEDOWN,		\
	[10] = KEY_CAMERA,		\
	[11] = KEY_FOCUS,		\
}	 
/*****************************************************************/
/*******************Preload Customation***************************/
/*****************************************************************/
#define KPD_PWRKEY_EINT_GPIO  GPIO61

#define KPD_PWRKEY_GPIO_DIN  0

#define  KPD_DL_KEY1  8    /* KEY_POWER */
#define KPD_DL_KEY2  0    /* KEY_VOLUMEUP */
#define KPD_DL_KEY3  2    /* KEY_VOLUMEDOWN */
/*****************************************************************/
/*******************Uboot Customation***************************/
/*****************************************************************/
#define MT65XX_RECOVERY_KEY  0    /* KEY_VOLUMEUP */
#define MT65XX_FACTORY_KEY  2    /* KEY_VOLUMEDOWN */
/*****************************************************************/
/*******************factory Customation***************************/
/*****************************************************************/
#define KEYS_PWRKEY_MAP		{ KEY_POWER, "Power" }
#define DEFINE_KEYS_KEYMAP(x)		\
 struct key x[] = {	\
 	KEYS_PWRKEY_MAP,		\
{ KEY_VOLUMEUP,   "VLUp"  }, \
{ KEY_VOLUMEDOWN,   "VLDown"  }, \
{KEY_CAMERA,   "camera"  }, \
{KEY_FOCUS,   "focus"  }, \
}
#define CUST_KEY_VOLUP  KEY_VOLUMEUP    
#define CUST_KEY_VOLDOWN  KEY_VOLUMEDOWN    
#define CUST_KEY_CENTER  KEY_HOME    
#define CUST_KEY_CONFIRM  KEY_HOME    
#define CUST_KEY_BACK  KEY_BACK    
/*****************************************************************/
/*******************recovery Customation****************************/
/*****************************************************************/
#define RECOVERY_KEY_VOLDOWN  KEY_VOLUMEDOWN   
#define RECOVERY_KEY_VOLUP  KEY_VOLUMEUP   
#define RECOVERY_KEY_CENTER  KEY_MENU   
#define RECOVERY_KEY_RIGHT  KEY_BACK   
#endif



