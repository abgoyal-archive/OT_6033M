
//#include <windows.h>
//#include <memory.h>
//#include <nkintr.h>
//#include <ceddk.h>
//#include <ceddk_exp.h>

//#include "kal_release.h"
//#include "i2c_exp.h"
//#include "gpio_exp.h"
//#include "msdk_exp.h"
//#include "msdk_sensor_exp.h"
//#include "msdk_isp_exp.h"
//#include "base_regs.h"
//#include "Sensor.h"
//#include "camera_sensor_para.h"
//#include "CameraCustomized.h"

//s_porting add
//s_porting add
//s_porting add
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/io.h>
#include <asm/system.h>


#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"


#include "s5k8aayx_Sensor.h"
#include "s5k8aayx_Camera_Sensor_para.h"
#include "s5k8aayx_CameraCustomized.h"

#define S5K8AAYX_DEBUG
#ifdef S5K8AAYX_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

typedef struct
{
  UINT16  iSensorVersion;
  UINT16  iNightMode;
  UINT16  iWB;
  UINT16  iEffect;
  UINT16  iEV;
  UINT16  iBanding;
  UINT16  iMirror;
  UINT16  iFrameRate;
} S5K8AAYXStatus;
S5K8AAYXStatus S5K8AAYXCurrentStatus;

static DEFINE_SPINLOCK(s5k8aayx_drv_lock);

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

static int sensor_id_fail = 0; 
static kal_uint32 zoom_factor = 0; 


inline S5K8AAYX_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
	char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,2,S5K8AAYX_WRITE_ID);
	return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);

}

inline int S5K8AAYX_write_cmos_sensor(u16 addr, u32 para)
{
   char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};

   iWriteRegI2C(puSendCmd , 4,S5K8AAYX_WRITE_ID);
}


#define WINMO_USE 0

#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT



#define	S5K8AAYX_LIMIT_EXPOSURE_LINES				(1253)
#define	S5K8AAYX_VIDEO_NORMALMODE_30FRAME_RATE       (30)
#define	S5K8AAYX_VIDEO_NORMALMODE_FRAME_RATE         (15)
#define	S5K8AAYX_VIDEO_NIGHTMODE_FRAME_RATE          (7.5)
#define BANDING50_30HZ
/* Global Valuable */


kal_bool S5K8AAYX_MPEG4_encode_mode = KAL_FALSE, S5K8AAYX_MJPEG_encode_mode = KAL_FALSE;

kal_uint32 S5K8AAYX_pixel_clk_freq = 0, S5K8AAYX_sys_clk_freq = 0;		// 480 means 48MHz

kal_uint16 S5K8AAYX_CAP_dummy_pixels = 0;
kal_uint16 S5K8AAYX_CAP_dummy_lines = 0;

kal_uint16 S5K8AAYX_PV_cintr = 0;
kal_uint16 S5K8AAYX_PV_cintc = 0;
kal_uint16 S5K8AAYX_CAP_cintr = 0;
kal_uint16 S5K8AAYX_CAP_cintc = 0;

kal_bool S5K8AAYX_night_mode_enable = KAL_FALSE;


//===============old============================================
static kal_uint8 S5K8AAYX_exposure_line_h = 0, S5K8AAYX_exposure_line_l = 0,S5K8AAYX_extra_exposure_line_h = 0, S5K8AAYX_extra_exposure_line_l = 0;

static kal_bool S5K8AAYX_gPVmode = KAL_TRUE; 
static kal_bool S5K8AAYX_VEDIO_encode_mode = KAL_FALSE; 
static kal_bool S5K8AAYX_sensor_cap_state = KAL_FALSE; 

static kal_uint16 S5K8AAYX_dummy_pixels=0, S5K8AAYX_dummy_lines=0;

static kal_uint16 S5K8AAYX_exposure_lines=0, S5K8AAYX_extra_exposure_lines = 0;



static kal_uint8 S5K8AAYX_Banding_setting = AE_FLICKER_MODE_50HZ;

/****** OVT 6-18******/
static kal_uint16 S5K8AAYX_Capture_Max_Gain16= 6*16;
static kal_uint16 S5K8AAYX_Capture_Gain16=0 ;    
static kal_uint16 S5K8AAYX_Capture_Shutter=0;
static kal_uint16 S5K8AAYX_Capture_Extra_Lines=0;

static kal_uint16  S5K8AAYX_PV_Dummy_Pixels =0, S5K8AAYX_Capture_Dummy_Pixels =0, S5K8AAYX_Capture_Dummy_Lines =0;
static kal_uint16  S5K8AAYX_PV_Gain16 = 0;
static kal_uint16  S5K8AAYX_PV_Shutter = 0;
static kal_uint16  S5K8AAYX_PV_Extra_Lines = 0;
kal_uint16 S5K8AAYX_sensor_gain_base=0,S5K8AAYX_FAC_SENSOR_REG=0,S5K8AAYX_iS5K8AAYX_Mode=0,S5K8AAYX_max_exposure_lines=0;
kal_uint32 S5K8AAYX_capture_pclk_in_M=520,S5K8AAYX_preview_pclk_in_M=390,S5K8AAYX_PV_dummy_pixels=0,S5K8AAYX_PV_dummy_lines=0,S5K8AAYX_isp_master_clock=0;
static kal_uint32  S5K8AAYX_sensor_pclk=720;
static kal_bool S5K8AAYX_AWB_ENABLE = KAL_TRUE; 
static kal_bool S5K8AAYX_AE_ENABLE = KAL_TRUE; 

//===============old============================================

kal_uint8 S5K8AAYX_sensor_write_I2C_address = S5K8AAYX_WRITE_ID;
kal_uint8 S5K8AAYX_sensor_read_I2C_address = S5K8AAYX_READ_ID;
kal_uint16 S5K8AAYX_Sensor_ID = 0;

//HANDLE S5K8AAYXhDrvI2C;
//I2C_TRANSACTION S5K8AAYXI2CConfig;

UINT8 S5K8AAYXPixelClockDivider=0;


MSDK_SENSOR_CONFIG_STRUCT S5K8AAYXSensorConfigData;


static void S5K8AAYXInitialPara(void)
{
  spin_lock(&s5k8aayx_drv_lock);
  S5K8AAYXCurrentStatus.iNightMode = 0xFFFF;
  S5K8AAYXCurrentStatus.iWB = AWB_MODE_AUTO;
  S5K8AAYXCurrentStatus.iEffect = MEFFECT_OFF;
  S5K8AAYXCurrentStatus.iBanding = AE_FLICKER_MODE_50HZ;
  S5K8AAYXCurrentStatus.iEV = AE_EV_COMP_n03;
  S5K8AAYXCurrentStatus.iMirror = IMAGE_NORMAL;
  S5K8AAYXCurrentStatus.iFrameRate = 0;
  spin_unlock(&s5k8aayx_drv_lock);
}


void S5K8AAYX_set_mirror(kal_uint8 image_mirror)
{

		if(S5K8AAYXCurrentStatus.iMirror == image_mirror)
		  return;

		S5K8AAYX_write_cmos_sensor(0x0028, 0x7000);
		S5K8AAYX_write_cmos_sensor(0x002a, 0x01E8); 
	
		switch (image_mirror)  
		{
			case IMAGE_NORMAL:
				S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000); 	// REG_0TC_PCFG_uPrevMirror
				S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000); 	// REG_0TC_PCFG_uCaptureMirror
				break;
			case IMAGE_H_MIRROR:
				S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001); 	// REG_0TC_PCFG_uPrevMirror
				S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001); 	// REG_0TC_PCFG_uCaptureMirror
				break;
			case IMAGE_V_MIRROR:
				S5K8AAYX_write_cmos_sensor(0x0F12, 0x0002); 	// REG_0TC_PCFG_uPrevMirror
				S5K8AAYX_write_cmos_sensor(0x0F12, 0x0002); 	// REG_0TC_PCFG_uCaptureMirror
				break;
			case IMAGE_HV_MIRROR:
				S5K8AAYX_write_cmos_sensor(0x0F12, 0x0003); 	// REG_0TC_PCFG_uPrevMirror
				S5K8AAYX_write_cmos_sensor(0x0F12, 0x0003); 	// REG_0TC_PCFG_uCaptureMirror
				break;
				
			default:
				ASSERT(0);
				break;
		}
	
		S5K8AAYX_write_cmos_sensor(0x002A,0x01A8); 
		S5K8AAYX_write_cmos_sensor(0x0F12,0x0000); // #REG_TC_GP_ActivePrevConfig // Select preview configuration_0
		S5K8AAYX_write_cmos_sensor(0x002A,0x01AC);
		S5K8AAYX_write_cmos_sensor(0x0F12,0x0001); // #REG_TC_GP_PrevOpenAfterChange
		S5K8AAYX_write_cmos_sensor(0x002A,0x01A6); 
		S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);  // #REG_TC_GP_NewConfigSync // Update preview configuration
		S5K8AAYX_write_cmos_sensor(0x002A,0x01AA); 
		S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);  // #REG_TC_GP_PrevConfigChanged
		S5K8AAYX_write_cmos_sensor(0x002A,0x019E); 
		S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);  // #REG_TC_GP_EnablePreview // Start preview
		S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);  // #REG_TC_GP_EnablePreviewChanged

		spin_lock(&s5k8aayx_drv_lock);
		S5K8AAYXCurrentStatus.iMirror = image_mirror;
		spin_unlock(&s5k8aayx_drv_lock);

}



void S5K8AAYX_set_isp_driving_current(kal_uint8 current)
{
}

void S5K8AAYX_set_dummy(kal_uint16 dummy_pixels, kal_uint16 dummy_lines)
{
		/****************************************************
		  * Adjust the extra H-Blanking & V-Blanking.
		  *****************************************************/
		S5K8AAYX_write_cmos_sensor(0x0028, 0x7000); 
		S5K8AAYX_write_cmos_sensor(0x002A, 0x044C); 
		
		S5K8AAYX_write_cmos_sensor(0x0F12, dummy_pixels); 
		//S5K8AAYX_write_cmos_sensor(0x0F1C, dummy_pixels); 	// Extra H-Blanking
		S5K8AAYX_write_cmos_sensor(0x0F12, dummy_lines); 	// Extra V-Blanking
}   /* S5K8AAYX_set_dummy */

	/*****************************************************************************
	* FUNCTION
	*  S5K8AAYX_Initialize_Setting
	* DESCRIPTION
	*
	* PARAMETERS
	*  void
	* RETURNS
	*  void
	*****************************************************************************/
	void S5K8AAYX_Initialize_Setting(void)
	{

	//*************************************/
	// 01.Start Setting					  */
	//*************************************/
	S5K8AAYX_write_cmos_sensor(0xFCFC, 0xD000);
	S5K8AAYX_write_cmos_sensor(0x0010, 0x0001);// S/W Reset */
	S5K8AAYX_write_cmos_sensor(0xFCFC, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0000, 0x0000);// Simmian bug workaround */

	S5K8AAYX_write_cmos_sensor(0xFCFC, 0xD000);
	S5K8AAYX_write_cmos_sensor(0x1030, 0x0000);// contint_host_int */
	S5K8AAYX_write_cmos_sensor(0x0014, 0x0001);

	mdelay(20);// Delay 1ms */                      

	//*************************************/          
	// 02.Analog Setting & ASP Control    */          
	//*************************************/          

	//*************************************/          
	// 03.Trap and Patch			 
	//*************************************/          
	// Start T&P part                                 
	// DO NOT DELETE T&P SECTION COMMENTS! They are required to debug T&P related issues.
	// https://svn/svn/SVNRoot/System/Software/tcevb/SDK+FW/ISP_8AA/Firmware
	// SVN Rev: 42166-42166                           
	// ROM Rev:                                       
	// Signature:                                     
	// md5 08808857c253b85b272ad1bae6844f68 .btp      
	// md5 6c3dd92aafd427ef96ddcecb8d9a0dcd .htp      
	// md5 92f4d9343102ef4aa626c999a25b1be9 .RegsMap.h
	// md5 6c9c1c1457ead489e12ee0feebb3adf1 .RegsMap.bin
	// md5 9d27e1e105fd2ae52c3f25336e875aa3 .base.RegsMap.h
	// md5 093c743f5c4be55ad550ddf32d7d8512 .base.RegsMap.bin
	//                                                
	S5K8AAYX_write_cmos_sensor(0x0028, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x002A, 0x2470);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xB510);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4910);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4810);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xF000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFA41);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4910);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4810);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xF000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFA3D);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4910);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4810);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x6341);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4910);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4811);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xF000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFA36);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4910);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4811);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xF000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFA32);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4910);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4811);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xF000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFA2E);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4810);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4911);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x6448);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4911);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4811);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xF000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFA27);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xBC10);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xBC08);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4718);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2870);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x8EDD);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x27E8);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x8725);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2788);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0080);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x26DC);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xA6EF);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x26A8);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xA0F1);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2674);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x058F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2568);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x24F4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xAC79);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4070);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE92D);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x5000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x23BC);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00BE);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x10BC);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE351);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000E);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0A00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x33A8);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x14BC);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D3);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE151);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000A);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x9A00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE041);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x11B0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0091);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1034);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE593);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0520);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0091);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1384);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1008);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE591);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00F0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xEB00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xEA00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE3A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00EE);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xEB00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0004);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE080);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE585);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4070);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE8BD);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF1E);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x401F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE92D);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00EB);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xEB00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE3A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1002);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE3A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0F86);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE3A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00EA);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xEB00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2010);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE3A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE28D);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0E3F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE3A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00E9);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xEB00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2001);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE3A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1002);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE3A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0F86);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE3A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00E2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xEB00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE5DD);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00C3);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE350);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1A00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE5DD);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE350);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1A00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000E);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE5DD);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0011);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE350);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0029);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1A00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE5DD);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0011);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE350);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0026);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1A00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02E8);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x10BA);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE351);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0022);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0A00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x12E4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE5D1);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE352);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0002);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1A00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00B8);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE350);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001B);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0A00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE3A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE5C1);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1002);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE28D);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0015);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xEA00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE5D1);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3001);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE5D1);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3403);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE182);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xC2B0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2080);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE08C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE7B4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x039E);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE004);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE80F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE3E0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4624);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE00E);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x47B4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1C2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4004);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE280);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xC084);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE08C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x47B4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1DC);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0493);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE004);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4624);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE00E);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x47B4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1CC);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xC8B4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x039C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE003);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3623);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE00E);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x38B4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1C2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE280);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1002);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE281);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0004);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE350);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFE7);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xBAFF);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x401F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE8BD);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF1E);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4010);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE92D);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00B1);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xEB00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0250);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00B2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE350);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0004);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0A00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0080);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE310);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0002);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1A00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x123C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE3A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0DB2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1C1);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4010);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE8BD);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF1E);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4010);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE92D);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE590);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0004);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00A5);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xEB00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x021C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE5D0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE350);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0002);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0A00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0004);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE594);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0004);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE584);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4010);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE8BD);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF1E);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4070);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE92D);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE590);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0800);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0820);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4041);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE280);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01E8);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x11B8);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x51B6);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0005);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE041);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0094);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1D11);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE3A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x007F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xEB00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x11C8);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE5D1);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE351);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0A00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x21B0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3FB0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE353);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0003);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0A00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x31AC);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x5BB2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1C3);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xC000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE085);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xCBB4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1C3);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE351);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0A00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0080);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1DBC);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3EB4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2EB2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0193);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE001);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0092);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2811);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE3A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0194);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE001);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0092);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x11A1);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0064);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xEB00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1168);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02B4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1C1);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4070);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE8BD);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF1E);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4010);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE92D);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0072);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xEB00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2150);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x14B0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0080);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE311);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0005);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0A00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0144);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00B0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE350);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x9A00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE3A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xEA00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE3A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3118);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE5C3);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE5D3);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE350);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0003);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0A00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0080);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE3C1);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1114);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x04B0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1C2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00B2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1C1);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4010);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE8BD);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF1E);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x41F0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE92D);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE590);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xC801);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xC82C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1004);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE590);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1801);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1821);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4008);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE590);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x500C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE590);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2004);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3005);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0052);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xEB00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x609C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00B2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D6);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE350);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000E);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0A00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00C0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x05B4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0002);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE350);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000A);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1A00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7080);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x10F4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D6);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x26B0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D7);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00F0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0048);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xEB00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00B0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1C4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x26B0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D7);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x10F6);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D6);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00F0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D5);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0043);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xEB00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00B0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1C5);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x41F0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE8BD);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF1E);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4010);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE92D);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1004);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE594);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00B0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1D0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE350);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0008);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0A00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0030);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3001);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE1A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2068);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE590);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0058);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1005);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE3A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0036);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xEB00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE584);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4010);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE8BD);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF1E);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE594);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0034);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xEB00);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE584);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFF9);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xEAFF);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3360);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x20D4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x16C8);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x299C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1272);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1728);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x112C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x29A0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x122C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xF200);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xD000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2340);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0E2C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xF400);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xD000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0CDC);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x06D4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4778);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x46C0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xC000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xC091);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xF004);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE51F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xD14C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xC000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xAC79);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xC000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0467);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xC000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2FA7);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xC000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xCB1F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xC000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x058F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xC000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xA0F1);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xC000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2B43);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xC000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x8725);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xC000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x6777);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xC000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x8E49);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xC000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x8EDD);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xA4B6);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
	//                                                
	// Parameters Defined in T&P:                     
	//                                14580 70000000 STRUCT
	// TnP_SvnVersion                     2 70002998 SHORT
	// Tune_TP                           18 70003360 STRUCT
	// afit_bUseNormBrForAfit             2 70003360 SHORT
	// awbb_bUseOutdoorGrid               2 70003362 SHORT
	// awbb_OutdoorGridCorr_R             2 70003364 SHORT
	// awbb_OutdoorGridCorr_B             2 70003366 SHORT
	// Tune_TP_bReMultGainsByNvm          2 70003368 SHORT
	// Tune_TP_bUseNvmMultGain            2 7000336A SHORT
	// Tune_TP_bCorrectBlueChMismatch     2 7000336C SHORT
	// Tune_TP_BlueGainOfs88              2 7000336E SHORT
	// Tune_TP_BlueGainFactor88           2 70003370 SHORT
	//                                                
	// End T&P part                                   

	//==================================================================================
	// 04.Analog Setting & APS Control                
	//==================================================================================
	//This register is for FACTORY ONLY.              
	//If you change it without prior notification     
	//YOU are RESPONSIBLE for the FAILURE that will happen in the future
	S5K8AAYX_write_cmos_sensor(0x0028, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0E38);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0476);	//senHal_RegCompBiasNormSf //CDS bias
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0476);	//senHal_RegCompBiasYAv //CDS bias
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0AA0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);	//setot_bUseDigitalHbin //1-Digital, 0-Analog
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0E2C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);	//senHal_bUseAnalogVerAv //2-Adding/averaging, 1-Y-Avg, 0-PLA
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0E66);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);	//senHal_RegBlstEnNorm
	S5K8AAYX_write_cmos_sensor(0x002A, 0x1250);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFFF); 	//senHal_Bls_nSpExpLines
	S5K8AAYX_write_cmos_sensor(0x002A, 0x1202);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0010); 	//senHal_Dblr_VcoFreqMHZ
	//ADLC Filter                            
	S5K8AAYX_write_cmos_sensor(0x002A, 0x1288);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x020F);	//gisp_dadlc_ResetFilterValue
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1C02);	//gisp_dadlc_SteadyFilterValue
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0006);	//gisp_dadlc_NResetIIrFrames	

	//*************************************/ 
	// 05.OTP Control                     */ 
	//*************************************/ 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x3368);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// Tune_TP_bReMultGainsByNvm */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);	// Tune_TP_bUseNvmMultGain            2 7000336A SHORT
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// Tune_TP_bCorrectBlueChMismatch     2 7000336C SHORT
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// Tune_TP_BlueGainOfs88              2 7000336E SHORT
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// Tune_TP_BlueGainFactor88           2 70003370 SHORT

	//*********************************************************************************
	// 06.GAS (Grid Anti-Shading)                                            
	//*********************************************************************************
	S5K8AAYX_write_cmos_sensor(0x002A, 0x1326);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// gisp_gos_Enable 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x063A);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASAlpha_0__0_ Horizon 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASAlpha_0__1_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASAlpha_0__2_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASAlpha_0__3_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASAlpha_1__0_ IncandA 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASAlpha_1__1_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASAlpha_1__2_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASAlpha_1__3_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASAlpha_2__0_ WW      
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASAlpha_2__1_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASAlpha_2__2_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASAlpha_2__3_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00E8);	// TVAR_ash_GASAlpha_3__0_ CW      
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASAlpha_3__1_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASAlpha_3__2_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASAlpha_3__3_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00C8);	// TVAR_ash_GASAlpha_4__0_ D50     
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00F8);	// TVAR_ash_GASAlpha_4__1_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00F8);	// TVAR_ash_GASAlpha_4__2_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASAlpha_4__3_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00F0);	// TVAR_ash_GASAlpha_5__0_ D65     
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASAlpha_5__1_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASAlpha_5__2_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASAlpha_5__3_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00F0);	// TVAR_ash_GASAlpha_6__0_ D75     
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASAlpha_6__1_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASAlpha_6__2_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASAlpha_6__3_         
	S5K8AAYX_write_cmos_sensor(0x002A, 0x067A);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_0__0_ Horizon 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_0__1_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_0__2_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_0__3_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_1__0_ IncandA 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_1__1_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_1__2_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_1__3_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_2__0_ WW      
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_2__1_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_2__2_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_2__3_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_3__0_ CW      
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_3__1_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_3__2_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_3__3_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_4__0_ D50     
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_4__1_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_4__2_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_4__3_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_5__0_ D65     
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_5__1_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_5__2_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_5__3_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_6__0_ D75     
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_6__1_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_6__2_         
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASBeta_6__3_         
	S5K8AAYX_write_cmos_sensor(0x002A, 0x06BA);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);	//ash_bLumaMode 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0632);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// ash_CGrasAlphas_0_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// ash_CGrasAlphas_1_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// ash_CGrasAlphas_2_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// ash_CGrasAlphas_3_ 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0672);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASOutdoorAlpha_0_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASOutdoorAlpha_1_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASOutdoorAlpha_2_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);	// TVAR_ash_GASOutdoorAlpha_3_ 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x06B2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASOutdoorBeta_0_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASOutdoorBeta_1_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASOutdoorBeta_2_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// ash_GASOutdoorBeta_3_ 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x06D0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000D);	// ash_uParabolicScalingA
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000F);	// ash_uParabolicScalingB
	S5K8AAYX_write_cmos_sensor(0x002A, 0x06CC);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0280);	// ash_uParabolicCenterX 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01E0);	// ash_uParabolicCenterY 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x06C6);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);	// ash_bParabolicEstimation 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0624);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x009D);	// TVAR_ash_AwbAshCord_0_ Horizon 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00D5);	// TVAR_ash_AwbAshCord_1_ IncandA 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0103);	// TVAR_ash_AwbAshCord_2_ WW      
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0128);	// TVAR_ash_AwbAshCord_3_ CW      
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0166);	// TVAR_ash_AwbAshCord_4_ D50     
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0193);	// TVAR_ash_AwbAshCord_5_ D65     
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01A0);	// TVAR_ash_AwbAshCord_6_ D75  

	S5K8AAYX_write_cmos_sensor(0x002A, 0x347C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x013B);	// 011A 012F Tune_wbt_GAS_0_   
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0116);	// 011A 0111 Tune_wbt_GAS_1_   
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00D9);	// 00EE 00D6 Tune_wbt_GAS_2_   
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00A6);	// 00C1 009E Tune_wbt_GAS_3_   
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0082);	// 009E 007A Tune_wbt_GAS_4_   
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x006C);	// 008A 0064 Tune_wbt_GAS_5_   
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0065);	// 0083 005D Tune_wbt_GAS_6_   
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x006C);	// 008A 0063 Tune_wbt_GAS_7_   
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0080);	// 009E 007B Tune_wbt_GAS_8_   
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00A3);	// 00BF 00A3 Tune_wbt_GAS_9_   
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00D4);	// 00E5 00E3 Tune_wbt_GAS_10_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x010D);	// 00F9 013B Tune_wbt_GAS_11_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x012E);	// 0124 018B Tune_wbt_GAS_12_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0138);	// 0126 012B Tune_wbt_GAS_13_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0104);	// 010E 00F6 Tune_wbt_GAS_14_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00BE);	// 00D3 00B2 Tune_wbt_GAS_15_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0088);	// 009F 007B Tune_wbt_GAS_16_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0062);	// 007C 0057 Tune_wbt_GAS_17_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x004D);	// 0068 0042 Tune_wbt_GAS_18_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0046);	// 0061 0039 Tune_wbt_GAS_19_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x004C);	// 0068 003F Tune_wbt_GAS_20_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0060);	// 007E 0053 Tune_wbt_GAS_21_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0084);	// 00A3 007A Tune_wbt_GAS_22_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00B8);	// 00C9 00B6 Tune_wbt_GAS_23_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00F9);	// 00F0 0110 Tune_wbt_GAS_24_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x012C);	// 0131 016A Tune_wbt_GAS_25_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x011A);	// 011C 0114 Tune_wbt_GAS_26_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00DB);	// 00EB 00D4 Tune_wbt_GAS_27_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0093);	// 00AA 008F Tune_wbt_GAS_28_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005F);	// 0075 005A Tune_wbt_GAS_29_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003C);	// 0053 0035 Tune_wbt_GAS_30_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0027);	// 003F 0020 Tune_wbt_GAS_31_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0020);	// 0038 0019 Tune_wbt_GAS_32_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0026);	// 0040 001F Tune_wbt_GAS_33_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003A);	// 0055 0032 Tune_wbt_GAS_34_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005C);	// 007A 0056 Tune_wbt_GAS_35_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x008E);	// 00A6 008E Tune_wbt_GAS_36_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00D2);	// 00D5 00E6 Tune_wbt_GAS_37_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x010E);	// 0126 0142 Tune_wbt_GAS_38_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0101);	// 00F6 0102 Tune_wbt_GAS_39_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00BF);	// 00C0 00BE Tune_wbt_GAS_40_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0077);	// 007D 0077 Tune_wbt_GAS_41_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0044);	// 004D 0045 Tune_wbt_GAS_42_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0023);	// 002C 0022 Tune_wbt_GAS_43_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0011);	// 001B 000D Tune_wbt_GAS_44_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000C);	// 0017 0006 Tune_wbt_GAS_45_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0010);	// 001B 000A Tune_wbt_GAS_46_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0022);	// 002D 001D Tune_wbt_GAS_47_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0043);	// 004E 003F Tune_wbt_GAS_48_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0074);	// 0080 0075 Tune_wbt_GAS_49_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00B7);	// 00C1 00CC Tune_wbt_GAS_50_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00F7);	// 00FF 0126 Tune_wbt_GAS_51_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00FC);	// 00EA 00FB Tune_wbt_GAS_52_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00B7);	// 00B0 00B7 Tune_wbt_GAS_53_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x006F);	// 006D 0070 Tune_wbt_GAS_54_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003C);	// 003D 003D Tune_wbt_GAS_55_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001C);	// 001E 001B Tune_wbt_GAS_56_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000A);	// 000F 0007 Tune_wbt_GAS_57_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0004);	// 000A 0000 Tune_wbt_GAS_58_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000A);	// 0010 0004 Tune_wbt_GAS_59_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001B);	// 001E 0015 Tune_wbt_GAS_60_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003B);	// 003E 0034 Tune_wbt_GAS_61_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x006C);	// 006F 006A Tune_wbt_GAS_62_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00B0);	// 00B2 00C0 Tune_wbt_GAS_63_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00F2);	// 00F1 011B Tune_wbt_GAS_64_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00EF);	// 00E0 0102 Tune_wbt_GAS_65_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00AB);	// 00A6 00BA Tune_wbt_GAS_66_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0065);	// 0063 0073 Tune_wbt_GAS_67_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0034);	// 0033 0040 Tune_wbt_GAS_68_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0015);	// 0016 001D Tune_wbt_GAS_69_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0004);	// 0008 0009 Tune_wbt_GAS_70_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// 0003 0001 Tune_wbt_GAS_71_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0004);	// 0009 0006 Tune_wbt_GAS_72_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0013);	// 0016 0017 Tune_wbt_GAS_73_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0033);	// 0035 0039 Tune_wbt_GAS_74_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0063);	// 0066 006F Tune_wbt_GAS_75_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00A5);	// 00A7 00C8 Tune_wbt_GAS_76_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00E5);	// 00E9 011E Tune_wbt_GAS_77_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00F7);	// 00D5 0111 Tune_wbt_GAS_78_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00B4);	// 009D 00C8 Tune_wbt_GAS_79_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x006D);	// 005D 0081 Tune_wbt_GAS_80_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003C);	// 002F 004D Tune_wbt_GAS_81_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001C);	// 0010 0028 Tune_wbt_GAS_82_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000B);	// 0004 0014 Tune_wbt_GAS_83_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0005);	// 0001 000B Tune_wbt_GAS_84_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000A);	// 0005 0010 Tune_wbt_GAS_85_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001B);	// 0013 0022 Tune_wbt_GAS_86_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003B);	// 0031 0047 Tune_wbt_GAS_87_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x006B);	// 005F 007E Tune_wbt_GAS_88_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00AD);	// 00A1 00D8 Tune_wbt_GAS_89_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00ED);	// 00DF 0131 Tune_wbt_GAS_90_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x010B);	// 00E9 0129 Tune_wbt_GAS_91_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00CB);	// 00B2 00E4 Tune_wbt_GAS_92_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0085);	// 006F 009E Tune_wbt_GAS_93_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0051);	// 003F 0068 Tune_wbt_GAS_94_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002F);	// 0020 0041 Tune_wbt_GAS_95_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001C);	// 000F 002B Tune_wbt_GAS_96_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0016);	// 000B 0023 Tune_wbt_GAS_97_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001C);	// 0010 0028 Tune_wbt_GAS_98_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002E);	// 0021 003B Tune_wbt_GAS_99_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x004F);	// 0041 0063 Tune_wbt_GAS_100_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0081);	// 0071 00A0 Tune_wbt_GAS_101_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00C4);	// 00B4 00FD Tune_wbt_GAS_102_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0102);	// 00F6 015A Tune_wbt_GAS_103_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0119);	// 00F9 014A Tune_wbt_GAS_104_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00DF);	// 00C2 010D Tune_wbt_GAS_105_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x009B);	// 0082 00CA Tune_wbt_GAS_106_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0067);	// 0053 008F Tune_wbt_GAS_107_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0045);	// 0033 0066 Tune_wbt_GAS_108_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0030);	// 0021 004F Tune_wbt_GAS_109_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0029);	// 001C 0048 Tune_wbt_GAS_110_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002F);	// 0022 004E Tune_wbt_GAS_111_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0043);	// 0035 0067 Tune_wbt_GAS_112_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0066);	// 0056 008F Tune_wbt_GAS_113_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0098);	// 0087 00D1 Tune_wbt_GAS_114_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00D9);	// 00CA 0132 Tune_wbt_GAS_115_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x010F);	// 0107 018D Tune_wbt_GAS_116_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0138);	// 0108 0159 Tune_wbt_GAS_117_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x010C);	// 00E0 0141 Tune_wbt_GAS_118_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00CB);	// 00A2 0103 Tune_wbt_GAS_119_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0097);	// 0072 00C9 Tune_wbt_GAS_120_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0073);	// 0052 009E Tune_wbt_GAS_121_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005C);	// 0040 0087 Tune_wbt_GAS_122_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0054);	// 003A 007D Tune_wbt_GAS_123_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005B);	// 0041 0087 Tune_wbt_GAS_124_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0070);	// 0055 00A2 Tune_wbt_GAS_125_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0096);	// 0077 00D4 Tune_wbt_GAS_126_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00C9);	// 00A8 011A Tune_wbt_GAS_127_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0106);	// 00E7 017D Tune_wbt_GAS_128_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x012D);	// 011E 01CF Tune_wbt_GAS_129_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0147);	// 0123 0181 Tune_wbt_GAS_130_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x012F);	// 0100 0169 Tune_wbt_GAS_131_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00F8);	// 00CB 0140 Tune_wbt_GAS_132_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00C5);	// 009A 0106 Tune_wbt_GAS_133_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00A1);	// 0079 00DD Tune_wbt_GAS_134_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x008B);	// 0067 00C5 Tune_wbt_GAS_135_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0083);	// 0060 00BE Tune_wbt_GAS_136_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x008B);	// 0068 00C8 Tune_wbt_GAS_137_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00A0);	// 007B 00E6 Tune_wbt_GAS_138_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00C2);	// 009D 011B Tune_wbt_GAS_139_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00F3);	// 00CD 015F Tune_wbt_GAS_140_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0124);	// 0108 01BC Tune_wbt_GAS_141_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0139);	// 0131 0206 Tune_wbt_GAS_142_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0093);	// 006C 00A8 Tune_wbt_GAS_143_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x007E);	// 006E 008F Tune_wbt_GAS_144_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0062);	// 005D 006F Tune_wbt_GAS_145_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x004D);	// 004C 0054 Tune_wbt_GAS_146_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003E);	// 0040 0041 Tune_wbt_GAS_147_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0034);	// 0037 0036 Tune_wbt_GAS_148_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0030);	// 0035 0033 Tune_wbt_GAS_149_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0032);	// 0036 0037 Tune_wbt_GAS_150_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003B);	// 003F 0045 Tune_wbt_GAS_151_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0049);	// 004B 005A Tune_wbt_GAS_152_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005C);	// 0053 007A Tune_wbt_GAS_153_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0077);	// 0053 00AA Tune_wbt_GAS_154_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x008A);	// 0070 00E2 Tune_wbt_GAS_155_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0093);	// 0075 009E Tune_wbt_GAS_156_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0077);	// 006D 007D Tune_wbt_GAS_157_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0059);	// 0056 005A Tune_wbt_GAS_158_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0042);	// 0043 0041 Tune_wbt_GAS_159_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0032);	// 0037 002E Tune_wbt_GAS_160_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0027);	// 002F 0022 Tune_wbt_GAS_161_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0024);	// 002C 001F Tune_wbt_GAS_162_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0026);	// 002F 0022 Tune_wbt_GAS_163_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002F);	// 0038 0030 Tune_wbt_GAS_164_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003D);	// 0045 0044 Tune_wbt_GAS_165_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0052);	// 004E 0063 Tune_wbt_GAS_166_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x006E);	// 0055 0091 Tune_wbt_GAS_167_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x008B);	// 007F 00CB Tune_wbt_GAS_168_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0083);	// 0077 0093 Tune_wbt_GAS_169_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0064);	// 0062 006D Tune_wbt_GAS_170_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0046);	// 004A 004A Tune_wbt_GAS_171_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0030);	// 0037 0031 Tune_wbt_GAS_172_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0020);	// 002A 001E Tune_wbt_GAS_173_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0016);	// 0021 0013 Tune_wbt_GAS_174_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0011);	// 001E 000F Tune_wbt_GAS_175_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0014);	// 0021 0013 Tune_wbt_GAS_176_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001E);	// 002B 001F Tune_wbt_GAS_177_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002D);	// 003B 0034 Tune_wbt_GAS_178_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0041);	// 0046 0051 Tune_wbt_GAS_179_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005D);	// 0051 007C Tune_wbt_GAS_180_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x007C);	// 007F 00B7 Tune_wbt_GAS_181_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0077);	// 0062 008A Tune_wbt_GAS_182_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0057);	// 004B 0061 Tune_wbt_GAS_183_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0039);	// 0034 003E Tune_wbt_GAS_184_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0024);	// 0022 0026 Tune_wbt_GAS_185_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0014);	// 0014 0013 Tune_wbt_GAS_186_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000A);	// 000E 0008 Tune_wbt_GAS_187_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0007);	// 000C 0004 Tune_wbt_GAS_188_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0009);	// 000D 0007 Tune_wbt_GAS_189_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0012);	// 0016 0013 Tune_wbt_GAS_190_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0021);	// 0024 0027 Tune_wbt_GAS_191_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0036);	// 0036 0045 Tune_wbt_GAS_192_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0051);	// 004E 0070 Tune_wbt_GAS_193_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0070);	// 0069 00A7 Tune_wbt_GAS_194_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0077);	// 005F 0085 Tune_wbt_GAS_195_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0056);	// 0048 005D Tune_wbt_GAS_196_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0038);	// 002F 003A Tune_wbt_GAS_197_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0022);	// 001C 0021 Tune_wbt_GAS_198_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0013);	// 0010 0010 Tune_wbt_GAS_199_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0009);	// 000B 0004 Tune_wbt_GAS_200_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0005);	// 0008 0000 Tune_wbt_GAS_201_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0008);	// 000B 0003 Tune_wbt_GAS_202_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0011);	// 0010 000E Tune_wbt_GAS_203_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0020);	// 001E 0021 Tune_wbt_GAS_204_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0035);	// 0031 003F Tune_wbt_GAS_205_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0051);	// 0049 006B Tune_wbt_GAS_206_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0071);	// 0066 00A1 Tune_wbt_GAS_207_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x006E);	// 005B 0089 Tune_wbt_GAS_208_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x004E);	// 0043 0060 Tune_wbt_GAS_209_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0032);	// 002B 003D Tune_wbt_GAS_210_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001C);	// 0019 0023 Tune_wbt_GAS_211_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000D);	// 000C 0012 Tune_wbt_GAS_212_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0004);	// 0007 0006 Tune_wbt_GAS_213_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// 0004 0002 Tune_wbt_GAS_214_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0003);	// 0007 0005 Tune_wbt_GAS_215_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000B);	// 000D 0011 Tune_wbt_GAS_216_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001A);	// 001B 0025 Tune_wbt_GAS_217_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002F);	// 002E 0043 Tune_wbt_GAS_218_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0049);	// 0046 0070 Tune_wbt_GAS_219_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0068);	// 0062 00A4 Tune_wbt_GAS_220_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0072);	// 0052 0091 Tune_wbt_GAS_221_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0053);	// 003D 0067 Tune_wbt_GAS_222_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0037);	// 0026 0044 Tune_wbt_GAS_223_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0021);	// 0014 002B Tune_wbt_GAS_224_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0012);	// 0007 0019 Tune_wbt_GAS_225_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0009);	// 0002 000D Tune_wbt_GAS_226_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0005);	// 0001 0009 Tune_wbt_GAS_227_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0008);	// 0002 000C Tune_wbt_GAS_228_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0010);	// 0007 0018 Tune_wbt_GAS_229_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001F);	// 0015 002D Tune_wbt_GAS_230_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0034);	// 0028 004B Tune_wbt_GAS_231_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x004E);	// 0040 007B Tune_wbt_GAS_232_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x006C);	// 005C 00B0 Tune_wbt_GAS_233_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x007F);	// 005E 00A1 Tune_wbt_GAS_234_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0060);	// 0049 0077 Tune_wbt_GAS_235_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0043);	// 0030 0054 Tune_wbt_GAS_236_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002D);	// 001E 003B Tune_wbt_GAS_237_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001D);	// 0010 0029 Tune_wbt_GAS_238_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0013);	// 0009 001C Tune_wbt_GAS_239_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0010);	// 0007 0018 Tune_wbt_GAS_240_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0013);	// 0009 001B Tune_wbt_GAS_241_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001C);	// 0012 0029 Tune_wbt_GAS_242_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002B);	// 0020 003F Tune_wbt_GAS_243_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0040);	// 0032 005F Tune_wbt_GAS_244_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005A);	// 004B 008F Tune_wbt_GAS_245_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0079);	// 0069 00C6 Tune_wbt_GAS_246_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0082);	// 0066 00B1 Tune_wbt_GAS_247_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0066);	// 004E 008E Tune_wbt_GAS_248_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0049);	// 0037 006D Tune_wbt_GAS_249_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0035);	// 0026 0050 Tune_wbt_GAS_250_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0025);	// 0019 003D Tune_wbt_GAS_251_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001B);	// 0011 0031 Tune_wbt_GAS_252_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0017);	// 000F 002F Tune_wbt_GAS_253_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0019);	// 0012 0032 Tune_wbt_GAS_254_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0023);	// 001B 0042 Tune_wbt_GAS_255_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0033);	// 0028 0058 Tune_wbt_GAS_256_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0046);	// 003B 007A Tune_wbt_GAS_257_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0060);	// 0054 00AC Tune_wbt_GAS_258_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x007B);	// 0072 00E5 Tune_wbt_GAS_259_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0092);	// 006A 00BD Tune_wbt_GAS_260_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x007C);	// 0058 00AA Tune_wbt_GAS_261_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0060);	// 0041 008B Tune_wbt_GAS_262_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x004B);	// 0030 006E Tune_wbt_GAS_263_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003C);	// 0025 005A Tune_wbt_GAS_264_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0032);	// 001E 004F Tune_wbt_GAS_265_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002D);	// 001B 004B Tune_wbt_GAS_266_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0030);	// 001F 0052 Tune_wbt_GAS_267_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0039);	// 0027 0062 Tune_wbt_GAS_268_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0049);	// 0034 007D Tune_wbt_GAS_269_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005D);	// 0046 00A2 Tune_wbt_GAS_270_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0076);	// 005D 00D6 Tune_wbt_GAS_271_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x008C);	// 0078 010C Tune_wbt_GAS_272_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x009F);	// 007C 00E5 Tune_wbt_GAS_273_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x008F);	// 006A 00C7 Tune_wbt_GAS_274_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0077);	// 0055 00B1 Tune_wbt_GAS_275_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0061);	// 0043 0093 Tune_wbt_GAS_276_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0052);	// 0037 007F Tune_wbt_GAS_277_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0048);	// 0030 0074 Tune_wbt_GAS_278_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0043);	// 002E 0071 Tune_wbt_GAS_279_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0047);	// 0030 0077 Tune_wbt_GAS_280_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0050);	// 0039 0089 Tune_wbt_GAS_281_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005E);	// 0045 00A7 Tune_wbt_GAS_282_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0071);	// 0056 00CC Tune_wbt_GAS_283_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0086);	// 006C 00FE Tune_wbt_GAS_284_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0097);	// 0084 0132 Tune_wbt_GAS_285_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0093);	// 006E 00A8 Tune_wbt_GAS_286_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x007C);	// 006D 008D Tune_wbt_GAS_287_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005F);	// 005B 006C Tune_wbt_GAS_288_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0049);	// 0046 004E Tune_wbt_GAS_289_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003A);	// 003A 003C Tune_wbt_GAS_290_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0030);	// 0033 0032 Tune_wbt_GAS_291_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002C);	// 002D 002D Tune_wbt_GAS_292_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002F);	// 0032 0032 Tune_wbt_GAS_293_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0037);	// 0039 0040 Tune_wbt_GAS_294_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0045);	// 0047 0056 Tune_wbt_GAS_295_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005A);	// 004F 0076 Tune_wbt_GAS_296_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0075);	// 0050 00A8 Tune_wbt_GAS_297_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x008A);	// 006E 00E6 Tune_wbt_GAS_298_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0094);	// 0077 00A2 Tune_wbt_GAS_299_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0077);	// 006C 007C Tune_wbt_GAS_300_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0057);	// 0054 0059 Tune_wbt_GAS_301_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0040);	// 0040 003E Tune_wbt_GAS_302_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002F);	// 0033 002A Tune_wbt_GAS_303_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0024);	// 002B 0020 Tune_wbt_GAS_304_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0020);	// 0027 001B Tune_wbt_GAS_305_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0023);	// 002A 0020 Tune_wbt_GAS_306_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002D);	// 0034 002D Tune_wbt_GAS_307_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003B);	// 0041 0042 Tune_wbt_GAS_308_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0051);	// 004C 0061 Tune_wbt_GAS_309_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x006E);	// 0052 0092 Tune_wbt_GAS_310_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x008C);	// 007E 00CE Tune_wbt_GAS_311_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0085);	// 0078 0094 Tune_wbt_GAS_312_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0066);	// 0063 006F Tune_wbt_GAS_313_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0046);	// 0049 004B Tune_wbt_GAS_314_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002F);	// 0035 002F Tune_wbt_GAS_315_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001F);	// 0028 001D Tune_wbt_GAS_316_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0014);	// 001E 0011 Tune_wbt_GAS_317_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000F);	// 001B 000D Tune_wbt_GAS_318_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0012);	// 001F 0012 Tune_wbt_GAS_319_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001C);	// 0028 001D Tune_wbt_GAS_320_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002B);	// 0037 0033 Tune_wbt_GAS_321_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0040);	// 0044 0051 Tune_wbt_GAS_322_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005C);	// 0050 007F Tune_wbt_GAS_323_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x007D);	// 0080 00BA Tune_wbt_GAS_324_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x007A);	// 0064 008B Tune_wbt_GAS_325_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005A);	// 004E 0064 Tune_wbt_GAS_326_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003A);	// 0035 0040 Tune_wbt_GAS_327_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0024);	// 0021 0025 Tune_wbt_GAS_328_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0014);	// 0013 0013 Tune_wbt_GAS_329_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0009);	// 000D 0007 Tune_wbt_GAS_330_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0006);	// 000B 0003 Tune_wbt_GAS_331_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0008);	// 000C 0006 Tune_wbt_GAS_332_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0011);	// 0014 0012 Tune_wbt_GAS_333_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0020);	// 0022 0027 Tune_wbt_GAS_334_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0036);	// 0036 0046 Tune_wbt_GAS_335_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0051);	// 004D 0073 Tune_wbt_GAS_336_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0072);	// 006B 00AB Tune_wbt_GAS_337_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x007B);	// 0066 008B Tune_wbt_GAS_338_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0059);	// 004C 0062 Tune_wbt_GAS_339_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003A);	// 0032 003F Tune_wbt_GAS_340_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0023);	// 001E 0023 Tune_wbt_GAS_341_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0012);	// 0010 0010 Tune_wbt_GAS_342_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0008);	// 000A 0004 Tune_wbt_GAS_343_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0004);	// 0007 0000 Tune_wbt_GAS_344_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0007);	// 000B 0003 Tune_wbt_GAS_345_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000F);	// 0010 000E Tune_wbt_GAS_346_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001F);	// 001E 0022 Tune_wbt_GAS_347_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0035);	// 0032 0041 Tune_wbt_GAS_348_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0051);	// 004B 006E Tune_wbt_GAS_349_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0072);	// 006B 00A4 Tune_wbt_GAS_350_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0073);	// 0061 008E Tune_wbt_GAS_351_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0053);	// 0049 0065 Tune_wbt_GAS_352_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0034);	// 002F 0040 Tune_wbt_GAS_353_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001D);	// 001B 0025 Tune_wbt_GAS_354_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000E);	// 000E 0013 Tune_wbt_GAS_355_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0004);	// 0008 0006 Tune_wbt_GAS_356_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// 0004 0001 Tune_wbt_GAS_357_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0002);	// 0008 0005 Tune_wbt_GAS_358_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000A);	// 000D 0010 Tune_wbt_GAS_359_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001A);	// 001C 0025 Tune_wbt_GAS_360_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002F);	// 002F 0044 Tune_wbt_GAS_361_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x004A);	// 0047 0074 Tune_wbt_GAS_362_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x006A);	// 0067 00AA Tune_wbt_GAS_363_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0077);	// 005A 0097 Tune_wbt_GAS_364_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0058);	// 0043 006D Tune_wbt_GAS_365_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0039);	// 002B 0048 Tune_wbt_GAS_366_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0022);	// 0017 002D Tune_wbt_GAS_367_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0012);	// 0009 0019 Tune_wbt_GAS_368_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0008);	// 0004 000E Tune_wbt_GAS_369_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0004);	// 0002 0009 Tune_wbt_GAS_370_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0007);	// 0004 000C Tune_wbt_GAS_371_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000F);	// 0008 0018 Tune_wbt_GAS_372_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001E);	// 0016 002E Tune_wbt_GAS_373_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0034);	// 002A 004E Tune_wbt_GAS_374_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x004F);	// 0042 007D Tune_wbt_GAS_375_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x006F);	// 005F 00B5 Tune_wbt_GAS_376_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0083);	// 0066 00A6 Tune_wbt_GAS_377_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0064);	// 0050 007C Tune_wbt_GAS_378_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0045);	// 0035 0058 Tune_wbt_GAS_379_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002E);	// 0022 003E Tune_wbt_GAS_380_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001D);	// 0013 0029 Tune_wbt_GAS_381_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0012);	// 000A 001D Tune_wbt_GAS_382_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000F);	// 0008 0018 Tune_wbt_GAS_383_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0011);	// 000B 001B Tune_wbt_GAS_384_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001A);	// 0014 0028 Tune_wbt_GAS_385_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002A);	// 0021 003F Tune_wbt_GAS_386_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003F);	// 0035 0062 Tune_wbt_GAS_387_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005B);	// 004D 0093 Tune_wbt_GAS_388_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x007B);	// 006E 00CC Tune_wbt_GAS_389_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0087);	// 006E 00B9 Tune_wbt_GAS_390_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x006A);	// 0057 0094 Tune_wbt_GAS_391_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x004B);	// 003E 0071 Tune_wbt_GAS_392_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0036);	// 002B 0052 Tune_wbt_GAS_393_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0025);	// 001C 003D Tune_wbt_GAS_394_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0019);	// 0013 0031 Tune_wbt_GAS_395_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0015);	// 0011 002D Tune_wbt_GAS_396_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0017);	// 0013 0031 Tune_wbt_GAS_397_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0022);	// 001D 0040 Tune_wbt_GAS_398_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0031);	// 002B 0058 Tune_wbt_GAS_399_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0045);	// 003D 007B Tune_wbt_GAS_400_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0060);	// 0057 00AE Tune_wbt_GAS_401_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x007D);	// 0077 00EA Tune_wbt_GAS_402_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0096);	// 0072 00C2 Tune_wbt_GAS_403_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x007F);	// 005F 00AE Tune_wbt_GAS_404_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0061);	// 0047 008E Tune_wbt_GAS_405_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x004B);	// 0035 006F Tune_wbt_GAS_406_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003B);	// 0028 0059 Tune_wbt_GAS_407_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002F);	// 0020 004E Tune_wbt_GAS_408_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002A);	// 001D 0049 Tune_wbt_GAS_409_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002D);	// 0020 004F Tune_wbt_GAS_410_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0036);	// 0029 005E Tune_wbt_GAS_411_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0046);	// 0035 007A Tune_wbt_GAS_412_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005B);	// 0047 009F Tune_wbt_GAS_413_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0075);	// 0060 00D5 Tune_wbt_GAS_414_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x008D);	// 007D 010C Tune_wbt_GAS_415_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00A1);	// 0084 00E5 Tune_wbt_GAS_416_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0091);	// 0072 00C8 Tune_wbt_GAS_417_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0077);	// 005B 00B0 Tune_wbt_GAS_418_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0060);	// 0046 0091 Tune_wbt_GAS_419_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0050);	// 003A 007D Tune_wbt_GAS_420_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0044);	// 0031 0070 Tune_wbt_GAS_421_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0040);	// 002E 006D Tune_wbt_GAS_422_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0043);	// 0032 0074 Tune_wbt_GAS_423_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x004C);	// 0039 0086 Tune_wbt_GAS_424_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005A);	// 0046 00A4 Tune_wbt_GAS_425_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x006D);	// 0056 00CA Tune_wbt_GAS_426_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0084);	// 006E 00FE Tune_wbt_GAS_427_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0094);	// 0087 0134 Tune_wbt_GAS_428_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0072);	// 004C 009F Tune_wbt_GAS_429_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0063);	// 004C 0089 Tune_wbt_GAS_430_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x004C);	// 0041 0067 Tune_wbt_GAS_431_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003A);	// 002F 004E Tune_wbt_GAS_432_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002D);	// 0024 003E Tune_wbt_GAS_433_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0025);	// 001D 0033 Tune_wbt_GAS_434_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0023);	// 001B 0031 Tune_wbt_GAS_435_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0025);	// 001E 0036 Tune_wbt_GAS_436_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002C);	// 0024 0045 Tune_wbt_GAS_437_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0038);	// 0032 0058 Tune_wbt_GAS_438_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x004A);	// 0037 0074 Tune_wbt_GAS_439_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005F);	// 0038 00A0 Tune_wbt_GAS_440_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x006B);	// 004C 00C9 Tune_wbt_GAS_441_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0079);	// 005B 0098 Tune_wbt_GAS_442_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0065);	// 0056 0077 Tune_wbt_GAS_443_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x004A);	// 0041 0055 Tune_wbt_GAS_444_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0037);	// 0030 003C Tune_wbt_GAS_445_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0029);	// 0026 002A Tune_wbt_GAS_446_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0021);	// 001F 0022 Tune_wbt_GAS_447_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001D);	// 001C 001F Tune_wbt_GAS_448_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001F);	// 001F 0024 Tune_wbt_GAS_449_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0027);	// 0027 0030 Tune_wbt_GAS_450_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0033);	// 0033 0044 Tune_wbt_GAS_451_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0044);	// 003C 0060 Tune_wbt_GAS_452_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005E);	// 0041 008B Tune_wbt_GAS_453_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x006E);	// 0060 00B2 Tune_wbt_GAS_454_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x006A);	// 005E 0088 Tune_wbt_GAS_455_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0055);	// 004F 0065 Tune_wbt_GAS_456_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003A);	// 003A 0044 Tune_wbt_GAS_457_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0028);	// 002A 002C Tune_wbt_GAS_458_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001A);	// 001D 001B Tune_wbt_GAS_459_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0011);	// 0016 0012 Tune_wbt_GAS_460_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000D);	// 0014 000F Tune_wbt_GAS_461_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000F);	// 0016 0013 Tune_wbt_GAS_462_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0017);	// 0021 001E Tune_wbt_GAS_463_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0024);	// 002D 0032 Tune_wbt_GAS_464_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0035);	// 0038 004E Tune_wbt_GAS_465_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x004E);	// 0040 0078 Tune_wbt_GAS_466_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0061);	// 0066 00A2 Tune_wbt_GAS_467_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0061);	// 004A 007F Tune_wbt_GAS_468_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x004A);	// 003C 005B Tune_wbt_GAS_469_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0031);	// 0028 0039 Tune_wbt_GAS_470_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001E);	// 0019 0021 Tune_wbt_GAS_471_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0011);	// 000D 0012 Tune_wbt_GAS_472_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0008);	// 0008 0007 Tune_wbt_GAS_473_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0005);	// 0007 0004 Tune_wbt_GAS_474_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0007);	// 0007 0006 Tune_wbt_GAS_475_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000E);	// 000F 0012 Tune_wbt_GAS_476_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001B);	// 001C 0024 Tune_wbt_GAS_477_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002D);	// 002C 0041 Tune_wbt_GAS_478_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0045);	// 0042 006A Tune_wbt_GAS_479_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0059);	// 0054 0092 Tune_wbt_GAS_480_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0062);	// 004B 007B Tune_wbt_GAS_481_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x004B);	// 003C 0057 Tune_wbt_GAS_482_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0031);	// 0027 0036 Tune_wbt_GAS_483_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001E);	// 0017 001E Tune_wbt_GAS_484_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0010);	// 000C 000F Tune_wbt_GAS_485_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0008);	// 0007 0003 Tune_wbt_GAS_486_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0004);	// 0006 0000 Tune_wbt_GAS_487_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0006);	// 0008 0002 Tune_wbt_GAS_488_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000E);	// 000D 000D Tune_wbt_GAS_489_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001B);	// 001A 0020 Tune_wbt_GAS_490_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002E);	// 002B 003C Tune_wbt_GAS_491_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0046);	// 0041 0067 Tune_wbt_GAS_492_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005A);	// 0054 008D Tune_wbt_GAS_493_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005B);	// 0049 007E Tune_wbt_GAS_494_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0045);	// 003A 005A Tune_wbt_GAS_495_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002C);	// 0025 0038 Tune_wbt_GAS_496_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001A);	// 0015 0022 Tune_wbt_GAS_497_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000C);	// 000A 0013 Tune_wbt_GAS_498_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0003);	// 0005 0006 Tune_wbt_GAS_499_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// 0003 0001 Tune_wbt_GAS_500_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0002);	// 0006 0004 Tune_wbt_GAS_501_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0009);	// 000B 000F Tune_wbt_GAS_502_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0016);	// 0018 0023 Tune_wbt_GAS_503_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0029);	// 0029 003E Tune_wbt_GAS_504_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0040);	// 003E 006A Tune_wbt_GAS_505_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0054);	// 0050 0091 Tune_wbt_GAS_506_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005F);	// 0044 0085 Tune_wbt_GAS_507_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x004A);	// 0033 0060 Tune_wbt_GAS_508_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0031);	// 0021 0041 Tune_wbt_GAS_509_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001F);	// 0011 002A Tune_wbt_GAS_510_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0010);	// 0005 0019 Tune_wbt_GAS_511_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0008);	// 0002 000D Tune_wbt_GAS_512_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0004);	// 0001 0008 Tune_wbt_GAS_513_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0007);	// 0003 000A Tune_wbt_GAS_514_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000E);	// 0008 0016 Tune_wbt_GAS_515_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001B);	// 0014 002A Tune_wbt_GAS_516_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002E);	// 0025 0047 Tune_wbt_GAS_517_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0045);	// 0039 0074 Tune_wbt_GAS_518_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0059);	// 004D 009A Tune_wbt_GAS_519_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x006C);	// 0050 0097 Tune_wbt_GAS_520_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0057);	// 0041 0070 Tune_wbt_GAS_521_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003E);	// 002C 0052 Tune_wbt_GAS_522_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002A);	// 001C 003C Tune_wbt_GAS_523_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001B);	// 0011 0028 Tune_wbt_GAS_524_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0012);	// 0009 001D Tune_wbt_GAS_525_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000F);	// 0008 0019 Tune_wbt_GAS_526_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0011);	// 000A 001A Tune_wbt_GAS_527_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0019);	// 0012 0026 Tune_wbt_GAS_528_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0027);	// 001F 003B Tune_wbt_GAS_529_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0039);	// 002F 005A Tune_wbt_GAS_530_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0050);	// 0045 0089 Tune_wbt_GAS_531_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0063);	// 005A 00AF Tune_wbt_GAS_532_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x006F);	// 0056 00A7 Tune_wbt_GAS_533_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005C);	// 0048 0088 Tune_wbt_GAS_534_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0044);	// 0035 006B Tune_wbt_GAS_535_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0031);	// 0026 0050 Tune_wbt_GAS_536_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0023);	// 0019 003D Tune_wbt_GAS_537_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0019);	// 0012 0032 Tune_wbt_GAS_538_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0016);	// 0011 002E Tune_wbt_GAS_539_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0017);	// 0012 0030 Tune_wbt_GAS_540_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0020);	// 001B 003E Tune_wbt_GAS_541_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002E);	// 0027 0052 Tune_wbt_GAS_542_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0040);	// 0037 0073 Tune_wbt_GAS_543_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0055);	// 004C 00A1 Tune_wbt_GAS_544_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0064);	// 0060 00C7 Tune_wbt_GAS_545_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x007E);	// 0058 00AE Tune_wbt_GAS_546_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0071);	// 0050 00A4 Tune_wbt_GAS_547_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0059);	// 003D 0088 Tune_wbt_GAS_548_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0046);	// 002E 006D Tune_wbt_GAS_549_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0039);	// 0023 0059 Tune_wbt_GAS_550_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002F);	// 001C 004D Tune_wbt_GAS_551_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002A);	// 001B 0049 Tune_wbt_GAS_552_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x002D);	// 001D 004E Tune_wbt_GAS_553_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0035);	// 0024 005B Tune_wbt_GAS_554_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0043);	// 002F 0073 Tune_wbt_GAS_555_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0054);	// 003E 0095 Tune_wbt_GAS_556_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0069);	// 0052 00C4 Tune_wbt_GAS_557_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0074);	// 0062 00E9 Tune_wbt_GAS_558_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0083);	// 0060 00D5 Tune_wbt_GAS_559_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x007D);	// 0057 00C1 Tune_wbt_GAS_560_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0068);	// 0048 00AB Tune_wbt_GAS_561_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0055);	// 0039 008D Tune_wbt_GAS_562_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0048);	// 002E 0079 Tune_wbt_GAS_563_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003E);	// 0028 0070 Tune_wbt_GAS_564_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003A);	// 0027 006A Tune_wbt_GAS_565_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003D);	// 0029 006F Tune_wbt_GAS_566_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0045);	// 002F 0080 Tune_wbt_GAS_567_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0051);	// 0039 0096 Tune_wbt_GAS_568_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0061);	// 0047 00B9 Tune_wbt_GAS_569_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0072);	// 0059 00E2 Tune_wbt_GAS_570_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0077);	// 0067 010F Tune_wbt_GAS_571_ 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x1348);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);	// gisp_gras_Enable 

	//==================================================================================
	// 07. Analog Setting 2                  
	//==================================================================================   
	//This register is for FACTORY ONLY.     
	//If you change it without prior notification     
	//YOU are RESPONSIBLE for the FAILURE that will happen in the future      
	S5K8AAYX_write_cmos_sensor(0x002A, 0x1278);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xAAF0);	//gisp_dadlc_config //Ladlc mode average

	//*************************************/ 
	// 08.AF Setting                      */ 
	//*************************************/ 

	//*************************************/ 
	// 09.AWB-BASIC setting               */ 
	//*************************************/ 

	// For WB Calibration */                 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0B36);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0005);// awbb_IndoorGrZones_ZInfo_m_GridStep */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0B3A);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00F3);// awbb_IndoorGrZones_ZInfo_m_BMin */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02CB);// awbb_IndoorGrZones_ZInfo_m_BMax */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0B38);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0010);// awbb_IndoorGrZones_ZInfo_m_GridSz */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0AE6);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0385);// 0352 03E1 awbb_IndoorGrZones_m_BGrid_0__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03D8);// 038C 0413 awbb_IndoorGrZones_m_BGrid_0__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x032A);// 0321 039E awbb_IndoorGrZones_m_BGrid_1__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03C5);// 03A6 0416 awbb_IndoorGrZones_m_BGrid_1__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02F5);// 02EC 0367 awbb_IndoorGrZones_m_BGrid_2__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x039D);// 03A0 03F3 awbb_IndoorGrZones_m_BGrid_2__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02D3);// 02CA 032D awbb_IndoorGrZones_m_BGrid_3__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0372);// 038D 03C5 awbb_IndoorGrZones_m_BGrid_3__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02B1);// 02A8 02FD awbb_IndoorGrZones_m_BGrid_4__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x033E);// 036E 038F awbb_IndoorGrZones_m_BGrid_4__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x028A);// 0281 02D3 awbb_IndoorGrZones_m_BGrid_5__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0322);// 0344 0365 awbb_IndoorGrZones_m_BGrid_5__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0268);// 025F 02AA awbb_IndoorGrZones_m_BGrid_6__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02FD);// 0327 033E awbb_IndoorGrZones_m_BGrid_6__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0248);// 023F 028D awbb_IndoorGrZones_m_BGrid_7__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02EF);// 0302 0310 awbb_IndoorGrZones_m_BGrid_7__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x022F);// 0226 0271 awbb_IndoorGrZones_m_BGrid_8__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02D5);// 02DC 02F1 awbb_IndoorGrZones_m_BGrid_8__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0219);// 0210 025A awbb_IndoorGrZones_m_BGrid_9__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02C2);// 02B9 02D2 awbb_IndoorGrZones_m_BGrid_9__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0206);// 01FD 0249 awbb_IndoorGrZones_m_BGrid_10__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02A3);// 029A 02B9 awbb_IndoorGrZones_m_BGrid_10__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01F0);// 01E7 0238 awbb_IndoorGrZones_m_BGrid_11__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0286);// 027D 02A2 awbb_IndoorGrZones_m_BGrid_11__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01E3);// 01DA 021B awbb_IndoorGrZones_m_BGrid_12__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0268);// 025F 0289 awbb_IndoorGrZones_m_BGrid_12__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01D6);// 01CD 0200 awbb_IndoorGrZones_m_BGrid_13__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x024E);// 0245 026C awbb_IndoorGrZones_m_BGrid_13__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01DD);// 01D4 01FC awbb_IndoorGrZones_m_BGrid_14__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x022A);// 0221 024F awbb_IndoorGrZones_m_BGrid_14__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0210);// 0207 021E awbb_IndoorGrZones_m_BGrid_15__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01F2);// 01E9 022C awbb_IndoorGrZones_m_BGrid_15__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 0000 awbb_IndoorGrZones_m_BGrid_16__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 0000 awbb_IndoorGrZones_m_BGrid_16__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 0000 awbb_IndoorGrZones_m_BGrid_17__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 0000 awbb_IndoorGrZones_m_BGrid_17__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 0000 awbb_IndoorGrZones_m_BGrid_18__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 0000 awbb_IndoorGrZones_m_BGrid_18__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 0000 awbb_IndoorGrZones_m_BGrid_19__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 0000 awbb_IndoorGrZones_m_BGrid_19__m_right */

	S5K8AAYX_write_cmos_sensor(0x002A, 0x0BAA);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0006);// awbb_LowBrGrZones_ZInfo_m_GridStep */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0BAE);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00CC);// 010E awbb_LowBrGrZones_ZInfo_m_BMin */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02F3);// 02E9 awbb_LowBrGrZones_ZInfo_m_BMax */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0BAC);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000A);// 0009 awbb_LowBrGrZones_ZInfo_m_GridSz */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0B7A);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x036C);// 0374 038C awbb_LowBrGrZones_m_BGrid_0__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03C6);// 03CE 03DA awbb_LowBrGrZones_m_BGrid_0__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02EE);// 02F6 030E awbb_LowBrGrZones_m_BGrid_1__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03F9);// 0401 03E9 awbb_LowBrGrZones_m_BGrid_1__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02BE);// 02C6 02A2 awbb_LowBrGrZones_m_BGrid_2__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03DF);// 03E7 03C2 awbb_LowBrGrZones_m_BGrid_2__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x027A);// 0282 0259 awbb_LowBrGrZones_m_BGrid_3__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03AE);// 03B6 038A awbb_LowBrGrZones_m_BGrid_3__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0234);// 023C 0218 awbb_LowBrGrZones_m_BGrid_4__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0376);// 037E 0352 awbb_LowBrGrZones_m_BGrid_4__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0204);// 020C 01F4 awbb_LowBrGrZones_m_BGrid_5__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x033E);// 0346 02E1 awbb_LowBrGrZones_m_BGrid_5__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01E0);// 01E8 01D7 awbb_LowBrGrZones_m_BGrid_6__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02CD);// 02D5 028E awbb_LowBrGrZones_m_BGrid_6__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01C3);// 01CB 01CB awbb_LowBrGrZones_m_BGrid_7__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x027A);// 0282 0258 awbb_LowBrGrZones_m_BGrid_7__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01B7);// 01BF 022B awbb_LowBrGrZones_m_BGrid_8__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0244);// 024C 01CC awbb_LowBrGrZones_m_BGrid_8__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01FE);// 01F8 0000 awbb_LowBrGrZones_m_BGrid_9__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01DD);// 0201 0000 awbb_LowBrGrZones_m_BGrid_9__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 0000 awbb_LowBrGrZones_m_BGrid_10__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 0000 awbb_LowBrGrZones_m_BGrid_10__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 0000 awbb_LowBrGrZones_m_BGrid_11__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 0000 awbb_LowBrGrZones_m_BGrid_11__m_right */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0B70);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0005);// awbb_OutdoorGrZones_ZInfo_m_GridStep */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0B74);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01E3);// awbb_OutdoorGrZones_ZInfo_m_BMin */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0270);// awbb_OutdoorGrZones_ZInfo_m_BMax */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0B72);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0006);// awbb_OutdoorGrZones_ZInfo_m_GridSz */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0B40);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x028A);// 029E awbb_OutdoorGrZones_m_BGrid_0__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02A1);// 02C8 awbb_OutdoorGrZones_m_BGrid_0__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0263);// 0281 awbb_OutdoorGrZones_m_BGrid_1__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02C0);// 02C8 awbb_OutdoorGrZones_m_BGrid_1__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x024C);// 0266 awbb_OutdoorGrZones_m_BGrid_2__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02BE);// 02AC awbb_OutdoorGrZones_m_BGrid_2__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x023D);// 0251 awbb_OutdoorGrZones_m_BGrid_3__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02A6);// 028E awbb_OutdoorGrZones_m_BGrid_3__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0243);// 023D awbb_OutdoorGrZones_m_BGrid_4__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0289);// 0275 awbb_OutdoorGrZones_m_BGrid_4__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x026F);// 0228 awbb_OutdoorGrZones_m_BGrid_5__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x025D);// 025D awbb_OutdoorGrZones_m_BGrid_5__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0228 awbb_OutdoorGrZones_m_BGrid_6__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0243 awbb_OutdoorGrZones_m_BGrid_6__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 awbb_OutdoorGrZones_m_BGrid_7__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 awbb_OutdoorGrZones_m_BGrid_7__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 awbb_OutdoorGrZones_m_BGrid_8__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 awbb_OutdoorGrZones_m_BGrid_8__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 awbb_OutdoorGrZones_m_BGrid_9__m_left   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 awbb_OutdoorGrZones_m_BGrid_9__m_right  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 awbb_OutdoorGrZones_m_BGrid_10__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 awbb_OutdoorGrZones_m_BGrid_10__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 awbb_OutdoorGrZones_m_BGrid_11__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 awbb_OutdoorGrZones_m_BGrid_11__m_right */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0BC8);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0005);// awbb_CWSkinZone_ZInfo_m_GridStep */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0BCC);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x010F);// awbb_CWSkinZone_ZInfo_m_BMin */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x018F);// awbb_CWSkinZone_ZInfo_m_BMax */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0BCA);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0005);// awbb_CWSkinZone_ZInfo_m_GridSz */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0BB4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03E7);// awbb_CWSkinZone_m_BGrid_0__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03F8);// awbb_CWSkinZone_m_BGrid_0__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03A7);// awbb_CWSkinZone_m_BGrid_1__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03FC);// awbb_CWSkinZone_m_BGrid_1__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0352);// awbb_CWSkinZone_m_BGrid_2__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03D0);// awbb_CWSkinZone_m_BGrid_2__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0322);// awbb_CWSkinZone_m_BGrid_3__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x039E);// awbb_CWSkinZone_m_BGrid_3__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x032B);// awbb_CWSkinZone_m_BGrid_4__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x034D);// awbb_CWSkinZone_m_BGrid_4__m_right */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0BE6);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0006);// awbb_DLSkinZone_ZInfo_m_GridStep */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0BEA);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x019E);// awbb_DLSkinZone_ZInfo_m_BMin */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0257);// awbb_DLSkinZone_ZInfo_m_BMax */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0BE8);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0004);// awbb_DLSkinZone_ZInfo_m_GridSz */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0BD2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x030B);// awbb_DLSkinZone_m_BGrid_0__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0323);// awbb_DLSkinZone_m_BGrid_0__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02C3);// awbb_DLSkinZone_m_BGrid_1__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x030F);// awbb_DLSkinZone_m_BGrid_1__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0288);// awbb_DLSkinZone_m_BGrid_2__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02E5);// awbb_DLSkinZone_m_BGrid_2__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x026A);// awbb_DLSkinZone_m_BGrid_3__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02A2);// awbb_DLSkinZone_m_BGrid_3__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// awbb_DLSkinZone_m_BGrid_4__m_left  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// awbb_DLSkinZone_m_BGrid_4__m_right */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0C2C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0139);// awbb_IntcR */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0122);// awbb_IntcB */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0BFC);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0378);// 03AD awbb_IndoorWP_0__r */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x011E);// 013F awbb_IndoorWP_0__b */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02F0);// 0341 awbb_IndoorWP_1__r */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0184);// 017B awbb_IndoorWP_1__b */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0313);// 038D awbb_IndoorWP_2__r */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0158);// 014B awbb_IndoorWP_2__b */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02BA);// 02C3 awbb_IndoorWP_3__r */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01BA);// 01CC awbb_IndoorWP_3__b */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0231);// 0241 awbb_IndoorWP_4__r */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0252);// 027F awbb_IndoorWP_4__b */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0237);// 0241 awbb_IndoorWP_5__r */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x024C);// 027F awbb_IndoorWP_5__b */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x020F);// 0214 awbb_IndoorWP_6__r */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0279);// 02A8 awbb_IndoorWP_6__b */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0268);// 0270 255 awbb_OutdoorWP_r */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x021A);// 0210 25B awbb_OutdoorWP_b */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0C4C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0450);// awbb_MvEq_RBthresh */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0C58);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x059C);// awbb_MvEq_RBthresh */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0BF8);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01AE);// awbb_LowTSep_m_RminusB */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0C28);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// awbb_SkinPreference */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0CAC);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0050);// awbb_OutDMaxIncr */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0C28);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// awbb_SkinPreference */ 

	S5K8AAYX_write_cmos_sensor(0x002A, 0x20BA);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0006);// Lowtemp bypass */

	S5K8AAYX_write_cmos_sensor(0x002A, 0x0D0E);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00B8);// awbb_GridCoeff_R_2 */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00B2);// awbb_GridCoeff_B_2 */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0CFE);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0FAB);// 0FAB awbb_GridConst_2_0_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0FF5);// 0FF5 0FF5 awbb_GridConst_2_1_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x10BB);// 10BB 10BB awbb_GridConst_2_2_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1117);// 1117 1123 1153 awbb_GridConst_2_3_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x116D);// 116D 11C5 awbb_GridConst_2_4_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x11D5);// 122A awbb_GridConst_2_5_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00A9);// awbb_GridCoeff_R_1 */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00C0);// awbb_GridCoeff_B_1 */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0CF8);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02CC);// awbb_GridConst_1_0_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x031E);// awbb_GridConst_1_1_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0359);// awbb_GridConst_1_2_ */

	S5K8AAYX_write_cmos_sensor(0x002A, 0x0CB0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0030);// 0000 awbb_GridCorr_R_0__0_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0040);// 0000 awbb_GridCorr_R_0__1_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0060);// 0078 awbb_GridCorr_R_0__2_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0040);// 00AA awbb_GridCorr_R_0__3_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0008);// 0000 awbb_GridCorr_R_0__4_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0008);// 0000 awbb_GridCorr_R_0__5_ */

	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0030);// 0000 awbb_GridCorr_R_1__0_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0040);// 0096 awbb_GridCorr_R_1__1_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0060);// 0000 awbb_GridCorr_R_1__2_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0040);// 0000 awbb_GridCorr_R_1__3_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0008);// 0000 awbb_GridCorr_R_1__4_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0008);// 0000 awbb_GridCorr_R_1__5_ */

	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0030);// 00E6 awbb_GridCorr_R_2__0_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0040);// 0000 awbb_GridCorr_R_2__1_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0060);// 0000 awbb_GridCorr_R_2__2_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0040);// 0000 awbb_GridCorr_R_2__3_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0008);// 0000 awbb_GridCorr_R_2__4_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0008);// 0000 awbb_GridCorr_R_2__5_ */

	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0018);// 0000 awbb_GridCorr_B_0__0_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 awbb_GridCorr_B_0__1_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0064 awbb_GridCorr_B_0__2_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 awbb_GridCorr_B_0__3_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF80);// 0000 awbb_GridCorr_B_0__4_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFEC0);// 0000 awbb_GridCorr_B_0__5_ */

	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0018);// 0000 awbb_GridCorr_B_1__0_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0032 awbb_GridCorr_B_1__1_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 awbb_GridCorr_B_1__2_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 awbb_GridCorr_B_1__3_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF80);// FF38 awbb_GridCorr_B_1__4_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFEC0);// 0000 awbb_GridCorr_B_1__5_ */

	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0018);// 0000 awbb_GridCorr_B_2__0_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0032 awbb_GridCorr_B_2__1_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 awbb_GridCorr_B_2__2_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 awbb_GridCorr_B_2__3_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF80);// 0000 awbb_GridCorr_B_2__4_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFEC0);// 0000 awbb_GridCorr_B_2__5_ */

	S5K8AAYX_write_cmos_sensor(0x002A, 0x0D30);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0002);// awbb_GridEnable */

	S5K8AAYX_write_cmos_sensor(0x002A, 0x3362);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);// awbb_bUseOutdoorGrid */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// awbb_OutdoorGridCorr_R */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// awbb_OutdoorGridCorr_B */

	// For Outdoor Detector */               
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0C86);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0005);// awbb_OutdoorDetectionZone_ZInfo_m_GridSz */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0C70);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF7B);// awbb_OutdoorDetectionZone_m_BGrid_0__m_left */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00CE);// awbb_OutdoorDetectionZone_m_BGrid_0__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF23);// awbb_OutdoorDetectionZone_m_BGrid_1__m_left */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x010D);// awbb_OutdoorDetectionZone_m_BGrid_1__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFEF3);// awbb_OutdoorDetectionZone_m_BGrid_2__m_left */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x012C);// awbb_OutdoorDetectionZone_m_BGrid_2__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFED7);// awbb_OutdoorDetectionZone_m_BGrid_3__m_left */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x014E);// awbb_OutdoorDetectionZone_m_BGrid_3__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFEBB);// awbb_OutdoorDetectionZone_m_BGrid_4__m_left */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0162);// awbb_OutdoorDetectionZone_m_BGrid_4__m_right */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1388);// awbb_OutdoorDetectionZone_ZInfo_m_AbsGridStep */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0C8A);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4ACB);// awbb_OutdoorDetectionZone_ZInfo_m_MaxNB */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0C88);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0A7C);// awbb_OutdoorDetectionZone_ZInfo_m_NBoffs */

	S5K8AAYX_write_cmos_sensor(0x002A, 0x0CA0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0030); //awbb_GnCurPntImmunity

	S5K8AAYX_write_cmos_sensor(0x002A, 0x0CA4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0030); //awbb_GnCurPntLongJump
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0180); //awbb_GainsMaxMove
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0002); //awbb_GnMinMatchToJump

	//==================================================================================
	// 10.Clock Setting                      
	//==================================================================================
	// Input Clock (Mclk)                    
	S5K8AAYX_write_cmos_sensor(0x002A, 0x012E);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x5DC0);	// REG_TC_IPRM_InClockLSBs 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// REG_TC_IPRM_InClockMSBs 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0146);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);	// REG_TC_IPRM_UseNPviClocks 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// REG_TC_IPRM_UseNMipiClocks

	// System Clock & Output clock (Pclk)    
	S5K8AAYX_write_cmos_sensor(0x002A, 0x014C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2AF8);	// REG_TC_IPRM_OpClk4KHz_0 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0152);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x55F0);	// REG_TC_IPRM_MinOutRate4KHz_0 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x014E);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x55F0);	// REG_TC_IPRM_MaxOutRate4KHz_0 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00FA);

	S5K8AAYX_write_cmos_sensor(0x002A, 0x0164);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);	// REG_TC_IPRM_InitParamsUpdated 

	//*************************************/ 
	// 11.Auto Flicker Detection          */ 
	//*************************************/ 
	S5K8AAYX_write_cmos_sensor(0x0028, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0408);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x067F); //REG_TC_DBG_AutoAlgEnBits //all AA are on
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0ABC);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000); //AFC_Default60Hz //0000: 50Hz, 0001:60Hz

	//*************************************/ 
	// 12.AE Setting                      */ 
	//*************************************/ 

	S5K8AAYX_write_cmos_sensor(0x002A, 0x0D40);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0036); //TVAR_ae_BrAve */

	// For LT Calibration */                 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0D46);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000F);// ae_StatMode */

	S5K8AAYX_write_cmos_sensor(0x002A, 0x0440);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3410);// lt_uMaxExp_0_ */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0444);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x6820);// lt_uMaxExp_1_ */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0448);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x8227);// lt_uMaxExp_2_ */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x044C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xC350);// lt_uMaxExp_3_ */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0450);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3410);// lt_uCapMaxExp_0_ */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0454);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x6820);// lt_uCapMaxExp_1_ */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0458);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x8227);// lt_uCapMaxExp_2_ */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x045C);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xC350);// lt_uCapMaxExp_3_ */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0460);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01B0);// lt_uMaxAnGain_0_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01B0);// lt_uMaxAnGain_1_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0280);// lt_uMaxAnGain_2_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0600);// lt_uMaxAnGain_3_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);// B0 0100 lt_uMaxDigGain */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3000);// lt_uMaxTotGain */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x042E);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x012E);// lt_uLimitHigh */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00D5);// lt_uLimitLow */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0DE0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0002);// ae_Fade2BlackEnable F2B off, F2W on */

	// For Illum Type Calibration */         
	// WRITE #SARR_IllumType_0_  0078 */     
	// WRITE #SARR_IllumType_1_  00C3 */     
	// WRITE #SARR_IllumType_2_  00E9 */     
	// WRITE #SARR_IllumType_3_  0128 */     
	// WRITE #SARR_IllumType_4_  016F */     
	// WRITE #SARR_IllumType_5_  0195 */     
	// WRITE #SARR_IllumType_6_  01A4 */     
	// WRITE #SARR_IllumTypeF_0_  0100 */    
	// WRITE #SARR_IllumTypeF_1_  0100 */    
	// WRITE #SARR_IllumTypeF_2_  0110 */    
	// WRITE #SARR_IllumTypeF_3_  00E5 */    
	// WRITE #SARR_IllumTypeF_4_  0100 */    
	// WRITE #SARR_IllumTypeF_5_  00ED */    
	// WRITE #SARR_IllumTypeF_6_  00ED */    

	S5K8AAYX_write_cmos_sensor(0x002A, 0x0D38);// bp_uMaxBrightnessFactor */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x019C);
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0D3E);// bp_uMinBrightnessFactor */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02A0);

	//*********************************************************************************
	// 13.AE Weight (Normal)                                                  
	//*********************************************************************************
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0D4E);//0D4E);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);//0000);//0000);//0100); //ae_WeightTbl_16_0_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);//0000);//0101);//0001); //ae_WeightTbl_16_1_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);//0000);//0101);//0100); //ae_WeightTbl_16_2_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);//0000);//0000);//0001); //ae_WeightTbl_16_3_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0101);//0000);//0101);//0101); //ae_WeightTbl_16_4_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0101);//0000);//0101);//0101); //ae_WeightTbl_16_5_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0101);//0000);//0101);//0101); //ae_WeightTbl_16_6_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0101);//0000);//0101);//0101); //ae_WeightTbl_16_7_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0101);//0000);//0201);//0101); //ae_WeightTbl_16_8_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0201);//0101);//0303);//0201); //ae_WeightTbl_16_9_  
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0202);//0101);//0303);//0202); //ae_WeightTbl_16_10_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0101);//0000);//0102);//0101); //ae_WeightTbl_16_11_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0101);//0000);//0201);//0101); //ae_WeightTbl_16_12_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0303);//0F01);//0403);//0303); //ae_WeightTbl_16_13_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0303);//010F);//0304);//0303); //ae_WeightTbl_16_14_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0101);//0000);//0102);//0101); //ae_WeightTbl_16_15_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0101);//0000);//0201);//0101); //ae_WeightTbl_16_16_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0503);//0F01);//0403);//0503); //ae_WeightTbl_16_17_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0305);//010F);//0304);//0305); //ae_WeightTbl_16_18_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0101);//0000);//0102);//0101); //ae_WeightTbl_16_19_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0101);//0000);//0201);//0101); //ae_WeightTbl_16_20_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0402);//0101);//0403);//0402); //ae_WeightTbl_16_21_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0204);//0101);//0304);//0204); //ae_WeightTbl_16_22_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0101);//0000);//0102);//0101); //ae_WeightTbl_16_23_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0101);//0000);//0201);//0101); //ae_WeightTbl_16_24_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0201);//0000);//0303);//0201); //ae_WeightTbl_16_25_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0102);//0000);//0303);//0102); //ae_WeightTbl_16_26_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0101);//0000);//0102);//0101); //ae_WeightTbl_16_27_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100);//0000);//0201);//0100); //ae_WeightTbl_16_28_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0101);//0000);//0202);//0101); //ae_WeightTbl_16_29_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0101);//0000);//0202);//0101); //ae_WeightTbl_16_30_ 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);//0000);//0102);//0001); //ae_WeightTbl_16_31_ 
                                             
	//*************************************/ 
	// 14.Flash Setting                   */ 
	//*************************************/ 

	//*************************************/ 
	// 15.CCM Setting                     */ 
	//*************************************/ 

	S5K8AAYX_write_cmos_sensor(0x002A, 0x33A4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01D0);//01D0// Tune_wbt_BaseCcms_0__0_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFA1);//FFA1// Tune_wbt_BaseCcms_0__1_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFFA);//FFFA// Tune_wbt_BaseCcms_0__2_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF6F);//FF6F// Tune_wbt_BaseCcms_0__3_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0140);//0140// Tune_wbt_BaseCcms_0__4_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF49);//FF49// Tune_wbt_BaseCcms_0__5_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFC1);//FFC1// Tune_wbt_BaseCcms_0__6_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001F);//001F// Tune_wbt_BaseCcms_0__7_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01BD);//01BD// Tune_wbt_BaseCcms_0__8_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x013F);//013F// Tune_wbt_BaseCcms_0__9_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00E1);//00E1// Tune_wbt_BaseCcms_0__10_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF43);//FF43// Tune_wbt_BaseCcms_0__11_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0191);//0191// Tune_wbt_BaseCcms_0__12_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFC0);//FFC0// Tune_wbt_BaseCcms_0__13_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01B7);//01B7// Tune_wbt_BaseCcms_0__14_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF30);//FF30// Tune_wbt_BaseCcms_0__15_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x015F);//015F// Tune_wbt_BaseCcms_0__16_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0106);//0106// Tune_wbt_BaseCcms_0__17_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01D0);//01D0// Tune_wbt_BaseCcms_1__0_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFA1);//FFA1// Tune_wbt_BaseCcms_1__1_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFFA);//FFFA// Tune_wbt_BaseCcms_1__2_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF6F);//FF6F// Tune_wbt_BaseCcms_1__3_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0140);//0140// Tune_wbt_BaseCcms_1__4_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF49);//FF49// Tune_wbt_BaseCcms_1__5_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFC1);//FFC1// Tune_wbt_BaseCcms_1__6_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001F);//001F// Tune_wbt_BaseCcms_1__7_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01BD);//01BD// Tune_wbt_BaseCcms_1__8_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x013F);//013F// Tune_wbt_BaseCcms_1__9_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00E1);//00E1// Tune_wbt_BaseCcms_1__10_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF43);//FF43// Tune_wbt_BaseCcms_1__11_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0191);//0191// Tune_wbt_BaseCcms_1__12_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFC0);//FFC0// Tune_wbt_BaseCcms_1__13_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01B7);//01B7// Tune_wbt_BaseCcms_1__14_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF30);//FF30// Tune_wbt_BaseCcms_1__15_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x015F);//015F// Tune_wbt_BaseCcms_1__16_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0106);//0106// Tune_wbt_BaseCcms_1__17_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01C4);//01C4// 01F3 01C2 01D0 Tune_wbt_BaseCcms_2__0_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFAB);//FFAB// FFCB FF93 FFA1 Tune_wbt_BaseCcms_2__1_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFFC);//FFFC// 0031 001C FFFA Tune_wbt_BaseCcms_2__2_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF6E);//FF6E// FF6F Tune_wbt_BaseCcms_2__3_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0145);//0145// 0140 Tune_wbt_BaseCcms_2__4_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF4A);//FF4A// FF49 Tune_wbt_BaseCcms_2__5_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFE1);//FFE1// FFE3 FFC1 Tune_wbt_BaseCcms_2__6_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFF6);//FFF6// FFF9 001F Tune_wbt_BaseCcms_2__7_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01BD);//01BD// 01C1 01BD Tune_wbt_BaseCcms_2__8_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x013E);//013E// 013F Tune_wbt_BaseCcms_2__9_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00E4);//00E4// 00E1 Tune_wbt_BaseCcms_2__10_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF46);//FF46// FF43 Tune_wbt_BaseCcms_2__11_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0190);//0190// 0191 Tune_wbt_BaseCcms_2__12_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFBC);//FFBC// FFC0 Tune_wbt_BaseCcms_2__13_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01B5);//01B5// 01B7 Tune_wbt_BaseCcms_2__14_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF30);//FF30// FF30 Tune_wbt_BaseCcms_2__15_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x015E);//015E// 015F Tune_wbt_BaseCcms_2__16_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0103);//0103// 0106 Tune_wbt_BaseCcms_2__17_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0187);//01C4);//01C4// 01D0 Tune_wbt_BaseCcms_3__0_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFD5);//FFAB);//FFAB// FFA1 Tune_wbt_BaseCcms_3__1_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0014);//FFFC);//FFFC// FFFA Tune_wbt_BaseCcms_3__2_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF81);//FF6E);//FF6E// FF6F Tune_wbt_BaseCcms_3__3_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0138);//0145);//0145// 0140 Tune_wbt_BaseCcms_3__4_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF62);//FF4A);//FF4A// FF49 Tune_wbt_BaseCcms_3__5_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFF2);//FFE1);//FFE1// FFE3 Tune_wbt_BaseCcms_3__6_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFFB);//FFF6);//FFF6// FFF9 Tune_wbt_BaseCcms_3__7_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0190);//01BD);//01BD// 01C1 Tune_wbt_BaseCcms_3__8_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x011B);//013E);//013E// 013F Tune_wbt_BaseCcms_3__9_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00E1);//00E4);//00E4// 00E1 Tune_wbt_BaseCcms_3__10_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF89);//FF46);//FF46// FF43 Tune_wbt_BaseCcms_3__11_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x017C);//0190);//0190// 0191 Tune_wbt_BaseCcms_3__12_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFD3);//FFBC);//FFBC// FFC0 Tune_wbt_BaseCcms_3__13_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0196);//01B5);//01B5// 01B7 Tune_wbt_BaseCcms_3__14_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF5C);//FF30);//FF30// FF30 Tune_wbt_BaseCcms_3__15_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0143);//015E);//015E// 015F Tune_wbt_BaseCcms_3__16_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00E9);//0103);//0103// 0106 Tune_wbt_BaseCcms_3__17_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0157);//01C6);//01C6// 01BF 01E9 01C7 01BF 01C7 01BF Tune_wbt_BaseCcms_4__0_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFF7);//FFC2);//FFC2// FFBF FFE4 FFA5 FFBF FFA5 FFBF Tune_wbt_BaseCcms_4__1_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0028);//0004);//0004// FFFE 0031 0006 FFFE 0006 FFFE Tune_wbt_BaseCcms_4__2_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF8E);//FF6F);//FF6F// FF6D FF6F FF6D Tune_wbt_BaseCcms_4__3_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0137);//01C9);//01C9// 01B4 01C9 01B4 Tune_wbt_BaseCcms_4__4_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF72);//FF4F);//FF4F// FF66 FF4F FF66 Tune_wbt_BaseCcms_4__5_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFFC);//FFDB);//FFDB// FFCA FFDB FFCA Tune_wbt_BaseCcms_4__6_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFFB);//FFC0);//FFC0// FFCE FFC0 FFCE Tune_wbt_BaseCcms_4__7_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0171);//019D);//019D// 017B 019D 017B Tune_wbt_BaseCcms_4__8_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0104);//0136);//0136// Tune_wbt_BaseCcms_4__9_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00E1);//0132);//0132// Tune_wbt_BaseCcms_4__10_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFBB);//FF85);//FF85// Tune_wbt_BaseCcms_4__11_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x016F);//018B);//018B// Tune_wbt_BaseCcms_4__12_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFDD);//FF73);//FF73// Tune_wbt_BaseCcms_4__13_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0180);//0191);//0191// Tune_wbt_BaseCcms_4__14_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF78);//FF3F);//FF3F// Tune_wbt_BaseCcms_4__15_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0131);//015B);//015B// Tune_wbt_BaseCcms_4__16_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00D6);//00D0);//00D0// Tune_wbt_BaseCcms_4__17_ */

	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0191);//01C6);//01C6// 01BF 01E9 01C7 01BF 01C7 01BF Tune_wbt_BaseCcms_5__0_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFE2);//FFC2);//FFC2// FFBF FFE4 FFA5 FFBF FFA5 FFBF Tune_wbt_BaseCcms_5__1_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0019);//0004);//0004// FFFE 0031 0006 FFFE 0006 FFFE Tune_wbt_BaseCcms_5__2_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF68);//FF56);//FF6F// FF6D FF6F FF6D Tune_wbt_BaseCcms_5__3_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01A5);//01C9);//01C9// 01B4 01C9 01B4 Tune_wbt_BaseCcms_5__4_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF78);//FF68);//FF4F// FF66 FF4F FF66 Tune_wbt_BaseCcms_5__5_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFE7);//FFDB);//FFDB// FFCA FFDB FFCA FFE7 FFCA Tune_wbt_BaseCcms_5__6_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFCE);//FFC0);//FFC0// FFCE FFC0 FFCE FFC2 FFCE Tune_wbt_BaseCcms_5__7_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0182);//019D);//019D// 017B 019D 017B 018C 017B Tune_wbt_BaseCcms_5__8_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0126);//0140);//0136// Tune_wbt_BaseCcms_5__9_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0116);//012C);//0132// Tune_wbt_BaseCcms_5__10_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFC5);//FF97);//FF85// Tune_wbt_BaseCcms_5__11_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x016A);//018B);//018B// Tune_wbt_BaseCcms_5__12_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFB5);//FF73);//FF73// Tune_wbt_BaseCcms_5__13_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x016F);//0191);//0191// Tune_wbt_BaseCcms_5__14_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF5E);//FF3F);//FF3F// Tune_wbt_BaseCcms_5__15_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0145);//015B);//015B// Tune_wbt_BaseCcms_5__16_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00C7);//00D0);//00D0// Tune_wbt_BaseCcms_5__17_ */

	S5K8AAYX_write_cmos_sensor(0x002A, 0x3380);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01D6); //023C// 021B 021B 023D 01E9 01C7 01BF 01AC Tune_wbt_OutdoorCcm_0_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF94); //FFE0// FFE9 0010 0038 FFE4 FFA5 FFBF FFD7 Tune_wbt_OutdoorCcm_1_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFCC); //0052// 0050 0073 0085 0031 0006 FFFE 0019 Tune_wbt_OutdoorCcm_2_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF1F); //FF6F// FF6D Tune_wbt_OutdoorCcm_3_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x021F); //01C9// 01B4 Tune_wbt_OutdoorCcm_4_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF1F); //FF4F// FF66 Tune_wbt_OutdoorCcm_5_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFE4); //0000// FFE9 FFDB FFD5 FFD5 FFE7 FFCA Tune_wbt_OutdoorCcm_6_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFED); //FFA4// FFBA FFC0 FFD6 FFD6 FFC2 FFCE Tune_wbt_OutdoorCcm_7_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01C8); //01A5// 01AC 019D 018D 018D 018C 017B Tune_wbt_OutdoorCcm_8_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0112); //0125// 0132 Tune_wbt_OutdoorCcm_9_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0125); //013B// 012E Tune_wbt_OutdoorCcm_10_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF3E); //FF8D// FF8D Tune_wbt_OutdoorCcm_11_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0210); //018B// Tune_wbt_OutdoorCcm_12_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF5D); //FF73// Tune_wbt_OutdoorCcm_13_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0244); //0191// Tune_wbt_OutdoorCcm_14_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF10); //FF3F// Tune_wbt_OutdoorCcm_15_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0190); //015B// Tune_wbt_OutdoorCcm_16_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0145); //00D0// Tune_wbt_OutdoorCcm_17_  */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0612);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x009D);// SARR_AwbCcmCord_0_       */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00D5);// SARR_AwbCcmCord_1_       */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0103);// SARR_AwbCcmCord_2_       */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0128);// SARR_AwbCcmCord_3_       */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0166);// SARR_AwbCcmCord_4_       */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0193);// SARR_AwbCcmCord_5_       */


	//*************************************/ 
	// 17.GAMMA                           */ 
	//*************************************/ 
	// For Pre Post Gamma Calibration */     
	S5K8AAYX_write_cmos_sensor(0x002A, 0x0538);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// seti_uGammaLutPreDemNoBin_0_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001F);// seti_uGammaLutPreDemNoBin_1_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0035);// seti_uGammaLutPreDemNoBin_2_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x005A);// seti_uGammaLutPreDemNoBin_3_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0095);// seti_uGammaLutPreDemNoBin_4_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00E6);// seti_uGammaLutPreDemNoBin_5_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0121);// seti_uGammaLutPreDemNoBin_6_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0139);// seti_uGammaLutPreDemNoBin_7_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0150);// seti_uGammaLutPreDemNoBin_8_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0177);// seti_uGammaLutPreDemNoBin_9_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x019A);// seti_uGammaLutPreDemNoBin_10_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01BB);// seti_uGammaLutPreDemNoBin_11_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01DC);// seti_uGammaLutPreDemNoBin_12_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0219);// seti_uGammaLutPreDemNoBin_13_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0251);// seti_uGammaLutPreDemNoBin_14_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02B3);// seti_uGammaLutPreDemNoBin_15_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x030A);// seti_uGammaLutPreDemNoBin_16_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x035F);// seti_uGammaLutPreDemNoBin_17_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03B1);// seti_uGammaLutPreDemNoBin_18_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03FF);// seti_uGammaLutPreDemNoBin_19_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// seti_uGammaLutPostDemNoBin_0_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);// seti_uGammaLutPostDemNoBin_1_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);// seti_uGammaLutPostDemNoBin_2_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0002);// seti_uGammaLutPostDemNoBin_3_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0004);// seti_uGammaLutPostDemNoBin_4_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000A);// seti_uGammaLutPostDemNoBin_5_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0012);// seti_uGammaLutPostDemNoBin_6_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0016);// seti_uGammaLutPostDemNoBin_7_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001A);// seti_uGammaLutPostDemNoBin_8_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0024);// seti_uGammaLutPostDemNoBin_9_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0031);// seti_uGammaLutPostDemNoBin_10_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003E);// seti_uGammaLutPostDemNoBin_11_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x004E);// seti_uGammaLutPostDemNoBin_12_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0075);// seti_uGammaLutPostDemNoBin_13_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00A8);// seti_uGammaLutPostDemNoBin_14_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0126);// seti_uGammaLutPostDemNoBin_15_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01BE);// seti_uGammaLutPostDemNoBin_16_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0272);// seti_uGammaLutPostDemNoBin_17_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0334);// seti_uGammaLutPostDemNoBin_18_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03FF);// seti_uGammaLutPostDemNoBin_19_ */

	// For Gamma Calibration */              

	S5K8AAYX_write_cmos_sensor(0x002A, 0x0498);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// SARR_usDualGammaLutRGBIndoor_0__0_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0002);// SARR_usDualGammaLutRGBIndoor_0__1_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0007);// SARR_usDualGammaLutRGBIndoor_0__2_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001D);// SARR_usDualGammaLutRGBIndoor_0__3_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x006E);// SARR_usDualGammaLutRGBIndoor_0__4_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00D3);// SARR_usDualGammaLutRGBIndoor_0__5_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0127);// SARR_usDualGammaLutRGBIndoor_0__6_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x014C);// SARR_usDualGammaLutRGBIndoor_0__7_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x016E);// SARR_usDualGammaLutRGBIndoor_0__8_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01A5);// SARR_usDualGammaLutRGBIndoor_0__9_   */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01D3);// SARR_usDualGammaLutRGBIndoor_0__10_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01FB);// SARR_usDualGammaLutRGBIndoor_0__11_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x021F);// SARR_usDualGammaLutRGBIndoor_0__12_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0260);// SARR_usDualGammaLutRGBIndoor_0__13_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x029A);// SARR_usDualGammaLutRGBIndoor_0__14_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02F7);// SARR_usDualGammaLutRGBIndoor_0__15_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x034D);// SARR_usDualGammaLutRGBIndoor_0__16_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0395);// SARR_usDualGammaLutRGBIndoor_0__17_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03CE);// SARR_usDualGammaLutRGBIndoor_0__18_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03FF);// SARR_usDualGammaLutRGBIndoor_0__19_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// SARR_usDualGammaLutRGBOutdoor_0__0_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0004);// SARR_usDualGammaLutRGBOutdoor_0__1_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000C);// SARR_usDualGammaLutRGBOutdoor_0__2_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0024);// SARR_usDualGammaLutRGBOutdoor_0__3_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x006E);// SARR_usDualGammaLutRGBOutdoor_0__4_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00D1);// SARR_usDualGammaLutRGBOutdoor_0__5_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0119);// SARR_usDualGammaLutRGBOutdoor_0__6_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0139);// SARR_usDualGammaLutRGBOutdoor_0__7_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0157);// SARR_usDualGammaLutRGBOutdoor_0__8_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x018E);// SARR_usDualGammaLutRGBOutdoor_0__9_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01C3);// SARR_usDualGammaLutRGBOutdoor_0__10_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01F3);// SARR_usDualGammaLutRGBOutdoor_0__11_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x021F);// SARR_usDualGammaLutRGBOutdoor_0__12_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0269);// SARR_usDualGammaLutRGBOutdoor_0__13_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02A6);// SARR_usDualGammaLutRGBOutdoor_0__14_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x02FF);// SARR_usDualGammaLutRGBOutdoor_0__15_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0351);// SARR_usDualGammaLutRGBOutdoor_0__16_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0395);// SARR_usDualGammaLutRGBOutdoor_0__17_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03CE);// SARR_usDualGammaLutRGBOutdoor_0__18_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03FF);// SARR_usDualGammaLutRGBOutdoor_0__19_ */

	//*************************************/ 
	// 16.AFIT                            */ 
	//*************************************/ 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x3360);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// afit_bUseNormBrForAfit */

	S5K8AAYX_write_cmos_sensor(0x002A, 0x06D4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0032);// afit_uNoiseIndInDoor_0_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0078);// afit_uNoiseIndInDoor_1_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00C8);// afit_uNoiseIndInDoor_2_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0190);// afit_uNoiseIndInDoor_3_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x028C);// afit_uNoiseIndInDoor_4_ */

	S5K8AAYX_write_cmos_sensor(0x002A, 0x0734);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_0__0_  Brightness[0] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_0__1_  Contrast[0] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_0__2_  Saturation[0] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_0__3_  Sharp_Blur[0] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_0__4_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0078);// AfitBaseVals_0__5_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x012C);// AfitBaseVals_0__6_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03FF);// AfitBaseVals_0__7_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0014);// AfitBaseVals_0__8_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0064);// AfitBaseVals_0__9_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000C);// AfitBaseVals_0__10_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0010);// AfitBaseVals_0__11_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01E6);// AfitBaseVals_0__12_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_0__13_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0070);// AfitBaseVals_0__14_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01FF);// AfitBaseVals_0__15_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0144);// AfitBaseVals_0__16_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000F);// AfitBaseVals_0__17_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000A);// AfitBaseVals_0__18_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0073);// AfitBaseVals_0__19_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0087);// AfitBaseVals_0__20_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0014);// AfitBaseVals_0__21_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000A);// AfitBaseVals_0__22_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0023);// AfitBaseVals_0__23_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001E);// AfitBaseVals_0__24_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0014);// AfitBaseVals_0__25_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000A);// AfitBaseVals_0__26_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0023);// AfitBaseVals_0__27_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0046);// AfitBaseVals_0__28_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2B32);// AfitBaseVals_0__29_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0601);// AfitBaseVals_0__30_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_0__31_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_0__32_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_0__33_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00FF);// AfitBaseVals_0__34_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x07FF);// AfitBaseVals_0__35_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFFF);// AfitBaseVals_0__36_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_0__37_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x050D);// AfitBaseVals_0__38_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1E80);// AfitBaseVals_0__39_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_0__40_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1408);// 1408 AfitBaseVals_0__41_ iNearGrayDesat[0] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0214);// AfitBaseVals_0__42_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF01);// AfitBaseVals_0__43_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x180F);// AfitBaseVals_0__44_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);// AfitBaseVals_0__45_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_0__46_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x8003);// AfitBaseVals_0__47_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0094);// AfitBaseVals_0__48_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0580);// AfitBaseVals_0__49_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0280);// AfitBaseVals_0__50_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0308);// AfitBaseVals_0__51_ iClustThresh_H[0] iClustMulT_H[0] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3186);// AfitBaseVals_0__52_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x707E);// AfitBaseVals_0__53_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0A02);// AfitBaseVals_0__54_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x080A);// AfitBaseVals_0__55_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0500);// AfitBaseVals_0__56_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x032D);// AfitBaseVals_0__57_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x324E);// AfitBaseVals_0__58_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001E);// AfitBaseVals_0__59_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0200);// AfitBaseVals_0__60_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0103);// AfitBaseVals_0__61_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x010C);// AfitBaseVals_0__62_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x9696);// AfitBaseVals_0__63_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4646);// AfitBaseVals_0__64_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0802);// AfitBaseVals_0__65_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0802);// AfitBaseVals_0__66_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_0__67_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x030F);// AfitBaseVals_0__68_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3202);// AfitBaseVals_0__69_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0F1E);// AfitBaseVals_0__70_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x020F);// AfitBaseVals_0__71_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0103);// AfitBaseVals_0__72_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x010C);// AfitBaseVals_0__73_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x9696);// AfitBaseVals_0__74_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x4646);// AfitBaseVals_0__75_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0802);// AfitBaseVals_0__76_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0802);// AfitBaseVals_0__77_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_0__78_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x030F);// AfitBaseVals_0__79_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3202);// AfitBaseVals_0__80_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0F1E);// AfitBaseVals_0__81_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x020F);// AfitBaseVals_0__82_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0003);// AfitBaseVals_0__83_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_1__0_  Brightness[1] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_1__1_  Contrast[1] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_1__2_  Saturation[1] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_1__3_  Sharp_Blur[1] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_1__4_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x006A);// AfitBaseVals_1__5_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x012C);// AfitBaseVals_1__6_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03FF);// AfitBaseVals_1__7_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0014);// AfitBaseVals_1__8_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0064);// AfitBaseVals_1__9_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000C);// AfitBaseVals_1__10_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0010);// AfitBaseVals_1__11_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01E6);// AfitBaseVals_1__12_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03FF);// AfitBaseVals_1__13_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0070);// AfitBaseVals_1__14_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x007D);// AfitBaseVals_1__15_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0064);// AfitBaseVals_1__16_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0014);// AfitBaseVals_1__17_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000A);// AfitBaseVals_1__18_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0073);// AfitBaseVals_1__19_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0087);// AfitBaseVals_1__20_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0014);// AfitBaseVals_1__21_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000A);// AfitBaseVals_1__22_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0023);// AfitBaseVals_1__23_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001E);// AfitBaseVals_1__24_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0014);// AfitBaseVals_1__25_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000A);// AfitBaseVals_1__26_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0023);// AfitBaseVals_1__27_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001E);// AfitBaseVals_1__28_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2B32);// AfitBaseVals_1__29_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0601);// AfitBaseVals_1__30_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_1__31_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_1__32_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_1__33_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00FF);// AfitBaseVals_1__34_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x07FF);// AfitBaseVals_1__35_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFFF);// AfitBaseVals_1__36_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_1__37_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x050D);// AfitBaseVals_1__38_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1E80);// AfitBaseVals_1__39_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_1__40_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1403);// 1408 AfitBaseVals_1__41_ iNearGrayDesat[1] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0214);// AfitBaseVals_1__42_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF01);// AfitBaseVals_1__43_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x180F);// AfitBaseVals_1__44_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0002);// AfitBaseVals_1__45_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_1__46_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x8003);// AfitBaseVals_1__47_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0080);// AfitBaseVals_1__48_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0080);// AfitBaseVals_1__49_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0280);// AfitBaseVals_1__50_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0308);// AfitBaseVals_1__51_ iClustThresh_H[1] iClustMulT_H[1] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1E65);// AfitBaseVals_1__52_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1A24);// AfitBaseVals_1__53_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0A03);// AfitBaseVals_1__54_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x080A);// AfitBaseVals_1__55_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0500);// AfitBaseVals_1__56_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x032D);// AfitBaseVals_1__57_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x324D);// AfitBaseVals_1__58_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001E);// AfitBaseVals_1__59_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0200);// AfitBaseVals_1__60_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0103);// AfitBaseVals_1__61_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x010C);// AfitBaseVals_1__62_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x9696);// AfitBaseVals_1__63_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2F34);// AfitBaseVals_1__64_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0504);// AfitBaseVals_1__65_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x080F);// AfitBaseVals_1__66_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_1__67_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x030F);// AfitBaseVals_1__68_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3208);// AfitBaseVals_1__69_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0F1E);// AfitBaseVals_1__70_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x020F);// AfitBaseVals_1__71_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0103);// AfitBaseVals_1__72_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x010C);// AfitBaseVals_1__73_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x9696);// AfitBaseVals_1__74_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1414);// AfitBaseVals_1__75_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0504);// AfitBaseVals_1__76_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x080F);// AfitBaseVals_1__77_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_1__78_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x030F);// AfitBaseVals_1__79_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3208);// AfitBaseVals_1__80_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0F1E);// AfitBaseVals_1__81_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x020F);// AfitBaseVals_1__82_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0003);// AfitBaseVals_1__83_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_2__0_  Brightness[2] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_2__1_  Contrast[2] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFFC);// 0000 AfitBaseVals_2__2_  Saturation[2] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_2__3_  Sharp_Blur[2] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_2__4_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0064);// AfitBaseVals_2__5_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x012C);// AfitBaseVals_2__6_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03FF);// AfitBaseVals_2__7_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0014);// AfitBaseVals_2__8_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0064);// AfitBaseVals_2__9_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000C);// AfitBaseVals_2__10_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0010);// AfitBaseVals_2__11_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01E6);// AfitBaseVals_2__12_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03FF);// AfitBaseVals_2__13_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0070);// AfitBaseVals_2__14_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x007D);// AfitBaseVals_2__15_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0064);// AfitBaseVals_2__16_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0032);// 0096 AfitBaseVals_2__17_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003C);// AfitBaseVals_2__18_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0073);// AfitBaseVals_2__19_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0087);// AfitBaseVals_2__20_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0014);// AfitBaseVals_2__21_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0019);// AfitBaseVals_2__22_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0023);// AfitBaseVals_2__23_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001E);// AfitBaseVals_2__24_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0014);// AfitBaseVals_2__25_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0019);// AfitBaseVals_2__26_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0023);// AfitBaseVals_2__27_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001E);// AfitBaseVals_2__28_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2B32);// AfitBaseVals_2__29_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0601);// AfitBaseVals_2__30_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_2__31_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_2__32_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_2__33_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00FF);// AfitBaseVals_2__34_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x07FF);// AfitBaseVals_2__35_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFFF);// AfitBaseVals_2__36_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_2__37_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x050D);// AfitBaseVals_2__38_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1E80);// AfitBaseVals_2__39_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_2__40_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0A03);// 0A08 AfitBaseVals_2__41_ iNearGrayDesat[2] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0200);// AfitBaseVals_2__42_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF01);// AfitBaseVals_2__43_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x180F);// AfitBaseVals_2__44_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0002);// AfitBaseVals_2__45_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_2__46_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x8003);// AfitBaseVals_2__47_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0080);// AfitBaseVals_2__48_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0080);// AfitBaseVals_2__49_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0280);// AfitBaseVals_2__50_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0208);// AfitBaseVals_2__51_ iClustThresh_H[2] iClustMulT_H[2] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1E4B);// AfitBaseVals_2__52_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1A24);// AfitBaseVals_2__53_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0A05);// AfitBaseVals_2__54_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x080A);// AfitBaseVals_2__55_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0500);// AfitBaseVals_2__56_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x032D);// AfitBaseVals_2__57_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x324D);// AfitBaseVals_2__58_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001E);// AfitBaseVals_2__59_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0200);// AfitBaseVals_2__60_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0103);// AfitBaseVals_2__61_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x010C);// AfitBaseVals_2__62_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x9696);// AfitBaseVals_2__63_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1E23);// AfitBaseVals_2__64_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0505);// AfitBaseVals_2__65_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x080F);// AfitBaseVals_2__66_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_2__67_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x030F);// AfitBaseVals_2__68_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3208);// AfitBaseVals_2__69_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0F1E);// AfitBaseVals_2__70_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x020F);// AfitBaseVals_2__71_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0103);// AfitBaseVals_2__72_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x010C);// AfitBaseVals_2__73_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x9696);// AfitBaseVals_2__74_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1E23);// AfitBaseVals_2__75_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0505);// AfitBaseVals_2__76_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x080F);// AfitBaseVals_2__77_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_2__78_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x030F);// AfitBaseVals_2__79_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3208);// AfitBaseVals_2__80_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0F1E);// AfitBaseVals_2__81_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x020F);// AfitBaseVals_2__82_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0003);// AfitBaseVals_2__83_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_3__0_  Brightness[3] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 AfitBaseVals_3__1_  Contrast[3] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFFA);// AfitBaseVals_3__2_  Saturation[3] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_3__3_  Sharp_Blur[3] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_3__4_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0064);// AfitBaseVals_3__5_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x012C);// AfitBaseVals_3__6_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03FF);// AfitBaseVals_3__7_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0014);// AfitBaseVals_3__8_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0064);// AfitBaseVals_3__9_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000C);// AfitBaseVals_3__10_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0010);// AfitBaseVals_3__11_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01E6);// AfitBaseVals_3__12_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_3__13_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0070);// AfitBaseVals_3__14_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x007D);// AfitBaseVals_3__15_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0064);// AfitBaseVals_3__16_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0032);// 0096 AfitBaseVals_3__17_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003C);// AfitBaseVals_3__18_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0073);// AfitBaseVals_3__19_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x009F);// AfitBaseVals_3__20_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0028);// AfitBaseVals_3__21_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0028);// AfitBaseVals_3__22_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0023);// AfitBaseVals_3__23_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0037);// AfitBaseVals_3__24_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0028);// AfitBaseVals_3__25_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0028);// AfitBaseVals_3__26_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0023);// AfitBaseVals_3__27_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0037);// AfitBaseVals_3__28_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2B32);// AfitBaseVals_3__29_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0601);// AfitBaseVals_3__30_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_3__31_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_3__32_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_3__33_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00FF);// AfitBaseVals_3__34_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x07A0);// AfitBaseVals_3__35_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFFF);// AfitBaseVals_3__36_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_3__37_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x050D);// AfitBaseVals_3__38_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1E80);// AfitBaseVals_3__39_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_3__40_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0A03);// 0A08 AfitBaseVals_3__41_ iNearGrayDesat[3] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0200);// AfitBaseVals_3__42_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF01);// AfitBaseVals_3__43_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x180F);// AfitBaseVals_3__44_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);// AfitBaseVals_3__45_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_3__46_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x8003);// AfitBaseVals_3__47_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0080);// AfitBaseVals_3__48_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0080);// AfitBaseVals_3__49_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0280);// AfitBaseVals_3__50_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0108);// AfitBaseVals_3__51_ iClustThresh_H[3] iClustMulT_H[3] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1E32);// AfitBaseVals_3__52_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1A24);// AfitBaseVals_3__53_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0A05);// AfitBaseVals_3__54_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x080A);// AfitBaseVals_3__55_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_3__56_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0328);// AfitBaseVals_3__57_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x324C);// AfitBaseVals_3__58_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001E);// AfitBaseVals_3__59_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0200);// AfitBaseVals_3__60_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0103);// AfitBaseVals_3__61_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x010C);// AfitBaseVals_3__62_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x9696);// AfitBaseVals_3__63_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0F0F);// AfitBaseVals_3__64_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0307);// AfitBaseVals_3__65_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x080F);// AfitBaseVals_3__66_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_3__67_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x030F);// AfitBaseVals_3__68_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3208);// AfitBaseVals_3__69_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0F1E);// AfitBaseVals_3__70_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x020F);// AfitBaseVals_3__71_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0103);// AfitBaseVals_3__72_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x010C);// AfitBaseVals_3__73_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x9696);// AfitBaseVals_3__74_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0F0F);// AfitBaseVals_3__75_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0307);// AfitBaseVals_3__76_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x080F);// AfitBaseVals_3__77_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_3__78_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x030F);// AfitBaseVals_3__79_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3208);// AfitBaseVals_3__80_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0F1E);// AfitBaseVals_3__81_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x020F);// AfitBaseVals_3__82_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0003);// AfitBaseVals_3__83_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_4__0_  Brightness[4] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// 0000 AfitBaseVals_4__1_  Contrast[4] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFF8);// AfitBaseVals_4__2_  Saturation[4] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_4__3_  Sharp_Blur[4] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_4__4_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0028);// AfitBaseVals_4__5_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x012C);// AfitBaseVals_4__6_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03FF);// AfitBaseVals_4__7_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0014);// AfitBaseVals_4__8_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0064);// AfitBaseVals_4__9_  */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x000C);// AfitBaseVals_4__10_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0010);// AfitBaseVals_4__11_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x01E6);// AfitBaseVals_4__12_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_4__13_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0070);// AfitBaseVals_4__14_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0087);// AfitBaseVals_4__15_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0073);// AfitBaseVals_4__16_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0032);// 0096 AfitBaseVals_4__17_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x003C);// AfitBaseVals_4__18_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0073);// AfitBaseVals_4__19_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00B4);// AfitBaseVals_4__20_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0028);// AfitBaseVals_4__21_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0028);// AfitBaseVals_4__22_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0023);// AfitBaseVals_4__23_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0046);// AfitBaseVals_4__24_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0028);// AfitBaseVals_4__25_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0028);// AfitBaseVals_4__26_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0023);// AfitBaseVals_4__27_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0046);// AfitBaseVals_4__28_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x2B23);// AfitBaseVals_4__29_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0601);// AfitBaseVals_4__30_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_4__31_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_4__32_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_4__33_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x00FF);// AfitBaseVals_4__34_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0B84);// AfitBaseVals_4__35_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFFFF);// AfitBaseVals_4__36_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_4__37_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x050D);// AfitBaseVals_4__38_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1E80);// AfitBaseVals_4__39_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_4__40_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0A03);// 0A08 AfitBaseVals_4__41_ iNearGrayDesat[4] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0200);// AfitBaseVals_4__42_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFF01);// AfitBaseVals_4__43_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x180F);// AfitBaseVals_4__44_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);// AfitBaseVals_4__45_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_4__46_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x8003);// AfitBaseVals_4__47_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0080);// AfitBaseVals_4__48_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0080);// AfitBaseVals_4__49_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0280);// AfitBaseVals_4__50_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0108);// AfitBaseVals_4__51_ iClustThresh_H[4] iClustMulT_H[4] */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1E1E);// AfitBaseVals_4__52_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x1419);// AfitBaseVals_4__53_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0A0A);// AfitBaseVals_4__54_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0800);// AfitBaseVals_4__55_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_4__56_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0328);// AfitBaseVals_4__57_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x324C);// AfitBaseVals_4__58_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x001E);// AfitBaseVals_4__59_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0200);// AfitBaseVals_4__60_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0103);// AfitBaseVals_4__61_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x010C);// AfitBaseVals_4__62_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x6464);// AfitBaseVals_4__63_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0F0F);// AfitBaseVals_4__64_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0307);// AfitBaseVals_4__65_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x080F);// AfitBaseVals_4__66_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_4__67_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x030F);// AfitBaseVals_4__68_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3208);// AfitBaseVals_4__69_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0F1E);// AfitBaseVals_4__70_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x020F);// AfitBaseVals_4__71_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0103);// AfitBaseVals_4__72_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x010C);// AfitBaseVals_4__73_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x6464);// AfitBaseVals_4__74_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0F0F);// AfitBaseVals_4__75_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0307);// AfitBaseVals_4__76_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x080F);// AfitBaseVals_4__77_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// AfitBaseVals_4__78_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x030F);// AfitBaseVals_4__79_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x3208);// AfitBaseVals_4__80_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0F1E);// AfitBaseVals_4__81_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x020F);// AfitBaseVals_4__82_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0003);// AfitBaseVals_4__83_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x7F5E);// ConstAfitBaseVals_0_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xFEEE);// ConstAfitBaseVals_1_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0xD9B7);// ConstAfitBaseVals_2_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0472);// ConstAfitBaseVals_3_ */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);// ConstAfitBaseVals_4_ */

	//*************************************/ 
	// 18.JPEG Thumnail Setting           */ 
	//*************************************/ 

	//*************************************/ 
	// 19.Input Size Setting              */ 
	//*************************************/ 

	//*********************************************************************************
	// 20.Preview & Capture Configration Setting                                       
	//*********************************************************************************
	// Preview config[0] 640X480  15~7.5fps //        
	S5K8AAYX_write_cmos_sensor(0x002A, 0x01BE);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0500);	// REG_0TC_PCFG_usWidth 500:1280; 280:640
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03C0);	// REG_0TC_PCFG_usHeight 3C0:960; 1E0:480
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0005);	// REG_0TC_PCFG_Format 5:YUV422; 7:RAW10  
	S5K8AAYX_write_cmos_sensor(0x002A, 0x01C8);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// REG_0TC_PCFG_uClockInd 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x01C4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0040);	// REG_0TC_PCFG_PVIMask 52:YUV422, 42:RAW10 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x01D4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0002);	// REG_0TC_PCFG_FrRateQualityType  1b:FR(bin) 2b:Quality(no-bin) 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x01D2);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// REG_0TC_PCFG_usFrTimeType  0:dynamic; 1:fixed not accurate; 2:fixed accurate 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x01D8);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x029A);	// REG_0TC_PCFG_usMaxFrTimeMsecMult10  30fps-014D; 15fps-029A; 7.5-0535; 6.0-0682; 3.75-0A6A
	S5K8AAYX_write_cmos_sensor(0x002A, 0x01D6);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0535);       // REG_0TC_PCFG_usMinFrTimeMsecMult10 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x029A);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// REG_0TC_PCFG_uPrevMirror 
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// REG_0TC_PCFG_uCaptureMirror 

	// Capture config[0] 1280x960  7.5fps    
	S5K8AAYX_write_cmos_sensor(0x002A, 0x02AE);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);	// Capture mode AE On 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x02B0);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0500);	// REG_0TC_CCFG_usWidth 500:1280; 280:640
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x03C0);	// REG_0TC_CCFG_usHeight 3C0:960; 1E0:480
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0005);	// REG_0TC_CCFG_Format 5:YUV422; 7:RAW10  
	S5K8AAYX_write_cmos_sensor(0x002A, 0x02BA);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// REG_0TC_CCFG_uClockInd
	S5K8AAYX_write_cmos_sensor(0x002A, 0x02B6);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0040);	// REG_0TC_CCFG_PVIMask 52:YUV422; 42:RAW10 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x02C6);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0002);	// REG_0TC_CCFG_FrRateQualityType  1b:FR(bin) 2b:Quality(no-bin)
	S5K8AAYX_write_cmos_sensor(0x002A, 0x02C4);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0002);	// REG_0TC_CCFG_usFrTimeType  0:dynamic; 1:fixed not accurate;  2:fixed accurate 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x02CA);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x014D);	// REG_0TC_CCFG_usMaxFrTimeMsecMult10  30fps-014D; 15fps-029A; 7.5-0535; 6.0-0682; 3.75-0A6A 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x02C8);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);	// REG_0TC_CCFG_usMinFrTimeMsecMult10 

	//*************************************/ 
	// 21.Select Cofigration Display      */ 
	//*************************************/ 
	S5K8AAYX_write_cmos_sensor(0x002A, 0x01A8);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);// REG_TC_GP_ActivePreviewConfig */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x01AA);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);// REG_TC_GP_PreviewConfigChanged */
	S5K8AAYX_write_cmos_sensor(0x002A, 0x019E);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);// REG_TC_GP_EnablePreview */
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);// REG_TC_GP_EnablePreviewChanged */

	S5K8AAYX_write_cmos_sensor(0x0028, 0xD000);
	S5K8AAYX_write_cmos_sensor(0x002A, 0x1000);
	S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);	// Set host interrupt 
	mdelay(150); //delay 150ms
	}

void S5K8AAYX_PV_Mode(void)
{
	S5K8AAYX_write_cmos_sensor(0x002A,0x01A8); 
	S5K8AAYX_write_cmos_sensor(0x0F12,0x0000);	// #REG_TC_GP_ActivePrevConfig // Select preview configuration_0
	S5K8AAYX_write_cmos_sensor(0x002A,0x01AC); 
	S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);	// #REG_TC_GP_PrevOpenAfterChange
	S5K8AAYX_write_cmos_sensor(0x002A,0x01A6); 
	S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);	// #REG_TC_GP_NewConfigSync // Update preview configuration
	S5K8AAYX_write_cmos_sensor(0x002A,0x01AA); 
	S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);	// #REG_TC_GP_PrevConfigChanged
	S5K8AAYX_write_cmos_sensor(0x002A,0x019E); 
	S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);	// #REG_TC_GP_EnablePreview // Start preview
	S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);	// #REG_TC_GP_EnablePreviewChanged	

}




void S5K8AAYX_CAP_Mode(void)
{
//
}

static void S5K8AAYX_set_AE_mode(kal_bool AE_enable)
{
     if(AE_enable==KAL_TRUE)
     {
               S5K8AAYX_write_cmos_sensor(0xFCFC, 0xD000); // Set page 
               S5K8AAYX_write_cmos_sensor(0x0028, 0x7000); // Set address  

               S5K8AAYX_write_cmos_sensor(0x002A, 0x214A); 
               S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
     }
     else
     {
               S5K8AAYX_write_cmos_sensor(0xFCFC, 0xD000); // Set page 
               S5K8AAYX_write_cmos_sensor(0x0028, 0x7000); // Set address  

               S5K8AAYX_write_cmos_sensor(0x002A, 0x214A); 
               S5K8AAYX_write_cmos_sensor(0x0F12, 0x0000);
     }

}


void S5K8AAYX_night_mode(kal_bool enable)
{
	/*----------------------------------------------------------------*/
	/* Local Variables												  */
	/*----------------------------------------------------------------*/

	/*----------------------------------------------------------------*/
	/* Code Body													  */
	/*----------------------------------------------------------------*/
	kal_uint16 video_frame_len = 0;
	kal_uint16 prev_line_len = 0;

    //if(S5K8AAYXCurrentStatus.iNightMode == enable)
    //    return;

	if (S5K8AAYX_sensor_cap_state == KAL_TRUE)
		return ;	

	//S5K8AAYX_write_cmos_sensor(0xFCFC, 0xD000); // Set page 
	S5K8AAYX_write_cmos_sensor(0x0028, 0x7000); // Set address	
		
	if (enable)
	{
		if (S5K8AAYX_MPEG4_encode_mode == KAL_TRUE)
		{
			S5K8AAYX_write_cmos_sensor(0x002A,0x01D2); 
			S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);	//REG_0TC_PCFG_usFrTimeType 
			S5K8AAYX_write_cmos_sensor(0x0F12,0x0002);	//REG_0TC_PCFG_FrRateQualityType
			S5K8AAYX_write_cmos_sensor(0x002A,0x01D6);
			S5K8AAYX_write_cmos_sensor(0x0F12,S5K8AAYX_VID_NIT_FIX_FR_TIME); //REG_0TC_PCFG_usMinFrTimeMsecMult10
			S5K8AAYX_write_cmos_sensor(0x0F12,S5K8AAYX_VID_NIT_FIX_FR_TIME); //REG_0TC_PCFG_usMaxFrTimeMsecMult10 fps
		}
		else
		{
			S5K8AAYX_write_cmos_sensor(0x002A,0x01D2); 
			S5K8AAYX_write_cmos_sensor(0x0F12,0x0000);	//REG_0TC_PCFG_usFrTimeType
			S5K8AAYX_write_cmos_sensor(0x0F12,0x0002);	//REG_0TC_PCFG_FrRateQualityType
			
			S5K8AAYX_write_cmos_sensor(0x002A,0x01D6);
			S5K8AAYX_write_cmos_sensor(0x0F12,0x0535); //REG_0TC_PCFG_usMinFrTimeMsecMult10
			S5K8AAYX_write_cmos_sensor(0x0F12,0x029A);	//REG_0TC_PCFG_usMaxFrTimeMsecMult10

		}	
		
		S5K8AAYX_write_cmos_sensor(0x002A,0x0468); 
        S5K8AAYX_write_cmos_sensor(0x0F12,0x0140);  //lt_uMaxDigGain

		spin_lock(&s5k8aayx_drv_lock);
		S5K8AAYX_night_mode_enable = KAL_TRUE;
		spin_unlock(&s5k8aayx_drv_lock);
	}
	else
	{
		if (S5K8AAYX_MPEG4_encode_mode == KAL_TRUE)
		{
			S5K8AAYX_write_cmos_sensor(0x002A,0x01D2); 
			S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);	//REG_0TC_PCFG_usFrTimeType 
			S5K8AAYX_write_cmos_sensor(0x0F12,0x0002);	//REG_0TC_PCFG_FrRateQualityType
			S5K8AAYX_write_cmos_sensor(0x002A,0x01D6);
			S5K8AAYX_write_cmos_sensor(0x0F12, S5K8AAYX_VID_NOM_FIX_FR_TIME); //#REG_0TC_PCFG_usMaxFrTimeMsecMult10
			S5K8AAYX_write_cmos_sensor(0x0F12, S5K8AAYX_VID_NOM_FIX_FR_TIME); //#REG_0TC_PCFG_usMinFrTimeMsecMult10
		}
		else
		{
			S5K8AAYX_write_cmos_sensor(0x002A,0x01D2); 
			S5K8AAYX_write_cmos_sensor(0x0F12,0x0000);	//REG_0TC_PCFG_usFrTimeType
			S5K8AAYX_write_cmos_sensor(0x0F12,0x0002);	//REG_0TC_PCFG_FrRateQualityType
			
			S5K8AAYX_write_cmos_sensor(0x002A,0x01D6);
			S5K8AAYX_write_cmos_sensor(0x0F12,0x0535); //REG_0TC_PCFG_usMinFrTimeMsecMult10
			S5K8AAYX_write_cmos_sensor(0x0F12,0x029A);	//REG_0TC_PCFG_usMaxFrTimeMsecMult10
		}	

		S5K8AAYX_write_cmos_sensor(0x002A,0x0468); 
		S5K8AAYX_write_cmos_sensor(0x0F12,0x0100);	//lt_uMaxDigGain

		spin_lock(&s5k8aayx_drv_lock);
		S5K8AAYX_night_mode_enable = KAL_FALSE;
		spin_unlock(&s5k8aayx_drv_lock);
	}
	
	// active preview configure
	//============================================================
	S5K8AAYX_write_cmos_sensor(0x002A,0x01A8); 
	S5K8AAYX_write_cmos_sensor(0x0F12,0x0000);	// #REG_TC_GP_ActivePrevConfig // Select preview configuration_0
	S5K8AAYX_write_cmos_sensor(0x002A,0x01AC); 
	S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);	// #REG_TC_GP_PrevOpenAfterChange
	S5K8AAYX_write_cmos_sensor(0x002A,0x01A6); 
	S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);	// #REG_TC_GP_NewConfigSync // Update preview configuration
	S5K8AAYX_write_cmos_sensor(0x002A,0x01AA); 
	S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);	// #REG_TC_GP_PrevConfigChanged
	S5K8AAYX_write_cmos_sensor(0x002A,0x019E); 
	S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);	// #REG_TC_GP_EnablePreview // Start preview
	S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);	// #REG_TC_GP_EnablePreviewChanged

	spin_lock(&s5k8aayx_drv_lock);
    S5K8AAYXCurrentStatus.iNightMode = enable;
	spin_unlock(&s5k8aayx_drv_lock);
}	/* S5K8AAYX_night_mode */

static kal_uint32 S5K8AAYX_GetSensorID(kal_uint32 *sensorID)
{
    volatile signed char i;
	kal_uint16 sensor_id=0,sensor_id_2=0;

     // check if sensor ID correct
	 S5K8AAYX_write_cmos_sensor(0xFCFC, 0x0000);
	 sensor_id=S5K8AAYX_read_cmos_sensor(0x0040);
	 SENSORDB("Sensor Read S5K8AAYX ID_use_FCFC %x \r\n",sensor_id);
	 
	 *sensorID=sensor_id;
	if (sensor_id != S5K8AAYX_SENSOR_ID)
	{
	    *sensorID=0xFFFFFFFF;
	    SENSORDB("Sensor Read ByeBye \r\n");
		return ERROR_SENSOR_CONNECT_FAIL;
	}
    return ERROR_NONE;    
}  


/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
UINT32 S5K8AAYXOpen(void)
{
	volatile signed char i;
	kal_uint16 sensor_id=0;

	 S5K8AAYX_write_cmos_sensor(0xFCFC, 0x0000);
	 sensor_id=S5K8AAYX_read_cmos_sensor(0x0040);
	 SENSORDB("Sensor Read S5K8AAYX ID %x \r\n",sensor_id);

	if (sensor_id != S5K8AAYX_SENSOR_ID)
	{
	    SENSORDB("Sensor Read ByeBye \r\n");
		return ERROR_SENSOR_CONNECT_FAIL;
	}

    S5K8AAYXInitialPara(); 
			
	S5K8AAYX_Initialize_Setting();
	return ERROR_NONE;
}

UINT32 S5K8AAYXClose(void)
{
	return ERROR_NONE;
}	/* S5K8AAYXClose() */

UINT32 S5K8AAYXPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	spin_lock(&s5k8aayx_drv_lock);
	S5K8AAYX_sensor_cap_state = KAL_FALSE;
	S5K8AAYX_PV_dummy_lines = 0;    
	
	if(sensor_config_data->SensorOperationMode==MSDK_SENSOR_OPERATION_MODE_VIDEO)		// MPEG4 Encode Mode
	{
		S5K8AAYX_MPEG4_encode_mode = KAL_TRUE;		
		S5K8AAYX_MJPEG_encode_mode = KAL_FALSE;
	}
	else
	{
		S5K8AAYX_MPEG4_encode_mode = KAL_FALSE;		
		S5K8AAYX_MJPEG_encode_mode = KAL_FALSE;
		
	}
	spin_unlock(&s5k8aayx_drv_lock);
	
	S5K8AAYX_PV_Mode();


	S5K8AAYX_set_mirror(sensor_config_data->SensorImageMirror);
 
	//just add for porting,please delete this when release
	S5K8AAYX_set_mirror(IMAGE_HV_MIRROR);
	
    image_window->ExposureWindowWidth = S5K8AAYX_IMAGE_SENSOR_PV_WIDTH;
    image_window->ExposureWindowHeight = S5K8AAYX_IMAGE_SENSOR_PV_HEIGHT;
	
	// copy sensor_config_data
	memcpy(&S5K8AAYXSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
  	return ERROR_NONE;
}	/* S5K8AAYXPreview() */

UINT32 S5K8AAYXCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                 MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
     kal_uint32 pv_integration_time = 0;       // Uinit - us
     kal_uint32 cap_integration_time = 0;
     kal_uint16 PV_line_len = 0;
     kal_uint16 CAP_line_len = 0;

	spin_lock(&s5k8aayx_drv_lock);
	S5K8AAYX_sensor_cap_state = KAL_TRUE;
	spin_unlock(&s5k8aayx_drv_lock);
               
    S5K8AAYX_CAP_Mode();         

    image_window->GrabStartX = S5K8AAYX_IMAGE_SENSOR_FULL_INSERTED_PIXELS;
    image_window->GrabStartY = S5K8AAYX_IMAGE_SENSOR_FULL_INSERTED_LINES;
    image_window->ExposureWindowWidth= S5K8AAYX_IMAGE_SENSOR_FULL_WIDTH;
    image_window->ExposureWindowHeight = S5K8AAYX_IMAGE_SENSOR_FULL_HEIGHT;

     // copy sensor_config_data
     memcpy(&S5K8AAYXSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
     return ERROR_NONE;
}        /* S5K8AAYXCapture() */

UINT32 S5K8AAYXGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	pSensorResolution->SensorFullWidth=S5K8AAYX_IMAGE_SENSOR_FULL_WIDTH;  //modify by yanxu
	pSensorResolution->SensorFullHeight=S5K8AAYX_IMAGE_SENSOR_FULL_HEIGHT;
	pSensorResolution->SensorPreviewWidth=S5K8AAYX_IMAGE_SENSOR_PV_WIDTH; 
	pSensorResolution->SensorPreviewHeight=S5K8AAYX_IMAGE_SENSOR_PV_HEIGHT;
	return ERROR_NONE;
}	/* S5K8AAYXGetResolution() */

UINT32 S5K8AAYXGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	
    switch(ScenarioId)
    {
        #if defined(MT6575)
    	case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 pSensorInfo->SensorPreviewResolutionX=S5K8AAYX_IMAGE_SENSOR_FULL_WIDTH;
	         pSensorInfo->SensorPreviewResolutionY=S5K8AAYX_IMAGE_SENSOR_FULL_HEIGHT;
			 pSensorInfo->SensorCameraPreviewFrameRate=15;
			 break;
		
		default:
		#endif
			 pSensorInfo->SensorPreviewResolutionX=S5K8AAYX_IMAGE_SENSOR_PV_WIDTH;
	         pSensorInfo->SensorPreviewResolutionY=S5K8AAYX_IMAGE_SENSOR_PV_HEIGHT;
			 pSensorInfo->SensorCameraPreviewFrameRate=30;
    }
	pSensorInfo->SensorFullResolutionX=S5K8AAYX_IMAGE_SENSOR_FULL_WIDTH;
	pSensorInfo->SensorFullResolutionY=S5K8AAYX_IMAGE_SENSOR_FULL_HEIGHT;
	pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=10;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=1;
	
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_VYUY;
	
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;

	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorInterruptDelayLines = 1;
	
	#ifdef MIPI_INTERFACE
   		pSensorInfo->SensroInterfaceType        = SENSOR_INTERFACE_TYPE_MIPI;
   	#else
   		pSensorInfo->SensroInterfaceType		= SENSOR_INTERFACE_TYPE_PARALLEL;
   	#endif
	
	pSensorInfo->SensorMasterClockSwitch = 0; 
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;//ISP_DRIVING_4MA;   	
	pSensorInfo->CaptureDelayFrame = 3; 
	pSensorInfo->PreviewDelayFrame = 3; 
	pSensorInfo->VideoDelayFrame = 4; 
	
	switch (ScenarioId) 
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			pSensorInfo->SensorClockFreq=24;
			
			pSensorInfo->SensorClockDividCount=	3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			
			pSensorInfo->SensorDataLatchCount= 2;
			
			pSensorInfo->SensorGrabStartX = 2; 
			pSensorInfo->SensorGrabStartY = 2; 
#ifdef MIPI_INTERFACE
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
	        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;
#endif
		break;

		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=	3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = 2; 
			pSensorInfo->SensorGrabStartY = 2; 					
			#ifdef MIPI_INTERFACE
	            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;			
	            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
		        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
		        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
	            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
	            pSensorInfo->SensorPacketECCOrder = 1;
	        	#endif			
		break;
				
		default:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
			pSensorInfo->SensorGrabStartX = 2; 
			pSensorInfo->SensorGrabStartY = 2; 					
		break;
	}
	memcpy(pSensorConfigData, &S5K8AAYXSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* S5K8AAYXGetInfo() */


UINT32 S5K8AAYXControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			S5K8AAYXPreview(pImageWindow, pSensorConfigData);
		break;
		
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			S5K8AAYXCapture(pImageWindow, pSensorConfigData);
		break;
		
		#if defined(MT6575)
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			   S5K8AAYXCapture(pImageWindow, pSensorConfigData);
			break;
		#endif
		
        default:
            return ERROR_INVALID_SCENARIO_ID;
	}
	return TRUE;
}	/* S5K8AAYXControl() */

/* [TC] YUV sensor */	
#if WINMO_USE
void S5K8AAYXQuery(PMSDK_FEATURE_INFO_STRUCT pSensorFeatureInfo)
{
	MSDK_FEATURE_TYPE_RANGE_STRUCT *pFeatureRange;
	MSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT *pFeatureMultiSelection;
	switch (pSensorFeatureInfo->FeatureId)
	{
		case ISP_FEATURE_DSC_MODE:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_SUPPORTED);
			pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
			pFeatureMultiSelection->TotalSelection = CAM_NO_OF_SCENE_MODE_MAX-2;
			pFeatureMultiSelection->DefaultSelection = CAM_AUTO_DSC_MODE;
			pFeatureMultiSelection->SupportedSelection = 
				(CAMERA_FEATURE_SUPPORT(CAM_AUTO_DSC_MODE)|
				CAMERA_FEATURE_SUPPORT(CAM_NIGHTSCENE_MODE));			
		break;
		case ISP_FEATURE_WHITEBALANCE:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_VIDEO_SUPPORTED);
			pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
			pFeatureMultiSelection->TotalSelection = CAM_NO_OF_WB;
			pFeatureMultiSelection->DefaultSelection = CAM_WB_AUTO;
			pFeatureMultiSelection->SupportedSelection = 
				(CAMERA_FEATURE_SUPPORT(CAM_WB_AUTO)|
				CAMERA_FEATURE_SUPPORT(CAM_WB_CLOUD)|
				CAMERA_FEATURE_SUPPORT(CAM_WB_DAYLIGHT)|
				CAMERA_FEATURE_SUPPORT(CAM_WB_INCANDESCENCE)|
				CAMERA_FEATURE_SUPPORT(CAM_WB_FLUORESCENT));
		break;
		case ISP_FEATURE_IMAGE_EFFECT:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_VIDEO_SUPPORTED);
			pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
			pFeatureMultiSelection->TotalSelection = CAM_NO_OF_EFFECT_ENC;
			pFeatureMultiSelection->DefaultSelection = CAM_EFFECT_ENC_NORMAL;
			pFeatureMultiSelection->SupportedSelection = 
				(CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_NORMAL)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_GRAYSCALE)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_COLORINV)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_GRAYINV)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_SEPIABLUE)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_SKETCH)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_EMBOSSMENT)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_SEPIA));	
		break;
		case ISP_FEATURE_AE_METERING_MODE:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;
		break;
		case ISP_FEATURE_BRIGHTNESS:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_RANGE;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_VIDEO_SUPPORTED);
			pFeatureRange = (PMSDK_FEATURE_TYPE_RANGE_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureRange);
			pFeatureRange->MinValue = CAM_EV_NEG_4_3;
			pFeatureRange->MaxValue = CAM_EV_POS_4_3;
			pFeatureRange->StepValue = CAMERA_FEATURE_ID_EV_STEP;
			pFeatureRange->DefaultValue = CAM_EV_ZERO;
		break;
		case ISP_FEATURE_BANDING_FREQ:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_VIDEO_SUPPORTED);
			pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
			pFeatureMultiSelection->TotalSelection = CAM_NO_OF_BANDING;
			pFeatureMultiSelection->DefaultSelection = CAM_BANDING_50HZ;
			pFeatureMultiSelection->SupportedSelection = 
				(CAMERA_FEATURE_SUPPORT(CAM_BANDING_50HZ)|
				CAMERA_FEATURE_SUPPORT(CAM_BANDING_60HZ));
		break;
		case ISP_FEATURE_AF_OPERATION:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;
		break;
		case ISP_FEATURE_AF_RANGE_CONTROL:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;
		break;
		case ISP_FEATURE_FLASH:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;			
		break;
		case ISP_FEATURE_VIDEO_SCENE_MODE:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_VIDEO_SUPPORTED);
			pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
			pFeatureMultiSelection->TotalSelection = 2;
			pFeatureMultiSelection->DefaultSelection = CAM_VIDEO_AUTO_MODE;
			pFeatureMultiSelection->SupportedSelection = 
				(CAMERA_FEATURE_SUPPORT(CAM_VIDEO_AUTO_MODE)|
				CAMERA_FEATURE_SUPPORT(CAM_VIDEO_NIGHT_MODE));
		break;
		case ISP_FEATURE_ISO:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;			
		break;
		default:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;			
		break;
	}
}
#endif 

BOOL S5K8AAYX_set_param_wb(UINT16 para)
{

	if(S5K8AAYXCurrentStatus.iWB == para)
		return TRUE;
	SENSORDB("[Enter]S5K8AAYX set_param_wb func:para = %d\n",para);

	S5K8AAYX_write_cmos_sensor(0xFCFC, 0xD000); // Set page 
	S5K8AAYX_write_cmos_sensor(0x0028, 0x7000); // Set address	
	
	switch (para)
	{
		case AWB_MODE_AUTO:
			S5K8AAYX_write_cmos_sensor(0x002A, 0x0408); //bit[3]:AWB Auto:1 menual:0
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x067F);
			break;

		case AWB_MODE_CLOUDY_DAYLIGHT:	
			//======================================================================
			//	MWB : Cloudy_D65										 
			//======================================================================
			S5K8AAYX_write_cmos_sensor(0xFCFC, 0xD000);
			S5K8AAYX_write_cmos_sensor(0x0028, 0x7000);
			S5K8AAYX_write_cmos_sensor(0x002A, 0x0408);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0677);							 
			S5K8AAYX_write_cmos_sensor(0x002A, 0x03DA);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0740);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0400);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0460);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001); 
			break;

		case AWB_MODE_DAYLIGHT:
			//==============================================
			//	MWB : sun&daylight_D50						
			//==============================================
			S5K8AAYX_write_cmos_sensor(0xFCFC, 0xD000);
			S5K8AAYX_write_cmos_sensor(0x0028, 0x7000);
			S5K8AAYX_write_cmos_sensor(0x002A, 0x0408);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0677);							 
			S5K8AAYX_write_cmos_sensor(0x002A, 0x03DA);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x05E0);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0400);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0530);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);  
			break;

		case AWB_MODE_INCANDESCENT:
			//==============================================									   
			//	MWB : Incand_Tungsten						
			//==============================================
			S5K8AAYX_write_cmos_sensor(0xFCFC, 0xD000);
			S5K8AAYX_write_cmos_sensor(0x0028, 0x7000);
			S5K8AAYX_write_cmos_sensor(0x002A, 0x0408); 
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0677);							
			S5K8AAYX_write_cmos_sensor(0x002A, 0x03DA); 
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x05C0); //Reg_sf_user_Rgain
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_RgainChanged
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0400); //Reg_sf_user_Ggain
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_GgainChanged
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x08B0); //Reg_sf_user_Bgain
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_BgainChanged		
			break;

		case AWB_MODE_FLUORESCENT:
			//==================================================================
			//	MWB : Florescent_TL84							  
			//==================================================================
			S5K8AAYX_write_cmos_sensor(0xFCFC, 0xD000);
			S5K8AAYX_write_cmos_sensor(0x0028, 0x7000);
			S5K8AAYX_write_cmos_sensor(0x002A, 0x0408);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0677);
			S5K8AAYX_write_cmos_sensor(0x002A, 0x03DA);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0575);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0400);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0800);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001); 
			break;

		case AWB_MODE_TUNGSTEN:	
			S5K8AAYX_write_cmos_sensor(0xFCFC, 0xD000);
			S5K8AAYX_write_cmos_sensor(0x0028, 0x7000);
			S5K8AAYX_write_cmos_sensor(0x002A, 0x0408);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0677);
								 
			S5K8AAYX_write_cmos_sensor(0x002A, 0x03DA);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0400);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0400);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0940);
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0001); 
			break;
		default:
			return KAL_FALSE;	

		}
		spin_lock(&s5k8aayx_drv_lock);
	    S5K8AAYXCurrentStatus.iWB = para;
		spin_unlock(&s5k8aayx_drv_lock);
return TRUE;
}

BOOL S5K8AAYX_set_param_effect(UINT16 para)
{
	/*----------------------------------------------------------------*/
	   /* Local Variables												 */
	   /*----------------------------------------------------------------*/
	   kal_uint32 ret = KAL_TRUE;
	
	   /*----------------------------------------------------------------*/
	   /* Code Body 													 */
	   /*----------------------------------------------------------------*/

	   if(S5K8AAYXCurrentStatus.iEffect == para)
		  return TRUE;
	   SENSORDB("[Enter]s5k8aayx set_param_effect func:para = %d\n",para);

	   S5K8AAYX_write_cmos_sensor(0x0028,0x7000);
	   S5K8AAYX_write_cmos_sensor(0x002A,0x019C);
	   switch (para)
	   {
		   case MEFFECT_OFF:
			   S5K8AAYX_write_cmos_sensor(0x0F12,0x0000); // Normal, 
			   break;
		   case MEFFECT_MONO:
			   S5K8AAYX_write_cmos_sensor(0x0F12,0x0001); // Monochrome (Black & White)
			   break;
		   case MEFFECT_SEPIA:
			   S5K8AAYX_write_cmos_sensor(0x0F12,0x0004); // Sepia
			   break;
		   case MEFFECT_SEPIABLUE:
			   S5K8AAYX_write_cmos_sensor(0x0F12,0x0005); // Aqua (Blue)
			   break;
		   case MEFFECT_NEGATIVE:
			   S5K8AAYX_write_cmos_sensor(0x0F12,0x0002); // Negative Mono
		   default:
			   ret = KAL_FALSE;
	   }
	   
	   spin_lock(&s5k8aayx_drv_lock);
	   S5K8AAYXCurrentStatus.iEffect = para;
	   spin_unlock(&s5k8aayx_drv_lock);
	   return ret;
} 


BOOL S5K8AAYX_set_param_banding(UINT16 para)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/

#if (defined(S5K8AAYX_MANUAL_ANTI_FLICKER))

	if(S5K8AAYXCurrentStatus.iBanding == para)
      return TRUE;

    switch (para)
	{
		case AE_FLICKER_MODE_50HZ:
				S5K8AAYX_write_cmos_sensor(0x0028,0x7000);
				S5K8AAYX_write_cmos_sensor(0x002a,0x0408);
				S5K8AAYX_write_cmos_sensor(0x0f12,0x065F);
				S5K8AAYX_write_cmos_sensor(0x002a,0x03F4);
				S5K8AAYX_write_cmos_sensor(0x0f12,0x0001); //REG_SF_USER_FlickerQuant 1:50hz  2:60hz
				S5K8AAYX_write_cmos_sensor(0x0f12,0x0001); //REG_SF_USER_FlickerQuantChanged active flicker
			break;
		case AE_FLICKER_MODE_60HZ:
				S5K8AAYX_write_cmos_sensor(0x0028,0x7000);
				S5K8AAYX_write_cmos_sensor(0x002a,0x0408);
				S5K8AAYX_write_cmos_sensor(0x0f12,0x065F);
				S5K8AAYX_write_cmos_sensor(0x002a,0x03F4);
				S5K8AAYX_write_cmos_sensor(0x0f12,0x0002); //REG_SF_USER_FlickerQuant 1:50hz  2:60hz
				S5K8AAYX_write_cmos_sensor(0x0f12,0x0001); //REG_SF_USER_FlickerQuantChanged active flicker
			break;
		default:
			return KAL_FALSE;
	}
	spin_lock(&s5k8aayx_drv_lock);
    S5K8AAYXCurrentStatus.iBanding = para;
	spin_unlock(&s5k8aayx_drv_lock);
	return TRUE;
	
#else
	/* Auto anti-flicker method is enabled, then nothing need to do in this function.  */

#endif	/* #if (defined(S5K8AAYX_MANUAL_ANTI_FLICKER)) */
	return KAL_TRUE;
} /* S5K8AAYX_set_param_banding */

BOOL S5K8AAYX_set_param_exposure(UINT16 para)
{

	if(S5K8AAYXCurrentStatus.iEV == para)
		return TRUE;
	
	SENSORDB("[Enter]s5k8aayx set_param_exposure func:para = %d\n",para);
	  
	S5K8AAYX_write_cmos_sensor(0x0028, 0x7000);
	S5K8AAYX_write_cmos_sensor(0x002a, 0x019A);		
    switch (para)
	{	
		case AE_EV_COMP_n10:
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0080); //EV -1
			break;
		case AE_EV_COMP_00:				   
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0100); //EV 0
			break;
		case AE_EV_COMP_10:					
			S5K8AAYX_write_cmos_sensor(0x0F12, 0x0200);  //EV +1
			break;
			
		case AE_EV_COMP_n13:					
		case AE_EV_COMP_n07:				   
		case AE_EV_COMP_n03:				   
		case AE_EV_COMP_03: 			
		case AE_EV_COMP_07: 
		case AE_EV_COMP_13:
			break;
			
		default:			
			return FALSE;
	}
	spin_lock(&s5k8aayx_drv_lock);
	S5K8AAYXCurrentStatus.iEV = para;
	spin_unlock(&s5k8aayx_drv_lock);
	return TRUE;

}/* S5K8AAYX_set_param_exposure */


UINT32 S5K8AAYXYUVSensorSetting(FEATURE_ID iCmd, UINT16 iPara)
{
	switch (iCmd) {
	case FID_SCENE_MODE:
	    if (iPara == SCENE_MODE_OFF)
	    {
	        S5K8AAYX_night_mode(0); 
	    }

         else if (iPara == SCENE_MODE_NIGHTSCENE)			
	    {
            S5K8AAYX_night_mode(1); 
	    }	    
	    break; 	    
	case FID_AWB_MODE:
         S5K8AAYX_set_param_wb(iPara);
	break;
	case FID_COLOR_EFFECT:
    	    
         S5K8AAYX_set_param_effect(iPara);
	break;
	case FID_AE_EV:  	    
         S5K8AAYX_set_param_exposure(iPara);
	break;
	case FID_AE_FLICKER:
	    	    	    
         S5K8AAYX_set_param_banding(iPara);
	break;
    case FID_AE_SCENE_MODE:  
		spin_lock(&s5k8aayx_drv_lock);
      if (iPara == AE_MODE_OFF) {
          S5K8AAYX_AE_ENABLE = KAL_FALSE; 
      }
      else {
          S5K8AAYX_AE_ENABLE = KAL_TRUE; 
      }
	  spin_unlock(&s5k8aayx_drv_lock);
      S5K8AAYX_set_AE_mode(S5K8AAYX_AE_ENABLE);
    break; 
	case FID_ZOOM_FACTOR:
		spin_lock(&s5k8aayx_drv_lock);
	    zoom_factor = iPara; 
		spin_unlock(&s5k8aayx_drv_lock);
	break; 
	default:
	break;
	}
	return TRUE;
}   /* S5K8AAYXYUVSensorSetting */


UINT32 S5K8AAYXYUVSetVideoMode(UINT16 u2FrameRate)
{

    if(S5K8AAYXCurrentStatus.iFrameRate == u2FrameRate)
      return TRUE;

	spin_lock(&s5k8aayx_drv_lock);
    S5K8AAYX_VEDIO_encode_mode = KAL_TRUE; 
	S5K8AAYX_MPEG4_encode_mode = KAL_TRUE;
	spin_unlock(&s5k8aayx_drv_lock);
	
	//S5K8AAYX_write_cmos_sensor(0xFCFC, 0xD000); // Set page 
	S5K8AAYX_write_cmos_sensor(0x0028, 0x7000); // Set address	

    if(20<=u2FrameRate && u2FrameRate<=30) //fix 30
    {		
		S5K8AAYX_write_cmos_sensor(0x002A,0x01D2); 
		S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);	//REG_0TC_PCFG_usFrTimeType 
		S5K8AAYX_write_cmos_sensor(0x0F12,0x0002);	//REG_0TC_PCFG_FrRateQualityType
		S5K8AAYX_write_cmos_sensor(0x002A,0x01D6);
		S5K8AAYX_write_cmos_sensor(0x0F12,S5K8AAYX_VID_NOM_FIX_FR_TIME); //REG_0TC_PCFG_usMinFrTimeMsecMult10
		S5K8AAYX_write_cmos_sensor(0x0F12,S5K8AAYX_VID_NOM_FIX_FR_TIME); //REG_0TC_PCFG_usMaxFrTimeMsecMult10 fps
    }
    else if(5<=u2FrameRate && u2FrameRate<20 )// fix 15
    {
		S5K8AAYX_write_cmos_sensor(0x002A,0x01D2); 
		S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);	//REG_0TC_PCFG_usFrTimeType 
		S5K8AAYX_write_cmos_sensor(0x0F12,0x0002);	//REG_0TC_PCFG_FrRateQualityType
		S5K8AAYX_write_cmos_sensor(0x002A,0x01D6);
		S5K8AAYX_write_cmos_sensor(0x0F12,S5K8AAYX_VID_NIT_FIX_FR_TIME); //REG_0TC_PCFG_usMinFrTimeMsecMult10
		S5K8AAYX_write_cmos_sensor(0x0F12,S5K8AAYX_VID_NIT_FIX_FR_TIME); //REG_0TC_PCFG_usMaxFrTimeMsecMult10 fps
    }
    else 
    {
        printk("Wrong Frame Rate \n"); 
    }
	
	// active preview configure
	S5K8AAYX_write_cmos_sensor(0x002A,0x01A8); 
	S5K8AAYX_write_cmos_sensor(0x0F12,0x0000);	// #REG_TC_GP_ActivePrevConfig // Select preview configuration_0
	S5K8AAYX_write_cmos_sensor(0x002A,0x01AC); 
	S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);	// #REG_TC_GP_PrevOpenAfterChange
	S5K8AAYX_write_cmos_sensor(0x002A,0x01A6); 
	S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);	// #REG_TC_GP_NewConfigSync // Update preview configuration
	S5K8AAYX_write_cmos_sensor(0x002A,0x01AA); 
	S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);	// #REG_TC_GP_PrevConfigChanged
	S5K8AAYX_write_cmos_sensor(0x002A,0x019E); 
	S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);	// #REG_TC_GP_EnablePreview // Start preview
	S5K8AAYX_write_cmos_sensor(0x0F12,0x0001);	// #REG_TC_GP_EnablePreviewChanged
    return TRUE;
}


kal_uint16 S5K8AAYXReadShutter()
{
   kal_uint16 temp_msb=0x0000,temp_lsb=0x0000;
   kal_uint32 temp=0x00000000;
   
   
   S5K8AAYX_write_cmos_sensor(0x002c,0x7000);
   S5K8AAYX_write_cmos_sensor(0x002e,0x16E2);//16bit

   temp_msb=S5K8AAYX_read_cmos_sensor(0x0f12);
   temp_msb = (temp_msb<<16)&0xffff0000;

   S5K8AAYX_write_cmos_sensor(0x002c,0x7000);
   S5K8AAYX_write_cmos_sensor(0x002e,0x16E0);
   temp_lsb=S5K8AAYX_read_cmos_sensor(0x0f12);
   temp_lsb = temp_lsb&0x0000ffff;

   temp = (temp_msb|temp_lsb)/400;

   temp = temp*72000/(S5K8AAYX_IMAGE_SENSOR_PV_WIDTH*2);
   return temp;
   
}
kal_uint16 S5K8AAYXReadGain()
{
    kal_uint16 temp=0x0000,Base_gain=64;
	S5K8AAYX_write_cmos_sensor(0x002c,0x7000);
    S5K8AAYX_write_cmos_sensor(0x002e,0x20D0);//AGain
    
	temp=S5K8AAYX_read_cmos_sensor(0x0f12);
	temp = Base_gain*temp/256;

	return temp;
}
kal_uint16 S5K8AAYXReadAwbRGain()
{
    kal_uint16 temp=0x0000,Base_gain=64;
	
	S5K8AAYX_write_cmos_sensor(0x002c,0x7000);
    S5K8AAYX_write_cmos_sensor(0x002e,0x20DC);

	temp=S5K8AAYX_read_cmos_sensor(0x0f12);
	temp = Base_gain*temp/1024;
	return temp;
}
kal_uint16 S5K8AAYXReadAwbBGain()
{
    kal_uint16 temp=0x0000,Base_gain=64;
	
	S5K8AAYX_write_cmos_sensor(0x002c,0x7000);
    S5K8AAYX_write_cmos_sensor(0x002e,0x20E0);

	temp=S5K8AAYX_read_cmos_sensor(0x0f12);
	temp = Base_gain*temp/1024;

	return temp;
}
#if defined(MT6575)

static void S5K8AAYXGetEvAwbRef(UINT32 pSensorAEAWBRefStruct/*PSENSOR_AE_AWB_REF_STRUCT Ref*/)
{
    PSENSOR_AE_AWB_REF_STRUCT Ref = (PSENSOR_AE_AWB_REF_STRUCT)pSensorAEAWBRefStruct;
    SENSORDB("S5K8AAYXGetEvAwbRef ref = 0x%x \n", Ref);
    	
	Ref->SensorAERef.AeRefLV05Shutter = 5422;
    Ref->SensorAERef.AeRefLV05Gain = 478; /* 128 base */
    Ref->SensorAERef.AeRefLV13Shutter = 80;
    Ref->SensorAERef.AeRefLV13Gain = 128; /*  128 base */
    Ref->SensorAwbGainRef.AwbRefD65Rgain = 186; /* 128 base */
    Ref->SensorAwbGainRef.AwbRefD65Bgain = 158; /* 128 base */
    Ref->SensorAwbGainRef.AwbRefCWFRgain = 196; /* 1.25x, 128 base */
    Ref->SensorAwbGainRef.AwbRefCWFBgain = 278; /* 1.28125x, 128 base */
}
static void S5K8AAYXGetCurAeAwbInfo(UINT32 pSensorAEAWBCurStruct/*PSENSOR_AE_AWB_CUR_STRUCT Info*/)
{
    PSENSOR_AE_AWB_CUR_STRUCT Info = (PSENSOR_AE_AWB_CUR_STRUCT)pSensorAEAWBCurStruct;
    SENSORDB("S5K8AAYXGetCurAeAwbInfo Info = 0x%x \n", Info);

    Info->SensorAECur.AeCurShutter = S5K8AAYXReadShutter();
    Info->SensorAECur.AeCurGain = S5K8AAYXReadGain() * 2; /* 128 base */
    
    Info->SensorAwbGainCur.AwbCurRgain = S5K8AAYXReadAwbRGain()<< 1; /* 128 base */
    
    Info->SensorAwbGainCur.AwbCurBgain = S5K8AAYXReadAwbBGain()<< 1; /* 128 base */
}

#endif

void S5K8AAYXGetAFMaxNumFocusAreas(UINT32 *pFeatureReturnPara32)
{	
    *pFeatureReturnPara32 = 0;    
    SENSORDB("S5K8AAYXGetAFMaxNumFocusAreas, *pFeatureReturnPara32 = %d\n",*pFeatureReturnPara32);

}

void S5K8AAYXGetAFMaxNumMeteringAreas(UINT32 *pFeatureReturnPara32)
{	
    *pFeatureReturnPara32 = 0;    
    SENSORDB("S5K8AAYXGetAFMaxNumMeteringAreas,*pFeatureReturnPara32 = %d\n",*pFeatureReturnPara32);

}

void S5K8AAYXGetExifInfo(UINT32 exifAddr)
{
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 28;
    pExifInfo->AEISOSpeed = AE_ISO_100;
    pExifInfo->AWBMode = S5K8AAYXCurrentStatus.iWB;
    pExifInfo->CapExposureTime = 0;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = AE_ISO_100;
}


UINT32 S5K8AAYXFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
	MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
	MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
	MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;
#if WINMO_USE	
	PMSDK_FEATURE_INFO_STRUCT pSensorFeatureInfo=(PMSDK_FEATURE_INFO_STRUCT) pFeaturePara;
#endif 


	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=S5K8AAYX_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16=S5K8AAYX_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PERIOD:
			//*pFeatureReturnPara16++=S5K8AAYX_PV_PERIOD_PIXEL_NUMS+S5K8AAYX_PV_dummy_pixels;
			//*pFeatureReturnPara16=S5K8AAYX_PV_PERIOD_LINE_NUMS+S5K8AAYX_PV_dummy_lines;
			//*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*pFeatureReturnPara32 = S5K8AAYX_sensor_pclk/10;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_ESHUTTER:
		break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			S5K8AAYX_night_mode((BOOL) *pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_GAIN:
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			S5K8AAYX_isp_master_clock=*pFeatureData32;
		break;
		case SENSOR_FEATURE_SET_REGISTER:
			S5K8AAYX_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = S5K8AAYX_read_cmos_sensor(pSensorRegData->RegAddr);
		break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &S5K8AAYXSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
			*pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
		break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
		case SENSOR_FEATURE_GET_CCT_REGISTER:
		case SENSOR_FEATURE_SET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:

		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
		
		case SENSOR_FEATURE_GET_GROUP_INFO:
		case SENSOR_FEATURE_GET_ITEM_INFO:
		case SENSOR_FEATURE_SET_ITEM_INFO:
		case SENSOR_FEATURE_GET_ENG_INFO:
					break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
                        *pFeatureReturnPara32++=0;
                        *pFeatureParaLen=4;	    

		break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_YUV_CMD:
			//S5K8AAYXYUVSensorSetting((MSDK_ISP_FEATURE_ENUM)*pFeatureData16, *(pFeatureData16+1));
			S5K8AAYXYUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
		break;
		break;
#if WINMO_USE		
		case SENSOR_FEATURE_QUERY:
			S5K8AAYXQuery(pSensorFeatureInfo);
			*pFeatureParaLen = sizeof(MSDK_FEATURE_INFO_STRUCT);
		break;		
		case SENSOR_FEATURE_SET_YUV_CAPTURE_RAW_SUPPORT:
			/* update yuv capture raw support flag by *pFeatureData16 */
		break;
#endif 				
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		    S5K8AAYXYUVSetVideoMode(*pFeatureData16);
		    break; 
		#if defined(MT6575)
		case SENSOR_FEATURE_GET_EV_AWB_REF:
			 S5K8AAYXGetEvAwbRef(*pFeatureData32);
				break;
  		case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:
			   S5K8AAYXGetCurAeAwbInfo(*pFeatureData32);	
			break;
		#endif
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
            S5K8AAYX_GetSensorID(pFeatureData32); 
            break; 	

		case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
			S5K8AAYXGetAFMaxNumFocusAreas(pFeatureReturnPara32); 		   
			*pFeatureParaLen=4;
			break;	   
		case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
			S5K8AAYXGetAFMaxNumMeteringAreas(pFeatureReturnPara32);			  
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_GET_EXIF_INFO:
			SENSORDB("SENSOR_FEATURE_GET_EXIF_INFO\n");
			SENSORDB("EXIF addr = 0x%x\n",*pFeatureData32); 		 
			S5K8AAYXGetExifInfo(*pFeatureData32);
			break;
			
		default:
			break;			
	}
	return ERROR_NONE;
}

SENSOR_FUNCTION_STRUCT	SensorFuncS5K8AAYX=
{
	S5K8AAYXOpen,
	S5K8AAYXGetInfo,
	S5K8AAYXGetResolution,
	S5K8AAYXFeatureControl,
	S5K8AAYXControl,
	S5K8AAYXClose
};

UINT32 S5K8AAYX_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncS5K8AAYX;
	return ERROR_NONE;
}	/* SensorInit() */
