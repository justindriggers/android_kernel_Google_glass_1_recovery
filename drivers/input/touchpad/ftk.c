/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
*
* File Name		: ftk.c
* Authors		: Sensor & MicroActuators BU - Application Team
*			    : Bela Somaiah  (bela.somaiah@st.com)
* Version		: V 1.0 
* Date			: 29/06/2011
* Description	: Capacitive touch screen controller (FingertipK)
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
********************************************************************************
* REVISON HISTORY
*
*VERSION | DATE 	  | AUTHORS	    | DESCRIPTION
* 1.0    |  29/06/2011| Bela Somaiah| First Release(link to ftk_patch.h)
*
*******************************************************************************/
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h> 
#include <linux/interrupt.h>
#include <linux/serio.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/i2c/ftk_patch.h>

#include <linux/fs.h>
#include <linux/uaccess.h>

#include <linux/earlysuspend.h>

/*
 * Definitions & global arrays.
 */
#define DRIVER_DESC	"ftk i2c touchscreen driver"
#define ftk_TS_DRV_NAME	"ftk"

#define X_AXIS_MAX		480
#define X_AXIS_MIN		0
#define Y_AXIS_MAX		800    // 272
#define Y_AXIS_MIN		0
#define PRESSURE_MIN	0
#define PRESSURE_MAX	256

#define LOAD_PATCH_FOR_FINGERTIPK   1 // 1


static struct i2c_driver stm_ts_driver;
static struct workqueue_struct *stmtouch_wq;
static int cor_xyz[10][3];
static int prev_xyz[10][3];
static u8 ID_Indx[10];
static u8 IDj = 0;
static u8 StylusTouchID=100;

struct B0_write {   
  u8 addr;
  u8 val;
};


static unsigned char ftk_keycode[] = {
	KEY_BACK,
	KEY_SEARCH,
	KEY_HOME,
	KEY_1,
	KEY_9,
};

//#define patch_file_path "/mnt/sdcard/ftk.patch"
//#define config_file_path "/mnt/sdcard/ftk.config"

// CMM elton pointers
#define patch_file_path "/system/etc/touchpad/ftk.patch"
#define config_file_path "/system/etc/touchpad/ftk.config"
// \CMM

//#define patch_file_path "/data/misc/wifi/ftk.patch"
//#define config_file_path "/data/misc/wifi/ftk.config"

static u32 strnum_to_digtial( char * inputStrData, int length)  ;


#ifdef LOAD_PATCH_FOR_FINGERTIPK
extern u16 PatchDataSize;
//extern u8 PatchData[648];  //for patch 1

extern u8 PatchData[15984];     //for patch 7  of 169;

#define P70_PATCH_LEN           PatchDataSize// 16*1024
#define WRITE_CHUNK_SIZE         64
#define P70_PATCH_ADDR_START     0x00420000
#endif

struct hrtimer test_timer;

struct ftk_ts {
	struct device *dev;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct hrtimer timer;
	struct work_struct  work;
	spinlock_t		lock;
	int reported_finger_count;
	int x;
	int y;
	int z;
	int pendown;
	int irq;
	int (*power)(int on);
	struct early_suspend early_suspend;
};

static int TouchID_EventXYZ[10][5] = {0};	   //add by michael HU; 03Nov2011;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void stm_ts_early_suspend(struct early_suspend *h);
static void stm_ts_late_resume(struct early_suspend *h);
#endif


static int init_ftk(struct ftk_ts *ftk_ts);


static void test_timer_func(void)
{
//	printk("Michael: test_timer_func\n");
//	ftk_gpio_test();

	printk("In the test_timer_func!\n" );

//	init_ftk(struct ftk_ts *ftk_ts);

	hrtimer_start(&test_timer, ktime_set(1, 0), HRTIMER_MODE_REL);      
}

static void start_test_timer(void)
{
	printk("enter start_test_timer\n");
	hrtimer_init(&test_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	test_timer.function = test_timer_func;
	hrtimer_start(&test_timer, ktime_set(1, 0), HRTIMER_MODE_REL);
}

static void stop_test_timer(void)
{
	hrtimer_cancel(&test_timer);	
}


static int ftk_write_reg(struct ftk_ts *ftk_ts, unsigned char * reg, u16 num_com)
{
	int ret, i;

		struct i2c_msg xfer_msg[2];

	        xfer_msg[0].addr = ftk_ts->client->addr;
	        xfer_msg[0].len = num_com;
	        xfer_msg[0].flags = 0;
	        xfer_msg[0].buf = reg;

	        return i2c_transfer(ftk_ts->client->adapter, xfer_msg, 1);

	

	return 0;
}

static int ftk_read_reg(struct ftk_ts *ftk_ts, unsigned char *reg, int cnum, u8 *buf, int num)
{
	#if 1
	        struct i2c_msg xfer_msg[2];

	        xfer_msg[0].addr = ftk_ts->client->addr;
	        xfer_msg[0].len = cnum;
	        xfer_msg[0].flags = 0;
	        xfer_msg[0].buf = reg;

	        xfer_msg[1].addr = ftk_ts->client->addr;
	        xfer_msg[1].len = num;
	        xfer_msg[1].flags = I2C_M_RD;
	        xfer_msg[1].buf = buf;

	        return i2c_transfer(ftk_ts->client->adapter, xfer_msg, 2);
	#endif

	
	return 0;
}
#define B0_COMMAND_WRITES     75

static struct B0_write B0_command_writes[B0_COMMAND_WRITES] = {
	

{0x2E,0x10},//</Touch-In Hysteresis" />
 {0x2C,0x5A},//</TVR" />
 {0x2D,0x00},//</" />
 {0x4E,0x10},//</EVR1" />
 {0x4F,0x00},//</" />
 {0x7F,0x40},//</EVR2" />
 {0x80,0x00},//</" />
 {0x13,0x15},//</Post Sync Noise Reduction" />
 {0x15,0x09},//</TS FE settings - Triangular window enable" />
 {0x16,0x83},//</FECONF: RTI selection and M-Cycle" />
 {0x17,0x8F},//</C_int Sel and T-Cycle" />
 {0x18,0x06},//</I-Cycle" />
 {0x19,0x07},//</R-Cycle (min)" />
 {0x1A,0x55},//</Protocol" />
 {0x1B,0x55},//</Protocol" />

// {0x1C,0x04},//</X start Channel" />
//{0x1D,0x0F},//</X Length" />
//{0x1E,0x05},//</Y Start Channel" />
// {0x1F,0x09},//</Y Length" />

 {0x1C,0x04},//</X start Channel" />
 {0x1D,0x14},//</X Length" />
 {0x1E,0x05},//</Y Start Channel" />
 {0x1F,0x0C},//</Y Length" />


 {0x30,0x10},//</Stylus Strength Hysteresis" />
 {0x32,0x20},//</Finger Strength Hysteresis" />
 {0x34,0xD1},//</Adaptive Filter - Rscan enable; Adaptive weight Enable; Sample count" />
 {0x35,0x0A},//</Std dev base" />
 {0x36,0x0A},//</Std dev threshold 1" />
 {0x37,0x50},//</Std dev threshold 2" />
 {0x3A,0x59},//</Frame Filter Weight (Noisy and Default)" />
 {0x3B,0x8A},//</Co-ord Filter weight:Z,XY" />
 {0x48,0x70},//</Stylus Strength Threshold" />
 {0x4A,0x00},//</Stylus Area Threshold" />
 {0x4B,0xA0},//</Finger Strength Threshold (Byte 0)" />
 {0x4C,0x00},//</Finger Strength Threshold (Byte 1)" />
 {0x4D,0x06},//</Finger Area Threshold" />
 //{0x57,0x60},//</Display Screen Resolution X (Byte 0)" />
 //{0x58,0x01},//</Display Screen Resolution X (Byte 1)" />
 //{0x59,0xE6},//</Display Screen Resolution Y (Byte 0)" />
// {0x5A,0x00},//</Display Screen Resolution Y (Byte 1)" />
 
 //{0x57,0x20},//Display Resolution for X	  //landscape mode;
 //{0x58,0x3},//Display Resolution for X
 //{0x59,0xE0},//Display Resolution for Y
 //{0x5A,0x1},//Display Resolution for Y
 
{0x57,0x20},//Display Resolution for X           //---0x 01 EO = 480;
{0x58,0x03},//Display Resolution for X
{0x59,0xE0},//Display Resolution for Y	    // ---0X 03 20 = 800
{0x5A,0x01},//Display Resolution for Y

 {0x61,0x05},//</TS Active Frame Rate" />
 {0x62,0x05},//</TS Idle Frame Rate" />
 {0x63,0xC8},//</Auto Cal Interval (between cal cycles)" />
 {0x64,0x05},//</Auto Cal Sampling Time (between samples)" />
 {0x65,0x70},//</Tracking Radius (Byte 0)" />
 {0x66,0x00},//</Tracking Radius (Byte 1)" />
 {0x68,0x2F},//</Touch-In Delay" />
 {0x69,0x1F},//</Touch-out Delay" />
 {0x6E,0x05},//</Motion Tolerance X" />
 {0x6F,0x05},//</Motion Tolerance Y" />
 {0x70,0x94},//</SW Feature En: Auto Cal_Dis; Peak Det_En; Frame Reconstruction On" />
 {0x7D,0x37},//</ACC and Bit shift (Active Mode)" />
 {0x7E,0x37},//</ACC and Bit shift (Idle Mode)" />
 {0x88,0x04},//</Comp Clk Count" />
 {0x89,0x83},//</FE Cap sel (TSComp_Enable)" />
 {0x8A,0x0F},//</C-Comp setting (default:20pF) " />
 {0x8C,0xFF},//</Cs Offset (Byte 0)" />
 {0x8D,0xC1},//</Cs Offset (Byte 1)" />
 {0xAE,0x00},//</In-Cell Disable" />
 {0xB6,0x02},//</Min Area for Touch reporting" />
 {0xB7,0x00},//</Stationary Touch Reporting Interval" />
 {0xB8,0x40},//</Merge In Threshold" />
 {0xB9,0x10},//</Merge Out Hysteresis" />
 {0xBA,0x6F},//</Touch-In delay 2" />
 {0xBB,0x0D},//</Noisy Coordinate Filter" />
 {0xBC,0x01},//</Senseline Compensation" />
 {0xBD,0x12},//</Noise Monitoring TX - 19" />
 {0xBE,0x01},//</Noise Reduction Step" />
 {0xBF,0x10},//</Frame/Co-ordinate IIR Noise Threshold" />
 {0xC0,0x10},//</R-start:03" />
 {0xC1,0x9C},//</" />
 {0xC2,0x05},//</R-end:21" />
 {0xC3,0x00},//</" />
 {0xC4,0x04},//</R1:03" />
 {0xC5,0x0C},//</R2:08" />
 {0xC6,0x10},//</R3:0D" />
 {0xC7,0x12},//</R4:13" />
 {0xC8,0x03},//</Standard Deviation Adaptive Threshold Slope" />
 {0xCA,0x0F},//</Upper limit for Standard Deviation" /> 

  
  
};


static u8 SendConfigByFile(struct ftk_ts *ftk_ts,char * inputStrData, int iLength)
{
	int i = 0;
	u8 regAdd[8];
	int rc;

      //printk( "Enter SendConfigByFile \n");

	for ( i = 0 ; i < iLength/2; i++ )
	{
		regAdd[i] =  strnum_to_digtial( &inputStrData[i*2], 2 );
		//printk("%02X ", regAdd[i]);
	}
     //  printk("\n");
	
	rc = ftk_write_reg(ftk_ts, &regAdd[0],iLength/2);

	

#if 1
	if ( ( regAdd[0] == 0xB0 ) &&  ( regAdd[1] == 0x04 ))
	{
		printk("mdelay(55) after 0xB004 \n");
		mdelay(50);	
	}
#endif
		
	mdelay(5);	


	return 0;
}


static u8 LoadConfigFromFile(struct ftk_ts *ftk_ts)
{
	u32 writeAddr, j = 0, iReadLen = 0, iDataPackLen = 0, iTemp = 0 ;
	loff_t iPos = 0;
	static char buf[WRITE_CHUNK_SIZE];
        struct file *fp;
        mm_segment_t fs;
	u8 regAdd[8];
	u8 val[8];
	int rc;


	printk("Michael HU 10Oct2011: Enter LoadConfigFromFile function\n");

	//return -1;

	fp = filp_open(config_file_path, O_RDONLY, 0644);
	
	if (IS_ERR(fp)) {
		 printk("Open patch  file error\n");
		 return -1;
	 }

	fs = get_fs();
	set_fs(KERNEL_DS);

	 iPos= 0;	
   	 iReadLen = vfs_read(fp, buf, sizeof(buf), &iPos);

	if ( iReadLen == 0 )
	{
		printk("The file is an empty file!\n");
		return -1;
	}

	 iPos = 0;	

	 while( iReadLen > 0 )
	 {
	 	iReadLen = vfs_read(fp, buf, 2, &iPos);

		//printk("buf[0] =%c, buf[1]=%c\n", buf[0], buf[1]);
		//printk("iReadLen=%d\n", iReadLen);
		
		if ( iReadLen == 2 )
			iDataPackLen = strnum_to_digtial(buf, 2);
		else if ( iReadLen == 0 )
		{
			printk("finish reading config file\n");
		}
		else
		{
			printk("Pack length is wrong!");
			return -1;
		}

	//	iPos = iPos + 2;
		iTemp = iPos;
	//	printk("first iPos = %d\n!", iTemp );
		
	 	iReadLen = vfs_read(fp, buf, iDataPackLen, &iPos);

		if ( iReadLen == iDataPackLen )
			SendConfigByFile(ftk_ts, buf,iDataPackLen);
		else if ( iReadLen == 0 )
		{
			printk("finish reading config file\n");
		}
		else
		{
			printk("Pack Data  is wrong!");
			return -1;			
		}

	//	iPos = iPos + iDataPackLen;
		iTemp = iPos;
	//	printk("Second iPos = %d\n!", iTemp );

	 }


#if 1
	
#ifdef LOAD_PATCH_FOR_FINGERTIPK
				regAdd[0] = 0xB0; regAdd[1] = 0x03;
					rc=ftk_read_reg(ftk_ts, regAdd, 2, val, 1);
				mdelay(5);	
				printk("Patch loaded, Version =%X \n", val[0]); 
#endif


				regAdd[0]=0x83; //TS Sense on
				regAdd[1]=0x00; 
				rc = ftk_write_reg(ftk_ts, &regAdd[0],1);
				mdelay(5);	
#endif

		printk("finish LoadConfigFromFile function \n"); 

		return 0;

}


static u8 LoadConfig(struct ftk_ts *ftk_ts)
{
	u8 val[8];	
	u8 regAdd[7];
	int rc;	
	u8 patchConfig = 0xC0;
	int b0_i = 0;
	int k;
	
		for (  k = 0 ; k <2; k++)
		{
			
	regAdd[0] = 0xB0; // All writes below are 1 addr, 1 value
	for (b0_i = 0; b0_i < B0_COMMAND_WRITES; ++b0_i ) {
		regAdd[1] = B0_command_writes[b0_i].addr;
		regAdd[2] = B0_command_writes[b0_i].val;
		ftk_write_reg(ftk_ts, &regAdd[0], 3);
		mdelay(5);
	}
	
	
	regAdd[0]=0xB3; // Set Upper byte Address
	regAdd[1]=0xFF;
	regAdd[2]=0xFF;	
	ftk_write_reg(ftk_ts, &regAdd[0],3);
	mdelay(5);
	
	regAdd[0]=0xB1; // Set lower byte Address
	regAdd[1]=0xFC;
	regAdd[2]=0x2F;	
	regAdd[3]=patchConfig;
	ftk_write_reg(ftk_ts, &regAdd[0],4);
	mdelay(5);
		
	regAdd[0]=0xB0; // Warm-Boot 
	regAdd[1]=0x04;
	regAdd[2]=0x20;	
	ftk_write_reg(ftk_ts, &regAdd[0],3);
	mdelay(12);
	
  regAdd[1]=0x05; // Set Interrupt Polarity
	regAdd[2]=0x01;   //'0' - level interrupt	'1' - edge  interrupt 
	ftk_write_reg(ftk_ts, &regAdd[0],3);
	mdelay(5);
	
	regAdd[1]=0x06;  // Enable Touch Detect Interrupt 
	regAdd[2]=0xC0;	
	ftk_write_reg(ftk_ts, &regAdd[0],3);
	mdelay(5);
	
	regAdd[1]=0x07; //Read 0x07 to clear ISR
	rc=ftk_read_reg(ftk_ts, &regAdd[0], 2, &val[0], 1);	
	mdelay(5);	
	
	regAdd[0]=0x85; //Read 0x85 to clear the buffer
	rc=ftk_read_reg(ftk_ts, &regAdd[0], 1, &val[0], 8);	
	mdelay(5);
		
	regAdd[0]=0x85; //Read 0x85 to clear the buffer
	rc=ftk_read_reg(ftk_ts, &regAdd[0], 1, &val[0], 8);	
	mdelay(5);	 
	
	
	
	regAdd[0]=0x81; //TS Sleep out
	regAdd[1]=0x00;	
	ftk_write_reg(ftk_ts, &regAdd[0],1);
	mdelay(5);
		
	
	
	regAdd[0]=0x8A; //Set Area Location
	regAdd[1]=0x00;	//Touch screen size
	regAdd[2]=0x00;
	regAdd[3]=0x00;		
	regAdd[4]=0xFF;	 //Full screen is used
	regAdd[5]=0xFF;
	regAdd[6]=0xFF;	
	ftk_write_reg(ftk_ts, &regAdd[0],7);
	mdelay(5);	
	
	regAdd[0]=0x8B; //Set Area layer
	regAdd[1]=0x00;	
	ftk_write_reg(ftk_ts, &regAdd[0],2);
	mdelay(5);
	
	regAdd[0]=0x8C; //Set Area Event
	regAdd[1]=0x00;	
	regAdd[2]=0x00;
	regAdd[3]=0x00;		
	regAdd[4]=0x00;	
	regAdd[5]=0x0F;
	regAdd[6]=0xFF;	
	ftk_write_reg(ftk_ts, &regAdd[0],7);
	mdelay(5);
	
	regAdd[0]=0x8D; //Set Area Touches 
	regAdd[1]=0x00;
	regAdd[2]=0xFF;
	ftk_write_reg(ftk_ts, &regAdd[0],3);
	mdelay(5);
	
	
	regAdd[0]=0x8F; //Touch screen orientation
//	regAdd[1]=0xE0;// (Portrait mode)
//	regAdd[1]=0x80;	//(Landscape Mode)
//	regAdd[1]=0x60;// (Portrait mode)
	//regAdd[1]=0x00;// (Portrait mode)
	regAdd[1]=0x20;// (Portrait mode)

	ftk_write_reg(ftk_ts, &regAdd[0],2);
	mdelay(5);

#ifdef LOAD_PATCH_FOR_FINGERTIPK
	regAdd[0] = 0xB0; regAdd[1] = 0x03;
    rc=ftk_read_reg(ftk_ts, regAdd, 2, val, 1);
	mdelay(5);	
	printk("Patch loaded, Version =%X \n", val[0]);	
#endif
	


	regAdd[0]=0x83; //TS Sense on
	regAdd[1]=0x00;	
	rc = ftk_write_reg(ftk_ts, &regAdd[0],1);
	mdelay(5);	
}
			
	
		if(rc<0)
			printk("ftk Not Initialised from loadconfig \n");
		else
			printk("ftk Initialised from loadconfig\n");		

}



#ifdef LOAD_PATCH_FOR_FINGERTIPK
static u8 LoadPatch(struct ftk_ts *ftk_ts)
{
	u32 writeAddr, j = 0;
	u8 i, byteWork0[2], byteWork1[67], regAdd[3] ={0xB3, 0xB1,0};
	
	while (j<P70_PATCH_LEN)
	{
		writeAddr = P70_PATCH_ADDR_START + j;
		
		byteWork0[0] = (writeAddr >> 24) & 0xFF;
		byteWork0[1] = (writeAddr >> 16) & 0xFF;
		regAdd[0] =0xB3;
		regAdd[1] =byteWork0[0];
		regAdd[2] =byteWork0[1];
		ftk_write_reg(ftk_ts, &regAdd[0],3);
		mdelay(10);	
		
		byteWork1[0]= 0xB1;
		byteWork1[1] = (writeAddr >> 8) & 0xFF;
		byteWork1[2] = writeAddr & 0xFF;

		i = 0;
		while ((j<P70_PATCH_LEN) && (i<WRITE_CHUNK_SIZE))
		{
			byteWork1[i+3] = PatchData[j]; 
			i++;
			j++;
		}
		ftk_write_reg(ftk_ts, &byteWork1[0],67);
		mdelay(10);			
		
	}
	mdelay(10);	
	return 0;
}

static u8 Is_file_exist()
{
        struct file *fp;


		fp = filp_open(patch_file_path, O_RDONLY, 0644);
		
		if (IS_ERR(fp)) {
			printk("Patch file doesn't exist %s\n", patch_file_path);
			 return -1;
		 }
		else
		{
			filp_close(fp, NULL);
			printk("Patch file	exist\n");	
		}


		fp = filp_open(config_file_path, O_RDONLY, 0644);
	
		if (IS_ERR(fp)) {
			printk("Config file doesn't exist %s\n", config_file_path);
			 return -1;
		 }
		else
		{
			filp_close(fp, NULL);
			printk("Config file  exist\n");
			return 0;
		}
}

//u32 ChangeNum(String str,int length)      
static u32 strnum_to_digtial( char * inputStrData, int length)      
{      
   // char  revstr[16] = {'0'};    
    int   num[16] = {0};      
    int   count = 1;      
    int   result = 0;
    int  i;

 // wsprintfA(revstr, "%S", str);

    for   ( i=length-1;i>=0;i--)      
    {      
        if ((inputStrData[i] >= '0') && (inputStrData[i] <= '9'))      
            num[i] = inputStrData[i] - 48;
        else if ((inputStrData[i] >= 'a') && (inputStrData[i] <= 'f'))      
            num[i] = inputStrData[i] - 'a' + 10;      
        else if ((inputStrData[i] >= 'A') && (inputStrData[i] <= 'F'))      
            num[i] = inputStrData[i] - 'A' + 10;      
        else      
            num[i] = 0;    
        result = result+num[i] * count;      
        count = count * 16;    
    }    

    return result;      
}   


u8 get_data_from_file(struct file *fp,  char * buffer)
{
	static char raw_data_buf[WRITE_CHUNK_SIZE*2];
	u32 iReadLen = 0, i = 0, iTemp = 0;
	u32 iValue = 0; 
	loff_t iPos = 0;


	iReadLen = vfs_read(fp, raw_data_buf, sizeof(raw_data_buf), &iPos);

	iTemp = iPos;		
	printk("get_data_from_file iPos = %d!\n", iTemp);


	for ( i = 0; i < WRITE_CHUNK_SIZE; i++)
	{
			buffer[i] = strnum_to_digtial( &raw_data_buf[i*2], 2 );
			//printk("%02X ",buffer[i]);
	}
	//printk("\n iReadLen = %d, div/6= %d\n", iReadLen, iReadLen/6);
	return (iReadLen/2);
}

static u8 LoadPatchFromFile(struct ftk_ts *ftk_ts)
{
	u32 writeAddr, j = 0, iReadLen = 0, iTemp = 0;
	u8 i, byteWork0[2], byteWork1[67], regAdd[3] ={0xB3, 0xB1,0};
	static char buf[WRITE_CHUNK_SIZE];
        struct file *fp;
        mm_segment_t fs;			
	loff_t iPos = 0;

	printk("Enter LoadPatchFromFile function\n");

	fp = filp_open(patch_file_path, O_RDONLY, 0644);
	
	if (IS_ERR(fp)) {
		 printk("Open patch  file error\n");
		 return -1;
	 }

	fs = get_fs();
	set_fs(KERNEL_DS);

	 iPos= 0;	
   	 iReadLen = vfs_read(fp, buf, sizeof(buf), &iPos);

	if ( iReadLen == 0 )
	{
		printk("The file is an empty file!\n");
		return -1;
	}
	
	iPos= 0;   
	while (1)
	{
		writeAddr = P70_PATCH_ADDR_START + j;

		//printk("j = %d\n, ", j);
		
		byteWork0[0] = (writeAddr >> 24) & 0xFF;
		byteWork0[1] = (writeAddr >> 16) & 0xFF;
		regAdd[0] =0xB3;
		regAdd[1] =byteWork0[0];
		regAdd[2] =byteWork0[1];
		ftk_write_reg(ftk_ts, &regAdd[0],3);
		mdelay(10);	
		
		byteWork1[0]= 0xB1;
		byteWork1[1] = (writeAddr >> 8) & 0xFF;
		byteWork1[2] = writeAddr & 0xFF;

		i = 0;
	
		//iReadLen = vfs_read(fp, buf, sizeof(buf), &iPos);
		//iReadLen = get_data_from_file(fp, buf);

		//printk("\niReadLen = %d!\n", iReadLen);
		{
			static char raw_data_buf[WRITE_CHUNK_SIZE*2];
			u32 k = 0;
			u32 iValue = 0;
	
			iReadLen = vfs_read(fp, raw_data_buf, sizeof(raw_data_buf), &iPos);

			iTemp = iPos;		
			//printk("get_data_from_file iPos = %d!\n", iTemp);


			for ( k  = 0; k < WRITE_CHUNK_SIZE; k++)
			{
				buf[k] = strnum_to_digtial( &raw_data_buf[k*2], 2 );
				//printk("%02X ",buffer[i]);
			}
			
			iReadLen = iReadLen / 2;
		}

		if ( iReadLen < sizeof(buf) )
		{
			while (i < iReadLen )
			{
				byteWork1[i+3] = buf[i]; 
				//printk("%02X, ", buf[i]);
				//if ( buf[i] != PatchData[j] )
					//printk("Read data is different: j=%d, data=%02X, patchData=%02X\n", j, buf[i], PatchData[j] );
				i++;
				j++;
			}
			ftk_write_reg(ftk_ts, &byteWork1[0],iReadLen+3);
			mdelay(10);	
		
		 	printk("It is the last group of data!j =%d\n", j);
		 
		 	break;
		}

		
		while (i<WRITE_CHUNK_SIZE)
		{
			byteWork1[i+3] = buf[i]; 
			//printk("%02X, ", buf[i]);
			//if ( buf[i] != PatchData[j] )
				//printk("Read data is different: j=%d, data=%02X, patchData=%02X\n", j, buf[i], PatchData[j] );
			i++;
			j++;
		}
		ftk_write_reg(ftk_ts, &byteWork1[0],WRITE_CHUNK_SIZE+3);
		mdelay(10);		

	//printk("\n---------------------------------\n ");

		
	}
	
	filp_close(fp, NULL);

	set_fs(fs);

	//mdelay(10);	
	
	return 0;
}


#endif


static int write_config_file(struct ftk_ts *ftk_ts)
{
	u8 regAdd[8];
	int b0_i = 0;
	
	regAdd[0] = 0xB0; // All writes below are 1 addr, 1 value
	for (b0_i = 0; b0_i < B0_COMMAND_WRITES; ++b0_i ) {
		regAdd[1] = B0_command_writes[b0_i].addr;
		regAdd[2] = B0_command_writes[b0_i].val;
		ftk_write_reg(ftk_ts, &regAdd[0], 3);
		mdelay(5);
	}

	regAdd[0] = 0xB3;
	regAdd[1] = 0xFF;
	regAdd[2] = 0xFF;
	ftk_write_reg(ftk_ts, &regAdd[0], 3);
	mdelay(5);

	regAdd[0] = 0xB1;
	regAdd[1] = 0xFC;
	regAdd[2] = 0x2F;
	regAdd[3] = 0xC0;
	ftk_write_reg(ftk_ts, &regAdd[0], 4);
	mdelay(5);

	regAdd[0] = 0xB0;
	regAdd[1] = 0x04;
	regAdd[2] = 0x20;
	ftk_write_reg(ftk_ts, &regAdd[0], 3);
	mdelay(5);


	regAdd[0]=0x81; //TS Sleep out
	regAdd[1]=0x00; 
	ftk_write_reg(ftk_ts, &regAdd[0],1);
	mdelay(5);
		


	regAdd[0]=0x8A; //Set Area Location
	regAdd[1]=0x00; //Touch screen size
	regAdd[2]=0x00;
	regAdd[3]=0x00; 	
	regAdd[4]=0xFF;  //Full screen is used
	regAdd[5]=0xFF;
	regAdd[6]=0xFF; 
	ftk_write_reg(ftk_ts, &regAdd[0],7);
	mdelay(5);	

	regAdd[0]=0x8B; //Set Area layer
	regAdd[1]=0x00; 
	regAdd[2]=0x00; 
	ftk_write_reg(ftk_ts, &regAdd[0],3);
	mdelay(5);

	regAdd[0]=0x8C; //Set Area Event
	regAdd[1]=0x00; 
	regAdd[2]=0x00;
	regAdd[3]=0x00; 	
	regAdd[4]=0x00; 
	regAdd[5]=0x0F;
	regAdd[6]=0xFF; 
	ftk_write_reg(ftk_ts, &regAdd[0],7);
	mdelay(5);

	regAdd[0]=0x8D; //Set Area Touches 
	regAdd[1]=0x00;
	regAdd[2]=0xFF;
	ftk_write_reg(ftk_ts, &regAdd[0],3);
	mdelay(5);

	regAdd[0]=0x83; //Set Area layer
	regAdd[1]=0x00; 
	ftk_write_reg(ftk_ts, &regAdd[0],2);
	mdelay(5);

	regAdd[0]=0x80; //Set Area layer
	regAdd[1]=0x00; 
	ftk_write_reg(ftk_ts, &regAdd[0],2);
	mdelay(5);

	regAdd[0]=0x81; //Set Area layer
	regAdd[1]=0x00; 
	ftk_write_reg(ftk_ts, &regAdd[0],2);
	mdelay(5);

}

static int init_ftk(struct ftk_ts *ftk_ts)
{
	u8 val[8];	
	u8 regAdd[7];
	int rc;	
	u8 patchConfig = 0x80;
	int b0_i = 0;


#if 1
	regAdd[0]=0xB0;
	regAdd[1]=0x00; 
	rc=ftk_read_reg(ftk_ts, regAdd, 2, val, 3);

	if (rc < 0) {
		printk(KERN_ERR " : i2c_transfer failed\n");
	}

	mdelay(5);		
	printk("Chip ID = %x %x %x\n" , val[0], val[1], val[2]);		

	printk(" Michael Hu 15Oct2011: Read Chip ID finished!\n");
#endif


#if 1	
	regAdd[0]=0x9E; //TS Soft Reset
	ftk_write_reg(ftk_ts, &regAdd[0],1);   
	mdelay(5);

	
		if( Is_file_exist() == 0 )
		{	
#ifdef LOAD_PATCH_FOR_FINGERTIPK
			if ( LoadPatchFromFile(ftk_ts) == 0)
#endif
			{
				LoadConfigFromFile(ftk_ts);
				mdelay(5);
		//		LoadConfigFromFile(ftk_ts);
		//		mdelay(5);
		//		LoadConfig(ftk_ts);
			}
		}
		
#ifdef LOAD_PATCH_FOR_FINGERTIPK
		else if (LoadPatch(ftk_ts) == 0)
#endif
	
		{
			LoadConfig(ftk_ts);
		}

	
	
    
    if(rc<0)
		printk("ftk Not Initialised\n");
	else
		printk("ftk Initialised\n");		
#endif
		
	return 0;
}

static void newdata(struct ftk_ts * ftkts, u8 id)
{
//	printk("id=%d, x=%d, y=%d  \n", id, ftkts->x, ftkts->y);
	
//	input_report_abs(ftkts->input_dev, ABS_MT_TRACKING_ID, id);
	input_report_abs(ftkts->input_dev, ABS_MT_POSITION_X, ftkts->x);
        input_report_abs(ftkts->input_dev, ABS_MT_POSITION_Y, ftkts->y);
	input_report_abs(ftkts->input_dev, ABS_MT_TOUCH_MAJOR, ftkts->z);
#if 0
	if (ftkts->z){
        input_report_abs(ftkts->input_dev, ABS_MT_TOUCH_MAJOR, 10);
   }
    else{
        input_report_abs(ftkts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	}
#endif
    input_mt_sync(ftkts->input_dev);  
}

static enum hrtimer_restart st_ts_timer_func(struct hrtimer *timer)
{
	struct ftk_ts *ftkts  = container_of(timer, struct ftk_ts, timer);
	queue_work(stmtouch_wq, &ftkts->work);
	return HRTIMER_NORESTART;
}

static irqreturn_t ts_interrupt(int irq, void *handle)
{
	struct ftk_ts *ftk_ts = handle;	
	disable_irq_nosync(ftk_ts->client->irq);	
	queue_work(stmtouch_wq, &ftk_ts->work);
	return IRQ_HANDLED;
}

int iPrintTimes = 0;  //add  by   ;

static void SendDataOut(struct ftk_ts *ftkts,   int x,  int y, int z, int EventID, int TouchID, int NumTouches )
{
	int i,j ;
	
		  if ((EventID == 0x03)||(EventID == 0x05))
		  { 
			//	printk("EventID=%d, IDj = %d, TouchID = %d, NumTouches = %d \n",EventID, IDj, TouchID, NumTouches);
				if ( EventID == 0x03 )
				{
						ID_Indx[IDj] = TouchID;   //Add New ID In
						IDj=IDj+1;
				}
				//printk("num1= %d \n",IDj);

							
				cor_xyz[TouchID][0] = x;
				cor_xyz[TouchID][1] = y;
				cor_xyz[TouchID][2] = z;  
	
				for( i = 0; i < IDj; i++ ) 
				{
				//	printk("IDj = %d; i = %d; \n", IDj, i );
					
				//        printk("ID_Indx[i] = %d, x = %d, y = %d \n", ID_Indx[i], cor_xyz[ID_Indx[i]][0], cor_xyz[ID_Indx[i]][1]);
					ftkts->x=cor_xyz[ID_Indx[i]][0];
					ftkts->y=cor_xyz[ID_Indx[i]][1];
					ftkts->z=cor_xyz[ID_Indx[i]][2];

					newdata(ftkts, ID_Indx[i]);
					
					//printk("EventID=%d, ID_Indx[%d] = %d, x = %d, y = %d \n",EventID, i, ID_Indx[i], cor_xyz[ID_Indx[i]][0], cor_xyz[ID_Indx[i]][1]);
				}
				input_sync(ftkts->input_dev);	
							
		 }	
	  else  if (EventID == 0x04) 
	  { 	

			for (j = 0; j < IDj; j++)
			{
			  if (ID_Indx[j]==TouchID)
			  {
				do 
				{
				  ID_Indx[j]=ID_Indx[j+1]; //update the Indx
				  j++;
				}while(j<10);				 
			   // j=IDj+1;
			   break;
			  } 						   
			}	 
			
			
			if ( IDj > 0 )
			  IDj = IDj - 1;			


#if 1
			if ( IDj== 0)
			{
			input_mt_sync(ftkts->input_dev);  	
				//input_sync(ftkts->input_dev);
			}
				
				for( i = 0; i < IDj; i++ ) 
				{
				//printk("EventID=%d, ID_Indx[%d] = %d, x = %d, y = %d \n",EventID, i, ID_Indx[i], cor_xyz[ID_Indx[i]][0], cor_xyz[ID_Indx[i]][1]);
					
					ftkts->x=cor_xyz[ID_Indx[i]][0];
					ftkts->y=cor_xyz[ID_Indx[i]][1];
					ftkts->z=cor_xyz[ID_Indx[i]][2];
	
					newdata(ftkts, ID_Indx[i]);
					
				}
#endif
	//input_mt_sync(ftkts->input_dev);
	input_sync(ftkts->input_dev);

		//printk("EventID=%d, IDj = %d, TouchID = %d \n",EventID, IDj, TouchID);
	  } 	
	 
}

static void ts_tasklet_proc(struct work_struct *work)
{  
//  struct stmpe32m28_ts *stmpe32m28ts = container_of(work, struct stmpe32m28_ts, work);
  struct ftk_ts *ftkts = container_of(work, struct ftk_ts, work);

  unsigned char data[129];        
  int x, y,z;
  int rc;
  u8 status;
  u8 regAdd;
  u8 writeCMD[3];
  u8 FoundFlag=0;
  u8 LeftEvent = 0;
  u8 FirstLeftEvent = 0;
  u8 EventNum=0;
  u8 NumTouches=0;
  u8 EventNum2=0;
  u8 TouchID, EventID;
  u8 i,j;  
  //u8 regAdd2[2];
  //u8 val[2];



	  writeCMD[0]=0xB0; //disable interrupt;
	  writeCMD[1]=0x06; 
	  writeCMD[2]=0x00; 
	  ftk_write_reg(ftkts, &writeCMD[0],3);
	//mdelay(1);

	  data[0]=0xB0;
	  data[1]=0x07;
	  rc=ftk_read_reg(ftkts, &data[0], 2, &status, 1);	  
	  ftkts->pendown = 0;  
 	EventNum=0;

do
{
	regAdd=0x85;  
	data[0]=0;	
	rc=ftk_read_reg(ftkts, &regAdd, 1, data, 8);	

	FirstLeftEvent=data[7+EventNum*8]&0x0F;				  
	NumTouches=(data[1+EventNum*8]&0xF0)>>4;  
	TouchID=data[1+EventNum*8]&0x0F;  
	EventID=data[EventNum*8]&0x0F;	
	x=((data[4+EventNum*8]&0xF0)>>4)|((data[2+EventNum*8])<<4); 	 
	y=((data[4+EventNum*8]&0x0F)|((data[3+EventNum*8])<<4));
	z=data[5+EventNum*8];  


//	printk("FirstLeftEvent=%d, EventID=%d, TouchID = %d, NumTouches = %d, x=%d, y=%d \n", FirstLeftEvent, EventID, TouchID, NumTouches, x, y);
	
	SendDataOut(ftkts,	x,	y, z,  EventID, TouchID, NumTouches);
}while( FirstLeftEvent > 0);


	  

  	  writeCMD[0]=0xB0; //disable interrupt;
	  writeCMD[1]=0x06; 
	  writeCMD[2]=0x50; 
	  ftk_write_reg(ftkts, &writeCMD[0],3);
  	  mdelay(1);
	  
  if (!ftkts->irq) {
    hrtimer_start(&ftkts ->timer, ktime_set(0, 10000000), HRTIMER_MODE_REL);      
  }
  else
  {
    enable_irq(ftkts->client->irq);  
  }
}

static int stm_ts_probe(struct i2c_client *client, const struct i2c_device_id *idp)
{
	struct ftk_ts *ftk_ts = NULL;
	struct ftk_i2c_platform_data *pdata;
	int ret = 0;
	int err = 0;
	int i;

	printk( "++ftkts_probe:04Nov 09:13  2011\n");

	#if 1
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
		printk(KERN_ERR " :err = EIO! \n");
		err = EIO;
		goto fail;
	}
	
	ftk_ts = kzalloc(sizeof(struct ftk_ts), GFP_KERNEL);
	if (!ftk_ts){
		printk(KERN_ERR " :err = ENOMEM! \n");
		err = ENOMEM; 
		goto fail;
	}
	
	INIT_WORK(&ftk_ts->work, ts_tasklet_proc);	
	
	ftk_ts->client=client;	
	i2c_set_clientdata(client,ftk_ts);
        #endif

//begin add by  ; 02Aug2011;
       #if 1
		pdata = client->dev.platform_data;
		
		if (pdata)
		{
			printk(" :Get pdata! \n");
			ftk_ts->power = pdata->power;  //remove for test the only power;
		}
		else
		{
			printk(" :not get pdata! \n");
		}
		
		if (ftk_ts->power) {
			printk("Power affect tested!\n");
			ret = ftk_ts->power(1);
			
			pdata->power(1);
			
			if (ret < 0) {
				pr_err(" :ftk_probe power on failed\n");
				goto fail;
			}
			else
				printk(" :ftk_probe power on successfully! \n");
		} 
		
        #else
	printk("Power affect tested!\n");
		
	pdata = client->dev.platform_data;
	
	if (pdata)
	{
		printk(" :Get pdata! \n");
		pdata->power(1);
		if (ret < 0) {
			pr_err(" :ftk_probe power on failed\n");
			goto fail;
		}
		else
			printk(" :ftk_probe power on successfully! \n");

	}
	else
	{
		printk(" :not get pdata! \n");
	}
	
//end add by  ; 02Aug2011;
          #endif


#if 1
			
	ftk_ts->dev = &ftk_ts->client->dev;
	ftk_ts->input_dev = input_allocate_device();
	ftk_ts->input_dev->dev.parent = &client->dev;
	if(!ftk_ts->input_dev){
		err = ENOMEM;
		goto fail;
	}
	ftk_ts->input_dev->name = "ftk";


	ftk_ts->input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ftk_ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	set_bit(EV_SYN, ftk_ts->input_dev->evbit);	
	set_bit(EV_KEY,ftk_ts->input_dev->evbit);
	set_bit(BTN_TOUCH,ftk_ts->input_dev->keybit);
	set_bit(BTN_2,ftk_ts->input_dev->keybit);	
	set_bit(EV_ABS,ftk_ts->input_dev->evbit);
	
	input_set_abs_params(ftk_ts->input_dev, ABS_X, X_AXIS_MIN, X_AXIS_MAX, 0, 0);
	input_set_abs_params(ftk_ts->input_dev, ABS_Y, Y_AXIS_MIN, Y_AXIS_MAX, 0, 0);
	input_set_abs_params(ftk_ts->input_dev, ABS_PRESSURE, PRESSURE_MIN, PRESSURE_MAX, 0, 0);
	input_set_abs_params(ftk_ts->input_dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0);
	input_set_abs_params(ftk_ts->input_dev, ABS_MT_TOUCH_MAJOR, PRESSURE_MIN, PRESSURE_MAX, 0, 0);
	input_set_abs_params(ftk_ts->input_dev, ABS_MT_WIDTH_MAJOR, PRESSURE_MIN, PRESSURE_MAX, 0, 0);	
	input_set_abs_params(ftk_ts->input_dev, ABS_MT_POSITION_X, X_AXIS_MIN, X_AXIS_MAX, 0, 0);
	input_set_abs_params(ftk_ts->input_dev, ABS_MT_POSITION_Y, Y_AXIS_MIN, Y_AXIS_MAX, 0, 0);
	
	err=input_register_device(ftk_ts->input_dev);
	if(err) {
		goto fail;
	}
  
	err = init_ftk(ftk_ts);    
	if(err) {
		goto fail;
	}		
	
	ftk_ts->irq=client->irq;	

	#if 1
		
	if (!ftk_ts->irq) {
		hrtimer_init(&ftk_ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ftk_ts->timer.function = st_ts_timer_func;
		hrtimer_start(&ftk_ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		printk(KERN_INFO " : hrtimer_start\n");
	}
	else
	{
		 printk(KERN_INFO " :ftk_ts->irq = %d! \n", ftk_ts->irq);

		if(request_irq(ftk_ts->irq, ts_interrupt, IRQF_TRIGGER_FALLING, client->name, ftk_ts)) {
			printk(KERN_INFO " :request_irq fail! \n");
			err = -EBUSY;
			goto fail;
		}				
		printk(KERN_INFO " :request_irq successfully! \n");
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
		ftk_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		ftk_ts->early_suspend.suspend = stm_ts_early_suspend;
		ftk_ts->early_suspend.resume =stm_ts_late_resume;
		register_early_suspend(&ftk_ts->early_suspend);
#endif

	#else
	printk(KERN_INFO " :remove interrupt and timer handler \n");
	#endif
#endif
	
	return 0;
fail:
	if(ftk_ts) {
		if(ftk_ts->input_dev)
			input_free_device(ftk_ts->input_dev);
		kfree(ftk_ts);
	}
	printk("--ftkts_probe ret=%d\n", err);
	return err;
}

static int stm_ts_remove(struct i2c_client *client)
{
//	struct ftk_ts *priv = dev_get_drvdata(&client->dev);


        struct ftk_ts *ts = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&ts->early_suspend);
#endif

	printk("Hello from Remove: 12Oct2011 \n"); 

	
//	if (priv->irq) 
//		free_irq(priv->irq, priv);

	if (ts->irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	
	input_unregister_device(ts->input_dev);
	//kfree(priv);		
	kfree(ts);
	//dev_set_drvdata(&client->dev, NULL);
	return 0;
}

static int stm_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	u8 regAdd[2];

	struct ftk_ts  *ts = i2c_get_clientdata(client);
	printk( "enter stm_ts_suspend\n");

	if (ts->irq)
	{
		printk( "enter disable_irq\n");
		disable_irq(client->irq);
	}
	else
	{
		printk( "enter hrtimer_cancel\n");
		hrtimer_cancel(&ts->timer);
	}
	
	regAdd[0]=0x80; //TS Sleep in
	regAdd[1]=0x00; 
	ret = ftk_write_reg(ts, &regAdd[0],1);
	mdelay(5);


	if (ret < 0)
		printk( "stm_ts_suspend: i2c_smbus_write_byte_data failed\n");

	return 0;
}

static int stm_ts_resume(struct i2c_client *client)
{
	int ret;
	u8 regAdd[2];
	struct ftk_ts  *ts = i2c_get_clientdata(client);
	
	printk( "enter stm_ts_resume\n");


	if (ts->irq)
	{
		printk( "enter disable_irq\n");
		enable_irq(client->irq);
	}
	else
	{
		printk( "enter hrtimer_cancel\n");		
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

	regAdd[0]=0x81; //TS Sleep out
	regAdd[1]=0x00; 
	ret = ftk_write_reg(ts, &regAdd[0],1);
	mdelay(5);

	if (ret < 0)
		printk(KERN_ERR "stm_ts_suspend: i2c_smbus_write_byte_data failed\n");


	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void stm_ts_early_suspend(struct early_suspend *h)
{
	struct ftk_ts *ts;
	ts = container_of(h, struct ftk_ts, early_suspend);
	stm_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void stm_ts_late_resume(struct early_suspend *h)
{
	struct ftk_ts *ts;
	ts = container_of(h, struct ftk_ts, early_suspend);
	stm_ts_resume(ts->client);
}
#endif




static const struct i2c_device_id stm_ts_id[] = {
	{ "ftk", 0 },
	{ }
};

static struct i2c_driver stm_ts_driver = {
	.driver = {
		.name = "ftk",
	},
	.probe = stm_ts_probe,
	.remove = stm_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = stm_ts_suspend,
	.resume = stm_ts_resume,
#endif
	.id_table = stm_ts_id,
};

static int __init stm_ts_init(void)
{
	stmtouch_wq = create_singlethread_workqueue("stmtouch_wq");
	if (!stmtouch_wq)
		return -ENOMEM;
		
	return i2c_add_driver(&stm_ts_driver);
}

static void __exit stm_ts_exit(void)
{
	
	i2c_del_driver(&stm_ts_driver);
	if (stmtouch_wq)
		destroy_workqueue(stmtouch_wq); 
}

MODULE_DESCRIPTION("STM MultiTouch IC Driver");
MODULE_AUTHOR("Bela Somaiah");
MODULE_LICENSE("GPL");

module_init(stm_ts_init);
module_exit(stm_ts_exit);
