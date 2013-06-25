#ifndef _GPS_H_
#define _GPS_H_

#define CTL_GPS_BAUD	0x30
#define CTL_GPS_OUTMODE	0x31	/*gps信息的输出模式*/


#define GPS_OUTMODE_TRIGGER	0x01	/*秒触发输出*/
#define GPS_OUTMODE_GPRMC	0x02
#define GPS_OUTMODE_GPGSV	0x04
#define GPS_OUTMODE_BD		0x08

#define GPS_OUTMODE_ALL		0xFFFFFFFF	/*全数据输出*/


/*
北斗更新的操作状态结果码，分为几个部分

0000 0000-0FFF FFFF  包序号


*/


#define BDUPG_RES_UART_OK		(0x10000000)	/*没有错误升级成功-升级成功*/
#define BDUPG_RES_UART_READY	(0x1000FFFF)	/*串口更新就绪*/


#define BDUPG_RES_USB_OK		(0x20000000)	/*没有错误升级成功-升级成功*/

#define BDUPG_RES_USB_FILE_ERR	(0x2000FFFC)	/*更新文件格式错误*/
#define BDUPG_RES_USB_NOFILE	(0x2000FFFD)	/*更新文件不存在*/
#define BDUPG_RES_USB_WAITUSB	(0x2000FFFD)	/*更新文件不存在*/
#define BDUPG_RES_USB_NOEXIST	(0x2000FFFE)	/*u盘不存在*/
#define BDUPG_RES_USB_READY		(0x2000FFFF)	/*u盘就绪*/


#define BDUPG_RES_USB_MODULE_H	(0x20010000)	/*模块型号高16bit*/
#define BDUPG_RES_USB_MODULE_L	(0x20020000)	/*模块型号低16bit*/
#define BDUPG_RES_USB_FILE_VER	(0x20030000)	/*软件版本*/


#define BDUPG_RES_THREAD	(0xFFFFFFFE)	/*创建升级线程失败-创建更新失败*/
#define BDUPG_RES_RAM		(0xFFFFFFFD)	/*分配RAM失败*/
#define BDUPG_RES_TIMEOUT	(0xFFFFFFFC)	/*超时失败*/




//=============================
typedef struct gps_rmc { 
char utc_year;
char utc_mon;
char utc_day;
char utc_hour;
char utc_min;
char utc_sec;
char status; 
float latitude_value; 
char latitude; 
float longtitude_value; 
char longtitude; 
float speed; 
float azimuth_angle; 
} GPS_RMC; 

typedef  struct  GPS_STATUS
{
   u8  Position_Moule_Status;  // 1: BD   2:  GPS   3: BD+GPS    定位模块的状态
   u8  Antenna_Flag;//显示提示开路 
   u8  Raw_Output;   //  原始数据输出  
}GPSSTATUS;

typedef struct  Gps_Abnormal
{
   u32   current_datacou;  // 当前计数值
   u32   last_datacou;       // 上次计数值
   u16   no_updateTimer; // 没有数据更新定时器
   u16   GPS_Rst_counter;
   u8     Reset_gps;           // GPS timer   
  
}GPS_ABNORMAL;
// -----  GPS   App  Related  -------
#define	GPSRX_SIZE	130

extern  uint8_t							flag_bd_upgrade_uart ;
extern  GPSSTATUS  GpsStatus;
extern  GPS_ABNORMAL   Gps_Exception;  

//============================
void gps_init( void );
extern void gps_baud( int baud );
extern void  gps_mode(u8 *str) ;
extern void  GpsIo_Init(void);
extern rt_err_t gps_onoff( uint8_t openflag );
extern void  GPS_Abnormal_process(void);  
void thread_gps_upgrade_uart( void* parameter );
void thread_gps_upgrade_udisk( void* parameter );


#endif

