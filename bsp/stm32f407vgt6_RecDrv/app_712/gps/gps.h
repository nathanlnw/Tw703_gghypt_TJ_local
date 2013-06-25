#ifndef _GPS_H_
#define _GPS_H_

#define CTL_GPS_BAUD	0x30
#define CTL_GPS_OUTMODE	0x31	/*gps��Ϣ�����ģʽ*/


#define GPS_OUTMODE_TRIGGER	0x01	/*�봥�����*/
#define GPS_OUTMODE_GPRMC	0x02
#define GPS_OUTMODE_GPGSV	0x04
#define GPS_OUTMODE_BD		0x08

#define GPS_OUTMODE_ALL		0xFFFFFFFF	/*ȫ�������*/


/*
�������µĲ���״̬����룬��Ϊ��������

0000 0000-0FFF FFFF  �����


*/


#define BDUPG_RES_UART_OK		(0x10000000)	/*û�д��������ɹ�-�����ɹ�*/
#define BDUPG_RES_UART_READY	(0x1000FFFF)	/*���ڸ��¾���*/


#define BDUPG_RES_USB_OK		(0x20000000)	/*û�д��������ɹ�-�����ɹ�*/

#define BDUPG_RES_USB_FILE_ERR	(0x2000FFFC)	/*�����ļ���ʽ����*/
#define BDUPG_RES_USB_NOFILE	(0x2000FFFD)	/*�����ļ�������*/
#define BDUPG_RES_USB_WAITUSB	(0x2000FFFD)	/*�����ļ�������*/
#define BDUPG_RES_USB_NOEXIST	(0x2000FFFE)	/*u�̲�����*/
#define BDUPG_RES_USB_READY		(0x2000FFFF)	/*u�̾���*/


#define BDUPG_RES_USB_MODULE_H	(0x20010000)	/*ģ���ͺŸ�16bit*/
#define BDUPG_RES_USB_MODULE_L	(0x20020000)	/*ģ���ͺŵ�16bit*/
#define BDUPG_RES_USB_FILE_VER	(0x20030000)	/*����汾*/


#define BDUPG_RES_THREAD	(0xFFFFFFFE)	/*���������߳�ʧ��-��������ʧ��*/
#define BDUPG_RES_RAM		(0xFFFFFFFD)	/*����RAMʧ��*/
#define BDUPG_RES_TIMEOUT	(0xFFFFFFFC)	/*��ʱʧ��*/




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
   u8  Position_Moule_Status;  // 1: BD   2:  GPS   3: BD+GPS    ��λģ���״̬
   u8  Antenna_Flag;//��ʾ��ʾ��· 
   u8  Raw_Output;   //  ԭʼ�������  
}GPSSTATUS;

typedef struct  Gps_Abnormal
{
   u32   current_datacou;  // ��ǰ����ֵ
   u32   last_datacou;       // �ϴμ���ֵ
   u16   no_updateTimer; // û�����ݸ��¶�ʱ��
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

