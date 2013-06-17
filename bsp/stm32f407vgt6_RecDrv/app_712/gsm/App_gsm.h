/*
     APP_GSM.H 
*/

#ifndef   _APP_GSM
#define  _APP_GSM  

#include <rtthread.h> 
#include <rthw.h>
#include "stm32f4xx.h"

#include  <stdlib.h>
#include  <stdio.h>
#include  <string.h>
#include "App_moduleConfig.h"


//----------- ModuleStatus----------
#define Status_GPS			0x01    // Bit 0:   1 ��λ   0 ����λ
#define Status_GPRS 		0x02    // Bit 1:   1 ����   0 �Ͽ�
#define Status_Battery      0x04    // Bit 2:   1 �õ�� 0 ����� 
#define Status_Pcheck       0x08    // Bit 3:   1 �ٶ�����ϵ��У���   0  δ��У��


// ------------  GSM  Device  define ------------
#define                     APN                         0x01
#define                     main_socket           0x02
#define                     aux_socket             0x03
#define                     isp_socket              0x04
#define                     DNSR1                     0x05
#define                     DNSR2                     0x06

#define                     power_on                0x11 
#define                     at_init                     0x12
#define                     dial_gprs                0x13 

#define                     query_online          0x21
#define                     send_gprsdata       0x22



typedef struct  _SOCKET
{
    u8 ip[4];
    u16  port;	

}SOCKET;




#define GSM_RAWINFO_SIZE 9000
ALIGN(RT_ALIGN_SIZE)
extern  uint8_t					GSM_rawinfo[GSM_RAWINFO_SIZE];
extern struct rt_messagequeue	mq_GSM; 


extern u8 	ModuleStatus;   //����״̬ 
extern u8       Mocule_Data_Come; // ģ���յ����� 



/*
    Ӧ�����  
*/
extern  u8 DataLink_Online;   //  DataLink  ���߱�־    ��ǰ��GPRS_Online
extern u8  DataLink_EndFlag;         // Close_DataLink
extern  u8 DataLink_EndCounter; 
extern u8  PositionInfo_sdFlag;     // ���Ͷ�λ��Ϣ��־
extern u8  Datalink_close;  //�ҶϺ��ٵ�½
extern u8  Current_UDP_sd;   // ��ʱ�ϱ� ��־λ



extern u8  COPS_Couter;             // COPS  ���ش���

extern u8  CSQ_counter;
extern u8  CSQ_Duration;    //��ѯCSQ �Ķ�ʱ��� 
extern u8  CSQ_flag;
extern u8  ModuleSQ;  //GSM ģ���ź�ǿ����ֵ
extern u8  ModuleStatus;   //����״̬ 

extern u8   Light;
extern u8   Send_DataFlag;  // ����GSM data Flag  
extern u8	LinkNum;  // ͨ����·	0	 LINK 1    1   LINK     
extern u8   Receive_DataFlag;// ��������  
extern  u8    GSM_HEX[1024];  
extern  u16   GSM_HEX_len;    

/*
    Ӧ����غ���
*/

extern u8      DataLink_Status(void);
extern void   DialLink_TimeOut_Clear(void);    
extern void   DialLink_TimeOut_Enable(void);      
extern u8     Close_DataLink(void);
extern  void  DataLinkOK_Process(void);    // �������ӳɹ��������״̬����

extern u8   PositionSD_Enable(void);
extern u8   PositionSD_Disable(void); 
extern u8   PositionSD_Status(void);
extern u8   Stop_Communicate(void);
extern void Gsm_rxAppData_SemRelease(u8* instr, u16 inlen, u8 link_num);


/*
         RT      ��� 
*/
extern void _gsm_startup(void);

#endif 

