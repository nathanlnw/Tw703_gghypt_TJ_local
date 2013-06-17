/*
     APP_485.H 
*/

#ifndef   _APP_485
#define  _APP_485

#include <rtthread.h> 
#include <rthw.h>
#include "stm32f4xx.h"

#include  <stdlib.h>
#include  <stdio.h>
#include  <string.h>
#include "App_moduleConfig.h"


//---------  ���շ���״̬  DB  ----------
#define  Photo_transIdle  0
#define  trans_fail       1
#define  trans_succeed    2
#define  takePhoto_fail   3

//----------- MultiTake State ----------
#define Take_idle      0
#define Take_Success   1
#define Take_Fail      2


//------  485  Rx State -------- 
#define    IDLE_485           0
#define    CAMERA_Related     1
#define    LARGE_LCD           3



/*
                  Photo take Related                          
*/
//-------------------  Multi Take ------------------
 //  �ȸ�·����һ���¼�����ճɹ���ʧ�ܵĽ����Ȼ������ϴ�
 #define   Max_CameraNum   4    

typedef struct
{
  u8 Taking;      // ��·����ͷ������              Ĭ��Ϊ   0   ʹ��Ϊ   1
  u8 Transfering; // ͼƬ������                   Ĭ��Ϊ   0    ʹ��Ϊ   1
  u8 CurretCameraNum; //��ǰ���յ�����ͷ���
  u8 TakeResult[Max_CameraNum];   // ÿ·����ͷ���ս��    Ĭ�ϳ�ʼ��Ϊ  0     ���ճɹ�Ϊ  1  ����ʧ��Ϊ 2
  u8 Take_retry;      // ��·�������Դ���
  u8 Take_success_counter;  // ��¼�ܹ����ճɹ��˶���ͨ��
}_MultiTake;  // ��·����ͷ���ղ����Ĵ���

typedef struct 
{
	 u8     camera_running;     //���ս����б�־
	 u16    timeoutCounter;     //������ʱ������
	 u16	block_counter;	    //�������ݰ��ļ�����
	 u8	    status;	  //  ���չ���״̬
	 u8     OperateFlag;  // ���չ����в���״̬��ʶ �� ��camera ������ set 1�� camera ��Ӧ�����  0
	 u8     create_Flag;  // �Ƿ񴴽���ͼƬ�ļ���־ ��     1 : ��Ҫ�����±�־     0:   ����Ҫ����
} Camera_state;

typedef struct  _VOICE_DEV
{
 u8  Work_State;  // ���������ݴ���״̬
 u8  CMD_Type;    //  0  ֹͣ�ɼ�   1: �ɼ���   2: ��������
 u8  Poll_Enble;  // ʹ����ѯ��־
 u8  Sd_Timer;    // ���Ͷ�ʱ��   
 u8  info_sdFlag; //  ��������������Ϣ       1  ������Ϣ  2 ¼������
 u16 Rec_Dur;     // ¼������ʱ�� 
 u16 Rec_Counter; //  ¼����ʱ��
 u8  Rec_runFlag; // �Ƿ�һֱ¼��
 u8  SaveOrNotFlag; // �Ƿ񱣴� 
 u8  Centre_RecordFlag; // ����¼����־λ
 u8  Voice_FileOperateFlag;//    ���������ļ�������־λ   0  Ϊδ�����ļ�����  1 ��ʼ���ļ����в���
 u32 Voice_FileSize;//  �ļ���С
 u32 Voice_PageCounter;
 u8  FileName[20]; // �洢�ļ�����   
 u8  Play_info[100]; // ����������Ϣ 
 u8  Voice_Reg[512]; // ��������Buffer
}VOICE_DEV;




extern  VOICE_DEV Dev_Voice;  
/*
                  �������  
*/
 extern  _MultiTake     MultiTake;	  //  ��·����״̬λ 
 extern u8  SingleCamera_TakeRetry; // ��·����ͷ����ʱ�����Ĵ�������
 extern Camera_state CameraState; 
 extern u8    TX_485const_Enable;   // ʹ�ܷ��ͱ�־λ  
 extern u8 	  last_package; // �������һ����ʶ
extern  Camera_state CameraState;  










//-----------------------------------------
extern void  Send_const485(u8  flag); 
extern u8     Camera_Take_Enable(void);
extern  u8    Check_MultiTakeResult_b4Trans(void);
extern void   Init_Camera(void);
extern void   End_Camera(void);
extern  u8    Start_Camera(u8  CameraNum);
extern  void  MultiTake_Start(void);
extern  void  MultiTake_End(void); 
extern u8     Camera_Take_Enable(void);
extern  void  Voice_Dev_Init(void);
extern  void  Voice_Dev_Rxprocess(void);
extern  void  Voice_Dev_Txprocess(void);  
extern  void  Camra_Take_Exception(void);
extern int str2ipport(char *buf, u8 *ip, u16 *port); 



#endif 
