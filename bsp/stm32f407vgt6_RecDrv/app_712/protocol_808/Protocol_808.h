/*
     Protocol 808 .h
*/
#ifndef  _PROTOCOL808
#define   _PROTOCOL808

#include <rthw.h>
#include <rtthread.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>
#include "App_moduleConfig.h" 
#include "gps.h"

 // ----   Media  Trans state ---
#define   MEDIA
#define  enable                1
#define  disable               0
#define  other                  2
#define  transfer             3 



//------------  GPS function------
#define  INIT         1
#define  PROCESS      0

#define  K_adjust_Duration       20                //  У��Kֵ����Ҫʱ��  


// -------  ISP  Address   -------------
#define    ISP_APP_Addr                        0x6400    //  512*50  Page
#define    ISP_RAM_Addr                       0x1400   // 512*10 Page
 

//------- ͨ��״̬---------
#define CallState_Idle 		        0
#define CallState_Ring 		        1
#define CallState_Connected	        2
#define CallState_Hangup		    4
#define CallState_rdytoDialLis      5
#define CallState_Dialing           8   //���в���


//-----------------�г���¼��  ��¼A ������ --------------------
// 1. ���������� 
#define  A_Up_DrvInfo          0x01        //  ��ǰ��ʻ����Ϣ
#define  A_Up_RTC              0x02        //  �ɼ���¼�ǵ�ʵʱʱ��
#define  A_Up_Dist             0x03        //  �ɼ����
#define  A_Up_PLUS             0x04        //  �ɼ���¼������ϵ��
#define  A_Up_AvrgMin          0x05        //  ÿ����ƽ���ٶȼ�¼
#define  A_Up_VechInfo         0x06        //  ������Ϣ
#define  A_Up_Doubt            0x07        //  �¹��ɵ�����
#define  A_Up_N2daysDist       0x08        //  �ɼ����2����������ۼ���ʻ���
#define  A_Up_N2daysSpd        0x09        //  �ɼ����2���������ڵ���ʻ�ٶ����� 
#define  A_Up_Tired            0x11        //  ƣ�ͼ�ʻ��¼   

// 2. �´����� 
#define  A_Dn_DrvInfo          0x81        //  ���ü�ʻԱ��Ϣ
#define  A_Dn_VehicleInfo      0x82        //  ���ó�����Ϣ
#define  A_Dn_RTC              0xC2        //  ���ü�¼��ʱ��
#define  A_Dn_Plus             0xC3        //  �����ٶ�����ϵ��

// 3.  ���䷽ʽ
#define   Trans_Serial         0           //  ���ڷ�ʽ����
#define   Trans_WireLess       1           //  ���ߴ���


// 4. 808 Э��   ��ϢID
#define   MSG_0x0001           0x0001      // �ն�ͨ��Ӧ��
#define   MSG_0x8001           0x8001      // ƽ̨ͨ��Ӧ��
#define   MSG_0x0002           0x0002      // �ն����� 
#define   MSG_0x0100           0x0100      // �ն�ע��
#define   MSG_0x8100           0x8100      // �ն�ע��Ӧ��
#define   MSG_0x0101           0x0101      // �ն�ע��
#define   MSG_0x0102           0x0102      // �ն˼�Ȩ
#define   MSG_0x8103           0x8103      // �����ն˲��� 
#define   MSG_0x8104           0x8104      // ��ѯ�ն˲���
#define   MSG_0x0104           0x0104      // ��ѯ�ն˲���Ӧ��
#define   MSG_0x8105           0x8105      // �ն˿���
#define   MSG_0x0200           0x0200      // λ����Ϣ�㱨 
#define   MSG_0x8201           0x8201      // λ����Ϣ��ѯ
#define   MSG_0x0201           0x0201      // λ����Ϣ��ѯӦ�� 
#define   MSG_0x8202           0x8202      // ��ʱλ�ø��ٿ���
#define   MSG_0x8300           0x8300      // �ı���Ϣ�·�
#define   MSG_0x8301           0x8301      // ---�¼�����
#define   MSG_0x0301           0x0301      // ---�¼�����
#define   MSG_0x8302           0x8302      // ---�����·�
#define   MSG_0x0302           0x0302      // ---����Ӧ��
#define   MSG_0x8303           0x8303      // ---��Ϣ�㲥�˵�����
#define   MSG_0x0303           0x0303      // ---��Ϣ�㲥 ȡ��
#define   MSG_0x8304           0x8304      // ---��Ϣ����
#define   MSG_0x8400           0x8400      // ---�绰�ز�
#define   MSG_0x8401           0x8401      // ---���õ绰��
#define   MSG_0x8500           0x8500      // ��������
#define   MSG_0x0500           0x0500      // ��������Ӧ��
#define   MSG_0x8600           0x8600      // ����Բ������
#define   MSG_0x8601           0x8601      // ɾ��Բ������
#define   MSG_0x8602           0x8602      // ���þ�������
#define   MSG_0x8603           0x8603      // ɾ����������
#define   MSG_0x8604           0x8604      // ���ö��������
#define   MSG_0x8605           0x8605      // ɾ�����������
#define   MSG_0x8606           0x8606      // ����·��
#define   MSG_0x8607           0x8607      // ɾ��·��
#define   MSG_0x8700           0x8700      // �г���¼�����ݲɼ�����
#define   MSG_0x0700           0x0700      // ��ʻ��¼�������ϴ�
#define   MSG_0x8701           0x8701      // ��ʻ��¼�ǵ����´�
#define   MSG_0x0701           0x0701      // ---�����˵��ϱ�
#define   MSG_0x0702           0x0702      // ��ʻԱ�����Ϣ�ɼ��ϱ�
#define   MSG_0x0800           0x0800      // ---��ý���¼���Ϣ�ϴ�
#define   MSG_0x0801           0x0801      // ---��ý�������ϴ�
#define   MSG_0x8800           0x8800      // ---��ý�������ϴ�Ӧ��
#define   MSG_0x8801           0x8801      // ����ͷ������������
#define   MSG_0x8802           0x8802      // �洢��ý�����ݼ���
#define   MSG_0x0802           0x0802      // �洢��ý�����ݼ���Ӧ��
#define   MSG_0x8803           0x8803      // �洢��ý�������ϴ�
#define   MSG_0x8804           0x8804      // ¼����ʼ���� 
#define   MSG_0x8900           0x8900      // ��������͸��
#define   MSG_0x0900           0x0900      // ��������͸��
#define   MSG_0x0901           0x0901      // ����ѹ���ϱ�
#define   MSG_0x8A00           0x8A00      // ƽ̨RSA��Կ
#define   MSG_0x0A00           0x0A00      // �ն�RSA��Կ

//--------  ���Ͱ��ķ�ʽ  ------
#define   Packet_Normal         0        //  ������     
#define   Packet_Divide         1        //  �ְ�

#define   SpxBuf_Size       512
#define   SpxGet_Size       100


//---------  ISP 	���   ---
#define  ISPACK_92             0x92
#define  ISPinfoACK_94         0x94
#define  ISPoverACK_96         0x96

typedef struct _A_AckFlag 
{
	// 1. ���������� 
u8  A_Flag__Up_Ver_00H             ;// 0x00   1     //  ��¼��ִ�б�׼�汾��
u8  A_Flag_Up_DrvInfo_01H          ;// 0x01   2     //  ��ǰ��ʻ����Ϣ
u8  A_Flag_Up_RTC_02H              ;// 0x02   3     //  �ɼ���¼�ǵ�ʵʱʱ��
u8  A_Flag_Up_Dist_03H             ;//0x03    4    //  �ɼ����
u8  A_Flag_Up_PLUS_04H             ;//0x04    5    //  �ɼ���¼���ٶ�����ϵ��
u8  A_Flag_Up_VechInfo_06H         ;//0x06    6    //  ������Ϣ
u8  A_Flag_Up_SetInfo_08H          ;//0x08    7    //  ״̬������Ϣ
u8  A_Flag_Up_DevID_16H            ;//0x16    8    //  ��¼��ΨһID
u8  A_Flag_Up_AvrgSec_09H          ;//0x09    9    //  ÿ����ƽ���ٶȼ�¼
u8  A_Flag_Up_AvrgMin_05H          ;//0x05    10    //  ÿ����ƽ���ٶȼ�¼
u8  A_Flag_Up_Posit_13H            ;//0x13    11    //  ָ��λ����Ϣ��¼
u8  A_Flag_Up_Doubt_07H            ;//0x07    12    //  �¹��ɵ�����
u8  A_Flag_Up_Tired_11H            ;//0x11    13    //  ƣ�ͼ�ʻ��¼
u8  A_Flag_Up_LogIn_10H            ;//0x10    14    //  ����ǳ���¼
u8  A_Flag_Up_Powercut_14H         ;//0x14    15    //  �ⲿ�����¼
u8  A_Flag_Up_SetMdfy_15H          ;//0x15    16    //  �����޸ļ�¼
	
	// 2. �´����� 
u8  A_Flag_Dn_DrvInfo_82H          ;//0x82    17    //  ���ó�����Ϣ
u8  A_Flag_Dn_SetupDate_83H        ;//0x83    18    //  ���ó�װ����
u8  A_Flag_Dn_Satus_84H            ;//0x84    19    //  ����״̬����Ϣ
u8  A_Flag_Dn_RTC_C2H              ;//0xC2    20    //  ���ü�¼��ʱ��
u8  A_Flag_Dn_Plus_C3H             ;//0xC3    21    //  �����ٶ�����ϵ��    
}A_AckFlag;

typedef struct _GPRMC_PRO
{
  void (*Time)(u8 *tmpinfo, u8 hour, u8 min , u8 sec);       
  void (*Status)(u8 *tmpinfo);     
  void (*Latitude)(u8 *tmpinfo);
  void (*Latitude_NS)(u8 *tmpinfo);  // S ��γ  N  ��γ
  void (*Longitude)(u8 *tmpinfo);
  void (*Longitude_WE)(u8 *tmpinfo); // E ����  W  ����
  void (*Speed)(u8 *tmpinfo,u8 Invalue,u8 Point);
  void (*Direction)(u8 *tmpinfo,u8 Invalue,u8 Point);
  void (*Date)(u8 *tmpinfo,u8 fDateModify, u8 hour, u8 min , u8 sec); 

}GPRMC_PRO; 

typedef struct _Position
{
  u8  latitude_BgEnd[4];
  u8  longitude_BgEnd[4];
}POSIT;


typedef struct _TCP_ACKFlag
{
u8  f_CentreCMDack_0001H;
u8  f_CentreCMDack_resualt;
u8  f_CurrentPosition_0201H;  
u8  f_CurrentEventACK_0301H; //  �¼�����
u8  f_MsgBroadCast_0303H;//  ��Ϣ�㲥/ ȡ��
u8  f_MediaIndexACK_0802H; //  ��ý��Ӧ��      0  idle  1  ��ѯͼƬ   2  ��ѯ��Ƶ  3 ��ѯ��Ƶ
u8  f_QueryEventCode;  //  0  ƽ̨�·�����   1 ��ʱ����   2  ���ٱ������� 3  ��ײ����   
u8  f_SettingPram_0104H;
u8  f_DriverInfoSD_0702H; //   ��ʻԱ �����Ϣ�ɼ��ϱ�
u8  f_Worklist_SD_0701H; //   �����˵��ϴ� 
     //  -----  ������չ----
u8  f_BD_Extend_7F02H; // ������Ϣ��ѯӦ��
u8  f_BD_Extend_7F00H;// ��չ�ն��Ϸ�ָ��


}TCP_ACKFlag;
 

typedef struct _NandSVflag
{
 u8  Doubt_SaveFlag;      // �洢�¹��ɵ� 
 u8  MintPosit_SaveFlag;  // �洢ÿ����λ��  
 u8  PowerCut_SaveFlag;   // �洢�ϵ��¼
 u8  Log_SaveFlag;        // �洢��¼��¼
 u8  Setting_SaveFlag;    // �洢�����޸ļ�¼  


}NANDSVFlag;

typedef struct _Stdver
{
   u8 stdverStr[14];   //��¼��ִ�еĹ���汾
   u8 MdfyID;          // �޸ĵ��� 

}STD_VER; 

typedef struct _DriveInfo
{  
    u8 DriveCode[3]; 
	u8 DriverCard_ID[19];  // ��ʻԱ��ʻ֤���� 18λ
	u8 DriveName[22];	   // ��ʻԱ ����21
	u8 Driver_ID[21];      // ��ʻԱ���֤����20 
	u8 Drv_CareerID[41];  // ��ʻԱ��ҵ�ʸ�֤40
	u8 Comfirm_agentID[41]; // ��֤�������� 40
}DRV_INFO; 

typedef struct _VechInfo
{
 u8     Vech_VIN[18];        // ����VIN��17
 u8     Vech_Num[13];	     // ���ƺ�12
 u8     Vech_Type[13];       // �������� 12
 u16    Dev_ProvinceID;      // ��������ʡID
 u16    Dev_CityID;          // ����������ID    
 u8     Dev_Color;           // ������ɫ           // JT415    1  �� 2 �� 3 �� 4 �� 9����
}VechINFO;



//-------

//-------��̨ע����� ------------
typedef struct _DevRegst
{
  u8  Sd_counter; //  ʹ�ܷ��ͼ��������
  u8  Enable_sd;  //  ʹ�ܷ��� ע����Ϣ
  u8  DeRegst_sd; //  ע����̨ 
}DevRegst;

//------- ��̨��Ȩ��� ----------
typedef struct _DevLOGIN
{
  u8  Operate_enable;  // ʹ�ܿ�ʼ������Ȩ����   1 :enable  0: idle
  u8  Sd_counter;      // ��ʱ���ͼ�����
  u8  Enable_sd;       // ʹ�ܷ��ͼ�Ȩ��Ϣ
}DevLOGIN;
//------- ���ٱ��� -------
typedef struct SPDEXP
{
  u8  excd_status;
  u16 current_maxSpd;    //  ���ٱ����� ��ǰ������ٶ�
  u32 dur_seconds;       //  ����״̬������    ��λ: s
  u8  speed_flag;        //  ���ٱ����������������ϱ�һ����־λ  
  u8  ex_startTime[6];
  u8  ex_endTime[6];     
}SPD_EXP;
//------- ƣ�ͼ�ʻ���� ----
typedef struct _TIRED_DOOR
{                                              //  ��λ:  s
  u32  Door_DrvKeepingSec;       // ������ʻʱ��
  u32  Door_DayAccumlateDrvSec;  // �����ۼӼ�ʻʱ��
  u32  Door_MinSleepSec;         // ��С��Ϣʱ�� 
  u32  Door_MaxParkingSec;       // �ͣ��ʱ��  
  u32  Parking_currentcnt;       // ͣ��ʵʱ������ 
}TIRED_DOOR;
//-------- �ϱ�������  ---------
typedef struct _SEND_DUR
{                                       //  ��λ: s
  u32  Heart_Dur;     //  �ն����������ͼ��
  u16  Heart_SDCnter; // ������������
  u8   Heart_SDFlag;   // ���������ͱ�־λ
  //------------------
  u32  TCP_ACK_Dur;   //  TCP ��ϢӦ��ʱ
  u16  TCP_ACK_DurCnter;// TCP ������
  u8   TCP_SD_state;    // ����һ������1.
  //------------------
  u32  TCP_ReSD_Num;  //  TCP �ش�����
  u16  TCP_ReSD_cnter; // ���ͼ�����    
  //------------------
  u32  UDP_ACK_Dur;   // UDP��ϢӦ��ʱ
  u32  UDP_ReSD_Num;  // UDP �ش����� 
  
  u32  NoDrvLogin_Dur;  //  ��ʻԱδ��¼�㱨ʱ����
 // u16  NoLogin_SDcnter;   //  ���ͼ�ʱ��
  //------------------	
  u32  Sleep_Dur;     //  ����ʱ���ϱ���ʱ����
  //u16  Sleep_SDcnter;   //  ���ͼ�����
  //------------------
  u32  Emegence_Dur;  //  ��������ʱ�ϱ���ʱ���� 
  //u16  Emegence_SDcnter;
  //------------------
  u32  Default_Dur;   //  ȱʡʱ��㱨���   
  
  u32  SD_Delta_maxAngle; // �յ㲹���Ƕ�   < 180

  u16  IllgleMovo_disttance; // �Ƿ��ƶ���ֵ   -- ����Э��Ҫ�����
}SEND_DUR;
// ----------- �������ϱ����  ---------
typedef struct _SEND_DIST
{                                     //   ��λ: m
  u32  Defalut_DistDelta; // ȱʡ�������ϱ�
  u32  NoDrvLogin_Dist;   // ��ʻԱδ��¼ �ϲ�����
  u32  Sleep_Dist;        // ��������� �ϱ�����
  u32  Emergen_Dist;      // ��������ʱ �ϱ�����
} SEND_DIST;

//----------- �ϱ�ģʽ  ---------------
typedef struct _SEND_MODE
{ 
  u8 DUR_TOTALMODE;     // -----   ��ʱ�ϱ���ʽ
  u8 Dur_DefaultMode;  
  u8 Dur_SleepMode;     
  u8 Dur_NologinMode;
  u8 Dur_EmegencMode;

  u8 DIST_TOTALMODE;    // ------   �����ϱ���ʽ  
  u8 Dist_DefaultMode;
  u8 Dist_NoLoginMode;
  u8 Dist_SleepMode;
  u8 Dist_EmgenceMode;
  
}SEND_MODE;

//------- ʵʱλ�ø��� ----
typedef struct _REALTIME_LOCK
{
  u8  Lock_state;   //  ʵʱ����������־λ
  u16 Lock_Dur;     //  �޶����ϱ�ʱ����
  u32 Lock_KeepDur; //  ʵʱ�ϱ�����ʱ��
  u32 Lock_KeepCnter;  
  
}REALTIME_LOCK;

//------- �ı���Ϣ --------
typedef struct _TEXT_INFO
{
  u8  TEXT_FLAG;          //  �ı���־
  u8  TEXT_SD_FLAG;       // ���ͱ�־λ  
  u8  TEXT_Content[100];  // �ı�����
}TEXT_INFO;

//----- ��Ϣ ----
typedef struct _MSG_TEXT
{
  u8   TEXT_mOld;     //  ���µ�һ����Ϣ  дΪ1���������µ�һ����Ϣ
  u8   TEXT_TYPE;     //  ��Ϣ����   1-8  �еڼ���
  u8   TEXT_LEN;      //  ��Ϣ����    
  u8   TEXT_STR[150]; //  ��Ϣ����
}MSG_TEXT;

//-----  ���� ------
typedef struct _CENTER_ASK
{
  u8  ASK_SdFlag; //  ��־λ           ���� TTS  1  ��   TTS ����  2
  u16 ASK_floatID; // ������ˮ��
  u8  ASK_infolen;// ��Ϣ����  
  u8  ASK_answerID;    // �ظ�ID
  u8  ASK_info[30];//  ��Ϣ����
  u8  ASK_answer[30];  // ��ѡ��  
  u8  ASK_disp_Enable;// С����ʾʹ��
}CENTRE_ASK;


//------- ��������  --------
typedef struct _VEHICLE_CONTROL
{
  u8   Control_Flag;  // ���� ��־
  u16  CMD_FloatID;   // �·�Ӧ�����Ϣ��ˮ��
  u8   ACK_SD_Flag;   // �ظ����ͱ�־λ

}VEHICLE_CONTROL;

//--------  �г���¼�����  -----
typedef struct  _RECODER
{
  u16  Float_ID;     //  ������ˮ��
  u8   CMD;     //  ���ݲɼ� 
  u8   SD_Data_Flag; //  ���ͷ��������ر�־
}RECODER;
//------  Camera  --------
typedef struct _CAMERA
{
   u8 Channel_ID;     //  ����ͨ��
   u8 Operate_state;  //  ���պ���״̬    0 : ����   1:  ʵʱ�ϴ�
   
}CAMERA;
//---------- ��ý��  ----------
typedef struct _MULTIMEDIA
{
  u32  Media_ID;           //   ��ý������ID
  u8   Media_Type;         //   0:   ͼ��    1 : ��Ƶ    2:  ��Ƶ 
  u8   Media_CodeType;     //   �����ʽ  0 : JPEG  1:TIF  2:MP3  3:WAV  4: WMV
  u8   Event_Code;         //   �¼�����  0: ƽ̨�·�ָ��  1: ��ʱ����  2 : ���ٱ������� 3: ��ײ�෭�������� ��������
  u8   Media_Channel;      //   ͨ��ID
  //----------------------
  u8   SD_Eventstate;          // �����¼���Ϣ�ϴ�״̬    0 ��ʾ����   1  ��ʾ���ڷ���״̬        
  u8   SD_media_Flag;     // ����û���¼���Ϣ��־λ
  u8   SD_Data_Flag;      // �������ݱ�־λ
  u8   SD_timer;          // ���Ͷ�ʱ��
  u8   MaxSd_counter;   // ����ʹ���  
  u8   Media_transmittingFlag;  // ��ý�崫������״̬  1: ��ý�崫��ǰ����1����λ��Ϣ    2 :��ý�����ݴ�����  0:  δ���ж�ý�����ݴ���
  u16  Media_totalPacketNum;    // ��ý���ܰ��� 
  u16  Media_currentPacketNum;  // ��ý�嵱ǰ���� 
  //----------------------
  u8   RSD_State;     //  �ش�״̬   0 : �ش�û������   1 :  �ش���ʼ    2  : ��ʾ˳���굫�ǻ�û�յ����ĵ��ش�����
  u8   RSD_Timer;     //  ��״̬�µļ�����   
  u8   RSD_Reader;    //  �ش���������ǰ��ֵ 
  u8   RSD_total;     //  �ش�ѡ����Ŀ  
  
   
  u8	Media_ReSdList[10]; //  ��ý���ش���Ϣ�б� 
}MULTIMEDIA;   
//------  Voice Record ¼����� ----
typedef struct _VOICE_RECODE
{
   u8   Operate_CMD; //  ��������  0: ֹͣ¼��    1: ��ʼ¼�� 
   u16  Voice_Dur;   //  ¼��ʱ�䣬 0 ��ʾһֱ¼
   u8   Save_Flag;   //  0:  ʵʱ�ϴ�   1  ����
   u8   SampleRate;  //  ��Ƶ������   0:8K   1: 11k  2:23k  3: 32K  , ��������
   
}VOICE_RECODE;

//----------   DataTrans  ����͸��  -----------
typedef struct _DATATRANS
{
  u8 TYPE;             // ͸����Ϣ����
  u8 Data_RxLen;       // ������Ϣ����
  u8 Data_TxLen;       // ������Ϣ���� 
  u8 DataRx[10];      // ͸����Ϣ���ݽ���
  u8 Data_Tx[20];     // ͸����Ϣ���ݷ���  
  u8 Tx_Wr;  
  
}DATATRANS;   
//------  �������򱨾� -------------------
typedef struct _INOUT
{
  u8  TYPE; //   1 Բ������ 2 �������� 3 ��������� 4 ·��
  u32 ID;   //    ����·��ID
  u8  InOutState; //  0 ��  1  ��   
}INOUT;
//-------  ��ý������ -------------------
typedef struct _MEDIA_INDEX
{
  u8  Effective_Flag; // �洢��Ŀ
  u32  MediaID;  // ��ý��ID 
  u8  Type;     // ����    0 ͼ��   1  ��Ƶ   2  ��Ƶ
  u8  ID  ;      // ͨ��
  u8  EventCode; // �¼�����    0  ƽ̨�·�����   1 ��ʱ����   2  ���ٱ������� 3  ��ײ����
  u8  FileName[12];  // �ļ���
  u8  PosInfo[28];//λ����Ϣ  

}MEDIA_INDEX;
//----------- ���ſ���������� ------------
typedef struct _DoorCamra
{  
  u8	  currentState;  // ��ǰ״̬ 	
  u8	  BakState;   // �洢״̬
} DOORCamera;

//     ISP  Module
typedef struct IspRSD
{  
  u8 	   ISP_ResendACK_state;  // Isp   �ط�����
  u16	   ISP_ResendACK_counter;// Isp  �ط�����  
  u8 	   ISP_ResendCounter; // ISP �ط����� ��󲻳���3��
}ISP_RSD;



typedef struct   TGPSInfo_GPRS
{
   //---------------  �ӱ����ͨЭ��  ---------------------------
    u8		Date[3];	// ������(HEX)
	u8		Time[3];	// ʮ����(HEX)
	u8		Latitude[4];    //γ��	  N/S
	u8		Longitude[4];   //����	 W/E 
} T_GPS_Info_GPRS;  

typedef struct TIRDdrv
{  
  u32	ACC_ONstate_counter;	 //  ACC �� ������
  u32	ACC_Offstate_counter;	  //  ACC �� ������
  u8    Tireddrv_status;         // �Ƿ���ƣ�ͼ�ʻ״̬��    ƣ�͵������Ϊ1 ��ƣ�ʹ�������ʱΪ2 ������ʱΪ0
  u8    Tgvoice_play;            // ʹ������������ʾ
  u8    voicePly_timer;          // ����������ʱ��
  u8    voicePly_counter;        // ������������  ��ֻ����10��
  u8    start_time[6];           // ƣ�ͼ�ʻ��ʼʱ��
  u8    end_time[6];             // ƣ�ͼ�ʻ����ʱ��
  u8    Flag;                    // ACCon =1  ,ACC_off=0; ACC�ر���ACCon=1��ֱ����Ϣ������ 0    
  u8    Status_TiredwhRst;    //  ����λʱ ƣ�ͼ�ʻ��״̬   0 :ͣ��  1:ͣ����û���� 2:�����˻�û����                              

}TID_DRV; 




/*
          APP   ���� ��Ӧ��    
*/

typedef struct  _SYSConfig           //  name:  config
{
    u16     Version_ID ;   //  ϵͳ�汾IDE
    u8       APN_str[40];  //   ���������
    u8       IP_Main[4];   //   ��IP ��ַ 
    u16     Port_main;   //   ��socket  �˿�
    u8       IP_Aux[4];   //   ��IP ��ַ 
    u16     Port_Aux;    //   ��socket  �˿�
    u8       DNSR[50];    //  DNSR 1  ��������
    u8       DNSR_Aux[50]; //   DNSR2   ��������     

	//  LINK2  Setting
	u8		Link2_IP[4]; 
	u16 	Link2_Port; 	      
	
	
    u16     AccOn_Dur;   //  ACC ��  �ϱ����
    u16     AccOff_Dur;  //   ACC ��  �ϱ����
    u8      TriggerSDsatus;  //  �����������ϱ�״̬
} SYS_CONF;


typedef struct  _JT808Config   //name:  jt808
{
    SEND_DIST   DISTANCE;	       // ������ش����   16 Bytes
    SEND_DUR    DURATION;		//	���ͼ�����
    SEND_MODE  SD_MODE;         // ��Ϣ�ϱ�ģʽ    ���ͷ�ʽ   
    u8       LOAD_STATE  ;           //ѡ�г����ĸ���״̬��־   1:�ճ�   2:���   3:�س�
    u8       ConfirmCode[20];       // ��Ȩ��
    u8       Regsiter_Status;        //  ע��״̬
    u8       LISTEN_Num[30];       //  ��������
    u32     Vech_Character_Value;   //  ����ϵ��

    u8       FirstSetupDate[6];           //  ���ΰ�װʱ��
    u8       DeviceOnlyID[35];           //   �г���¼�ǵ�ΨһID
    u16     Msg_Float_ID;                 //   ��Ϣ��ˮ��

    u32     Distance_m_u32;            //  ��ʻ��¼����ʻ���  ��λ: ��
    u32     DayStartDistance_32;     //  ÿ�����ʼ�����Ŀ

    u32     Speed_warn_MAX;           //  �ٶȱ�������
    u32     Spd_Exd_LimitSeconds;  //  ���ٱ�������ʱ������
    u8       Speed_GetType;             //  ��¼�ǻ�ȡ�ٶȵķ�ʽ  00  gpsȡ�ٶ�  01 ��ʾ�Ӵ�����ȥ�ٶ� 
    u8       DF_K_adjustState; // ����ϵ���Զ�У׼״̬˵��  1:�Զ�У׼��    0:��δ�Զ�У׼   

   	
    u8       OutGPS_Flag;     //  0  Ĭ��  1  ���ⲿ��Դ���� 
    u8       concuss_step;    //------add by  xijing
    u8       password_flag;   //���������־     
    u8       relay_flag;      // �̵�������״̬

	u8       Link_Frist_Mode;       //   �״�����ģʽ        0  : dnsr first     1: mainlink  first 
    
     //--------  ʵʱ�ϱ� ---------  
    REALTIME_LOCK    RT_LOCK;     // ʵʱ����      

      //----------  Standard Version -----------------------
    STD_VER               StdVersion;   // ��׼���Ұ汾 
  
    DRV_INFO             Driver_Info;        //  ��ʻԱ��Ϣ
	
    VechINFO             Vechicle_Info;     //  ������Ϣ  
}JT808_CONF;

    //------------- ƣ�ͼ�ʻ���----------------

typedef  struct   _TIRED_Config    // name: tired
{
   TID_DRV         Tired_drive; 
   TIRED_DOOR   TiredDoor;  
} TIRED_CONF;

//------ �¼�  -------
typedef struct _EVENT               //  name: event
{
  u8 Event_ID;   //  �¼�ID
  u8 Event_Len;  //  �¼�����
  u8 Event_Effective; //  �¼��Ƿ���Ч��   1 ΪҪ��ʾ  0     
  u8 Event_Str[20];  //  �¼�����
}EVENT; 

//----- ��Ϣ ----
typedef struct _MSG_BROADCAST    // name: msg_broadcast
{
  u8   INFO_TYPE;     //  ��Ϣ����
  u16  INFO_LEN;      //  ��Ϣ����
  u8   INFO_PlyCancel; // �㲥/ȡ����־      0 ȡ��  1  �㲥
  u8   INFO_SDFlag;    //  ���ͱ�־λ
  u8   INFO_Effective; //  ��ʾ�Ƿ���Ч   1 ��ʾ��Ч    0  ��ʾ��Ч     
  u8   INFO_STR[30];  //  ��Ϣ����
}MSG_BRODCAST;


//------ �绰�� -----
typedef struct _PHONE_BOOK            // name: phonebook
{
  u8 CALL_TYPE ;    // ��������  1 ���� 2 ���� 3 ����/����
  u8 NumLen;        // ���볤��   
  u8 UserLen;       // ��ϵ�˳���
  u8 Effective_Flag;// ��Ч��־λ   ��Ч 0 ����Ч  1
  u8 NumberStr[20]; // �绰����
  u8 UserStr[10];   // ��ϵ������  GBK ����
}PHONE_BOOK;

//------  Բ��Χ��  -----------
typedef struct _CIRCLE_RAIL           // name: Rail_cycle
{
  //----------------------- 
  u32  Area_ID;        //  ����ID
  u16  Area_attribute; //  ��������
  u32  Center_Latitude;//  ����γ��
  u32  Center_Longitude;//  ���ľ���
  u32  Radius;          //  �뾶
  u16  MaxSpd;          //  ����ٶ�
  u8   KeepDur;         //  ���ٳ���ʱ��  
  //------------------------
  u8   Rail_Num;        // ��ǰΧ����Ŀ
  u8   Effective_flag;  // ��ǰΧ���Ƿ� ֹͣ    0 : δ����   1 :  ����  
  u8   StartTimeBCD[6]; //  ��ʼʱ��
  u8   EndTimeBCD[6];   //  ����ʱ��

}CIRCLE_RAIL;
//------  ����Χ��  -----------
typedef struct _RECT_RAIL                // name:  Rail_rect
{
  //-------------------
  u32  Area_ID;        //  ����ID
  u16  Area_attribute; //  ��������
  u32  LeftUp_Latitude;//  ����γ��
  u32  LeftUp_Longitude;//  ���Ͼ���
  u32  RightDown_Latitude;//  ����γ��
  u32  RightDown_Longitude;//  ���¾���     
  u16  MaxSpd;          //  ����ٶ�
  u8   KeepDur;         //  ���ٳ���ʱ��  
  //------------------  
  u8   Rail_Num;        // ��ǰΧ����Ŀ
  u8   Effective_flag;  // ��ǰΧ���Ƿ� ֹͣ    0 : δ����   1 :  ����  
  u8   StartTimeBCD[6]; //  ��ʼʱ��
  u8   EndTimeBCD[6];   //  ����ʱ��

}RECT_RAIL;
//------  �����Χ��  --------
typedef struct _POLYGEN_RAIL             //name: Rail_polygen       
{
   //----------------
  u32  Area_ID;        //  ����ID
  u16  Area_attribute; //  ��������
  u16  MaxSpd;          //  ����ٶ�
  u8   KeepDur;         //  ���ٳ���ʱ��  
  u16  Acme_Num;        //  ������Ŀ  ����3��
  u32  Acme1_Latitude; //  ����1γ��
  u32  Acme1_Longitude;//  ����1����
  u32  Acme2_Latitude; //  ����2γ��
  u32  Acme2_Longitude;//  ����2����
  u32  Acme3_Latitude; //  ����3γ��
  u32  Acme3_Longitude;//  ����3����
   //---------------- 
   u8	Rail_Num;		 // ��ǰΧ����Ŀ
   u8	Effective_flag;  // ��ǰΧ���Ƿ� ֹͣ	 0 : δ����   1 :  ����   
   u8	StartTimeBCD[6]; //  ��ʼʱ��
   u8	EndTimeBCD[6];	 //  ����ʱ��

}POLYGEN_RAIL; 

// -------  ·���йյ�  ------
typedef struct _POINT                               //  name:  turn_point      
{
  u32  POINT_ID;        //  ����ID
  u32  Line_ID;        //   ·��ID
  u32  POINT_Latitude;//   �յ�γ��
  u32  POINT_Longitude;//  �յ㾭��
  u8   Width;          //  ���
  u8   Atribute;       //  ����
  u16  TooLongValue;   //  ·����ʻ������ֵ
  u16  TooLessValue;   //  ·����ʻ������ֵ	
  u16  MaxSpd;          //  ����ٶ�
  u8   KeepDur;         //  ���ٳ���ʱ��  
}POINT; 
//----------  ·������  -------
typedef  struct _ROUTE                              // name: route_line
{
 //--------------------------
  u32  Route_ID;        //  ��·ID
  u16  Route_attribute; //  ��·����  
  u16  Points_Num;      //  �յ���Ŀ  ��С3
  u8   Effective_flag;  //  ��Ч״̬
  u8   StartTimeBCD[6]; //  ��ʼʱ��
  u8   EndTimeBCD[6];   //  ����ʱ��
  POINT RoutePoints[3];  //  �յ���Ϣ 
 //--------------------------
}ROUTE;

//--------------ÿ���ӵ�ƽ���ٶ� 
typedef  struct AvrgMintSpeed
{
  u8 datetime[6]; //current
  u8 datetime_Bak[6];//Bak
  u8 avgrspd[60];
  u8 saveFlag;  
}Avrg_MintSpeed;
//----------  �¹��ɵ�����
typedef struct DoubtTYPE
{
  u8  DOUBTspeed;   // 0x00-0xFA  KM/h
  u8  DOUBTstatus;  // status 
} DOUBT_TYPE;



//-------------------- Media_SD_State 
typedef struct
{
  u8  SD_flag;        // ͼƬ���ݷ��ͱ�־
  u8  photo_sending; //  ������Ƭ����״̬
  u16 SD_packetNum;  //  �������ݵİ����
  u16 Total_packetNum; // ���յ��ܰ���
  u8  Data_SD_counter; //  ���ݷ��ͼ��    
  u16  Exeption_timer;   //  �쳣����¶�ʱ��
  
}_Media_SD_state;


typedef union TIPAddr
{
	u8				ip8[4];	// access the ip as either 4 seperate bytes
	u32				ip32;	// ... or as a single 32-bit dword
} T_IP_Addr;


//-------- ���� ��չ  -------------
typedef struct _BD_EXTEND
{
   //  ----  ��̨������� ----
   u32     Termi_Type;    // �ն�����
   u32     Software_Ver; //  ����汾
   u32     GNSS_Attribute;   // BD  ģ������
   u32     GSMmodule_Attribute; // GSM ģ������
   u32     Device_Attribute;   //  �ն�����  

  //    ��������
     u32   BD_Mode;     //  ����ģ��ͨ��ģʽ
     u32   BD_Baud;     //  ����ģ�� ͨ�Ų�����
     u32   BD_OutputFreq;  // ����ģ��������������� 
     u32   BD_SampleFrea; //  ����ģ��ɼ� NMEA  ����Ƶ��

  //   CAN   �������
     u32   CAN_1_Mode;    //  CAN  ģʽ  01:����ģʽ  10  : ��ͨģʽ   11:  ��Ĭģʽ  
     u32   CAN_1_ID;
     u8    CAN_1_Type;	
     u8    CAN_1_Duration;  //  ��λ��
     
		 
     u32   CAN_2_Mode;   //   CAN  ģʽ
     u32   CAN_2_ID;
     u8     CAN_2_Type;    	 
     u8     CAN_2_Duration;  // s	 
     u32   Collision_Check;     //  bit 31 : �����ر���ײ  b7-b0  ��ײʱ������ b8-b15  ��ײ���ٶ�����

     u8     CloseAllcan;  

  //   λ�ø�����Ϣ
       // 1. �ź�ǿ��
       u16  FJ_SignalValue;  //  �ź�ǿ��   ���ֽ� 0 �����ֽ�  ��4Ϊ X2 gprsǿ�� ����4λ ���ǿ���
      //  2. �Զ���״̬��ģ�����ϴ�  
       u8    FJ_IO_1;
	u8    FJ_IO_2;
	u16  AD_0;  //  2 Byte
	u16  AD_1; //   2 Byte

 //    Χ������ж�
       u8    Close_CommunicateFlag;  //  ������ �ر�ͨ��   
       u8    Trans_GNSS_Flag;    //  ������ɼ�GNSS
  
} BD_EXTEND;

//------- ������չЭ��  ------------
extern BD_EXTEND     BD_EXT;     //  ������չЭ��

//===============================================================================================
extern _Media_SD_state Photo_sdState;   //  ͼƬ����״̬
extern _Media_SD_state Sound_sdState;	//��������  
extern _Media_SD_state Video_sdState;	//��Ƶ����
//------ Photo -----
extern  u32 PicFileSize; // ͼƬ�ļ���С  
extern  u8  PictureName[40];       



//------  voice -----



//------  video  --------



//-------  �г���¼�����extern BD_EXTEND     BD_EXT;     //  ������չЭ��

extern Avrg_MintSpeed  Avrgspd_Mint; 
extern u32         PerMinSpdTotal; //��¼ÿ�����ٶ�����  
extern u8          avgspd_Mint_Wr;       // ��дÿ����ƽ���ٶȼ�¼�±�
extern u8          avgspd_Sec_Wr;       // ��дÿ����ƽ���ٶȼ�¼�±�
extern u8          avgWriteOver;   // д�����־λ
extern u8          AspdCounter;    // ÿ�����ٶ���Ч���������� 
extern  u8         Vehicle_sensor; // ����������״̬   0.2s  ��ѯһ��  
extern  u8		   Vehicle_sensor_BAK; // ����������״̬	0.2s  ��ѯһ��	

extern  DOUBT_TYPE        Sensor_buf[100];// 20s ״̬��¼   
extern 	u8		   save_sensorCounter,sensor_writeOverFlag;  
extern u16  Delta_1s_Plus;
extern 	u32 	   total_plus; 



extern u8		 fCentre_ACK; 			  // ---------�ж�������Ӧ���־λ����
extern u8		 ACK_timer;				   //---------	ACK timer ��ʱ��--------------------- 
extern u8        f_ISP_ACK;   // Զ������Ӧ��	
extern u8        ISP_FCS[2];    //  �·���У�� 
extern u16       ISP_total_packnum;  // ISP  �ܰ���
extern u16       ISP_current_packnum;// ISP  ��ǰ����
extern u32       ISP_content_fcs;    // ISP  ������У��
extern u8		 ISP_ack_resualt;    // ISP ��Ӧ
extern u8		 ISP_rxCMD;		   // ISP �յ�������
extern u8		 f_ISP_88_ACK;	   // Isp  ����Ӧ��
extern u8        ISP_running_state;  // Isp  ��������״̬
extern u8        f_ISP_23_ACK;    //  Isp  ���� �ļ���ɱ�ʶ
extern u16       ISP_running_counter;// Isp  ����״̬�Ĵ��� 
extern u8        ISP_RepeatCounter; //   ISP ���������ظ����� ����5��У��ʧ�ܲ������� 
extern u8        ISP_NeedtoProcessFlag;   //   ��Ҫ����ISP ����
extern u8        ISP_Raw[600];                    //  ����ISP ����δ������ַ� 

extern ISP_RSD   Isp_Resend;

extern u8 TextInforCounter;//�ı���Ϣ����
extern DOORCamera   DoorOpen;    //  ���س�������
extern MEDIA_INDEX  MediaIndex;  // ��ý����Ϣ
extern INOUT        InOut_Object;  // ����Χ��״̬ 
extern DATATRANS    DataTrans;     // ������Ϣ͸��
extern MULTIMEDIA   MediaObj;      // ��ý����Ϣ 
extern VOICE_RECODE VoiceRec_Obj;   //  ¼������  
extern CAMERA       Camera_Obj;     //  �����������
extern RECODER      Recode_Obj;     // �г���¼��
extern POINT        POINT_Obj;      // ·�ߵĹյ�
extern ROUTE        ROUTE_Obj;      // ·�����
extern POLYGEN_RAIL Rail_Polygen;   // �����Χ��
extern RECT_RAIL    Rail_Rectangle; // ����Χ��
extern CIRCLE_RAIL  Rail_Cycle;  // Բ��Χ��
extern VEHICLE_CONTROL Vech_Control; //  ��������
extern PHONE_BOOK    PhoneBook;  //  �绰��
extern PHONE_BOOK	 PhoneBook_8[8];
extern MSG_BRODCAST  MSG_BroadCast_Obj;    // ��Ϣ�㲥  
extern MSG_BRODCAST   MSG_Obj_8[8];  // ��Ϣ�㲥  
extern CENTRE_ASK     ASK_Centre;  // ��������
extern EVENT          EventObj;    // �¼�
extern EVENT          EventObj_8[8]; // �¼�  
//-------�ı���Ϣ-------
extern MSG_TEXT       TEXT_Obj;
extern MSG_TEXT       TEXT_Obj_8[8],TEXT_Obj_8bak[8];

extern TEXT_INFO      TextInfo;    // �ı���Ϣ�·�

extern SPD_EXP    speed_Exd;      // ���ٱ���
extern DevRegst   DEV_regist;  // ע��
extern DevLOGIN   DEV_Login;   //  ��Ȩ


extern NANDSVFlag   NandsaveFlg;
extern A_AckFlag    Adata_ACKflag;
extern TCP_ACKFlag  SD_ACKflag;  



extern u8           SubCMD_8103H;            //  02 H���� ���ü�¼�ǰ�װ�����ظ� ������ 
extern u32         SubCMD_FF01H;            //  FF02 ������Ϣ��չ
extern u32         SubCMD_FF03H;     //  FF03  ������չ�ն˲�������1

extern u8          SubCMD_10H;            //  10H   ���ü�¼�Ƕ�λ�澯����
extern u8	       OutGPS_Flag; // 0  Ĭ��  1  ���ⲿ��Դ����
extern u8	       Spd_senor_Null;  // �ֶ��������ٶ�Ϊ0
extern u32         Centre_DoubtRead;     //  ���Ķ�ȡ�¹��ɵ����ݵĶ��ֶ�
extern u32         Centre_DoubtTotal;    //  ���Ķ�ȡ�¹��ɵ�����ֶ�  


extern u8 		   FCS_GPS_UDP;						//UDP ��������
extern u8          FCS_RX_UDP;                       // UDP ���ݽ���У�� 
extern u8          Centre_IP_modify;            //  ���޸�IP�� 
extern u8          IP_change_counter;           //   �����޸�IP ������
extern u8		   Down_Elec_Flag;				 //   ���Ͷϵ�ʹ�ܱ�־λ 


extern ALIGN(RT_ALIGN_SIZE) u8          GPRS_info[900]; 
extern u16         GPRS_infoWr_Tx;


//------ phone
extern u8       CallState; // ͨ��״̬


extern  GPRMC_PRO GPRMC_Funs;


//--------  GPS prototcol------------------------------------------------------------------------------------------------
extern u8   UDP_dataPacket_flag;			   /*V	   0X03 	 ;		   A	  0X02*/
extern u8   DispContent;   // ����ʱ�Ƿ���ʾ��������  
extern u8  Camera_Number;



extern u8	GPS_getfirst; 		 //  �״��о�γ�� 
extern u8	HDOP_value;		 //  Hdop ��ֵ	  
extern u8   Satelite_num;   // ���ǿ���

extern u8   CurrentTime[3];
extern u8   BakTime[3];
extern u8   Sdgps_Time[3];  // GPS ���� ʱ���¼  


extern  ALIGN(RT_ALIGN_SIZE) u8  UDP_HEX_Rx[1024];    // EM310 ��������hex     
extern u16 UDP_hexRx_len;    // hex ���� ����
extern u16 UDP_DecodeHex_Len;// UDP���պ�808 ���������ݳ���   



extern GPS_RMC  GPRMC;                   // GPMC��ʽ  
//---------- 808 Э�� -----------------------------------------------------------------------------------------------
extern 	u16 	        GPS_Hight;			   //	808Э��-> �߳�	 m 
extern  u16		        Speed_gps;    // ͨ��GPS����������ٶ� km/h   
extern 	u16	            GPS_speed;								  //  GPS��λʱ����ٶ� km/h
extern  u16			    GPS_direction;							  //  GPS����		 ��λ2��       

//---------- ��GPSУ׼����ϵ����� ----------------------------
extern u8		Speed_area; // У��Kֵ��Χ
extern u16		Speed_cacu; // ͨ��Kֵ����������ٶ� 
extern u16 	    Spd_adjust_counter; // ȷ������״̬������ 
extern u16      Former_DeltaPlus[K_adjust_Duration]; // ǰ����������� 
extern u8       Former_gpsSpd[K_adjust_Duration];// ǰ������ٶ�        
extern u8		DF_K_adjustState; // ����ϵ���Զ�У׼״̬˵��
 //------- ��������״̬ ---------------
extern u8      CarLoadState_Flag;//ѡ�г���״̬�ı�־   1:�ճ�   2:���   3:�س�
 //------- ��ý����Ϣ����---------------
extern u8  Multimedia_Flag;//��Ҫ�ϴ��Ķ�ý����Ϣ����	1:��Ƶ	 2:��Ƶ   3:ͼ��
extern u8  SpxBuf[SpxBuf_Size];  
extern u16 Spx_Wr,Spx_Rd; 
extern u8	Duomeiti_sdFlag; 

//------- ¼����ʼ���߽���---------------
extern u8  Recor_Flag; //  1:¼����ʼ   2:¼������  


// ---- �յ� -----
extern u16  Inflexion_Current;      
extern u16  Inflexion_Bak;       
extern u16  Inflexion_chgcnter; //�仯������
extern u16  InflexLarge_or_Small;	    // �ж�curent �� Bak ��С	0 equql  1 large  2 small
extern u16  InflexDelta_Accumulate;	//	��ֵ�ۼ� 


// ----����״̬  ------------
extern u8   SleepState;  //   0  ������ACC on        1  ����Acc Off     
extern u8	SleepConfigFlag; //  ����ʱ���ͼ�Ȩ��־λ

//---- �̶��ļ���С --- 
extern u32 mp3_fsize;   
extern u32 wmv_fsize;    
extern u8  mp3_sendstate;
extern u8  wmv_sendstate;



//---------- �ӱ����ͨЭ�� ------------------------------------------------------------------------------------------	
extern  u8		SIM_code[6];							   // Ҫ���͵�IMSI	����
extern  u8		IMSI_CODE[15];							//SIM ����IMSI ����
extern  u8		Warn_Status[4]; // ����״̬��Ϣ
extern  u8		Car_Status[4];  // ����״̬��Ϣ   
extern T_GPS_Info_GPRS 	 Gps_Gprs;	 
extern T_GPS_Info_GPRS	 Temp_Gps_Gprs;
extern u8	    A_time[6]; // ��λʱ�̵�ʱ��  
// ��λʱ�̵�ʱ��
extern u8       ReadPhotoPageTotal;
extern u8       SendPHPacketFlag; ////�յ���������������һ��blockʱ��λ

//-------- �������� --------
extern u8		warn_flag;		  
extern u8		f_Exigent_warning;//0;     //�Ŷ� ��������װ�� (INT0 PD0)
extern u8		Send_warn_times;     //   �豸�������ϱ��������������3 ��
extern u32  	    fTimer3s_warncount;


extern POSIT Posit[60];                //  ÿ����λ����Ϣ�洢
extern u8    PosSaveFlag;             //  �洢Pos ״̬λ

extern u8   Vehicle_RunStatus;    //  bit 0: ACC �� ��             1 ��  0��
                                  //  bit 1: ͨ���ٶȴ�������֪    1 ��ʾ��ʻ  0 ��ʾֹͣ   
                                  //  bit 2: ͨ��gps�ٶȸ�֪       1 ��ʾ��ʻ  0 ��ʾֹͣ  
extern u8   Status_TiredwhRst;    //  ����λʱ ƣ�ͼ�ʻ��״̬   0 :ͣ��  1:ͣ����û���� 2:�����˻�û����                              


extern u8    SpxSrcName[13];
extern u32   SrcFileSize,DestFilesize,SrcFile_read; 
extern u8	 SleepCounter;  

extern u16   DebugSpd;	//������GPS�ٶ�      
extern u8	 MMedia2_Flag;
extern u8     Send_Rdy4ok;


//==================================================================================================
// ��һ���� :   ������GPS ����ת����غ��� 
//==================================================================================================

extern void Time_pro(u8 *tmpinfo, u8 hour, u8 min , u8 sec);
extern void Status_pro(u8 *tmpinfo);
extern void Latitude_pro(u8 *tmpinfo);
extern void Lat_NS_pro(u8 *tmpinfo);
extern void Longitude_pro(u8 *tmpinfo);
extern void Long_WE_pro(u8 *tmpinfo);
extern void Speed_pro(u8 *tmpinfo,u8 Invalue,u8 Point);
extern void Direction_pro(u8 *tmpinfo,u8 Invalue,u8 Point);
extern void Date_pro(u8 *tmpinfo,u8 fDateModify, u8 hour, u8 min , u8 sec); 
extern void HDop_pro(u8 *tmpinfo); 
extern void   GPS_Delta_DurPro(void);    //��GPS �����ϱ������� 

//==================================================================================================
// �ڶ����� :   �������ⲿ������״̬���
//==================================================================================================
/*    
     -----------------------------
     2.1   ��Э����صĹ��ܺ���
     ----------------------------- 
*/
extern u8 ISP_running_Status(void);
extern int IP_Str(char *buf, u32 IP);
extern void strtrim(u8 *s, u8 c);
extern int str2ip(char *buf, u8 *ip);
/*    
     -----------------------------
     2.2 ����ܽ�״̬���
     ----------------------------- 
*/

/*    
     -----------------------------
    2.3  �������
     ----------------------------- 
*/
extern void  Enable_Relay(void);
extern void  Disable_Relay(void);


/*    
     -----------------------------
    2.4  ��ͬЭ��״̬�Ĵ����仯
     ----------------------------- 
*/

extern void StatusReg_WARN_Enable(void);
extern void StatusReg_WARN_Clear(void);
extern void StatusReg_ACC_ON(void);
extern void StatusReg_ACC_OFF(void);
extern void StatusReg_POWER_CUT(void);
extern void StatusReg_POWER_NORMAL(void);
extern void StatusReg_GPS_A(void);
extern void StatusReg_GPS_V(void);
extern void StatusReg_SPD_WARN(void);
extern void StatusReg_SPD_NORMAL(void);
extern void StatusReg_Relay_Cut(void);
extern void StatusReg_Relay_Normal(void);
extern void StatusReg_Default(void);  



	





//==================================================================================================
// �������� :   ������GPRS���ߴ������Э��
//==================================================================================================

extern  u8  Do_SendGPSReport_GPRS(void);   
extern void  Save_GPS(void);    
extern u8    Stuff_DevCommmonACK_0001H(void);        
extern u8    Stuff_RegisterPacket_0100H(u8  LinkNum);  
extern u8    Stuff_DeviceDeregister_0101H(void);      
extern u8    Stuff_DeviceHeartPacket_0002H(void);
extern u8    Stuff_DevLogin_0102H(void);      
extern u8    Stuff_Normal_Data_0200H(void); 
extern u8    Stuff_Current_Data_0200H(void);   //  ���ͼ�ʱ���ݲ��洢���洢���� 
extern u8    Stuff_Current_Data_0201H(void);   //   λ����Ϣ��ѯ��Ӧ
extern u8    Stuff_SettingPram_0104H(void); 
extern u8    Stuff_EventACK_0301H(void);   
extern u8    Stuff_ASKACK_0302H(void);    
extern u8	 Stuff_MSGACK_0303H(void);	  
extern u8    Stuff_ControlACK_0500H(void);    //   ��������Ӧ��
extern u8    Stuff_RecoderACK_0700H(void);   //   �г���¼�������ϴ�
extern u8    Stuff_MultiMedia_InfoSD_0800H(void);      
extern u8    Stuff_MultiMedia_Data_0801H(void);    
extern u8    Stuff_MultiMedia_IndexAck_0802H(void);  
extern u8    Stuff_DataTransTx_0900H(void);     
extern u8    Stuff_DriverInfoSD_0702H(void);  
extern u8    Stuff_Worklist_0701H(void); 

extern u8    Stuff_DataTrans_0900_ISP_ACK(u8  AckType);  
extern u8    Update_HardSoft_Version_Judge(u8 * instr);
extern void  ISP_file_Check(void);
extern void  ISP_Process(void);  


extern void delay_us(u16 j);
extern void delay_ms(u16 j);


#if 0
extern void  Stuff_ISP_ack(void);      // ISP  
extern void  Stuff_ISPinfo_ack(void);      // ISP  
#endif

extern  void  Media_Start_Init( u8  MdType , u8  MdCodeType);
extern  void  Media_Clear_State(void);
extern  void  Media_Timer(void); 
extern  void  Media_RSdMode_Timer(void);
extern  void  Media_Timer_Service(void); 
extern  void  Meida_Trans_Exception(void);

extern void   Photo_send_start(u16 Numpic);  
extern  void  DataTrans_Init(void);
extern void  TIRED_Drive_Init(void);
extern u8     Sound_send_start(void);

extern void  TCP_RX_Process(u8  LinkNum);  
extern u16    AsciiToGb(u8 *dec,u8 InstrLen,u8 *scr);
extern void  Time2BCD(u8 *dest);     



extern void SpeedWarnJudge(void);
extern void Process_GPRSIN_DeviceData(u8 *instr, u16  infolen);
extern u8   Send_Device_Data(void);
extern void SpeedSensorProcess(void);   // ͨ���������ٶȴ�������� �ٶ� ��������� 


extern void  Sleep_Mode_ConfigEnter(void); 
extern void  Sleep_Mode_ConfigExit(void);

//extern u16   WaveFile_EncodeHeader(u32 inFilesize ,u8* DestStr);   
extern void  CycleRail_Judge(u8* LatiStr,u8* LongiStr);
extern void  RectangleRail_Judge(u8* LatiStr,u8* LongiStr);       
extern u8    Save_MediaIndex( u8 type, u8* name, u8 ID,u8 Evencode);  
extern void  vin_set(u8 *instr);
extern void OutPrint_HEX(u8 * Descrip, u8 *instr, u16 inlen); 
//extern void  Sound_SaveStart(void);
//extern void  Sound_SaveEnd(void); 
extern void    DoorCameraInit(void);
extern void    MSG_BroadCast_Read(void); 
extern u8      Time_FastJudge(void);   
//extern u8  RecordSerial_output_Str(const char *fmt,...); 

//==================================================================================================
// ���Ĳ��� :   808  ��ش洢����  
//==================================================================================================

extern void  JT808_Related_Save_Process(void);
extern void  Save_Status(u8 year,u8 mon,u8 day,u8 hour,u8 min,u8 sec);
extern void  Spd_Exp_Wr(void);


//==================================================================================================
// ���岿�� :   �������г���¼�����Э�� �� ��¼A   
//==================================================================================================









#endif
