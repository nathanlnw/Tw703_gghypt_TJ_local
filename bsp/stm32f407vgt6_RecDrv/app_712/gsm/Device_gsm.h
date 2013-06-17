/*
     App_gsm.h
*/
#ifndef  _RT_GSM
#define _RT_GSM

#include <rthw.h>
#include <rtthread.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>


//#define      MG323_GSM
#define      M66_GSM
      #define    APN_initSTR_LEN     19


#define flash const    

//-----  Dial Step-------
typedef enum DialStage {Dial_DialInit0, Dial_DialInit1, Dial_DialInit2, Dial_DialInit3,Dial_DialInit4,Dial_DialInit5,Dial_DialInit6,Dial_DNSR1,Dial_DNSR2,Dial_DialInit7,Dial_MainLnk,Dial_AuxLnk,Dial_ISP,Dial_Idle} T_Dial_Stage;



//#define MULTI_LINK



#define GPRS_GSM_Power    GPIO_Pin_13
#define GPRS_GSM_PWKEY    GPIO_Pin_12  
#define GPRS_GSM_RST      GPIO_Pin_11


#define  Dial_Timeout			   100		// 3 seconds        500
#define  Dial_Dial_Retry_Time	   100		// 6 seconds
#define  Dial_max_Timeout		300		// 30 seconds        1000
#define  Dial_Step_MAXRetries			4

//
#define  Max_DIALSTEP            20         //  ��󲦺Ŵ��� 


 //-----------  GSM_RX  buffer size -------------
#define GSMRX_SIZE		2500
#define GSM_AsciiTX_SIZE   2500 



typedef   struct
{
  u8   GSM_PowerEnable;      // ʹ�ܿ�ʼ�ϵ� ����ʼ��Ϊ 1    
  u16  GSM_powerCounter;   //  �ϵ������
  u8   GSM_power_over;     //   1: module on  2:   IMSI get        
} GSM_POWER;

typedef struct
{
u8   check_simcard;     //  ���SIM��
u8   Get_state;        //  0: idle    1: Get IMSI_CODE successfully    2: No SIM card
u8   imsi_error_count;
u8   Checkcounter;   
}IMSI_GET;

typedef struct      //  ��ȡ��IMSI �� ����ǰ ��ʼ����������
{
u8     Total_initial; 
u8     Initial_step;
u8     Execute_couter; 
u8     Execute_enable; 

}COMM_AT; 

typedef  struct
{
	//---- Dial  Datalink----------
	u8     Dial_ON;                       //  ���ź�����ģʽ�µ�״̬��Ϣ���������
	u8     Pre_Dial_flag;              //  ׼������ʹ��
	u8     Pre_dial_counter;        //  ׼�����ż�ʱ��
	u8     start_dial_stateFLAG;  //�ӿ�ʼ����ʱдΪ1 
	u16     start_dial_counter;           //���ڲ���״̬��־λ
       u8     Dial_GPRS_Flag;           // ��ʼ��¼GPRS 
	u8     Dial_step;                         //���Ų�������� 

        //----Single Setp  Operate  related  -----
        u16   Dial_step_RetryTimer;         //  ����������ʱ������
        u8     Dial_step_Retry;                  //  �����������Դ���
        //-----  close  related   -----------
	u8      DataLink_EndFlag;    
	u8      DataLink_end_counter; 
	u8      Connect_counter;         //  ��û�е���ǰ�ز��������Ƽ�����    11-3-4 ���ӵ� ERROR 20 
 
} DATA_DIAL;



//  TTS   ���
#define   TTS_BUSY        1
#define   TTS_OK            0
#define   Speak_Ctrl             GPIO_Pin_9 


#define  Speak_OFF         GPIO_SetBits(GPIOD,Speak_Ctrl)
#define  Speak_ON           GPIO_ResetBits(GPIOD,Speak_Ctrl)       


typedef   struct  
{
     u8  HEX_BUF[400];     // GBK hex
     u16  HEX_len;
     u8   Save;    //  �����ǰ������Ϣæ��ô�洢��HEX_BUF������ ����������Ϣ1 ��û�� 0	 
     u8  ASCII_BUF[800];  // ASCII GBK  
     u16  ASCII_Len;
     u8  NeedtoPlay;  	 
     u8  Playing;                 //   1: pLaying    0:  idle     ����״̬���ع���
     u16  TimeOut_limt;               //  ���ų�ʱ ��λ:s      ��������180 ��ÿ�֣�3����ÿ�����
     u16  TimeCounter;               // �����쳣��ʱ��ʱ��
     
}TTS;


//   Voice Recrod  
 #define   VOICEREC_IDLE                  0      //  ����
 #define   VOICEREC_RECORDING       1       // ¼����
 #define   VOICEREC_DataRXing         2       //  ���ݽ�����
 #define   VOICEREC_DEL                   3       //  ���ݽ�����ϣ��ȴ�ɾ��

typedef struct
{
     u8    Sate;   // Operate  step
     u8   ExcuteFlag;  // ִ�б�־
     u8    file_name[20]; //ʱ����ASCII.AMR 
     u32  filesize;      //  ¼���ļ���С
     u32  file_read;    //  �ļ���ȡλ�ü�¼
     u8    excption_counter;  // �쳣����¶�ʱ�ָ�
     u8    running;        //   ����״̬�� 1 æ 0 ����     
     

		//at  &   save  related  
     u8   SendString[50];  //  AT ��������      
     u8   ASCII_in[2100];  //    ����ASCII ����
     u8   HEX[600];        //   HEX ԭʼ��Ϣ 
     u16  curren_PageCNT;  // �洢�ռ������
     u16  pic_PageIn_offset;// ҳ��ƫ��	 
     u16  Get_Len;  // ��ȡ�������Ĵ�С
     u8   half_page;     //   0 ʱ ���洢 1ʱ�洢
           //  808
     u16  JT808_Duration;   //  ����Ҫ��¼��ʱ��
     u8   JT808_SaveOrNot; //  �洢��־ 0: ʵʱ�ϴ�  1: ����
     u8   JT808_SampleRate;//  ��������  
     
} VOC_REC;

#define  GSM_TYPEBUF_SIZE  1500
typedef __packed struct
{
	u16	       gsm_wr;
	u8		gsm_content[GSM_TYPEBUF_SIZE];  
}GSM_typeBUF;

//  Voice  Record 
extern VOC_REC       VocREC;    // ¼���ϴ����  
//   TTS  
extern   TTS              TTS_Var;  //  TTS ���ͱ���  

extern COMM_AT       CommAT; 
extern DATA_DIAL     DataDial;  


//-------- TCP2 send ---------
extern u8       TCP2_ready_dial;  
extern u16     Ready_dial_counter2;
extern u16     TCP2_notconnect_counter;
extern u8       TCP2_Connect;
extern u8	      TCP2_sdFlag;		//��ʱ����GPSλ����Ϣ��־
extern u16     TCP2_sdDuration;
extern u8       TCP2_Coutner;  // ��ʱ������
extern u8       TCP2_login;       // TCP ���������Ӻ�ı�־λ   


ALIGN(RT_ALIGN_SIZE)
extern  u8     GSM_rx[GSMRX_SIZE];       
extern  u8  GSM_AsciiTx[GSM_AsciiTX_SIZE];

extern void GSM_CSQ_timeout(void);
extern void GSM_CSQ_Query(void);
extern void  DataLink_MainSocket_set(u8 *IP, u16  PORT, u8 DebugOUT);
extern void  DataLink_AuxSocket_set(u8 *IP, u16  PORT,u8 DebugOUT) ;
extern void  DataLink_IspSocket_set(u8 *IP, u16  PORT,u8 DebugOUT); 
extern void  DataLink_APN_Set(u8* apn_str,u8 DebugOUT);
extern void  DataLink_DNSR_Set(u8* Dns_str,u8 DebugOUT); 
extern void  DataLink_DNSR2_Set(u8* Dns_str,u8 DebugOUT);
extern void  DataLink2_Socket_set(u8 *IP, u16  PORT,u8 DebugOUT); 

extern  void  Gsm_RegisterInit(void);
extern  void  GSM_RxHandler(u8 data);     
extern  void  GSM_Buffer_Read_Process(void);


extern u8    GPRS_GSM_PowerON(void);   
extern void  Data_Send(u8* DataStr, u16  Datalen,u8  Link_Num);
extern void  End_Datalink(void);
extern void  ISP_Timer(void); 
extern void Redial_Init(void);
extern void rt_hw_gsm_output(const char *str);
extern void rt_hw_gsm_output_Data(u8 *Instr,u16 len);  
extern  u16  GSM_AsciitoHEX_Convert(u8 *Src_str,u16 Src_infolen,u8* Out_Str); 
extern void GSM_Module_TotalInitial(void);
extern void DataLink_Process(void);
extern void   Dial_step_Single_10ms_timer(void);
extern void  IMSIcode_Get(void);
extern void  rt_hw_gsm_init(void);


extern u8    TTS_Data_Play(void);
extern u8    TTS_Get_Data(u8* Instr,u16 LEN) ; 
extern void TTS_Exception_TimeLimt(void);     //  ��λ: s 

//   VOC REC 
extern void    VOC_REC_Stop(void);     //  ¼������
extern  void   VOC_REC_Start(void);     // ¼����ʼ 
extern void    VOC_REC_process(void); 


#endif 

