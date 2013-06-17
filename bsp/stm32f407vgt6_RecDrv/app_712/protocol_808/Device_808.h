/*
     Device  808 .h
*/
#ifndef  _DEVICE808
#define   _DEVICE808

#include <rtthread.h> 
#include <rthw.h>
#include "stm32f4xx.h"

#include  <stdlib.h>
#include  <stdio.h>
#include  <string.h>
#include "App_moduleConfig.h"


#define  PageSIZE          512

//---------ISP    Ӧ�÷���Dataflash �����ļ��е�--------------
#define DF_BL_PageNo		                 10             /*DF_BL_RAM run PageNo:   10  ~ 49  page */
#define DF_APP1_PageNo		                 50             /*DF_APP_flah run PageNo:   50  ~ 903  page*/


//--------  Protocol IO define -------------
//----- in pins  -------
#define  ACC_IO_Group          GPIOE               // ACC �ܽ�����
#define  ACC_Group_NUM         GPIO_Pin_9

#define  WARN_IO_Group         GPIOE               // �������� �ܽ�����
#define  WARN_Group_NUM        GPIO_Pin_8  


//---- ����״̬  ------------------
   /*  
     -------------------------------------------------------------
              F4  �г���¼�� TW703   �ܽŶ���
     -------------------------------------------------------------
     ��ѭ  GB10956 (2012)  Page26  ��A.12  �涨
    -------------------------------------------------------------
    | Bit  |      Note       |  �ر�|   MCUpin  |   PCB pin  |   Colour | ADC
    ------------------------------------------------------------
        D7      ɲ��           *            PE11             9                ��
        D6      ��ת��     *             PE10            10               ��
        D5      ��ת��     *             PC2              8                ��
        D4      Զ���     *             PC0              4                ��
        D3      �����     *             PC1              5                ��
        D2      ���          add          PC3              7                ��      *
        D1      ����          add          PA1              6                ��      *
        D0      Ԥ��
   */

#define BREAK_IO_Group                GPIOE                 //  ɲ����
#define BREAK_Group_NUM             GPIO_Pin_11

#define LEFTLIGHT_IO_Group         GPIOE                // ��ת��
#define LEFTLIGHT_Group_NUM       GPIO_Pin_10

#define RIGHTLIGHT_IO_Group       GPIOC               // ��ת��
#define RIGHTLIGHT_Group_NUM      GPIO_Pin_2

#define FARLIGHT_IO_Group           GPIOC              // Զ���
#define FARLIGHT_Group_NUM        GPIO_Pin_0 

#define NEARLIGHT_IO_Group          GPIOC             // �����
#define NEARLIGHT_Group_NUM       GPIO_Pin_1 

#define FOGLIGHT_IO_Group            GPIOC             //  ���
#define FOGLIGHT_Group_NUM         GPIO_Pin_3    

#define DOORLIGHT_IO_Group          GPIOA             // ���ŵ�   Ԥ��
#define DOORLIGHT_Group_NUM       GPIO_Pin_1

 
//#define WARNLIGHT_IO_Group        GPIOD             // ������
//#define WARNLIGHT_Group_NUM       GPIO_Pin_9

#define SPEAKER_IO_Group          GPIOA             // ����
#define SPEAKER_Group_NUM         GPIO_Pin_1


#define RAINBRUSH_IO_Group        GPIOC//  ��ˢ
#define RAINBRUSH_Group_NUM       GPIO_Pin_3



//------  out pins ---
#define RELAY_IO_Group           GPIOB               //�̵���
#define RELAY_Group_NUM          GPIO_Pin_1
 
#define Buzzer_IO_Group          GPIOB               //������ 
#define Buzzer_Group_Num         GPIO_Pin_6  



#define  timer1_dur         14111        // 168*84=1412      -1  =14111


//-----  WachDog related----
extern u8    wdg_reset_flag;    //  Task Idle Hook ���
extern u16   ADC_ConvertedValue; //��ص�ѹAD��ֵ    




/*    
     -----------------------------
    1.    ����ܽ�״̬���
     ----------------------------- 
*/
extern void  WatchDog_Feed(void);
extern void  WatchDogInit(void); 
extern void  APP_IOpinInit(void) ;
//  INPUT
extern u8    ACC_StatusGet(void);
extern u8    WARN_StatusGet(void);
extern u8    MainPower_cut(void);
extern u8   FarLight_StatusGet(void);
extern u8  NearLight_StatusGet(void);
extern u8  FogLight_StatusGet(void);
extern u8   WarnLight_StatusGet(void);
extern u8  Speaker_StatusGet(void);
extern u8  LeftLight_StatusGet(void);
extern u8  DoorLight_StatusGet(void);
extern u8  RightLight_StatusGet(void);
extern u8  BreakLight_StatusGet(void);
extern u8 RainBrush_StatusGet(void);

//   OUTPUT
extern void  Enable_Relay(void);
extern void  Disable_Relay(void);
extern void  IO_statusCheck(void); 
extern void  ACC_status_Check(void);


/*    
     -----------------------------
    2.  Ӧ�����
     ----------------------------- 
*/
extern   void TIM2_Configuration(void); 
extern   void Init_ADC(void); 
/*    
     -----------------------------
    3.  RT �������
     ----------------------------- 
*/


/*
       -----------------------------
       ������Ӧ��
      -----------------------------
*/
//  1 .  ѭ���洢 
extern   u8       Api_cycle_write(u8 *buffer, u16 len);
extern   u8       Api_cycle_read(u8 *buffer, u16 len); 
extern   u8       Api_cycle_Update(void);
extern   u8       Api_CHK_ReadCycle_status(void);  

 // 2. Config 
 extern   u8    Api_Config_write(u8 *name,u16 ID,u8* buffer, u16 wr_len);

 //  3.  ���� 
 extern   u8   Api_DFdirectory_Create(u8* name, u16 sectorNum);
 extern   void   Api_MediaIndex_Init(void);
 extern   u32  Api_DFdirectory_Query(u8 *name, u8  returnType);
 extern   u8   Api_DFdirectory_Write(u8 *name,u8 *buffer, u16 len); 
 extern   u8    Api_DFdirectory_Read(u8 *name,u8 *buffer, u16 len, u8  style ,u16 numPacket);  // style  1. old-->new   0 : new-->old 
 extern   u8   Api_DFdirectory_Delete(u8* name); 
 extern   u8   API_List_Directories(void );
 extern  void  Api_WriteInit_var_rd_wr(void);    //   д��ʼ���������Ͷ�д��¼��ַ
 extern  void  Api_Read_var_rd_wr(void);    //   ����ʼ���������Ͷ�д��¼��ַ 



 extern   u8     Api_Config_Recwrite_Large(u8 *name,u16 ID,u8* buffer, u16 wr_len);
 extern  u8      Api_Config_read(u8 *name,u16 ID,u8* buffer, u16 Rd_len); 
 extern   u8     Api_RecordNum_Write( u8 *name,u8 Rec_Num,u8 *buffer, u16 len);  
 extern    u8    Api_RecordNum_Read( u8 *name,u8 Rec_Num,u8 *buffer, u16 len);  

 
 extern   u8     ISP_Read( u32  Addr, u8*  Instr, u16 len);        
 extern   u8     ISP_Write( u32  Addr, u8*  Instr, u16 len);
 extern   u8     ISP_Format(u16 page_counter,u16 page_offset,u8 *p,u16 length);

 extern u8     TF_Card_Status(void);
 

#endif
