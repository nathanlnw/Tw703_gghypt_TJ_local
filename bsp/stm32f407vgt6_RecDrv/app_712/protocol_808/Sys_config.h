#ifndef   SYS_CONFIG
#define  SYS_CONFIG

#include <rthw.h>
#include <rtthread.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>

//------------------------ RT_Thread  Config   ID  list  --------------------------------




 #define    ID_CONF_SYS                              0                           // ϵͳ��ʼ�� ID ,���������״γ�ʼ��


/*   
         Directory  Name    &    Size (SectorNum)                 ---Start 
*/
  //   1.  ����
#define   config                                                "config"
#define   config_size                                      1

#define    jt808                                                 "jt808"
#define    jt808_size                                       2

#define    tired_config                                     "tired_config"
#define    tired_config_size                            1

#define    BD_ext_config                                      "BD_extent"
#define    BD_ext_size                                      1

//  2.  ѭ���洢
#define   cyc_gps                                            "cyc_gps" 
#define   cyc_gps_size                                    900           /* 1 record=128Byte    1 sector=4KB=32 ��
                                                                                             1��30s ��� 2880 ��=>  2880/32=90 Sector/Day
                                                                                             ���ٴ洢10 ������, ����  size=900   Sector
                                                                                        */ 

// 2.  �̶����	 
#define    event_808                                       "event"  
#define    event_size                                      1 

#define    msg_broadcast                               "msg_broadcast"
#define    msg_broadcast_size                      1

#define    phonebook                                      "phonebook"
#define    phonebook_size                             1

#define    Rail_cycle                                        "Rail_cycle"
#define    Rail_cycle_size                               1

#define    Rail_rect                                          "Rail_rect"
#define    Rail_rect_size                                 1

#define    Rail_polygen                                   "Rail_polygen"
#define    Rail_polygen_size                           1

#define    turn_point                                      "turn_point"
#define    turn_point_size                               1

#define    route_line                                        "route_line"
#define    route_line_size                                      1

#define    ask_quesstion                                 "ask_quesstion"
#define    ask_quesstion_size                               1 

#define    text_msg                                         "text_msg"
#define    text_msg_size                                       1 

 // 3.  ��¼ 

#define    spd_warn                                         "spd_warn"
#define    spd_warn_size                                 1

#define    tired_warn                                       "tired_warn"
#define    tired_warn_size                              1

#define    doubt_data                                     "doubt_data"
#define    doubt_data_size                             3

#define    spdpermin                                      "spdpermin"
#define    spdpermin_size                              3

#define    pospermin                                       "pospermin"
#define    pospermin_size                               3

#define    pic_index                                     "pic_index"
#define    pic_index_size                                2

#define    voice_index                                     "voice_index"
#define    voice_index_size                            2


  // 4. ����
#define    camera_1                                        "camera_1"
#define    camera_1_size                                 8

#define    camera_2                                        "camera_2"
#define    camera_2_size                                 8

#define    camera_3                                        "camera_3"
#define    camera_3_size                                 8

#define    camera_4                                        "camera_4"
#define    camera_4_size                                 8


// 5. ¼�� 
#define    voice                                                "voice"
#define    voice_size                                         8
/*   
         Directory  Name    &    Size (SectorNum)                 ---End
*/








//------------------------------------------------------------------------------
#define   SOFTWARE_VER     0x0001
#define   SYSID            0x0324
                                /*        
                                                        0x0000   -----   0x00FF  �������з���
                                                        0x0100   -----   0x0FFF  ��Ʒ������
                                                        0x1000   -----   0xF000  Զ��������
                                                       */  

#define  STM32F103_Recoder_16MbitDF            0x00000005        // �г���¼��ID ��ʶ  16Mit
#define  STM32F103_Recoder_32MbitDF            0x00000007        // �г���¼��ID ��ʶ  16Mit 
#define  STM32F103_Recoder_32MbitMG323         0x00000008        // �г���¼��ID ��ʶ  16Mit   
#define  STM32F407_Recoder_32MbitDF            0x00000009      




#define   Max_SystemCounter            28800 // 86400   //��ʱ����ʱ��24Сʱ һ�� 

//-----------------------  Max  Add    ---------------------------------
#define   Max_CycleNum                  16384
                                                 /*
												   ÿһ����¼Ϊ31�ֽڣ�����ȡÿ����¼32���ֽڣ�ÿpage 16����¼
												   1024page=1024x512 Bytes=1024*16=16284Record 
                                                         */
#define   Max_PicNum                    400
                                                 /*
												  ÿ��ͼƬ32Page��1Page ������31page ͼƬ����
												  32768page=32768/32=1024 pics 
												  */
#define   Max_RecoderNum                400
                                                 /*
                                                            ÿ����¼256���ֽڣ�1page =8 Record
                                                         */
#define   Max_CommonNum                  128 
                                                 /*
                                                             1page=64Records    2048 page =131072 Records 
                                                         */
#define  Max_SPDSperMin                  2800
                                                 /*
                                                             1record=70Bytes   1page=29Records  2048 page=59292 Records   
                                                         */
#define  Max_SPDerSec                    560      
                                                 /*
                                                             1record=70Bytes   1page=29Records  16000 page=464000 Records   
                                                         */
#define  Max_MintPos                     48   
                                                 /*  ��λСʱ��ÿ���ӵ�λ����Ϣ
                                                             1record=485Byte   1page=1Record   2048Page=8192records
                                                          */


/* Function  Select  Define  */
#define        IP2
#define        IP3 

// status
#define  AccON_Over                1                          //   1 : ACC ���������     0 :  ACC  �ػ������� 
#define  AccOFF_Over               0
#define  LOG_IN                    1                          //   01H:��¼��02H���˳���03H��������ʻԱ
#define  LOG_OUT                   2 
#define  LOG_Change                3
#define  Power_Normal              1                          //01H:�ϵ磬02H���ϵ�
#define  Power_Cut                 2
#define  SETTING_INFO              0x82            
#define  SETTING_Status            0x84 
#define  SETTING_Time              0xC2 
#define  SETTING_Plus              0xC3
#define  Route_Mum                  16

                                                              /*
																	82H:���ó�����Ϣ��84H������״̬��
																	C2H:���ü�¼��ʱ�� 
																	C3H:���ü�¼���ٶ�����ϵ��
															  */





/* System Basic  Define   */

//---------------------  Flash ��д������ �궨��  --------------
//#define    FlashWrite         FSMC_NAND_Write_withOffset      // Flash  д��  (uint8_t *pBuffer, NAND_ADDRESS Address, u16 RdOffset,u16 Rdlen)
//#define    FlashRead	         FSMC_NAND_Read_withOffset       // Flash  ��ȡ
//#define    FlashErase         FSMC_NAND_EraseBlock_InPage     // Flash  ����  


#define Init_RecordSize 40



extern ALIGN(RT_ALIGN_SIZE)  SYS_CONF        SysConf_struct;   //  ϵͳ����
extern ALIGN(RT_ALIGN_SIZE)  JT808_CONF      JT808Conf_struct;   //  JT 808   ������� 
extern ALIGN(RT_ALIGN_SIZE)  TIRED_CONF      TiredConf_struct;    //  ƣ�ͼ�ʻ������� 



//----------  Basic  Config---------------------------
extern u8		DeviceNumberID[13];//="800130100001";	 // ����DeviceID	---- �ӱ����ͨ��
extern u8       RemoteIP_Dnsr[4]; 
extern u8		RemoteIP_main[4];//
extern u16		RemotePort_main;
extern u8		RemoteIP_aux[4];    
extern u16		RemotePort_aux; 

//      Link2  Related 
extern u8      Remote_Link2_IP[4]; 
extern u16     Remote_Link2_Port;     



extern u8		APN_String[30];	  // APN  �ַ���   
extern u8		DomainNameStr[50];	// ����  ���ͨ  
extern u8       DomainNameStr_aux[50];        


extern u32		   Current_SD_Duration;  //GPS ��Ϣ�����ʱ����       
extern u32 		   Current_SD_Distance; // GPS ��Ϣ�����ϱ�����  
extern u8		   Current_State;		 // �ϱ�ʵʱ��־λ��Ϣ	 Ԥ��DF ��  
extern u32 		   DistanceAccumulate;	 // �����ϱ��ۼ���
extern u16         ACC_on_sd_Duration;    //  ACC ������ʱ�� �ϱ���ʱ����
extern u16         ACC_off_sd_Duration;    //  ACC �ر�ʱ���ϱ���ʱ����
extern u8		   TriggerSDsatus;   // �����������ϱ�״̬λ 

extern u16         StopLongDuration;    //����ͣ�������ʱ��
extern u16         Stop_counter;               //����ͣ������ʱ������


extern u8       EmergentWarn;               // ��������





extern u8     Vechicle_TYPE;                //   ��������    1:���ͻ���  2: С�ͻ���  3:���Ϳͳ�  4: ���Ϳͳ�   5:С�Ϳͳ�
extern u8	  OnFire_Status; 					//      1 : ACC ���������	   0 :	ACC  �ػ������� 
extern u8     Login_Status;                    //   01H:��¼��02H���˳���03H��������ʻԱ
extern u8     Powercut_Status;                 //01H:�ϵ磬02H���ϵ�
extern u8     Settingchg_Status;              /*
												82H:���ó�����Ϣ��84H������״̬��
												C2H:���ü�¼��ʱ�� 
												C3H:���ü�¼���ٶ�����ϵ��
											   */

											   


                                            /*
                                                            4  bytes                     4 bytes
                                                            д��ַ                      �洢��Nand�еĵ�ַ
                                                           
                                                    */

//extern u16    DaySpdMax;                                //  ��������ٶ�
//extern u16    DayDistance;                              //  ������ʻ����


//--------------  �����ϱ�  --------------------
extern u32  former_distance_meter;     //   ��һ�ξ���    ����ش�ʱ������
extern u32  current_distance_meter;    //   ��ǰ����    

//---------  SytemCounter ------------------
extern u32  Systerm_Reset_counter; 
extern u8   SYSTEM_Reset_FLAG;        // ϵͳ��λ��־λ  

extern u32      Device_type;    // Ӳ������   STM32103  ��A1 
extern u32      Firmware_ver;   // ����汾 
extern u8	    ISP_resetFlag;		 //Զ��������λ��־λ 


extern void TEXTMSG_Write_Init(void);
extern void TEXTMsg_Read (void);

//-------------------------------------------------------------------------------------------------------------   	
extern void TIRED_DoorValue_Read(void);
extern void SendMode_ConterProcess(void);
extern void SendMode_Config(void)  ;
extern void JT808_Vehicleinfo_Init(void);
extern void JT808_RealTimeLock_Init(void);
extern void Event_Init(u8  Intype);
extern void Question_Read(void);
extern void Event_Read(void);
extern void PhoneBook_Init(u8  Intype);
extern void PhoneBook_Read(void);    
extern void  RailCycle_Init(void);
extern void  RailCycle_Read(void);
extern void  RailRect_Init(void);
extern void  RailRect_Read(void);
extern void  RailPolygen_Init(void);
extern void  RailPolygen_Read(void);
extern void  RouteLine_Init(void);
extern void  RouteLine_Read(void);  






extern void SysConfiguration(void);
extern void SetConfig(void);
extern void DefaultConfig(void);
extern void RstWrite_ACConoff_counter(void);
extern void BD_EXT_Write(void);
extern void MSG_BroadCast_Init(u8  Intype);


//д���ı���Ϣ1-8
extern void TEXTMSG_Write(u8 num,u8 new_state,u8 len,u8 *str);


#endif
