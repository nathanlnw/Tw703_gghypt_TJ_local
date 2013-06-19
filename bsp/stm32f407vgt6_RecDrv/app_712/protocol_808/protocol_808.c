/*
     Protocol_808.C
*/

#include <rtthread.h> 
#include <rthw.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>
#include "math.h"
#include  <stdlib.h>
#include  <stdio.h>
#include  <string.h>
#include "App_moduleConfig.h"
#include "math.h"
#include "stdarg.h"
#include "string.h"
#include "SMS.h"



#define    ROUTE_DIS_Default            0x3F000000


//----   ��ý�巢��״̬ -------
_Media_SD_state Photo_sdState;   //  ͼƬ����״̬
_Media_SD_state Sound_sdState;	//��������  
_Media_SD_state Video_sdState;	//��Ƶ����


//------ Photo -----
 u32 PicFileSize=0; // ͼƬ�ļ���С  
 u8  PictureName[40];      



//------  voice -----



//------  video  --------


/*
             ��
*/
//------ phone
u8       CallState=CallState_Idle; // ͨ��״̬

//   ASCII  to   GB    ---- start  
//0-9        10
u8  arr_A3B0[20]={0xA3,0xB0,0xA3,0xB1,0xA3,0xB2,0xA3,0xB3,0xA3,0xB4,0xA3,0xB5,0xA3,0xB6,0xA3,0xB7,0xA3,0xB8,0xA3,0xB9};

//@ A-O      16
u8  arr_A3C0[32]={0xA3,0xC0,0xA3,0xC1,0xA3,0xC2,0xA3,0xC3,0xA3,0xC4,0xA3,0xC5,0xA3,0xC6,0xA3,0xC7,0xA3,0xC8,0xA3,0xC9,0xA3,0xCA,0xA3,0xCB,0xA3,0xCC0,0xA3,0xCD,0xA3,0xCE,0xA3,0xCF};

//P-Z         11��
u8  arr_A3D0[22]={0xA3,0xD0,0xA3,0xD1,0xA3,0xD2,0xA3,0xD3,0xA3,0xD4,0xA3,0xD5,0xA3,0xD6,0xA3,0xD7,0xA3,0xD8,0xA3,0xD9,0xA3,0xDA};

//.  a-0       16
u8  arr_A3E0[32]={0xA3,0xE0,0xA3,0xE1,0xA3,0xE2,0xA3,0xE3,0xA3,0xE4,0xA3,0xE5,0xA3,0xE6,0xA3,0xE7,0xA3,0xE8,0xA3,0xE9,0xA3,0xEA,0xA3,0xEB,0xA3,0xEC,0xA3,0xED,0xA3,0xEE,0xA3,0xEF};

//p-z          11
u8  arr_A3F0[22]={0xA3,0xF0,0xA3,0xF1,0xA3,0xF2,0xA3,0xF3,0xA3,0xF4,0xA3,0xF5,0xA3,0xF6,0xA3,0xF7,0xA3,0xF8,0xA3,0xF9,0xA3,0xFA};
//-------  ASCII to GB ------



//----------- �г���¼�����  -----------------
Avrg_MintSpeed  Avrgspd_Mint; 
u32         PerMinSpdTotal=0; //��¼ÿ�����ٶ�����  
u8          avgspd_Mint_Wr=0;       // ��дÿ����ƽ���ٶȼ�¼�±�
u8          avgspd_Sec_Wr=0;       // ��дÿ����ƽ���ٶȼ�¼�±�
u8          avgWriteOver=0;   // д�����־λ
u8          AspdCounter=0;    // ÿ�����ٶ���Ч���������� 
u8          Vehicle_sensor=0; // ����������״̬   0.2s  ��ѯһ��                                    
							  /*	   
							  D7  ɲ��
							  D6  ��ת��
							  D5  ��ת��
							  D4  ����
							  D3  Զ���
							  D2  ��ˢ
							  D1  Ԥ��
							  D0  Ԥ��
							  */
u8		  Vehicle_sensor_BAK=0; // ����������״̬	0.2s  ��ѯһ��	
                              
DOUBT_TYPE  Sensor_buf[100];// 20s ״̬��¼
u8          save_sensorCounter=0,sensor_writeOverFlag=0;;
u32     total_plus=0; 






u8  Camera_Number=1;
u8       DispContent=1;   // ����ʱ�Ƿ���ʾ��������  
                             /*                          
                                         1 <->  ������ʾ 
                                         2 <->  ��ʾ������Ϣ�� 
                                         3 <->  ��ʾ ������������  
                                         0<-> ����ʾ���������ֻ��ʾЭ������
                                  */ 

u8         TextInforCounter=0;//�ı���Ϣ����

u8 		   FCS_GPS_UDP=0;						//UDP ��������
u8         FCS_RX_UDP=0;                       // UDP ���ݽ���У��

u8          Centre_IP_modify=0;               //  ���޸�IP�� 
u8          IP_change_counter=0;             //   �����޸�IP ������
u8          Down_Elec_Flag=0;                //   ���Ͷϵ�ʹ�ܱ�־λ 



//------------ ���ٱ���---------------------  
SPD_EXP speed_Exd;



 GPRMC_PRO GPRMC_Funs=
 {
   Time_pro,
   Status_pro,
   Latitude_pro,
   Lat_NS_pro,
   Longitude_pro,
   Long_WE_pro,
   Speed_pro,
   Direction_pro,
   Date_pro
 };


//--------  GPS prototcol----------------------------------------------------------------------------------
static u32 	fomer_time_seconds, tmp_time_secnonds, delta_time_seconds;
u8	        UDP_dataPacket_flag= 0x03;			   /*V	   0X03 	 ;		   A	  0X02*/
u8	        GPS_getfirst=0; 		 //  �״��о�γ�� 
u8          HDOP_value=99;         //  Hdop ��ֵ    
u8          Satelite_num=0;   // ���ǿ���
u8 CurrentTime[3];
u8 BakTime[3];
u8 Sdgps_Time[3];  // GPS ���� ʱ���¼   BCD ��ʽ

//static u8      UDP_AsciiTx[1800];     
 ALIGN(RT_ALIGN_SIZE)
u8      GPRS_info[900];  
u16     GPRS_infoWr_Tx=0; 

 ALIGN(RT_ALIGN_SIZE)
u8  UDP_HEX_Rx[1024];    // EM310 ��������hex 
u16 UDP_hexRx_len=0;    // hex ���� ����
u16 UDP_DecodeHex_Len=0;// UDP���պ�808 ���������ݳ���


GPS_RMC GPRMC; // GPMC��ʽ

        /*                         pGpsRmc->status,\
								pGpsRmc->latitude_value,\
								pGpsRmc->latitude,\
								pGpsRmc->longtitude_value,\
								pGpsRmc->longtitude,\
								pGpsRmc->speed,\
								pGpsRmc->azimuth_angle);
								*/


		
//----------808 Э�� -------------------------------------------------------------------------------------
u16	   GPS_Hight=0;               //   808Э��-> �߳�   m    
u16	   GPS_speed=0;			   //   808Э��-> �ٶ�   0.1km/h    
u16     GPS_direction=0;           //   808Э��-> ����   ��           
u16     Centre_FloatID=0; //  ������Ϣ��ˮ��
u16     Centre_CmdID=0;   //  ��������ID

u8      Original_info[850]; // û��ת�崦��ǰ��ԭʼ��Ϣ  
u16     Original_info_Wr=0; // ԭʼ��Ϣд��ַ
//---------- ��GPSУ׼����ϵ����� ----------------------------
u8      Speed_area=60; // У��Kֵ��Χ
u16     Speed_gps=0;  // ͨ��GPS����������ٶ� 0.1km/h
u8      Speed_Rec=0;  // �ٶȴ����� У��K�õĴ洢��
u16     Speed_cacu=0; // ͨ��Kֵ����������ٶ�
u16     Spd_adjust_counter=0; // ȷ������״̬������
u16     Former_DeltaPlus[K_adjust_Duration]; // ǰ����������� 
u8      Former_gpsSpd[K_adjust_Duration];// ǰ������ٶ�      
u8      DF_K_adjustState=0; // ����ϵ���Զ�У׼״̬˵��  1:�Զ�У׼��    0:��δ�Զ�У׼   
//-----  ��̨ע�ᶨʱ��  ----------
DevRegst   DEV_regist;  // ע��
DevLOGIN   DEV_Login;   //  ��Ȩ  




 
//------- �ı���Ϣ�·� -------
TEXT_INFO      TextInfo;    // �ı���Ϣ�·� 
//------- �¼� ----
EVENT          EventObj;    // �¼�   
EVENT          EventObj_8[8]; // �¼�  
//-------�ı���Ϣ-------
MSG_TEXT       TEXT_Obj;
MSG_TEXT       TEXT_Obj_8[8],TEXT_Obj_8bak[8];

//------ ����  --------
CENTRE_ASK     ASK_Centre;  // ��������
//------  ��Ϣ�㲥  ---
MSG_BRODCAST   MSG_BroadCast_Obj;    // ��Ϣ�㲥         
MSG_BRODCAST   MSG_Obj_8[8];  // ��Ϣ�㲥    
//------  �绰��  -----
PHONE_BOOK    PhoneBook,Rx_PhoneBOOK;   //  �绰��
PHONE_BOOK    PhoneBook_8[8];

//-----  �������� ------
VEHICLE_CONTROL Vech_Control; //  ��������     
//-----  ����Χ��  -----
POLYGEN_RAIL Rail_Polygen;   // �����Χ��
RECT_RAIL    Rail_Rectangle; // ����Χ��
CIRCLE_RAIL  Rail_Cycle;     // Բ��Χ��
//------- ��·���� -----
POINT        POINT_Obj;      // ·�ߵĹյ�
ROUTE        ROUTE_Obj;      // ·����� 
//-------    �г���¼��  -----
RECODER      Recode_Obj;     // �г���¼��  
//-------  ����  ----  
CAMERA        Camera_Obj;     //  �����������      
//-----   ¼��  ----
VOICE_RECODE  VoiceRec_Obj;   //  ¼������     
//------ ��ý��  --------
MULTIMEDIA    MediaObj;       // ��ý����Ϣ  
//-------  ������Ϣ͸��  -------
DATATRANS     DataTrans;      // ������Ϣ͸��   
//-------  ����Χ��״̬ --------
INOUT        InOut_Object;    // ����Χ��״̬    
//-------- ��ý�����  ------------
MEDIA_INDEX  MediaIndex;  // ��ý����Ϣ    
//------- ��������״̬ ---------------
u8  CarLoadState_Flag=1;//ѡ�г���״̬�ı�־   1:�ճ�   2:���   3:�س�

//------- ��ý����Ϣ����---------------
u8  Multimedia_Flag=1;//��Ҫ�ϴ��Ķ�ý����Ϣ����   1:��Ƶ   2:��Ƶ   3:ͼ��
u8  SpxBuf[SpxBuf_Size];  
u16 Spx_Wr=0,Spx_Rd=0;
u8  Duomeiti_sdFlag=0; 

//------- ¼����ʼ���߽���---------------
u8  Recor_Flag=1; //  1:¼����ʼ   2:¼������


//----------808Э�� -------------------------------------------------------------------------------------	
u8		SIM_code[6];							   // Ҫ���͵�IMSI	����
u8		IMSI_CODE[15]="000000000000000";							//SIM ����IMSI ����
u8		Warn_Status[4]		=
{
		0x00, 0x00,0x00,0x00
}; //  ������־λ״̬��Ϣ
u8		Car_Status[4]		=
{
		0x00, 0x00,0x00,0x00 
}; //  ����״̬��Ϣ	
T_GPS_Info_GPRS 	Gps_Gprs,Bak_GPS_gprs;	 
T_GPS_Info_GPRS	Temp_Gps_Gprs; 
u8   A_time[6]; // ��λʱ�̵�ʱ��

u8      ReadPhotoPageTotal=0;
u8      SendPHPacketFlag=0; ////�յ���������������һ��blockʱ��λ


//-------- �������� -------- 
u8		warn_flag= 0;		  
u8		f_Exigent_warning= 0;//0;     //�Ŷ� ��������װ�� (INT0 PD0)
u8		Send_warn_times= 0;     //   �豸�������ϱ��������������3 ��
u32  	fTimer3s_warncount=0;


//------  ���ſ������� -------
DOORCamera   DoorOpen;    //  ���س�������

//------- ������չЭ��  ------------
BD_EXTEND     BD_EXT;     //  ������չЭ��  

// ---- �յ� -----
u16  Inflexion_Current=0;
u16  Inflexion_Bak=0;      
u16  Inflexion_chgcnter=0; //�仯������
u16  InflexLarge_or_Small=0;     // �ж�curent �� Bak ��С    0 equql  1 large  2 small  
u16  InflexDelta_Accumulate=0;  //  ��ֵ�ۼ� 

// ----����״̬  ------------
u8  SleepState=0;  //   0  ������ACC on            1  ����Acc Off 
u8  SleepConfigFlag=0; //  ����ʱ���ͼ�Ȩ��־λ

//---- �̶��ļ���С --- 
u32 mp3_fsize=5616;
u8  mp3_sendstate=0;
u32 wmv_fsize=25964;
u8  wmv_sendstate=0;

//-------------------   ���� ---------------------------------------		
static u8 GPSsaveBuf[40];     // �洢GPS buffer
static u8	ISP_buffer[520];
static u16 GPSsaveBuf_Wr=0;


POSIT Posit[60];           // ÿ����λ����Ϣ�洢
u8    PosSaveFlag=0;      // �洢Pos ״̬λ 

NANDSVFlag   NandsaveFlg;
A_AckFlag    Adata_ACKflag;  // ����GPRSЭ�� ������� RS232 Э�鷵��״̬�Ĵ���
TCP_ACKFlag  SD_ACKflag;     // ����GPRSЭ�鷵��״̬��־ 
u8  SubCMD_8103H=0;            //  02 H���� ���ü�¼�ǰ�װ�����ظ� ������
u32  SubCMD_FF01H=0;            //  FF02 ������Ϣ��չ
u32  SubCMD_FF03H=0;     //  FF03  ������չ�ն˲�������1

u8  SubCMD_10H=0;            //  10H   ���ü�¼�Ƕ�λ�澯����
u8  OutGPS_Flag=0;     //  0  Ĭ��  1  ���ⲿ��Դ����
u8   Spd_senor_Null=0;  // �ֶ��������ٶ�Ϊ0
u32  Centre_DoubtRead=0;     //  ���Ķ�ȡ�¹��ɵ����ݵĶ��ֶ� 
u32  Centre_DoubtTotal=0;    //  ���Ķ�ȡ�¹��ɵ�����ֶ�
u8   Vehicle_RunStatus=0;    //  bit 0: ACC �� ��             1 ��  0��
                             //  bit 1: ͨ���ٶȴ�������֪    1 ��ʾ��ʻ  0 ��ʾֹͣ   
                             //  bit 2: ͨ��gps�ٶȸ�֪       1 ��ʾ��ʻ  0 ��ʾֹͣ  
u8   Status_TiredwhRst=0;    //  ����λʱ ƣ�ͼ�ʻ��״̬   0 :ͣ��  1:ͣ����û���� 2:�����˻�û����                              



u32   SrcFileSize=0,DestFilesize=0,SrcFile_read=0;
u8    SleepCounter=0; 

u16   DebugSpd=0;   //������GPS�ٶ�  
u8    MMedia2_Flag=0;  // �ϴ�������Ƶ ��ʵʱ��Ƶ  �ı�־λ    0 ������ 1 ��ʵʱ






//-----  ISP    Զ��������� -------
u8       f_ISP_ACK=0;   // Զ������Ӧ��	
u8       ISP_FCS[2];    //  �·���У��
u16      ISP_total_packnum=0;  // ISP  �ܰ���
u16      ISP_current_packnum=0;// ISP  ��ǰ����
u32      ISP_content_fcs=0;    // ISP  ������У��
u8       ISP_ack_resualt=0;    // ISP ��Ӧ
u8       ISP_rxCMD=0;          // ISP �յ�������
u8       f_ISP_88_ACK=0;       // Isp  ����Ӧ��
u8       ISP_running_state=0;  // Isp  ��������״̬
u8       f_ISP_23_ACK=0;    //  Isp  ���� �ļ���ɱ�ʶ
u16      ISP_running_counter=0;// Isp  ����״̬�Ĵ���
u8       ISP_RepeatCounter=0; //   ISP ���������ظ����� ����5��У��ʧ�ܲ�������


 u8     ISP_NeedtoProcessFlag=0;   //   ��Ҫ����ISP ����
 u8     ISP_Raw[600];                        //  ����ISP ����δ������ַ�


ISP_RSD  Isp_Resend;
unsigned short int FileTCB_CRC16=0;
unsigned short int Last_crc=0,crc_fcs=0;



//---------  ����Ӧ��  -----------
u8		 fCentre_ACK=0; 			  // ---------�ж�����Ӧ���־λ����
u8		 ACK_timer=0;				   //---------	ACK timer ��ʱ��--------------------- 
u16         ACKFromCenterCounter=0;//ʮ����Ӧ�����²���  
u8           Send_Rdy4ok=0;


//---------------  �ٶ��������--------------
u16  Delta_1s_Plus=0;
u16  Delta_1s_Plus2=0; 
u16  Sec_counter=0;


void K_AdjustUseGPS(u32 sp, u32  sp_DISP);  // ͨ��GPS У׼  K ֵ  (������ʻ1KM ��������Ŀ) 
u16  Protocol_808_Encode(u8 *Dest,u8 *Src, u16 srclen);
void Protocol_808_Decode(void);  // ����ָ��buffer :  UDP_HEX_Rx  
void Photo_send_end(void);
void Sound_send_end(void);
void Video_send_end(void);  
unsigned short int CRC16_file(unsigned short int num);  
void Spd_ExpInit(void);
void AvrgSpd_MintProcess(u8 hour,u8 min, u8 sec) ; 
u32   Distance_Point2Line(u32 Cur_Lat, u32  Cur_Longi,u32 P1_Lat,u32 P1_Longi,u32 P2_Lat,u32 P2_Longi);
void  RouteRail_Judge(u8* LatiStr,u8* LongiStr);  

//  A.  Total  

void delay_us(u16 j)
{
	u8 i;
	  while(j--)
	  	{
	  	i=3;
	  	while(i--);
	  	}  
}

void delay_ms(u16 j )
{
  while(j--)
  	{
           DF_delay_us(2000); // 1000
  	} 
}

 u8  Do_SendGPSReport_GPRS(void)   
{  	
  unsigned short int crc_file=0;
// 	  u8 i=0;
            if(DEV_Login.Operate_enable !=2) 
			 	{                  
				  if(1==DEV_Login.Enable_sd)
					  {
						Stuff_DevLogin_0102H();   //  ��Ȩ   ==2 ʱ��Ȩ���
						DEV_Login.Enable_sd=0;
						//------ ���ͼ�Ȩ���ж� ------------------
						//DEV_Login.Operate_enable=2;  //  �����жϼ�Ȩ��   
						return true; 
					  }
			 	}
            //------------------------ MultiMedia Send--------------------------
            if(MediaObj.Media_transmittingFlag==2)
            {
			    if(1==MediaObj.SD_Data_Flag) 
			 	{
                  Stuff_MultiMedia_Data_0801H();     
				  MediaObj.SD_Data_Flag=0; 
                  return true;
			 	}
			   return true; // ����808 Э��Ҫ�� �������ý������в������ͱ����Ϣ��
            } 
            if(1==DEV_regist.Enable_sd)
               {									
                  Stuff_RegisterPacket_0100H(0);   // ע��
                   JT808Conf_struct.Msg_Float_ID=0;
                  Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
		    DEV_regist.Enable_sd=0;
		    JT808Conf_struct.Regsiter_Status=1; //��עע�ᣬ�����洢		  
                return true;
               }
            if(1==DEV_regist.DeRegst_sd)
            	{
            	  Stuff_DeviceDeregister_0101H(); 
                  DEV_regist.DeRegst_sd=0;		     		  
				  return true;
            	}
            if((1==JT808Conf_struct.DURATION.Heart_SDFlag)&&(DataLink_Status())&&(SleepState==0)) //  ����
            	{
                  Stuff_DeviceHeartPacket_0002H();
                  JT808Conf_struct.DURATION.Heart_SDFlag=0;
				  JT808Conf_struct.DURATION.TCP_SD_state=1;  //��������� 1   
				  return true;  
            	} 			
            if((1==SleepConfigFlag)&&(DataLink_Status())&&(SleepState==1)) //  ����ʱ����
            	{
                 // Stuff_DevLogin_0102H();   //  ��Ȩ   ==2 ʱ��Ȩ���
                 // rt_kprintf("\r\n	 ����ʱ�ü�Ȩ�������� ! \r\n");  
                  SleepConfigFlag=0;  
				  return true;   
            	} 
			
            if(1==SD_ACKflag.f_CurrentPosition_0201H)    // λ����Ϣ��ѯ
            	{
                  Stuff_Current_Data_0201H();
				  SD_ACKflag.f_CurrentPosition_0201H=0;
				  return true;
            	} 
            if(1==SD_ACKflag.f_CurrentEventACK_0301H)   //  �¼�����
            	{
                  Stuff_EventACK_0301H();
				  SD_ACKflag.f_CurrentEventACK_0301H=0;
				  return true;                  
            	}
				            
	         if(2==ASK_Centre.ASK_SdFlag)      //  ����Ӧ�� 
	             	{
	                  Stuff_ASKACK_0302H();
	                  ASK_Centre.ASK_SdFlag=0; 
	                  return true;  
	             	}             	
             	
             if(1==Vech_Control.ACK_SD_Flag)   //  ����Ӧ�����
             	{
                  Stuff_ControlACK_0500H(); 
				  Vech_Control.ACK_SD_Flag=0;
				  return true;                  
             	}
			 
			 if(1==Recode_Obj.SD_Data_Flag)
			 	{
                   Stuff_RecoderACK_0700H();   //   �г���¼�������ϴ�  
                   Recode_Obj.SD_Data_Flag=0;
                   return true;  
			 	} 
			 if(SD_ACKflag.f_MsgBroadCast_0303H==1)
			 	{
                   Stuff_MSGACK_0303H(); 
                   SD_ACKflag.f_MsgBroadCast_0303H=0;
                   return true;
			 	}
			 if(1==MediaObj.SD_media_Flag)
			 	{
				   Stuff_MultiMedia_InfoSD_0800H();
				   MediaObj.SD_media_Flag=0;
                   return true;    
			 	}	
             if(DataTrans.Data_TxLen)
             	{ 
                  Stuff_DataTransTx_0900H();  // ����͸�� ��Զ������
                  DataTrans_Init();     //clear 
                  return true;
             	} 		
             if(SD_ACKflag.f_MediaIndexACK_0802H)
             	{
                   Stuff_MultiMedia_IndexAck_0802H();   // ��ý�������ϱ� 
                   SD_ACKflag.f_MediaIndexACK_0802H=0;
				   return true;
             	}			 
			 if(SD_ACKflag.f_DriverInfoSD_0702H)
			 	{
			 	   Stuff_DriverInfoSD_0702H();  //  ��ʻԱ��Ϣ�ϱ�
                   SD_ACKflag.f_DriverInfoSD_0702H=0;
                   return true;
			 	}
			  if(SD_ACKflag.f_Worklist_SD_0701H)
			 	{
			 	   Stuff_Worklist_0701H();  //   �����˵� 
                   SD_ACKflag.f_Worklist_SD_0701H=0;
                   return true;
			 	}
	        if(SD_ACKflag.f_CentreCMDack_0001H)
	         {
                               Stuff_DevCommmonACK_0001H();        
				   if(SD_ACKflag.f_CentreCMDack_0001H==2)  //  �޸�IP��������Ҫ�ز�
				   	   Close_DataLink();    //  AT_END    
				   else	   
				   if(SD_ACKflag.f_CentreCMDack_0001H==3) //   Զ�̸�λ
				   	{
                                       Systerm_Reset_counter=Max_SystemCounter;
					  ISP_resetFlag=2;    //   ����Զ�������������Ƹ�λϵͳ
				   	}
				   else
				   if(SD_ACKflag.f_CentreCMDack_0001H==5) //   �ر�����ͨ��
				   	{
				   	   Close_DataLink();  
                                      Stop_Communicate();      
				   	}				 
                               SD_ACKflag.f_CentreCMDack_0001H=0; 
				   SD_ACKflag.f_CentreCMDack_resualt=0;    
                   
				   
                   return true;    
				}  	 
	        if(SD_ACKflag.f_SettingPram_0104H)
	        	{
	        	  Stuff_SettingPram_0104H();
                  SD_ACKflag.f_SettingPram_0104H=0;
				   return true;
	        	}	

	     //----------   Զ��������� ------------		
            if((1==f_ISP_ACK)&&(1==TCP2_Connect))
             	{
				   Stuff_DataTrans_0900_ISP_ACK(ISPACK_92);
				   f_ISP_ACK=0;
				   return true;
             	} 
		    if((1==f_ISP_23_ACK)&&(1==TCP2_Connect)) 
             	{
				   Stuff_DataTrans_0900_ISP_ACK(ISPoverACK_96);  
				   
				   crc_file=CRC16_file(ISP_total_packnum); 
				   
				   if(crc_file==FileTCB_CRC16)
						ISP_file_Check();//׼�����³���
				   else
						{    // ���Զ���������� 
						     
							 rt_kprintf("\r\n���鲻�����  Caculate_crc=%x ReadCrc=%x ",crc_file,FileTCB_CRC16); 
						        DF_ClearUpdate_Area();   
						}
							   
				   f_ISP_23_ACK=0;
				   return true; 
             	}
			//---------------------------------------------------
			 if((1==f_ISP_88_ACK)&&(1==TCP2_Connect))
                    {	
				Stuff_DataTrans_0900_ISP_ACK(ISPinfoACK_94);
				rt_kprintf( "\r\n CurrentPack_num:%u TotalPack_num %u \r\n",ISP_current_packnum,ISP_total_packnum); 
			 	f_ISP_88_ACK=0;
				return true;
			  }	
	            if((1==TCP2_Connect)&&(TCP2_sdFlag==1)&&(ISP_running_state==0))
		   	{
		   	      
                          Stuff_RegisterPacket_0100H(1);  //    ISP ������  
                          rt_kprintf("\r\nISP--heart\r\n"); 
                          TCP2_sdFlag=0;
                           return true;
		   	}
	         //----------------------Զ������������------------------ 		
			
			// if((Current_SD_Duration<=10)||(Current_State==1))   // ����ʱ30  ʵ����10 Current_SD_Duration
			// {	
			 if(PositionSD_Status()&&(DEV_Login.Operate_enable==2)&&((enable==BD_EXT.Trans_GNSS_Flag)||(DispContent==6))||(Current_UDP_sd&&PositionSD_Status()&&(DEV_Login.Operate_enable==2))||((DF_LOCK==enable)&&PositionSD_Status()))	  //�״ζ�λ�ٷ� 
			   //  if((PositionSD_Status())&&(DataLink_Status())&&(DEV_Login.Operate_enable==2))	                                                                                                                     // DF  �������͵�ǰλ����Ϣ  
		      {
		                  PositionSD_Disable();
				   Current_UDP_sd=0; 
                              //1.   ʱ�䳬ǰ�ж�
                              //if(Time_FastJudge()==false)
							//return false;		
				  // 2. 			  
			 	   Stuff_Current_Data_0200H();  // �ϱ���ʱ����  				   
				   //----Ӧ����� ----		   
				  // ACKFromCenterCounter++; // ֻ��עӦ������������Ӧ��ʱ�� 
				   //---------------------------------------------------------------------------------
				   if(DispContent)	
					    rt_kprintf("\r\n ���� GPS -current !\r\n");     
			    }   
			 //}
			 else 
             if((RdCycle_RdytoSD==ReadCycle_status)&&(0==ISP_running_state)&&(DataLink_Status())&&(DEV_Login.Operate_enable==2))    // ��ȡ����--------- ����GPS 
             {                                       /* Զ������ʱ�������ϱ�GPS ����Ϊ�п��ܷ��Ͷ�λ����ʱ
                                                          ���ڽ��մ����������������ݰ�,����GSMģ�鴦�������������ǵ�Ƭ����������*/
                  if (false==Stuff_Normal_Data_0200H())        
				 	return false;
				  fCentre_ACK=1;// �ж�Ӧ��
				 //-------- change status  Ready  ACK  ------ 
			         ReadCycle_status=RdCycle_SdOver;
				  Send_Rdy4ok=1;  // enable
				  //----Ӧ����� ----		  
				  ACKFromCenterCounter++; 
				   if(DispContent)	
					  rt_kprintf("\r\n ���� GPS --saved  OK!\r\n");    
				 return true; 
             }			 
 			//-------------------------------------------------------------
			return  false; 
             
}
void strtrim(u8 *s, u8 c)
{
	u8		 *p1, *p2;
	u16  i, j;

	if (s == 0) return;

	// delete the trailing characters
	if (*s == 0) return;
	j = strlen((char const *)s);
	p1 = s + j;
	for (i = 0; i < j; i++)
	{
	   	p1--;
	   	if (*p1 != c) break;
	}
	if (i < j) p1++;
	*p1 = 0;	// null terminate the undesired trailing characters

	// delete the leading characters
  	p1 = s;
	if (*p1 == 0) return;
	for (i = 0; *p1++ == c; i++);
	if (i > 0)
	{
		p2 = s;
	 	p1--;
		for (; *p1 != 0;) *p2++ = *p1++;
		*p2 = 0;
	}
} 

int str2ip(char *buf, u8 *ip)
{	// convert an ip:port string into a binary values
	int	i;
	u16	_ip[4]; 
	

	memset(_ip, 0, sizeof(_ip));

	strtrim((u8*)buf, ' ');
   
	i = sscanf(buf, "%u.%u.%u.%u", (u32*)&_ip[0], (u32*)&_ip[1], (u32*)&_ip[2], (u32*)&_ip[3]);

	*(u8*)(ip + 0) = (u8)_ip[0];
	*(u8*)(ip + 1) = (u8)_ip[1];
	*(u8*)(ip + 2) = (u8)_ip[2];
	*(u8*)(ip + 3) = (u8)_ip[3];

	return i;
}



int IP_Str(char *buf, u32 IP)
{
	T_IP_Addr	ip;
    
	if (!buf) return 0;

	ip.ip32 = IP;
    
	return sprintf(buf, "%u.%u.%u.%u", ip.ip8[0], ip.ip8[1], ip.ip8[2], ip.ip8[3]);
}

u16 AsciiToGb(u8 *dec,u8 InstrLen,u8 *scr)
{
u16 i=0,j=0,m=0;
u16 Info_len=0;


for(i=0,j=0;i<InstrLen;i++,j++) 
	{
	m=scr[i];
	if((m>=0x30)&&(m<=0x39))
		{
		memcpy(&dec[j],&arr_A3B0[(m-'0')*2],2);
		j++;
		}
	else if((m>=0x41)&&(m<=0x4f))
		{
		memcpy(&dec[j],&arr_A3C0[(m-0x41+1)*2],2);
		j++;
		}
	else if((m>=0x50)&&(m<=0x5a))
		{
		memcpy(&dec[j],&arr_A3D0[(m-0x50)*2],2) ;
		j++;
		}
	else if((m>=0x61)&&(m<=0x6f))
		{
		memcpy(&dec[j],&arr_A3E0[(m-0x61+1)*2],2) ;
		j++;
		}
	else if((m>=0x70)&&(m<=0x7a))
		{
		memcpy(&dec[j],&arr_A3F0[(m-0x70)*2],2)  ;
		j++;
		}
	else
		{
		dec[j]=m;
		}	
	}
Info_len=j;
return Info_len;
}


// B.   Protocol
 
  
//==================================================================================================
// ��һ���� :   ������GPS ����ת����غ��� 
//==================================================================================================

void Time_pro(u8 *tmpinfo, u8 hour, u8 min , u8 sec)
{
  	            //---- record  to memory
				GPRMC.utc_hour=hour;
				GPRMC.utc_min=min;
				GPRMC.utc_sec=sec;
                
				CurrentTime[0] = hour;
				CurrentTime[1] = min;
				CurrentTime[2] = sec; 
                
				//-----------  ���ͨЭ�� ------------- 	 
				Temp_Gps_Gprs.Time[0] = hour;
				Temp_Gps_Gprs.Time[1] = ( tmpinfo[2] - 0x30 ) * 10 + tmpinfo[3] - 0x30;
				Temp_Gps_Gprs.Time[2] = ( tmpinfo[4] - 0x30 ) * 10 + tmpinfo[5] - 0x30;  
            
}

void Status_pro(u8 *tmpinfo) 
{
 	                GPRMC.status=tmpinfo[0];
  
					//-------------------------���ͨЭ��-----------------------------
						   if ( tmpinfo[0] == 'V' || tmpinfo[0] == 'v' )
						  {
								UDP_dataPacket_flag=0X03;
								StatusReg_GPS_V();
						  }
						  else if ( tmpinfo[0] == 'A' || tmpinfo[0] == 'a' )
						  {
								UDP_dataPacket_flag=0X02;		
								StatusReg_GPS_A(); 
								  
						  }

					//---------------------------------------------------------
	

}

void Latitude_pro(u8 *tmpinfo)
{
  u32  latitude;
   GPRMC.latitude_value=atof((char *)tmpinfo);
   /*     Latitude  
          ddmm.mmmm
    */
  
	//--------	808 Э�� --------------------
  if(UDP_dataPacket_flag==0X02)    //��ȷ�������֮һ��
  {  
    
    //------------  dd part   -------- 
	latitude = ( u32 ) ( ( tmpinfo[0] - 0x30 ) * 10 + ( u32 ) ( tmpinfo[1] - 0x30 ) ) * 1000000;
	//------------  mm  part  -----------
	/*    ת���ɰ����֮һ��
	      mm.mmmm   *  1000000/60=mm.mmmm*50000/3=mm.mmmm*10000*5/3
	*/
	latitude = latitude + ( u32 )( (( tmpinfo[2] - 0x30 ) * 100000+ (tmpinfo[3] - 0x30 ) * 10000+(tmpinfo[5] - 0x30 ) * 1000 + ( tmpinfo[6] - 0x30 ) * 100 + ( tmpinfo[7] - 0x30 ) * 10 +( tmpinfo[8] - 0x30 ))*5/3);  

	 if(latitude==0)
	 	{
	 	   GPS_getfirst=0;
	          StatusReg_GPS_V();
	          return;
		}

	Temp_Gps_Gprs.Latitude[0] = ( u8 ) ( latitude >> 24 );
	Temp_Gps_Gprs.Latitude[1] = ( u8 ) ( latitude >> 16 );
	Temp_Gps_Gprs.Latitude[2] = ( u8 ) ( latitude >> 8 );
	Temp_Gps_Gprs.Latitude[3] = ( u8 ) latitude;
  } 
   //----------------------------------------------
}
 
void Lat_NS_pro(u8 *tmpinfo)
{
   GPRMC.latitude=tmpinfo[0];
}

void Longitude_pro(u8 *tmpinfo)
{
 u32  longtitude;
 GPRMC.longtitude_value=atof((char *)tmpinfo);  
  /*     Latitude  
          dddmm.mmmm
    */
 //--------  808Э��  ---------
 if(UDP_dataPacket_flag==0X02)  //��ȷ�������֮һ��
 {
     //------  ddd part -------------------
	 longtitude = ( u32 )( ( tmpinfo[0] - 0x30 ) * 100 + ( tmpinfo[1] - 0x30 ) * 10 + ( tmpinfo[2] - 0x30 ) ) * 1000000;  
	 //------  mm.mmmm --------------------	 
	 /*    ת���ɰ����֮һ��
		   mm.mmmm	 *	1000000/60=mm.mmmm*50000/3=mm.mmmm*10000*5/3
	 */
	 longtitude = longtitude + ( u32 ) (( ( tmpinfo[3] - 0x30 ) * 100000 + ( tmpinfo[4] - 0x30 )*10000+(tmpinfo[6] - 0x30 ) * 1000 + ( tmpinfo[7] - 0x30 ) * 100 + ( tmpinfo[8] - 0x30 ) * 10 + ( tmpinfo[9] - 0x30 ))*5/3);
	 if(longtitude==0)
	 	{GPS_getfirst=0; StatusReg_GPS_V();return;} 
	 
	 Temp_Gps_Gprs.Longitude[0] = ( u8 ) ( longtitude >> 24 );
	 Temp_Gps_Gprs.Longitude[1] = ( u8 ) ( longtitude >> 16 );
	 Temp_Gps_Gprs.Longitude[2] = ( u8 ) ( longtitude >> 8 );
	 Temp_Gps_Gprs.Longitude[3] = ( u8 ) longtitude; 
 }			   
	 
   //---------------------------------------------------  
}

void Long_WE_pro(u8 *tmpinfo)
{

	GPRMC.longtitude=tmpinfo[0]; 
}


void Speed_pro(u8 *tmpinfo,u8 Invalue,u8 Point)
{ 
  u32	  sp=0,sp_DISP=0;
  u32     reg=0;  

  
    //-------------------------------------------------------------------------------------------------------------
	if(Invalue==INIT)
	{	  
	  return;
	}
	else//---------------------------------------------------------------------------------------------------------
	{
	 GPRMC.speed=atof((char *)tmpinfo);
	 //---------------------------------------------------					  
	 if(UDP_dataPacket_flag==0x02 )
	 {
			  //-----808 Э�� -------------- 
			 //�����ֽڵ�λ0.1 km/h  
			 if ( Point == 1 )	//0.0-9.9=>
			 {
												  //++++++++  Nathan Modify on 2008-12-1   ++++++++++
					if((tmpinfo[0]>=0x30)&&(tmpinfo[0]<=0x39)&&(tmpinfo[2]>=0x30)&&(tmpinfo[2]<=0x39))
					 {
					   sp = ( tmpinfo[0] - 0x30 ) * 10 + ( tmpinfo[2] - 0x30 );  //����10��
					 }	   
					else
					   return;			  
				 
			 }
			 else if ( Point == 2 )  //10.0-99.9
			 {
											  //++++++++  Nathan Modify on 2008-12-1   ++++++++++
					if((tmpinfo[0]>=0x30)&&(tmpinfo[0]<=0x39)&&(tmpinfo[1]>=0x30)&&(tmpinfo[1]<=0x39)&&(tmpinfo[3]>=0x30)&&(tmpinfo[3]<=0x39)) 
					{
					  sp = ( tmpinfo[0] - 0x30 ) * 100 + ( tmpinfo[1] - 0x30 ) * 10 + tmpinfo[3] - 0x30;
					}
					else
					 return; 									 
	 
			 }
			 else if( Point == 3 ) //100.0-999.9
			 {
												   //++++++++  Nathan Modify on 2008-12-1	++++++++++
					if((tmpinfo[0]>=0x30)&&(tmpinfo[0]<=0x39)&&(tmpinfo[1]>=0x30)&&(tmpinfo[1]<=0x39)&&(tmpinfo[2]>=0x30)&&(tmpinfo[2]<=0x39)&&(tmpinfo[4]>=0x30)&&(tmpinfo[4]<=0x39)) 
					 {
					   sp = ( tmpinfo[0] - 0x30 ) * 1000 + ( tmpinfo[1] - 0x30 ) * 100 + ( tmpinfo[2] - 0x30 ) * 10 + tmpinfo[4] - 0x30;
					  }
					else
					  return;  
	 
			 }
		 else		 
			 {
			     if(JT808Conf_struct.Speed_GetType==0)
			          GPS_speed=0;
			 }
	 
		 // --------  sp ��ǰ��0.1 knot------------------	 
		  sp= (u32)(sp * 185.6) ;  //  1 ����=1.856 ǧ��  ������m/h
								 
		  if(sp>220000)   //ʱ�ٴ���220km/h���޳�
			  return;  
	 
		  sp_DISP=sp/100;   //  sp_Disp ��λ�� 0.1km/h 
							  
	         //------------------------------ ͨ��GPSģ�����ݻ�ȡ�����ٶ� --------------------------------
                   Speed_gps=(u16)sp_DISP;
	         //---------------------------------------------------------------------------
       if(JT808Conf_struct.Speed_GetType)  // ͨ���ٶȴ����� ��ȡ�ٶ�
       	{ 
              K_AdjustUseGPS(sp,sp_DISP);  //  ����Kֵ    
              if(JT808Conf_struct.DF_K_adjustState==0)
			  {
			     // ---  ��δУ׼ǰ����õ����ٶ���ͨ��GPS����õ���
			     GPS_speed=Speed_gps;      
			       //------- GPS	��̼���-------- 
				 if(sp>=5000)		//	�������Ư��  �ٶȴ���
				 {
					 reg=sp/3600;  // ����3600 ��m/s 
					 JT808Conf_struct.Distance_m_u32+=reg;
					 if(JT808Conf_struct.Distance_m_u32>0xFFFFFF)
							  JT808Conf_struct.Distance_m_u32=0; 	  //������ô����	

					 //----- ����ش�����---  
                     if(1==JT808Conf_struct.SD_MODE.DIST_TOTALMODE)
                     {
                        DistanceAccumulate+=reg;
                        if(DistanceAccumulate>=Current_SD_Distance)
                        	{
                               DistanceAccumulate=0;
				   PositionSD_Enable();        //����
				   Current_UDP_sd=1;
                        	}
                     }
					 //------- ���ദ����� -----
							  
				 }	
				
              }  
       	}  
	   else
	   	{  // ��GPS ȡ�ٶ�
			     //------- GPS	��̼���-------- 
				 if(sp>=5000)		//	�������Ư��  �ٶȴ���
				 {
					 JT808Conf_struct.Distance_m_u32+=sp/3600;  // ����3600 ��m/s 
					 if(JT808Conf_struct.Distance_m_u32>0xFFFFFF)
							  JT808Conf_struct.Distance_m_u32=0; 	  //������ô����	  

					 //----- ����ش�����---  
                     if(1==JT808Conf_struct.SD_MODE.DIST_TOTALMODE)
                     {
                        DistanceAccumulate+=reg;
                        if(DistanceAccumulate>=Current_SD_Distance)
                        	{
                               DistanceAccumulate=0;
					PositionSD_Enable();          //����
					Current_UDP_sd=1;
                        	}
                     }
					 //------- ���ദ����� -----		   
				 }			 
					
					GPS_speed=Speed_gps;    // ��GPS���ݼ���õ��ٶ� ��λ0.1km/h
					
		            //-----------------------------------------------
					
	   	     }
			 // if(DispContent==2) 
			  //  rt_kprintf("\r\n				  �ٶ�: %d Km/h\r\n",GPS_speed/10);     
	 }
	 else if ( UDP_dataPacket_flag == 0x03 )
	 { 
       if(0==JT808Conf_struct.Speed_GetType)  
		{
			 //----- GPS ��ʱ�ٶ�	km/h  ---------
		     GPS_speed=0;	 
       	} 
        if(JT808Conf_struct.Speed_GetType)  // ͨ���ٶȴ����� ��ȡ�ٶ�
       	{ 
              K_AdjustUseGPS(sp,sp_DISP);  //  ����Kֵ  
              GPS_speed=Speed_cacu;      
       	}      
	   Speed_gps=0;
	   if(DispContent==2)
		   rt_kprintf("\r\n 2 GPSû��λ\r\n"); 
	 }  		
	}
   //---------------------------------------------------
}


void Direction_pro(u8 *tmpinfo,u8 Invalue,u8 Point)
{
   u32	  sp=0;  
  //------------------------------------------------------------------------------------------------
  if(Invalue==INIT)
  {    
    return;
  }
  else//-------------------------------------------------------------------------------------------
  {
   GPRMC.azimuth_angle=atof((char *)tmpinfo);
	
   
   //--------------808 Э��  1 ��------------------------- 	 
   if ( UDP_dataPacket_flag== 0x02 )
   {	  
		  
		
		   if ( Point == 1 )   //5.8 
		   {
					  if((tmpinfo[0]>=0x30)&&(tmpinfo[0]<=0x39)&&(tmpinfo[2]>=0x30)&&(tmpinfo[2]<=0x39))
				   sp = ( tmpinfo[0] - 0x30 ) ;
				  else 
				   return;
				   
		   }
		   else if ( Point == 2 )  // 14.7
		   {
					  if((tmpinfo[0]>=0x30)&&(tmpinfo[0]<=0x39)&&(tmpinfo[1]>=0x30)&&(tmpinfo[1]<=0x39)&&(tmpinfo[3]>=0x30)&&(tmpinfo[3]<=0x39)) 
				   sp = ( tmpinfo[0] - 0x30 ) * 10 + ( tmpinfo[1] - 0x30 );
				  else
				   return;
		   
		   }
		   else    //357.38
		   if ( Point == 3 )
		   {
			      if((tmpinfo[0]>=0x30)&&(tmpinfo[0]<=0x39)&&(tmpinfo[1]>=0x30)&&(tmpinfo[1]<=0x39)&&(tmpinfo[2]>=0x30)&&(tmpinfo[2]<=0x39)&&(tmpinfo[4]>=0x30)&&(tmpinfo[4]<=0x39)) 
				   sp = ( tmpinfo[0] - 0x30 ) * 100 + ( tmpinfo[1] - 0x30 ) * 10 + ( tmpinfo[2] - 0x30 ) ;
				  else
				   return;
   
	       }
		   else
		   	{
               sp=0;   
		   	}
		   GPS_direction=sp;  //  ��λ 1��

	   //----------  �յ㲹�����   ----------
	  // Inflexion_Process();
       
	   
		   
   }
   else if ( UDP_dataPacket_flag == 0x03 )
   {
          GPS_direction=0; 		  
 
   }


    return;
  }
 


}

void Date_pro(u8 *tmpinfo,u8 fDateModify, u8 hour, u8 min , u8 sec)
{
 uint8_t  year=0,mon=0,day=0;  
 TDateTime now;

 
 day = (( tmpinfo[0] - 0x30 ) * 10 ) + ( tmpinfo[1] - 0x30 );
 mon = (( tmpinfo[2] - 0x30 ) * 10 ) + ( tmpinfo[3] - 0x30 );
 year = (( tmpinfo[4] - 0x30 ) * 10 ) + ( tmpinfo[5] - 0x30 );
 
 if(fDateModify){
	 //sscanf(tmpinfo,"%2d%2d%2d",&day,&mon,&year);
	 day++;
	 if(mon == 2){
		 if ( ( year % 4 ) == 0 ){
			 if ( day == 30 ){day = 1;mon++;}
		 }else if ( day == 29 ){ day = 1;mon++;}
	 }else if (( mon == 4 ) || ( mon == 6 ) || ( mon == 9 ) || ( mon == 11 )){
		 if ( day == 31 ){mon++;day = 1;}
	 }else{
		 if ( day == 32 ){mon++;day = 1;}	 
		 if( mon == 13 ){mon = 1;year++; }
	 }
 }
 GPRMC.utc_year=year;
 GPRMC.utc_mon=mon;
 GPRMC.utc_day=day;
 if((sec==0)&&(GPRMC.status=='A')){
	 now.year = year;
	 now.month = mon;	   
	 now.day = day;
	 now.hour = hour;
	 now.min = min;
	 now.sec = sec;
	 now.week=1;      
	 Device_RTC_set(now);  
 }
 //------------------------------------------------ 	
 if(GPRMC.status=='A')     //  ��¼��λʱ��
{
  Time2BCD(A_time);  
  //------- Debug �洢 ÿ��ľ�γ��  || ʵ��Ӧ���� �洢ÿ���ӵ�λ��  -----   
  //  ���ݳ���55��ÿ����£���Ĵ����м�¼������ÿ���������һ����λ�ľ�γ�� ,Ԥ��5�����ڴ洢��һСʱ��λ�� 
  if(sec<55)
  	{
      memcpy(Posit[min].latitude_BgEnd,Gps_Gprs.Latitude,4); //��γ
      memcpy(Posit[min].longitude_BgEnd,Gps_Gprs.Longitude,4); //����	  
      Posit[min].longitude_BgEnd[0]|=0x80;//  ����
  	}
  if((min==59)&&(sec==55))
    { // ÿ��Сʱ��λ����Ϣ               
       NandsaveFlg.MintPosit_SaveFlag=1; 
	}  
 }   
  //---- �洢��ǰ����ʼ���  ����ʱ------------
   if((hour==0)&&(min==0)&&(sec<3))   // �洢3��ȷ���洢�ɹ� 
  	{ 
	  JT808Conf_struct.DayStartDistance_32=JT808Conf_struct.Distance_m_u32;
          Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct)); 
       } 
 
 //-------------------------------------------------
      //---------  ���ͨЭ��  -------
                       
						//if(systemTick_TriggerGPS==0)
 {
   Temp_Gps_Gprs.Date[0] = year;
   Temp_Gps_Gprs.Date[1] = mon;
   Temp_Gps_Gprs.Date[2] = day;   						
  }

  
 //-------------- ������ƽ���ٶ� ----------------
 AvrgSpd_MintProcess(hour,min,sec); 
}


//---------  GGA --------------------------
void HDop_pro(u8 *tmpinfo)
{ 
 float dop;
 
  dop=atof((char *)tmpinfo);				        
  HDOP_value=dop;		 //  Hdop ��ֵ

}

void  GPS_Delta_DurPro(void)    //��GPS �����ϱ�������
{
  if(1==JT808Conf_struct.SD_MODE.DUR_TOTALMODE)   // ��ʱ�ϱ�ģʽ
  {
	 	//----- ��һ�����ݼ�¼��ʱ��
		fomer_time_seconds = ( u32 ) ( BakTime[0] * 60 * 60 ) + ( u32 ) ( BakTime[1] * 60 ) + ( u32 ) BakTime[2];  
		
		//-----  ��ǰ���ݼ�¼��ʱ��
		tmp_time_secnonds = ( u32 ) ( CurrentTime[0] * 60 * 60 ) + ( u32 ) ( CurrentTime[1] * 60 ) + ( u32 )  CurrentTime[2];
		
		//һ��86400��
		
		if ( tmp_time_secnonds > fomer_time_seconds )
		{
				delta_time_seconds = tmp_time_secnonds - fomer_time_seconds;
				//systemTickGPS_Clear();
		}
		else if(tmp_time_secnonds < fomer_time_seconds)
		{
				delta_time_seconds = 86400 - fomer_time_seconds + tmp_time_secnonds;
				//systemTickGPS_Clear();
		}
		else 
		 {
			 // systemTickGPS_Set();  
			  UDP_dataPacket_flag=0X03; 
			  StatusReg_GPS_V();
		 }	
		
		if((SleepState==1)&&(delta_time_seconds==(Current_SD_Duration-5)))  //  --  ����ʱ �ȷ���Ȩ
		   {
			  SleepConfigFlag=1;  //����ǰ5 ����һ����Ȩ
		   }            
		
		if((delta_time_seconds >= Current_SD_Duration))//limitSend_idle
		  {
			  PositionSD_Enable();    
			  memcpy(BakTime,CurrentTime,3); // update   
		  }
  	}  
  
    //------------------------------ do this every  second-----------------------------------------    
	memcpy((char*)&Gps_Gprs,(char*)&Temp_Gps_Gprs,sizeof(Temp_Gps_Gprs));  

   //------  ����Χ�� �ж�  ----------
  /*  if((Temp_Gps_Gprs.Time[2]%20)==0) //   ��֤ʱ�����Բ�ε���Χ��
    {
        CycleRail_Judge(Temp_Gps_Gprs.Latitude,Temp_Gps_Gprs.Longitude);
		//rt_kprintf("\r\n --- �ж�Բ�ε���Χ��");
    }	*/
    // if((Temp_Gps_Gprs.Time[2]==5)||(Temp_Gps_Gprs.Time[2]==25)||(Temp_Gps_Gprs.Time[2]==45)) //  
    if(Temp_Gps_Gprs.Time[2]%2==0)//    ��֤ʱҪ��2 ��
   {
        RectangleRail_Judge(Temp_Gps_Gprs.Latitude,Temp_Gps_Gprs.Longitude);
		//rt_kprintf("\r\n -----�жϾ��ε���Χ��"); 
    }
      if((Temp_Gps_Gprs.Time[2]%5)==0) //    
    {
          // printf("\r\n --- �ж�Բ�ε���Χ��");
           RouteRail_Judge(Temp_Gps_Gprs.Latitude,Temp_Gps_Gprs.Longitude);
    }
	//rt_kprintf("\r\n Delta_seconds %d \r\n",delta_time_seconds);    
}

//----------------------------------------------------------------------------------------------------
void  SpeedSensorProcess(void)   // ͨ���������ٶȴ�������� �ٶ� ���������
{
  u32  Distance_1s_m=0;  // һ�������� ������
//  u32 sp=0;
  //  1. ��Kֵ�����ٶ�   -----------------------------------------------------------
     /*
           K ��ʾ ÿ���� ������ 
           1��/������ :   K/1000 
           Delta_1s_Plus: ÿ���Ӳɼ�����������    
           ÿ����ʻ����:  Delta_1s_Plus *1000/K
           => Delta_1s_Plus *1000/K*3.6*10 ��λ:0.1  km/h  =>Delta_1s_Plus *36000/K  ��λ:0.1 km/h
      */
    Speed_cacu=(Delta_1s_Plus*36000)/JT808Conf_struct.Vech_Character_Value;  // ͨ������õ����ٶ� 
    GPS_speed=Speed_cacu;  //�Ѽ���õ��Ĵ������ٶȸ� Э�� �Ĵ���
    Distance_1s_m=(Delta_1s_Plus*1000)/JT808Conf_struct.Vech_Character_Value;  // ÿ�����ж�����
  // 2. ����������  -------------------------------------------------------------
          //------------------------------------
		ModuleStatus|=Status_Pcheck;	
		 
   
		  //------- GPS  ��̼���  -------- 
		  JT808Conf_struct.Distance_m_u32+=Distance_1s_m;	// ����3600 ��m/s 
		  if(JT808Conf_struct.Distance_m_u32>0xFFFFFF)
				   JT808Conf_struct.Distance_m_u32=0;	   //������ô����	   

  // ------------------------------------------------------------------------------ 
}
//---------------------------------------------------------------------------------------------------
void K_AdjustUseGPS(u32 sp, u32  sp_DISP)  // ͨ��GPS У׼  K ֵ  (������ʻ1KM ��������Ŀ) 
{
      
  u32 Reg_distance=0;
  u32 Reg_plusNum=0;
  u16 i=0;
  
  if(JT808Conf_struct.DF_K_adjustState)   // ֻ��ûУ׼ʱ����Ч
			return;
		
   Speed_Rec=(u8)(sp_DISP/10);	// GPS�ٶ�    ��λ:km/h
   // -------	Ҫ���ٶ���60��65km/h  -------------
   if(((Speed_Rec>=Speed_area)&&(Speed_Rec<=(Speed_area+8)))||((Speed_Rec>=40)&&(Speed_Rec<=(40+8)))||((Speed_Rec>=70)&&(Speed_Rec<=(70+8))))   // Speed_area=60   
 // if(Speed_Rec>=Speed_area) 
//   if((Speed_Rec>=40)&&(Speed_Rec<=48))   // Speed_area=60    
   {
	   Spd_adjust_counter++;    
	   if(Spd_adjust_counter>K_adjust_Duration)  //�������ٶ���60~65����Ϊ�Ѿ���������
	   {
		   // �û�ȡ��������GPS�ٶ���Ϊ��׼���͸��ݴ���������������ٶȣ���Kֵ��У׼
		   Reg_distance=0;	// clear
		   Reg_plusNum=0;	// clear 
		   for(i=0;i<K_adjust_Duration;i++)
		   {
			  Reg_distance+=Former_gpsSpd[i];  // ����3.6km/h ��ʾ���������˶�����
			  Reg_plusNum+=Former_DeltaPlus[i]; 						   
		   }
           /*
                ��һ���ж�  �� ����ٶȴ����������ã� ��ô���أ� 
             */
           if(Reg_plusNum<20)
		   	 {
		   	   Spd_adjust_counter=0; 
			   rt_kprintf("\r\n    �ٶȴ����� û������!\r\n");
		   	   return; 
           	 }  		   	
		   //===================================================================
		   // ת���ɸ���GPS�ٶȼ�����ʻ�˶����ף�(�ܾ���) ������ڳ���3.6 ��Ϊ�˼��㷽�� ��x10  �ٳ���36 
		   Reg_distance=(u32)(Reg_distance*10/36); // ת���ɸ���GPS�ٶȼ�����ʻ�˶����ף�(�ܾ���)
		   // (Reg_plusNum/Reg_distance) ��ʾ�������������Ծ���(��)= ÿ�ײ������ٸ����� ����ΪKֵ��1000��������������Ӧ�ó���1000
		   JT808Conf_struct.Vech_Character_Value=1000*Reg_plusNum/Reg_distance;
		   //-------  �洢�µ�����ϵ�� -------------------------------- 						
		   JT808Conf_struct.DF_K_adjustState=1;					// clear  Flag
		   ModuleStatus|=Status_Pcheck;
		   Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
		   
		  Spd_adjust_counter=0; // clear  counter
	   }
	   else
	   {   //-------- ��¼�涨ʱ���ڵ���������GPS�ٶ�----------
		   Former_gpsSpd[Spd_adjust_counter]=Speed_Rec; 
		   Former_DeltaPlus[Spd_adjust_counter]=Delta_1s_Plus;
	   }
  
   }  
   else
	  Spd_adjust_counter=0;  // ֻҪ�ٶȳ���Ԥ�跶Χ ��������0
}
//==================================================================================================
// �ڶ����� :   �������ⲿ������״̬���
//==================================================================================================
/*    
     -----------------------------
     2.1   ��Э����صĹ��ܺ���
     ----------------------------- 
*/


/*    
     -----------------------------
    2.4  ��ͬЭ��״̬�Ĵ����仯
     ----------------------------- 
*/

void StatusReg_WARN_Enable(void)
{
  //     ��������״̬�� �Ĵ����ı仯
    Warn_Status[3] |= 0x01;//BIT( 0 );		   
}

void StatusReg_WARN_Clear(void)
{
  //     ��������Ĵ���  
    Warn_Status[3] &= ~0x01;//BIT( 0 );  	  	
}

void StatusReg_ACC_ON(void)
{  //    ACC ��
  Car_Status[3]|=0x01; //  Bit(0)     Set  1  ��ʾ ACC��

}

void StatusReg_ACC_OFF(void)
{  //    ACC ��  
  
  Car_Status[3]&=~0x01; //  Bit(0)     Set  01  ��ʾ ACC��
}

void StatusReg_POWER_CUT(void)
{  //  ����Դ�Ͽ�
   Warn_Status[2]	|= 0x01;//BIT( 0 );
   ModuleStatus|=Status_Battery;
}

void StatusReg_POWER_NORMAL(void)
{  // ����Դ����
   Warn_Status[2] &= ~0x01;//BIT( 0 );     
   ModuleStatus&=~Status_Battery;
}

void StatusReg_GPS_A(void)
{     // GPS ��λ 
   if(GPS_getfirst==0)
   {	    
	 #ifdef LCD_5inch
                   DwinLCD.Type=LCD_SETTIME;       
	  #endif
   }	  
 GPS_getfirst=1;
 Car_Status[3]|=0x02;   //Bit(1)
 ModuleStatus |= Status_GPS;	
}

void StatusReg_GPS_V(void)
{    //  GPS ����λ  
  Car_Status[3]&=~0x02;   //Bit(1)
  ModuleStatus &= ~Status_GPS;
}
	
void StatusReg_SPD_WARN(void)
{    //  ���ٱ���
   Warn_Status[3] |= 0x02;//BIT( 1 );
}

void StatusReg_SPD_NORMAL(void)
{    //  �ٶ�����
  Warn_Status[3] &=~ 0x02;//BIT( 1 );
}

void StatusReg_Relay_Cut(void)
{  // ���Ͷϵ�״̬ 

}

 void StatusReg_Relay_Normal(void)
{ //  ���͵�״̬����
 
}


void StatusReg_Default(void)
{    //   ״̬�Ĵ�����ԭĬ������
  
   Warn_Status[0]=0x00; //HH
   Warn_Status[1]=0x00; //HL  
   Warn_Status[2]=0x00; //LH 
   Warn_Status[3]=0x00; //LL	      
}

//==================================================================================================
// �������� :   ������GPRS���ߴ������Э��
//==================================================================================================
void  Save_GPS(void)     
{
   u16 counter_mainguffer,i;
   u8  lati_reg[4];//,regstatus;
   
		  if (PositionSD_Status())	
		  {
		         if(DF_LOCK==enable)    // ����ļ�����ʱ ����ֹ����DF
				 	return ;
		     //-------------------------------------------------------	 
		     //1.   ʱ�䳬ǰ�ж�
                              //if(Time_FastJudge()==false)    
							//return ;		 
				  //----------------------- Save GPS --------------------------------------
				  memset(GPSsaveBuf,0,40);
				  GPSsaveBuf_Wr=0;				  
				   //------------------------------- Stuff ----------------------------------------
					counter_mainguffer = GPSsaveBuf_Wr;
				   // 1. �澯״̬   4 Bytes
				   memcpy( ( char * ) GPSsaveBuf + GPSsaveBuf_Wr, ( char * )Warn_Status,4 );   
				   GPSsaveBuf_Wr += 4;  
                   // 2. ����״̬   4 Bytes
				   memcpy( ( char * ) GPSsaveBuf + GPSsaveBuf_Wr, ( char * )Car_Status,4 );   
				   GPSsaveBuf_Wr += 4; 
				   // 3.   γ��     4 Bytes
				   memcpy(lati_reg,Gps_Gprs.Latitude,4);
				   memcpy( ( char * ) GPSsaveBuf + GPSsaveBuf_Wr, ( char * )Gps_Gprs.Latitude, 4 );//γ��   modify by nathan
				   GPSsaveBuf_Wr += 4;
				   // 4.   ����     4 Bytes
				   memcpy( ( char * ) GPSsaveBuf + GPSsaveBuf_Wr, ( char * )  Gps_Gprs.Longitude, 4 );	 //����    ����  Bit 7->0	���� Bit 7 -> 1
				   GPSsaveBuf_Wr += 4;
				   // 5.  �߶�	  2 Bytes    m
				   GPSsaveBuf[GPSsaveBuf_Wr++]=(GPS_Hight>>8);	// High 			   
				   GPSsaveBuf[GPSsaveBuf_Wr++]=(u8)GPS_Hight;  // Low  
				   // 6.  �ٶ�	  2 Bytes     0.1Km/h
				   GPSsaveBuf[GPSsaveBuf_Wr++]=(Speed_gps>>8);	// High 			   
				   GPSsaveBuf[GPSsaveBuf_Wr++]=(u8)Speed_gps;  // Low 
				   // 7.  ����	  2 Bytes	    1�� 				   
				   GPSsaveBuf[GPSsaveBuf_Wr++]=(GPS_direction>>8);	//High
				   GPSsaveBuf[GPSsaveBuf_Wr++]=GPS_direction; // Low				   
				   // 8.  ����ʱ��	  6 Bytes 
				    GPSsaveBuf[GPSsaveBuf_Wr++]=(((Gps_Gprs.Date[0])/10)<<4)+((Gps_Gprs.Date[0])%10);  	   	
					GPSsaveBuf[GPSsaveBuf_Wr++]=((Gps_Gprs.Date[1]/10)<<4)+(Gps_Gprs.Date[1]%10); 
					GPSsaveBuf[GPSsaveBuf_Wr++]=((Gps_Gprs.Date[2]/10)<<4)+(Gps_Gprs.Date[2]%10);
					GPSsaveBuf[GPSsaveBuf_Wr++]=((Gps_Gprs.Time[0]/10)<<4)+(Gps_Gprs.Time[0]%10);
					GPSsaveBuf[GPSsaveBuf_Wr++]=((Gps_Gprs.Time[1]/10)<<4)+(Gps_Gprs.Time[1]%10);
					GPSsaveBuf[GPSsaveBuf_Wr++]=((Gps_Gprs.Time[2]/10)<<4)+(Gps_Gprs.Time[2]%10);    
                   //------------------------------------------------------------------ 
                   //   ע:   ��Ϊ�����������Դ洢ʱֻ�洢�������ٶȣ�����д ���ͺͳ��� 
                  //----------- ������Ϣ  ------------
				    //  ������Ϣ 1  -----------------------------    
					//	������Ϣ ID
					//Original_info[Original_info_Wr++]=0x03; // ��ʻ��¼�ǵ��ٶ�
					//	������Ϣ����
					//Original_info[Original_info_Wr++]=2;
					//	����
					//GPSsaveBuf[GPSsaveBuf_Wr++]=(u8)(Speed_cacu>>8);  
					//GPSsaveBuf[GPSsaveBuf_Wr++]=(u8)(Speed_cacu);	           
				   //--------------------------------------------------------------------
				if(strncmp(( char * )GPSsaveBuf + GPSsaveBuf_Wr-3,(const char*)Sdgps_Time,3)==0)						
					{
					/*if(strncmp((const char*)GPSsaveBuf + GPSsaveBuf_Wr-3,(const char*)Sdgps_Time,3)==0)
						{
						PositionSD_Disable();
						rt_kprintf("\r\n  -->���洢�ϱ�ʱ����ͬ�ĳ�\r\n");
						return;
						}
					else*/
						//{    //-------- ��RTC ʱ�� -----     
				        time_now=Get_RTC();  
						Time2BCD(GPSsaveBuf + GPSsaveBuf_Wr-6);  
						rt_kprintf("\r\n    ����RTCʱ����! \r\n");
						//}
					}
					memcpy(Sdgps_Time,GPSsaveBuf + GPSsaveBuf_Wr-3,3); //��������һ�δ洢ʱ��

                   //-------------  Caculate  FCS  -----------------------------------
					FCS_GPS_UDP=0;  
					for ( i = counter_mainguffer; i < 30; i++ )  
					{
							FCS_GPS_UDP ^= *( GPSsaveBuf + i ); 
					}			   //���ϱ����ݵ�����
					GPSsaveBuf[30] = FCS_GPS_UDP;  	  	
			     //-------------------------------- Save  ------------------------------------------
			      if(Api_cycle_write(GPSsaveBuf,31))
			      { 
					  if(DispContent) 	
					       rt_kprintf("\r\n    GPS Save succed\r\n");
			      	}
				 else
			       {
				  	  if(DispContent) 
				     	         rt_kprintf("\r\n GPS save fail\r\n"); 
				 }
                 //---------------------------------------------------------------------------------
				  if(PositionSD_Status())
				         PositionSD_Disable();
				 if(SleepState==1)	   
				 {	 
					 rt_kprintf("\r\n����ʱ�洢ʱ�� %d-%d-%d %02d:%02d:%02d\r\n", time_now.year+2000, time_now.month, time_now.day, \
					 time_now.hour, time_now.min, time_now.sec);
				 }
				  //-----------------------------------------------------
		  }
   

}
//----------------------------------------------------------------------
u8  Protocol_Head(u16 MSG_ID, u8 Packet_Type)
{
     //----  clear --------------
       Original_info_Wr=0;
	//	1. Head  
	
	    //  original info
	   Original_info[Original_info_Wr++]=(MSG_ID>>8); // ��ϢID 
	   Original_info[Original_info_Wr++]=(u8)MSG_ID;
	   
	   Original_info[Original_info_Wr++]=0x00; // �ְ������ܷ�ʽ��״̬λ 
	   Original_info[Original_info_Wr++]=28; // ��Ϣ�峤��   λ����Ϣ����Ϊ28���ֽ�
	   
	   memcpy(Original_info+Original_info_Wr,SIM_code,6); // �ն��ֻ��� ���豸��ʶID	BCD
	   Original_info_Wr+=6;  
	   
	   Original_info[Original_info_Wr++]=( JT808Conf_struct.Msg_Float_ID>>8); //��Ϣ��ˮ��
	   Original_info[Original_info_Wr++]= JT808Conf_struct.Msg_Float_ID;

       if(Packet_Type==Packet_Divide) 
       	{
            switch (MediaObj.Media_Type)
	         {
	          case 0 : // ͼ��
	                  MediaObj.Media_totalPacketNum=Photo_sdState.Total_packetNum;  // ͼƬ�ܰ���
					  MediaObj.Media_currentPacketNum=Photo_sdState.SD_packetNum;  // ͼƬ��ǰ����
					  MediaObj.Media_ID=1;   //  ��ý��ID
					  MediaObj.Media_Channel=Camera_Number;  // ͼƬ����ͷͨ����
			          break;
			  case 1 : // ��Ƶ
					  MediaObj.Media_totalPacketNum=Sound_sdState.Total_packetNum;	// ��Ƶ�ܰ���
					  MediaObj.Media_currentPacketNum=Sound_sdState.SD_packetNum;  // ��Ƶ��ǰ����
					  MediaObj.Media_ID=1;	 //  ��ý��ID
					  MediaObj.Media_Channel=1;  // ��Ƶͨ����   

			          break;
			  case 2 : // ��Ƶ
					  MediaObj.Media_totalPacketNum=Video_sdState.Total_packetNum;	// ��Ƶ�ܰ���
					  MediaObj.Media_currentPacketNum=Video_sdState.SD_packetNum;  // ��Ƶ��ǰ����
					  MediaObj.Media_ID=1;	 //  ��ý��ID
					  MediaObj.Media_Channel=1;  // ��Ƶͨ���� 
			          break;
			  default:
			  	      return false;
	         }
			 
			 Original_info[Original_info_Wr++]=(MediaObj.Media_totalPacketNum&0xff00)>>8;//��block
			 Original_info[Original_info_Wr++]=(u8)MediaObj.Media_totalPacketNum;//��block
			 
			 
			 Original_info[Original_info_Wr++]=((MediaObj.Media_currentPacketNum)&0xff00)>>8;//��ǰblock
			 Original_info[Original_info_Wr++]=(u8)((MediaObj.Media_currentPacketNum)&0x00ff);//��ǰblock

       	}  
	    return true;

}  

void Protocol_End(u8 Packet_Type,u8 LinkNum)   
{                   
 u16 packet_len=0;  
 u16  i=0;		//Ҫ���͵�UDP �������ݵĳ���
 u8   Gfcs=0;
 u16   Msg_bodyLen=0; //  Э�������Ϣֻ��ʾ��Ϣ��     ��������Ϣͷ ��ϢͷĬ�ϳ�����12 , �ְ���Ϣͷ���� 20   
 
   Gfcs=0;				   //  �������Ϣͷ��ʼ��У��ǰ���ݵ�����  808Э��У��  1Byte
   //---  ��д��Ϣ���� ---
   if(Packet_Normal==Packet_Type)
   {
     Msg_bodyLen=Original_info_Wr-12;
     Original_info[2]=(Msg_bodyLen>>8) ;
     Original_info[3]=Msg_bodyLen;
   }
   else
    if(Packet_Divide==Packet_Type)
   {
     Msg_bodyLen=Original_info_Wr-16;
	// rt_kprintf("\r\n Divide Infolen=%d  \r\n",Msg_bodyLen);  
     Original_info[2]=(Msg_bodyLen>>8)|0x20 ;  // Bit 13  0x20 ����Bit 13
     Original_info[3]=Msg_bodyLen;  
   }
   //---- ����У��  -----
   for(i=0;i<Original_info_Wr;i++)  
		Gfcs^=Original_info[i]; 
   Original_info[Original_info_Wr++]=Gfcs;  // ��дGУ��λ     


  // 1.stuff start
   GPRS_infoWr_Tx=0;	
   GPRS_info[GPRS_infoWr_Tx++]=0x7e;   // Start ��ʶλ
    if(Packet_Divide==Packet_Type)
   {   
       //rt_kprintf("\r\n Tx=%d  Divide Infolen=%d  \r\n",GPRS_infoWr_Tx,Original_info_Wr);   
	  /* rt_kprintf("\r\n PacketContent: ");
				   for(i=0;i<Original_info_Wr;i++)
                       rt_kprintf(" %X",Original_info[i]);    
				   rt_kprintf("\r\n");
	  */
   }
  // 2.  convert 
   packet_len=Protocol_808_Encode(GPRS_info+GPRS_infoWr_Tx,Original_info,Original_info_Wr);
   GPRS_infoWr_Tx+=packet_len;	   
    if(Packet_Divide==Packet_Type)
   {
	 //rt_kprintf("\r\n Divide  Send Infolen=%d  \r\n",packet_len);  
	 
	/* rt_kprintf("\r\n EncodeContent: ");
	 for(i=0;i<packet_len;i++)
                       rt_kprintf(" %X",GPRS_info[i+1]);      
	rt_kprintf("\r\n"); 
	*/
	//rt_kprintf("\r\n GPRStx  Send Infolen=%d  \r\n",GPRS_infoWr_Tx+1);    
   }
   GPRS_info[GPRS_infoWr_Tx++]=0x7e;  //  End  ��ʶ
  //  4. Send   

    // 4.1 ������Ϣ����1
     if(DispContent==2)
	 {
	    rt_kprintf("\r\n App to GSM info:");    
	    for(i=0;i<GPRS_infoWr_Tx;i++)
			 rt_kprintf(" %02X",GPRS_info[i]);  
		rt_kprintf("\r\n"); 
	 }
    // 4.2   MsgQueue
    WatchDog_Feed();
   Gsm_rxAppData_SemRelease(GPRS_info,GPRS_infoWr_Tx,LinkNum);  
   //--------��Ϣ��� ���� --------
   JT808Conf_struct.Msg_Float_ID++;   
  //------------------------------ 
} 
//--------------------------------------------------------------------------------------
u8  Stuff_DevCommmonACK_0001H(void)      
{    
	// 1. Head
	  if(!Protocol_Head(MSG_0x0001,Packet_Normal))  return false;     //�ն�ͨ��Ӧ��
	 // 2. content  is null 
	   //   float ID
	    Original_info[Original_info_Wr++]=(u8)(Centre_FloatID>>8);
	    Original_info[Original_info_Wr++]=(u8)Centre_FloatID;
	    //  cmd  ID
	    Original_info[Original_info_Wr++]=(u8)(Centre_CmdID>>8);
	    Original_info[Original_info_Wr++]=(u8)Centre_CmdID;   
		//   resualt
	    Original_info[Original_info_Wr++]=SD_ACKflag.f_CentreCMDack_resualt;
	 //  3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	      rt_kprintf("\r\n	Common CMD ACK! \r\n");  
	 return true;    
}


//-------------------------------------------------------------------------
u8  Stuff_RegisterPacket_0100H_oldTJ(u8  LinkNum)  
{  
  
// 1. Head
  if(!Protocol_Head(MSG_0x0100,Packet_Normal)) 
 	return false; 	
    
 // 2. content 
    //  province ID
    Original_info[Original_info_Wr++]=(u8)(JT808Conf_struct.Vechicle_Info.Dev_ProvinceID>>8);
    Original_info[Original_info_Wr++]=(u8)JT808Conf_struct.Vechicle_Info.Dev_ProvinceID;
    //  county  ID
    Original_info[Original_info_Wr++]=(u8)(JT808Conf_struct.Vechicle_Info.Dev_CityID>>8);
    Original_info[Original_info_Wr++]=(u8)JT808Conf_struct.Vechicle_Info.Dev_CityID;   
	//  product Name	
    memcpy(Original_info+Original_info_Wr,"TCBBD",5);
    Original_info_Wr+=5;
	//  �ն��ͺ� 20 Bytes      -- ����Э����������
	memcpy(Original_info+Original_info_Wr,"Tianjin TCB TW701-BD",20);  
    Original_info_Wr+=20;      
    //  �ն�ID   7 Bytes    ,    
    memcpy(Original_info+Original_info_Wr,DeviceNumberID+5,7);   
    Original_info_Wr+=7;  
	//  ������ɫ  
	Original_info[Original_info_Wr++]=JT808Conf_struct.Vechicle_Info.Dev_Color;
	//  ����
	memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Vechicle_Info.Vech_Num,13);  
	Original_info_Wr+=13;

 
 //  3. Send 
  Protocol_End(Packet_Normal,LinkNum);   
  if(DispContent)
 rt_kprintf("\r\n	SEND Reigster Packet! \r\n");  
 return true;  

}

u8  Stuff_RegisterPacket_0100H(u8  LinkNum)  
{  
  u8  i=0;
// 1. Head
  if(!Protocol_Head(MSG_0x0100,Packet_Normal)) 
 	return false; 	
    
  // 2. content 
    //  province ID
    Original_info[Original_info_Wr++]=0x00;//(u8)(JT808Conf_struct.Vechicle_Info.Dev_ProvinceID>>8);
    Original_info[Original_info_Wr++]=10;;//(u8)JT808Conf_struct.Vechicle_Info.Dev_ProvinceID;
    //  county  ID
    Original_info[Original_info_Wr++]=(u8)(1010>>8);
    Original_info[Original_info_Wr++]=(u8)1010;   
	//  product Name	
    memcpy(Original_info+Original_info_Wr,"70103",5);  //�����ж�  70104
    Original_info_Wr+=5;
	//  �ն��ͺ� 20 Bytes      -- ����Э����������
	memcpy(Original_info+Original_info_Wr,"HVT100BD3",9);   //ZD-V01H  HVT100BD1
    Original_info_Wr+=9;      
    for(i=0;i<11;i++)
	 Original_info[Original_info_Wr++]=0x00;	 
    //  �ն�ID   7 Bytes    ,    
    memcpy(Original_info+Original_info_Wr,IMSI_CODE+8,7);     
    Original_info_Wr+=7;  
	//  ������ɫ  
	Original_info[Original_info_Wr++]=2;//JT808Conf_struct.Vechicle_Info.Dev_Color;
	
	if(JT808Conf_struct.Vechicle_Info.Dev_Color!=0)
	{
		//  ����
		memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Vechicle_Info.Vech_Num,13);  
		Original_info_Wr+=13;
	}
	else
	{
	       rt_kprintf("\r\n  ������ɫ:0     Need Vin\r\n");
		// ����VIN 
		memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Vechicle_Info.Vech_VIN,17);
		Original_info_Wr+=17;
    }

 
 //  3. Send 
  Protocol_End(Packet_Normal,LinkNum);   
  if(DispContent)
  {      
       rt_kprintf("\r\n	SEND Reigster Packet! \r\n");  
	 rt_kprintf("\r\n  ע�ᷢ��ʱ�� %d-%d-%d %02d:%02d:%02d\r\n", time_now.year+2000, time_now.month, time_now.day, \
      time_now.hour, time_now.min, time_now.sec);
   

  }

 
 //  3. Send 
  Protocol_End(Packet_Normal,LinkNum);   
  if(DispContent)
 rt_kprintf("\r\n	SEND Reigster Packet! \r\n");  
 return true;  

}

//--------------------------------------------------------------------------------------
u8  Stuff_DeviceHeartPacket_0002H(void)      
{  
  
	// 1. Head
	if(!Protocol_Head(MSG_0x0002,Packet_Normal))  
 	  return false; 
	 // 2. content  is null
	 
	 //  3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	Send Dev Heart! \r\n");    
	 return true;  

}
//--------------------------------------------------------------------------------------
u8  Stuff_DeviceDeregister_0101H(void)      
{    
	// 1. Head
	if(!Protocol_Head(MSG_0x0101,Packet_Normal)) 
 	  return false; //�ն�ע�� 
	 // 2. content  is null 
	 //  3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	Deregister  ע��! \r\n");  
	 return true;    
}
//------------------------------------------------------------------------------------
u8  Stuff_DevLogin_0102H(void)      
{    
	// 1. Head
	if(!Protocol_Head(MSG_0x0102,Packet_Normal))  
 	  return false; //�ն˼�Ȩ 
	 // 2. content  
	  
	 memcpy(Original_info+Original_info_Wr,JT808Conf_struct.ConfirmCode,strlen((const char*)JT808Conf_struct.ConfirmCode));   // ��Ȩ��  string Type
	 Original_info_Wr+=strlen((const char*)JT808Conf_struct.ConfirmCode); 
	 //  3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	 ���ͼ�Ȩ! \r\n");   
	 return true;    
}

//--------------------------------------------------------------------------------------
u8  Stuff_Normal_Data_0200H(void)  
{
  u8 spd_sensorReg[2];
  u32  Dis_01km=0;
 //  1. Head  
   if(!Protocol_Head(MSG_0x0200,Packet_Normal)) 
 	  return false;  
 // 2. content 
 if(Api_cycle_read(Original_info+Original_info_Wr, 31)==false)            
   return false;
 Original_info_Wr+=28;   // ����ֻ��28 
 memcpy(spd_sensorReg,Original_info+Original_info_Wr,2);// �Ѷ�ȡ�������ٶ� 
 
 
  //----------- ������Ϣ  ------------ 
  //  ������Ϣ 1  -----------------------------    
  //  ������Ϣ ID
  Original_info[Original_info_Wr++]=0x03; // ��ʻ��¼�ǵ��ٶ�
  //  ������Ϣ����
  Original_info[Original_info_Wr++]=2;
  //  ����
  Original_info[Original_info_Wr++]=(u8)(Speed_cacu>>8); 
  Original_info[Original_info_Wr++]=(u8)(Speed_cacu);	   
  //rt_kprintf("\r\n GPS�ٶ�=%d km/h , �������ٶ�=%d km/h\r\n",Speed_gps,Speed_cacu); 
   //  ������Ϣ 2  -----------------------------	
   //  ������Ϣ ID
   Original_info[Original_info_Wr++]=0x01; // ���ϵ���ʻ���
   //  ������Ϣ����
   Original_info[Original_info_Wr++]=4; 
   //  ����
   Dis_01km=JT808Conf_struct.Distance_m_u32/100;
   Original_info[Original_info_Wr++]=(Dis_01km>>24); 
   Original_info[Original_info_Wr++]=(Dis_01km>>16); 
   Original_info[Original_info_Wr++]=(Dis_01km>>8); 
   Original_info[Original_info_Wr++]=Dis_01km; 
 
 
  //  ������Ϣ 3 
  if(Warn_Status[1]&0x10)
 {
   //  ������Ϣ ID
   Original_info[Original_info_Wr++]=0x12; //  ��������/·�߱���
   //  ������Ϣ���� 
   Original_info[Original_info_Wr++]=6;
   //  ����
   Original_info[Original_info_Wr++]=InOut_Object.TYPE;
   Original_info[Original_info_Wr++]=(InOut_Object.ID>>24);
   Original_info[Original_info_Wr++]=(InOut_Object.ID>>16);
   Original_info[Original_info_Wr++]=(InOut_Object.ID>>8);
   Original_info[Original_info_Wr++]=InOut_Object.ID;
   Original_info[Original_info_Wr++]=InOut_Object.InOutState;  
   rt_kprintf("\r\n ----- 0x0200 ������Ϣ \r\n");	 
 }
 
  //  ������Ϣ4
  if(Warn_Status[3]&0x02)
  { 	 
	//	������Ϣ ID
	Original_info[Original_info_Wr++]=0x11; //	��������/·�߱���
	//	������Ϣ���� 
	Original_info[Original_info_Wr++]=1; 
	//	����
	Original_info[Original_info_Wr++]=0; //  ���ض�λ��   
 
  }


	   //  ������Ϣ 5  -----------------------------	  
	 //  ������Ϣ ID
	 Original_info[Original_info_Wr++]=0xFE; //�ź�ǿ��
	 //  ������Ϣ����
	 Original_info[Original_info_Wr++]=2; 
	 //  ����
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_SignalValue; 
	 Original_info[Original_info_Wr++]=0x00;  //  ���� 

        //if(DispContent)
         //     printf("\r\n---- Satelitenum: %d , CSQ:%d\r\n",Satelite_num,ModuleSQ);  

        //  ������Ϣ 6  -----------------------------	  
	 //  ������Ϣ ID
	 Original_info[Original_info_Wr++]=0xFF; //�Զ���ģ�����ϴ�
	 //  ������Ϣ����
	 Original_info[Original_info_Wr++]=6; 
	 //  ����
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_IO_1; 
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_IO_2;  
	 Original_info[Original_info_Wr++]=(BD_EXT.AD_0>>8);  // ģ���� 1
	 Original_info[Original_info_Wr++]=BD_EXT.AD_0;
	 Original_info[Original_info_Wr++]=(BD_EXT.AD_1>>8);  // ģ���� 2
	 Original_info[Original_info_Wr++]=BD_EXT.AD_1;

 
 //  3. Send 
 Protocol_End(Packet_Normal ,0);
   	
 return true; 

}
//------------------------------------------------------------------------------------
u8  Stuff_Current_Data_0200H(void)   //  ���ͼ�ʱ���ݲ��洢���洢����
{  
  
  u32  Dis_01km=0;
  
    if( GPS_speed <=( JT808Conf_struct.Speed_warn_MAX*10) )
		StatusReg_SPD_NORMAL(); 

 //  1. Head	
 if(!Protocol_Head(MSG_0x0200,Packet_Normal)) 
 	  return false; 
 // 2. content 
    //------------------------------- Stuff ----------------------------------------
	// 1. �澯��־  4
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Warn_Status,4 );    
	Original_info_Wr += 4;
	// 2. ״̬  4
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Car_Status,4 );   
	Original_info_Wr += 4;
	// 3.  γ��
    memcpy( ( char * ) Original_info+ Original_info_Wr,( char * )  Gps_Gprs.Latitude, 4 );//γ��   modify by nathan
	Original_info_Wr += 4;
	// 4.  ����
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )  Gps_Gprs.Longitude, 4 );	  //����    ����  Bit 7->0   ���� Bit 7 -> 1
	Original_info_Wr += 4;
	// 5.  �߳�
	Original_info[Original_info_Wr++]=(u8)(GPS_Hight<<8);
	Original_info[Original_info_Wr++]=(u8)GPS_Hight;
	// 6.  �ٶ�    0.1 Km/h
	Original_info[Original_info_Wr++]=(u8)(Speed_gps>>8);//(GPS_speed>>8); 
	Original_info[Original_info_Wr++]=(u8)(Speed_gps);//GPS_speed;     
	// 7. ����   ��λ 1��
	Original_info[Original_info_Wr++]=(GPS_direction>>8);  //High 
	Original_info[Original_info_Wr++]=GPS_direction; // Low
	// 8.  ����ʱ��	
	Original_info[Original_info_Wr++]=(((Gps_Gprs.Date[0])/10)<<4)+((Gps_Gprs.Date[0])%10);		
	Original_info[Original_info_Wr++]=((Gps_Gprs.Date[1]/10)<<4)+(Gps_Gprs.Date[1]%10); 
	Original_info[Original_info_Wr++]=((Gps_Gprs.Date[2]/10)<<4)+(Gps_Gprs.Date[2]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[0]/10)<<4)+(Gps_Gprs.Time[0]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[1]/10)<<4)+(Gps_Gprs.Time[1]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[2]/10)<<4)+(Gps_Gprs.Time[2]%10);	 

	//----------- ������Ϣ  ------------
    //  ������Ϣ 1  -----------------------------    
	//	������Ϣ ID
	Original_info[Original_info_Wr++]=0x03; // ��ʻ��¼�ǵ��ٶ�
	//	������Ϣ����
	Original_info[Original_info_Wr++]=2;
	//	����
	Original_info[Original_info_Wr++]=(u8)(Speed_cacu>>8); 
	Original_info[Original_info_Wr++]=(u8)(Speed_cacu);	     
	//rt_kprintf("\r\n GPS�ٶ�=%d km/h , �������ٶ�=%d km/h\r\n",Speed_gps,Speed_cacu); 
     //  ������Ϣ 2  -----------------------------	  
	 //  ������Ϣ ID
	 Original_info[Original_info_Wr++]=0x01; // ���ϵ���ʻ���
	 //  ������Ϣ����
	 Original_info[Original_info_Wr++]=4; 
	 //  ����
	 Dis_01km=JT808Conf_struct.Distance_m_u32/100;
	 Original_info[Original_info_Wr++]=(Dis_01km>>24); 
	 Original_info[Original_info_Wr++]=(Dis_01km>>16); 
	 Original_info[Original_info_Wr++]=(Dis_01km>>8); 
	 Original_info[Original_info_Wr++]=Dis_01km; 
  
   
	//  ������Ϣ 3 
	if(Warn_Status[1]&0x10)
   {
     //  ������Ϣ ID
     Original_info[Original_info_Wr++]=0x12; //  ��������/·�߱���
     //  ������Ϣ���� 
     Original_info[Original_info_Wr++]=6;
	 //  ����
	 Original_info[Original_info_Wr++]=InOut_Object.TYPE;
	 Original_info[Original_info_Wr++]=(InOut_Object.ID>>24);
	 Original_info[Original_info_Wr++]=(InOut_Object.ID>>16);
	 Original_info[Original_info_Wr++]=(InOut_Object.ID>>8);
	 Original_info[Original_info_Wr++]=InOut_Object.ID;
	 Original_info[Original_info_Wr++]=InOut_Object.InOutState;  
	 rt_kprintf("\r\n ----- 0x0200 current ������Ϣ \r\n");    
   }

    //  ������Ϣ4
    if(Warn_Status[3]&0x02)
    {      
	  //  ������Ϣ ID
	  Original_info[Original_info_Wr++]=0x11; //  ��������/·�߱���
	  //  ������Ϣ���� 
	  Original_info[Original_info_Wr++]=1; 
	  //  ����
	  Original_info[Original_info_Wr++]=0; //  ���ض�λ��   

    }


	   //  ������Ϣ 5  -----------------------------	  
	 //  ������Ϣ ID
	 Original_info[Original_info_Wr++]=0xFE; //�ź�ǿ��
	 //  ������Ϣ����
	 Original_info[Original_info_Wr++]=2; 
	 //  ����
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_SignalValue; 
	 Original_info[Original_info_Wr++]=0x00;  //  ���� 

        //if(DispContent)
         //     printf("\r\n---- Satelitenum: %d , CSQ:%d\r\n",Satelite_num,ModuleSQ);  

        //  ������Ϣ 6  -----------------------------	  
	 //  ������Ϣ ID
	 Original_info[Original_info_Wr++]=0xFF; //�Զ���ģ�����ϴ�
	 //  ������Ϣ����
	 Original_info[Original_info_Wr++]=6; 
	 //  ����
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_IO_1; 
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_IO_2;  
	 Original_info[Original_info_Wr++]=(BD_EXT.AD_0>>8);  // ģ���� 1
	 Original_info[Original_info_Wr++]=BD_EXT.AD_0;
	 Original_info[Original_info_Wr++]=(BD_EXT.AD_1>>8);  // ģ���� 2
	 Original_info[Original_info_Wr++]=BD_EXT.AD_1;
	
 //  3. Send 
 Protocol_End(Packet_Normal ,0);

  if(SleepState==1)  	
  {   
      rt_kprintf("\r\n����ʱ����ʱ�� %d-%d-%d %02d:%02d:%02d\r\n", time_now.year+2000, time_now.month, time_now.day, \
      time_now.hour, time_now.min, time_now.sec);
  }
 return true; 

}
//-----------------------------------------------------------------------
u8  Stuff_Current_Data_0201H(void)   //   λ����Ϣ��ѯ��Ӧ
{  
   u32  Dis_01km=0; 
 //  1. Head	
 if(!Protocol_Head(MSG_0x0201,Packet_Normal)) 
 	  return false; 
 // 2. content 
    //------------------------------- Stuff ----------------------------------------
    //   float ID                                                // ��Ӧ����Ӧ����Ϣ����ˮ��
	Original_info[Original_info_Wr++]=(u8)(Centre_FloatID>>8);
	Original_info[Original_info_Wr++]=(u8)Centre_FloatID;

	// 1. �澯��־  4
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Warn_Status,4 );    
	Original_info_Wr += 4;
	// 2. ״̬  4
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Car_Status,4 );   
	Original_info_Wr += 4;
	// 3.  γ��
    memcpy( ( char * ) Original_info+ Original_info_Wr,( char * )  Gps_Gprs.Latitude, 4 );//γ��   modify by nathan
	Original_info_Wr += 4;
	// 4.  ����
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )  Gps_Gprs.Longitude, 4 );	  //����    ����  Bit 7->0   ���� Bit 7 -> 1
	Original_info_Wr += 4;
	// 5.  �߳�
	Original_info[Original_info_Wr++]=(u8)(GPS_Hight<<8);
	Original_info[Original_info_Wr++]=(u8)GPS_Hight;
	// 6.  �ٶ�    0.1 Km/h
	Original_info[Original_info_Wr++]=(u8)(Speed_gps>>8);
	Original_info[Original_info_Wr++]=(u8)Speed_gps;   
	// 7. ����   ��λ 1��
	Original_info[Original_info_Wr++]=(GPS_direction>>8);  //High 
	Original_info[Original_info_Wr++]=GPS_direction; // Low
	// 8.  ����ʱ��	
	Original_info[Original_info_Wr++]=(((Gps_Gprs.Date[0])/10)<<4)+((Gps_Gprs.Date[0])%10);		
	Original_info[Original_info_Wr++]=((Gps_Gprs.Date[1]/10)<<4)+(Gps_Gprs.Date[1]%10); 
	Original_info[Original_info_Wr++]=((Gps_Gprs.Date[2]/10)<<4)+(Gps_Gprs.Date[2]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[0]/10)<<4)+(Gps_Gprs.Time[0]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[1]/10)<<4)+(Gps_Gprs.Time[1]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[2]/10)<<4)+(Gps_Gprs.Time[2]%10);	 

	
		//----------- ������Ϣ  ------------
    //  ������Ϣ 1  -----------------------------    
	//	������Ϣ ID
	Original_info[Original_info_Wr++]=0x03; // ��ʻ��¼�ǵ��ٶ�
	//	������Ϣ����
	Original_info[Original_info_Wr++]=2;
	//	����
	Original_info[Original_info_Wr++]=(u8)(Speed_cacu>>8); 
	Original_info[Original_info_Wr++]=(u8)(Speed_cacu);	     
	//rt_kprintf("\r\n GPS�ٶ�=%d km/h , �������ٶ�=%d km/h\r\n",Speed_gps,Speed_cacu); 
     //  ������Ϣ 2  -----------------------------	  
	 //  ������Ϣ ID
	 Original_info[Original_info_Wr++]=0x01; // ���ϵ���ʻ���
	 //  ������Ϣ����
	 Original_info[Original_info_Wr++]=4; 
	 //  ����
	 Dis_01km=JT808Conf_struct.Distance_m_u32/100;
	 Original_info[Original_info_Wr++]=(Dis_01km>>24); 
	 Original_info[Original_info_Wr++]=(Dis_01km>>16); 
	 Original_info[Original_info_Wr++]=(Dis_01km>>8); 
	 Original_info[Original_info_Wr++]=Dis_01km; 
  
   
	//  ������Ϣ 3 
	if(Warn_Status[1]&0x10)
   {
     //  ������Ϣ ID
     Original_info[Original_info_Wr++]=0x12; //  ��������/·�߱���
     //  ������Ϣ���� 
     Original_info[Original_info_Wr++]=6;
	 //  ����
	 Original_info[Original_info_Wr++]=InOut_Object.TYPE;
	 Original_info[Original_info_Wr++]=(InOut_Object.ID>>24);
	 Original_info[Original_info_Wr++]=(InOut_Object.ID>>16);
	 Original_info[Original_info_Wr++]=(InOut_Object.ID>>8);
	 Original_info[Original_info_Wr++]=InOut_Object.ID;
	 Original_info[Original_info_Wr++]=InOut_Object.InOutState;  
	 rt_kprintf("\r\n ----- 0x0201 ������Ϣ \r\n");    
   }

    //  ������Ϣ4
    if(Warn_Status[3]&0x02)
    {      
	  //  ������Ϣ ID
	  Original_info[Original_info_Wr++]=0x11; //  ��������/·�߱���
	  //  ������Ϣ���� 
	  Original_info[Original_info_Wr++]=1; 
	  //  ����
	  Original_info[Original_info_Wr++]=0; //  ���ض�λ��   

    }

    
	   //  ������Ϣ 5  -----------------------------	  
	 //  ������Ϣ ID
	 Original_info[Original_info_Wr++]=0xFE; //�ź�ǿ��
	 //  ������Ϣ����
	 Original_info[Original_info_Wr++]=2; 
	 //  ����
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_SignalValue; 
	 Original_info[Original_info_Wr++]=0x00;  //  ���� 

        //if(DispContent)
         //     printf("\r\n---- Satelitenum: %d , CSQ:%d\r\n",Satelite_num,ModuleSQ);  

        //  ������Ϣ 6  -----------------------------	  
	 //  ������Ϣ ID
	 Original_info[Original_info_Wr++]=0xFF; //�Զ���ģ�����ϴ�
	 //  ������Ϣ����
	 Original_info[Original_info_Wr++]=6; 
	 //  ����
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_IO_1; 
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_IO_2;  
	 Original_info[Original_info_Wr++]=(BD_EXT.AD_0>>8);  // ģ���� 1
	 Original_info[Original_info_Wr++]=BD_EXT.AD_0;
	 Original_info[Original_info_Wr++]=(BD_EXT.AD_1>>8);  // ģ���� 2
	 Original_info[Original_info_Wr++]=BD_EXT.AD_1;
	
 //  3. Send 
Protocol_End(Packet_Normal ,0);
 if(DispContent)
   rt_kprintf("\r\n	SEND GPS CMD=81H ! \r\n"); 
   	
 return true; 

}


//-----------------------------------------------------------------------
u8  Stuff_SettingPram_0104H(void)
{
   u8  reg_str[30];
  
  //  1. Head     
  if(!Protocol_Head(MSG_0x0104,Packet_Normal)) 
 	  return false; // �ն˲����ϴ�

  //  2. content 
       //   float ID
	    Original_info[Original_info_Wr++]=(u8)(Centre_FloatID>>8);
	    Original_info[Original_info_Wr++]=(u8)Centre_FloatID;
	   //   ��������	
	    Original_info[Original_info_Wr++]=4;
	   //   �����б�

	   //   2.1   ���ƺ�
       /* Original_info[Original_info_Wr++]=0x00;   // ����ID 4Bytes
        Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x83;
		Original_info[Original_info_Wr++]=strlen((const char*)JT808Conf_struct.Vechicle_Info.Vech_Num); // ��������
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )JT808Conf_struct.Vechicle_Info.Vech_Num,strlen((const char*)JT808Conf_struct.Vechicle_Info.Vech_Num) ); // ����ֵ
		Original_info_Wr+=strlen((const char*)JT808Conf_struct.Vechicle_Info.Vech_Num); 
        */
		//   2.2  ��������IP    
        Original_info[Original_info_Wr++]=0x00;   // ����ID 4Bytes
        Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x13;    
		                                         // ��������			    
		memset(reg_str,0,sizeof(reg_str));
	    IP_Str((char*)reg_str, *( u32 * ) RemoteIP_aux);   
		Original_info[Original_info_Wr++]=strlen((const char*)reg_str);
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )reg_str,strlen((const char*)reg_str));  // ����ֵ	 
		Original_info_Wr+=strlen((const char*)reg_str);

		//   2.3   ������TCP�˿�
        Original_info[Original_info_Wr++]=0x00;   // ����ID 4Bytes
        Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x18;
		Original_info[Original_info_Wr++]=4  ; // ��������
		Original_info[Original_info_Wr++]=0x00;   // ����ֵ
		Original_info[Original_info_Wr++]=0x00; 
		Original_info[Original_info_Wr++]=(RemotePort_main>>8); 
		Original_info[Original_info_Wr++]=RemotePort_main; 
													    
		 //   2.4  APN �ַ���
        Original_info[Original_info_Wr++]=0x00;   // ����ID 4Bytes
        Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x10;
		Original_info[Original_info_Wr++]=strlen((const char*)APN_String); // ��������
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )APN_String,strlen((const char*)APN_String)); // ����ֵ
		Original_info_Wr+=strlen((const char*)APN_String);

        //  2.5   ����IP		 
        Original_info[Original_info_Wr++]=0x00;   // ����ID 4Bytes
        Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x17;    
		                                         // ��������		 	    
		memset(reg_str,0,sizeof(reg_str));
	    IP_Str((char*)reg_str, *( u32 * ) RemoteIP_aux);
		Original_info[Original_info_Wr++]=strlen((const char*)reg_str);
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )reg_str,strlen((const char*)reg_str));  // ����ֵ	 
		Original_info_Wr+=strlen((const char*)reg_str);
		
       /*   
		 //   2.4   ȱʡʱ���ϱ����
        Original_info[Original_info_Wr++]=0x00;   // ����ID 4Bytes
        Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x29;
		Original_info[Original_info_Wr++]=4  ; // ��������
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Default_Dur>>24);   // ����ֵ 
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Default_Dur>>16);
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Default_Dur>>8);
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Default_Dur);     
		
		 //   2.5   ���ļ�غ���
        Original_info[Original_info_Wr++]=0x00;   // ����ID 4Bytes
        Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x40;
		Original_info[Original_info_Wr++]=strlen((const char*)JT808Conf_struct.LISTEN_Num); // ��������
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )JT808Conf_struct.LISTEN_Num,strlen((const char*)JT808Conf_struct.LISTEN_Num)); // ����ֵ
		Original_info_Wr+=strlen((const char*)JT808Conf_struct.LISTEN_Num);
		 //   2.6   ����ٶ�����
        Original_info[Original_info_Wr++]=0x00;   // ����ID 4Bytes
        Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x55;
		Original_info[Original_info_Wr++]=4  ; // ��������
		Original_info[Original_info_Wr++]=( JT808Conf_struct.Speed_warn_MAX>>24);   // ����ֵ
		Original_info[Original_info_Wr++]=( JT808Conf_struct.Speed_warn_MAX>>16);  
		Original_info[Original_info_Wr++]=( JT808Conf_struct.Speed_warn_MAX>>8);
		Original_info[Original_info_Wr++]=( JT808Conf_struct.Speed_warn_MAX);
		 //   2.7   ������ʻ����
        Original_info[Original_info_Wr++]=0x00;   // ����ID 4Bytes
        Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x57;
		Original_info[Original_info_Wr++]=4  ; // ��������
		Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DrvKeepingSec>>24);   // ����ֵ
		Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DrvKeepingSec>>16);  
		Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DrvKeepingSec>>8);
		Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DrvKeepingSec);
		 //   2.8   ��С��Ϣʱ��
		 Original_info[Original_info_Wr++]=0x00;   // ����ID 4Bytes
		 Original_info[Original_info_Wr++]=0x00;
		 Original_info[Original_info_Wr++]=0x00;
		 Original_info[Original_info_Wr++]=0x59;
		 Original_info[Original_info_Wr++]=4  ; // ��������
		 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MinSleepSec>>24);	 // ����ֵ
		 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MinSleepSec>>16);	
		 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MinSleepSec>>8);
		 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MinSleepSec);         
		 */
  //  3. Send 
  Protocol_End(Packet_Normal ,0);
  if(DispContent)
	rt_kprintf("\r\n	���Ͳ�����ѯ��Ϣ! \r\n");   

  return true;
}
//--------------------------------------------------------------------------
u8  Stuff_EventACK_0301H(void)      
{    
	// 1. Head
	if(!Protocol_Head(MSG_0x0301,Packet_Normal)) 
 	  return false; 
	 // 2. content   
	   Original_info[Original_info_Wr++]=EventObj.Event_ID;// �����¼�ID
	    
	 //  3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	 �¼��������  \r\n");    
	 return true; 

}

u8  Stuff_ASKACK_0302H(void)      
{  
  
	// 1. Head
	if(!Protocol_Head(MSG_0x0302,Packet_Normal)) 
 	  return false; 
	 // 2. content   
	   //  Ӧ����ˮ��
	   Original_info[Original_info_Wr++]=(ASK_Centre.ASK_floatID>>8);// �����¼�ID
	   Original_info[Original_info_Wr++]=ASK_Centre.ASK_floatID;
       Original_info[Original_info_Wr++]=ASK_Centre.ASK_answerID;	    
	 //  3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	��������ѡ���� \r\n");     
	 return true; 

}

u8  Stuff_MSGACK_0303H(void)      
{  
 
	// 1. Head
	if(!Protocol_Head(MSG_0x0303,Packet_Normal)) 
 	  return false; 
	 // 2. content   
	   //  Ӧ����ˮ��
	   Original_info[Original_info_Wr++]=MSG_BroadCast_Obj.INFO_TYPE; 
	   Original_info[Original_info_Wr++]=MSG_BroadCast_Obj.INFO_PlyCancel;  //  0  ȡ��  1 �㲥 
	 //  3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	�㲥ȡ���ظ�  \r\n");     
	 return true;    

}

u8  Stuff_ControlACK_0500H(void)   //   ��������Ӧ��
{  

 //  1. Head	
   if(!Protocol_Head(MSG_0x0500,Packet_Normal)) 
 	  return false; 
 // 2. content 
    //------------------------------- Stuff ----------------------------------------
    //   float ID                                                // ��Ӧ����Ӧ����Ϣ����ˮ��
	Original_info[Original_info_Wr++]=(u8)(Vech_Control.CMD_FloatID>>8);
	Original_info[Original_info_Wr++]=(u8)Vech_Control.CMD_FloatID;  

	// 1. �澯��־  4
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Warn_Status,4 );    
	Original_info_Wr += 4;
	// 2. ״̬  4
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Car_Status,4 );   
	Original_info_Wr += 4;
	// 3.  γ��
    memcpy( ( char * ) Original_info+ Original_info_Wr,( char * )  Gps_Gprs.Latitude, 4 );//γ��   modify by nathan
	Original_info_Wr += 4;
	// 4.  ����
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )  Gps_Gprs.Longitude, 4 );	  //����    ����  Bit 7->0   ���� Bit 7 -> 1
	Original_info_Wr += 4;
	// 5.  �߳�
	Original_info[Original_info_Wr++]=(u8)(GPS_Hight<<8);
	Original_info[Original_info_Wr++]=(u8)GPS_Hight;
	// 6.  �ٶ�    0.1 Km/h
	Original_info[Original_info_Wr++]=(u8)(Speed_gps>>8);
	Original_info[Original_info_Wr++]=(u8)Speed_gps;   
	// 7. ����   ��λ 1��
	Original_info[Original_info_Wr++]=(GPS_direction>>8);  //High 
	Original_info[Original_info_Wr++]=GPS_direction; // Low
	// 8.  ����ʱ��	
	Original_info[Original_info_Wr++]=(((Gps_Gprs.Date[0])/10)<<4)+((Gps_Gprs.Date[0])%10);		
	Original_info[Original_info_Wr++]=((Gps_Gprs.Date[1]/10)<<4)+(Gps_Gprs.Date[1]%10); 
	Original_info[Original_info_Wr++]=((Gps_Gprs.Date[2]/10)<<4)+(Gps_Gprs.Date[2]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[0]/10)<<4)+(Gps_Gprs.Time[0]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[1]/10)<<4)+(Gps_Gprs.Time[1]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[2]/10)<<4)+(Gps_Gprs.Time[2]%10);	 

 //  3. Send 
 Protocol_End(Packet_Normal ,0);
 if(DispContent)
   rt_kprintf("\r\n	SEND  Vech  Control  ! \r\n"); 
   	
 return true; 

}

u8  Stuff_RecoderACK_0700H(void)   //   �г���¼�������ϴ�
{  

   u16	SregLen=0,Swr=0;//,Gwr=0; // S:serial  G: GPRS   
   u8	       Sfcs=0;     
   u16	i=0;   
   u32	regdis=0,reg2=0;   
   u8   	Reg[70];  
   u8       QueryRecNum=0;  // ��ѯ��¼��Ŀ

 //  1. Head	
 if(!Protocol_Head(MSG_0x0700,Packet_Normal)) 
 	  return false; 
 // 2. content 
    //------------------------------- Stuff ----------------------------------------
    //   float ID                                                // ��Ӧ����Ӧ����Ϣ����ˮ��
	Original_info[Original_info_Wr++]=(u8)(Recode_Obj.Float_ID>>8);
	Original_info[Original_info_Wr++]=(u8)Recode_Obj.Float_ID;  
	//   ������
	Original_info[Original_info_Wr++]=Recode_Obj.CMD;   // ������  

    //   ��¼�� ���ݿ�
	//---------------��д A Э��ͷ ------------------------
	Swr=Original_info_Wr;	// reg save 
	Original_info[Original_info_Wr++]=0x55;  // ��ʼͷ
	Original_info[Original_info_Wr++]=0x7A; 
	//---------------�������ͷ�����д����------------------
	switch(Recode_Obj.CMD)  
	  {
		  //---------------- �ϴ�������  -------------------------------------
		  case	A_Up_DrvInfo  : 	 //  ��ǰ��ʻԱ���뼰��Ӧ�Ļ�������ʻ֤��
							   Original_info[Original_info_Wr++]=Recode_Obj.CMD; //������
	
							   SregLen=0x00;		   // ��Ϣ����
							   Original_info[Original_info_Wr++]=0x00;    // Hi
							   Original_info[Original_info_Wr++]=21;	   // Lo
							   
							   Original_info[Original_info_Wr++]=0x00;    // ������ 
							   
							   
							   memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Driver_Info.DriveCode,3);  
							   Original_info_Wr+=3;							   
							   memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Driver_Info.DriverCard_ID,18); //��Ϣ����
							   Original_info_Wr+=18;  
							   break;
		  case	A_Up_RTC	  : 	 //  �ɼ���¼�ǵ�ʵʱʱ��
							  Original_info[Original_info_Wr++]=Recode_Obj.CMD; //������
	
							   SregLen=0x00;		   // ��Ϣ����
							   Original_info[Original_info_Wr++]=0x00;    // Hi
							   Original_info[Original_info_Wr++]=6;	  // Lo
							   
							   Original_info[Original_info_Wr++]=0x00;    // ������
							   
							   Time2BCD(Original_info+Original_info_Wr);	//��Ϣ����	
							   Original_info_Wr+=6;
							   break;
		  case	A_Up_Dist	  :    //  �ɼ�360h�����
							   Original_info[Original_info_Wr++]=Recode_Obj.CMD; //������
	
							   SregLen=0x00;		   // ��Ϣ����
							   Original_info[Original_info_Wr++]=0x00;    // Hi
							   Original_info[Original_info_Wr++]=8;	   // Lo
							   
							   Original_info[Original_info_Wr++]=0x00;    // ������
							   //	��Ϣ����
							    // -- ��� 3���ֽ� ��λ0.1km    6λ
							   regdis=JT808Conf_struct.Distance_m_u32/100;  //��λ0.1km 
							   reg2=regdis/100000;
							  Original_info[Original_info_Wr++]=(reg2<<4)+(regdis%100000/10000);
							  Original_info[Original_info_Wr++]=((regdis%10000/1000)<<4)+(regdis%1000/100);
							  Original_info[Original_info_Wr++]=((regdis%100/10)<<4)+(regdis%10);   
							    //  --�������ʱ��RTC --- 
							  Time2BCD(Original_info+Original_info_Wr);	//��Ϣ����	
							  Original_info_Wr+=5;   // ֻҪ�󵽷� ������ 5���ֽ�
							  break; 
	
		  case	A_Up_PLUS	  :    //  �ɼ���¼������ϵ��
							   Original_info[Original_info_Wr++]=Recode_Obj.CMD; //������
	
							   SregLen=0x00;		   // ��Ϣ����
							   Original_info[Original_info_Wr++]=0x00;    // Hi
							   Original_info[Original_info_Wr++]=3;	   // Lo
							   
							   Original_info[Original_info_Wr++]=0x00;    // ������
	
							   //  ��Ϣ����
							   Original_info[Original_info_Wr++]=(u8)(JT808Conf_struct.Vech_Character_Value>>16);
							   Original_info[Original_info_Wr++]=(u8)(JT808Conf_struct.Vech_Character_Value>>8); 
							   Original_info[Original_info_Wr++]=(u8)(JT808Conf_struct.Vech_Character_Value);  
							   
							   break;
		  case	A_Up_VechInfo :    //  ������Ϣ
							   Original_info[Original_info_Wr++]=Recode_Obj.CMD; //������
	
							   SregLen=0x00;		   // ��Ϣ����
							   Original_info[Original_info_Wr++]=0x00;    // Hi
							   Original_info[Original_info_Wr++]=41;	   // Lo
							   
							   Original_info[Original_info_Wr++]=0x00;    // ������							  
							   
							   memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Vechicle_Info.Vech_VIN,17); //��Ϣ����
							   Original_info_Wr+=17;
							   memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Vechicle_Info.Vech_Num,12);
							   Original_info_Wr+=12;
							   memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Vechicle_Info.Vech_Type,12);	
							   Original_info_Wr+=12; 
	
							   break;
	      case	A_Up_Doubt	  :    //  �¹��ɵ�����
						     Original_info[Original_info_Wr++]=Recode_Obj.CMD; //������
  
							 SregLen=206;		 // ��Ϣ����
							 Original_info[Original_info_Wr++]=(u8)(SregLen>>8);	 // Hi
							 Original_info[Original_info_Wr++]=(u8)SregLen;		 // Lo		 
							 
							 Original_info[Original_info_Wr++]=0x00;	 // ������		
							 //------- ��Ϣ���� ------		
							 if(Api_DFdirectory_Read(doubt_data, Original_info+Original_info_Wr,207,0,0)==1)
							 	  Original_info_Wr+=206; //	 ������206
							 else
							 {  //����д����
				                                   Original_info[Original_info_Wr-3]=0x00;//Hi
				                                   Original_info[Original_info_Wr-2]=0x00;//lo
							 } 
	
							   break;							   
		  case	A_Up_N2daysDist:    //  ���2 ���ۼ���ʻ���
							   Original_info[Original_info_Wr++]=Recode_Obj.CMD; //������
	
							   SregLen=0x00;		   // ��Ϣ����
							   Original_info[Original_info_Wr++]=0x00;    // Hi
							   Original_info[Original_info_Wr++]=8;	   // Lo
							   
							   Original_info[Original_info_Wr++]=0x00;    // ������
							   //	��Ϣ����
							    // -- ��� 3���ֽ� ��λ0.1km    6λ
							   regdis=(JT808Conf_struct.Distance_m_u32-JT808Conf_struct.DayStartDistance_32)/100;  //��λ0.1km 
							   reg2=regdis/100000;
							   Original_info[Original_info_Wr++]=(reg2<<4)+(regdis%100000/10000);
							   Original_info[Original_info_Wr++]=((regdis%10000/1000)<<4)+(regdis%1000/100);
							   Original_info[Original_info_Wr++]=((regdis%100/10)<<4)+(regdis%10);   
							    //  --�������ʱ��RTC --- 
							  Time2BCD(Original_info+Original_info_Wr);	//��Ϣ����	
							  Original_info_Wr+=5;   // ֻҪ�󵽷� ������ 5���ֽ�
							  // Original_info_Wr+=18;		   //  18�ֽ��ڲ�����д00H
							   break; 
	      case  A_Up_N2daysSpd:	  // ���2���ڵ���ʻ�ٶ�   7Сʱ
									  Original_info[Original_info_Wr++]=Recode_Obj.CMD; //������
		   
									  SregLen=455;		  // ��Ϣ����
									  Original_info[Original_info_Wr++]=(u8)(SregLen>>8);	  // Hi
									  Original_info[Original_info_Wr++]=(u8)SregLen;	  // Lo    65x7    
									  
									  Original_info[Original_info_Wr++]=0x00;	  // ������ 	
									  //-----------  ��Ϣ����  --------------		
                                                                   //----------------------
										  QueryRecNum=Api_DFdirectory_Query(spdpermin,0);   //��ѯ��ǰƣ�ͼ�ʻ��¼��Ŀ
										 if(QueryRecNum>7)								   
										        QueryRecNum=7;			
								
										    SregLen=QueryRecNum*65;
										      // ��д��Ϣ����
			                                                      Original_info[Original_info_Wr-3]=(u8)(SregLen>>8);	  // Hi
			                                                      Original_info[Original_info_Wr-2]=(u8)SregLen;	  // Lo    65x7    
										 for(i=0;i<QueryRecNum;i++)			   // �����´���ȡ�洢��д
										  {
											 Api_DFdirectory_Read(spdpermin,Reg,70,0,i); // ��new-->old  ��ȡ
											  memcpy(Original_info+Original_info_Wr,Reg+5,60);	// ֻ��д�ٶ�
											Original_info_Wr+=65;	    
										  }
									       //------------------------------
									  
							break; 

		  case	A_Up_AvrgMin  : 	 //  360h ��Ӧÿ���ӵ�ƽ���ٶ�	 // Ĭ����д����7hour��¼ 
										 Original_info[Original_info_Wr++]=Recode_Obj.CMD; //������
			  
										 SregLen=435;		 // ��Ϣ���� 
										 Original_info[Original_info_Wr++]=(u8)(SregLen>>8);	 // Hi
										 Original_info[Original_info_Wr++]=(u8)SregLen;		 // Lo	  65x7	  
										 
										 Original_info[Original_info_Wr++]=0x00;	 // ������	   
										 //-----------	��Ϣ����  --------------	
                                                                                  //----------------------
										  QueryRecNum=Api_DFdirectory_Query(spdpermin,0);   //��ѯ��ǰƣ�ͼ�ʻ��¼��Ŀ
										 if(QueryRecNum>7)								   
										        QueryRecNum=7;			
								
										    SregLen=QueryRecNum*65;     // ��д��Ϣ����
			                                                      Original_info[Original_info_Wr-3]=(u8)(SregLen>>8);	  // Hi
			                                                      Original_info[Original_info_Wr-2]=(u8)SregLen;	  // Lo    65x7    

																  
										 for(i=0;i<QueryRecNum;i++)			   // �����´���ȡ�洢��д
										  {
											 Api_DFdirectory_Read(spdpermin,Reg,70,1,i); // ��new-->old  ��ȡ
											  memcpy(Original_info+Original_info_Wr,Reg+5,60);	// ֻ��д�ٶ�
											Original_info_Wr+=65;	    
										  }
									       //------------------------------
										 
							   break; 
	
		  case	A_Up_Tired	  : 	//	ƣ�ͼ�ʻ��¼
										 Original_info[Original_info_Wr++]=Recode_Obj.CMD; //������
			  
										 SregLen=90;		 // ��Ϣ����
										 Original_info[Original_info_Wr++]=(u8)(SregLen>>8);	 // Hi
										 Original_info[Original_info_Wr++]=(u8)SregLen;		 // Lo	  30x6	 
										 
										 Original_info[Original_info_Wr++]=0x00;	 // ������		 
										 //------------ 
										 memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Driver_Info.DriverCard_ID,18); //��������ʻ֤��
							                      Original_info_Wr+=18;  
										 QueryRecNum=Api_DFdirectory_Query(tired_warn,0);   //��ѯ��ǰƣ�ͼ�ʻ��¼��Ŀ
										 if(QueryRecNum>6)								   
										        QueryRecNum=6;		
										 
										    SregLen=QueryRecNum*12+18;     // ��д��Ϣ����
			                                                      Original_info[Original_info_Wr-3]=(u8)(SregLen>>8);	  // Hi
			                                                      Original_info[Original_info_Wr-2]=(u8)SregLen;	  // Lo    65x7    

										 
										 for(i=0;i<QueryRecNum;i++)			   // �����´���ȡ�洢��д
										  {
											 Api_DFdirectory_Read(tired_warn,Reg,31,0,i); // ��new-->old  ��ȡ
											  memcpy(Original_info+Original_info_Wr,Reg+18,12);	//ֻҪ��ʼʱ��
											  Original_info_Wr+=12;	     
										  }
	
							   break; 
		  //------------------------ �´������������ ---------------------
		  case	A_Dn_DrvInfo  : 	  //  ���� ��ʻԱ��Ϣ
		  case  A_Dn_VehicleInfo:     //  ���ó�����Ϣ							 
		  case	A_Dn_RTC	  : 	   //  ���ü�¼��ʱ��
		  case	A_Dn_Plus	  : 	   //  �����ٶ�����ϵ��
							   
						  //   JT808Conf_struct.Vech_Character_Value=((u32)(*InStr)<<24)+((u32)(*InStr+1)<<16)+((u32)(*InStr+2)<<8)+(u32)(*InStr+3); // ����ϵ��   �ٶ�����ϵ��
						  //   Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
							   //-------------------------------------------------------------	
							   Original_info[Original_info_Wr++]=Recode_Obj.CMD; //������
												  // ��Ϣ����
							   Original_info[Original_info_Wr++]=0x00;    // Hi
							   Original_info[Original_info_Wr++]=0;	  // Lo   20x8							   
							   Original_info[Original_info_Wr++]=0x00;    // ������   
							   break;
		  default :
							 //  rt_kprintf("Error:	Device Type Error! \r\n");
							   return  false; 
							   
	  }
	//---------------  ��д���� A Э��	Serial Data   У��λ  -------------------------------------
	Sfcs=0; 					  //  ����SУ�� ��Ox55 ��ʼ 
	for(i=Swr;i<Original_info_Wr;i++)
	   Sfcs^=Original_info[i];
	Original_info[Original_info_Wr++]=Sfcs;		   // ��дFCS  


 //  3. Send 
 Protocol_End(Packet_Normal ,0);
 if(DispContent)
   rt_kprintf("\r\n	SEND Recorder Data ! \r\n"); 
   	
 return true; 

}
//------------------------------------------------------------------
u8  Stuff_DataTransTx_0900H(void)      
{  
	
	// 1. Head
	if(!Protocol_Head(MSG_0x0900,Packet_Normal)) 
 	  return false; 
	 // 2. content   
	 //  Ӧ����ˮ��
	Original_info[Original_info_Wr++]=DataTrans.TYPE;// ����͸�����ݵ�����
	memcpy(Original_info+Original_info_Wr,DataTrans.Data_Tx,DataTrans.Data_TxLen);
	Original_info_Wr+=DataTrans.Data_TxLen;
	 //  3. Send 
	Protocol_End(Packet_Normal ,0);
    if(DispContent)
	     rt_kprintf("\r\n	����͸��  \r\n");    
	return true; 

}

//----------------------------------------------------------------------
u8  Stuff_MultiMedia_InfoSD_0800H(void)      
{  
  
	// 1. Head
	if(!Protocol_Head(MSG_0x0800,Packet_Normal)) 
 	  return false; 
	 // 2. content          
		 switch (MediaObj.Media_Type)
		  {
		   case 0 : // ͼ��
				   MediaObj.Media_totalPacketNum=Photo_sdState.Total_packetNum;  // ͼƬ�ܰ���
				   MediaObj.Media_currentPacketNum=Photo_sdState.SD_packetNum;	// ͼƬ��ǰ����
				   MediaObj.Media_ID=1;   //  ��ý��ID
				   MediaObj.Media_Channel=Camera_Number;  // ͼƬ����ͷͨ����
				   break;
		   case 1 : // ��Ƶ
		          
				  MediaObj.Media_totalPacketNum=Sound_sdState.Total_packetNum;	// ͼƬ�ܰ���
				  MediaObj.Media_currentPacketNum=Sound_sdState.SD_packetNum;  // ͼƬ��ǰ����
				  MediaObj.Media_ID=1;	 //  ��ý��ID
				  MediaObj.Media_Channel=1;  // ͼƬ����ͷͨ����
		         // rt_kprintf(" \r\n �����ϴ���Ƶ��Ϣ \r\n");  
				   break;
		   case 2 : // ��Ƶ
		          MediaObj.Media_totalPacketNum=Video_sdState.Total_packetNum;	// ͼƬ�ܰ���
				  MediaObj.Media_currentPacketNum=Video_sdState.SD_packetNum;  // ͼƬ��ǰ����
				  MediaObj.Media_ID=1;	 //  ��ý��ID
				  MediaObj.Media_Channel=1;  // ͼƬ����ͷͨ����
		         // rt_kprintf(" \r\n �����ϴ���Ƶ��Ϣ \r\n");  
		 
		 
				   break;
		   default:
				   return false;
		  }
	 
	   //  MediaID
	   Original_info[Original_info_Wr++]=(MediaObj.Media_ID>>24);// �����¼�ID
	   Original_info[Original_info_Wr++]=(MediaObj.Media_ID>>16);
       Original_info[Original_info_Wr++]=(MediaObj.Media_ID>>8);    	
	   Original_info[Original_info_Wr++]=MediaObj.Media_ID;
	   //  Type	   
	   Original_info[Original_info_Wr++]=MediaObj.Media_Type;
	   //  MediaCode Type 
	    Original_info[Original_info_Wr++]=MediaObj.Media_CodeType;
	    Original_info[Original_info_Wr++]=MediaObj.Event_Code;
	    Original_info[Original_info_Wr++]=MediaObj.Media_Channel;
	   
	   
	 //  3. Send 
	Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	���Ͷ�ý���¼���Ϣ�ϴ�  \r\n");    
	 return true; 

}

//--------------------------------------------------------------------------
u8  Stuff_MultiMedia_Data_0801H(void)      
{  
	u16 inadd=0,readsize=0;//,soundpage=0,sounddelta=0;
//	u8  instr[SpxGet_Size];  

	  
	//  rt_kprintf("\r\n  1--- pic_total_num:  %d	current_num:  %d  MediaObj.Media_Type: %d \r\n ",MediaObj.Media_totalPacketNum,MediaObj.Media_currentPacketNum,MediaObj.Media_Type);
	 // 1. Head 
	 if(!Protocol_Head(MSG_0x0801,Packet_Divide))   
 	  return false;    
	 // 2. content1  ==>  MediaHead   
	 if(MediaObj.Media_currentPacketNum==1)
	 {
	   //  MediaID
	   Original_info[Original_info_Wr++]=(MediaObj.Media_ID>>24);//  ��ý��ID
	   Original_info[Original_info_Wr++]=(MediaObj.Media_ID>>16);
       Original_info[Original_info_Wr++]=(MediaObj.Media_ID>>8);    	
	   Original_info[Original_info_Wr++]=MediaObj.Media_ID;
	   //  Type	   
	   Original_info[Original_info_Wr++]=MediaObj.Media_Type;    // ��ý������
	   //  MediaCode Type 
	    Original_info[Original_info_Wr++]=MediaObj.Media_CodeType; // ��ý������ʽ
	    Original_info[Original_info_Wr++]=MediaObj.Event_Code;     // ��ý���¼�����
	    Original_info[Original_info_Wr++]=MediaObj.Media_Channel;  // ͨ��ID  

	    //  Position Inifo	    
		//  �澯��־  4
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Warn_Status,4 );    
		Original_info_Wr += 4;
		// . ״̬  4
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Car_Status,4 );   
		Original_info_Wr += 4;
		//   γ��
	    memcpy( ( char * ) Original_info+ Original_info_Wr,( char * )  Gps_Gprs.Latitude, 4 );//γ��   modify by nathan
		Original_info_Wr += 4;
		//   ����
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )  Gps_Gprs.Longitude, 4 );	  //����    ����  Bit 7->0   ���� Bit 7 -> 1
		Original_info_Wr += 4;
		//   �߳�
		Original_info[Original_info_Wr++]=(u8)(GPS_Hight<<8);
		Original_info[Original_info_Wr++]=(u8)GPS_Hight;
		//   �ٶ�    0.1 Km/h
		Original_info[Original_info_Wr++]=(u8)(Speed_gps>>8);//(GPS_speed>>8); 
		Original_info[Original_info_Wr++]=(u8)(Speed_gps);//GPS_speed;     
		//   ����   ��λ 1��
		Original_info[Original_info_Wr++]=(GPS_direction>>8);  //High 
		Original_info[Original_info_Wr++]=GPS_direction; // Low
		//   ����ʱ��	
		Original_info[Original_info_Wr++]=(((Gps_Gprs.Date[0])/10)<<4)+((Gps_Gprs.Date[0])%10);		
		Original_info[Original_info_Wr++]=((Gps_Gprs.Date[1]/10)<<4)+(Gps_Gprs.Date[1]%10); 
		Original_info[Original_info_Wr++]=((Gps_Gprs.Date[2]/10)<<4)+(Gps_Gprs.Date[2]%10);
		Original_info[Original_info_Wr++]=((Gps_Gprs.Time[0]/10)<<4)+(Gps_Gprs.Time[0]%10);
		Original_info[Original_info_Wr++]=((Gps_Gprs.Time[1]/10)<<4)+(Gps_Gprs.Time[1]%10);
		Original_info[Original_info_Wr++]=((Gps_Gprs.Time[2]/10)<<4)+(Gps_Gprs.Time[2]%10);	 
		//------------
	 } 	
      // 4. content3  ==> Media Info
     switch (MediaObj.Media_Type) 
         {
          case 0 : // ͼ��
                   
				   if(((Photo_sdState.photo_sending)==enable)&&((Photo_sdState.SD_flag)==enable))
				   {
					   Photo_sdState.SD_flag=disable;// clear    
				   }
				   else
				   	 return false; 
				   //  ---------------  ��д����  ---------------
				   //			read		Photo_sdState.SD_packetNum��1��ʼ����
				   //			content_startoffset 	picpage_offset				 contentpage_offset
                  if(TF_Card_Status()==0) 
                  {
                   if(Camera_Number==1)
				      Api_DFdirectory_Read(camera_1, Original_info + Original_info_Wr, 512,1,MediaObj.Media_currentPacketNum); 
                   else
				   if(Camera_Number==2)
				      Api_DFdirectory_Read(camera_2, Original_info + Original_info_Wr, 512,1,MediaObj.Media_currentPacketNum); 
                   else
				   if(Camera_Number==3)
				      Api_DFdirectory_Read(camera_3, Original_info + Original_info_Wr, 512,1,MediaObj.Media_currentPacketNum); 
                   else
		     if(Camera_Number==4)
				      Api_DFdirectory_Read(camera_4, Original_info + Original_info_Wr, 512,1,MediaObj.Media_currentPacketNum);  
 
				   	
      
				      inadd=(Photo_sdState.SD_packetNum-1)<<9; //����512
				   	  if(PicFileSize>inadd)
				   	  	{
                            if((PicFileSize-inadd)>512)
                                 readsize=512;
							else
							 {	 
							    readsize=PicFileSize-inadd; // ���һ��  
							    rt_kprintf("\r\n   ���һ�� readsize =%d \r\n",readsize);
							 }
				   	  	}
					  else
					  	 return false;
                  	}
				  else
				   if(TF_Card_Status()==1)  
				   	{  ;
				   	 /* inadd=(Photo_sdState.SD_packetNum-1)<<9; //����512
				   	  if(PicFileSize>inadd)
				   	  	{
                            if((PicFileSize-inadd)>512)
                                 readsize=512;
							else
							 {	 
							    readsize=PicFileSize-inadd; // ���һ��  
							    rt_kprintf("\r\n   ���һ�� readsize =%d \r\n",readsize);
							 }
				   	  	}
					  else
					  	 return false;
                      i=read_file(PictureName,inadd,readsize,Original_info + Original_info_Wr); 
					  if(i==false)
					  	{
                          rt_kprintf("\r\n ͼƬ�ļ�: %s   ��ȡʧ��\r\n",PictureName); 
                          return false;
					  	} */
				   	} 
			 Original_info_Wr+=readsize;		 //         
		          break;
		  case 1 : // ��Ƶ
			  if(((Sound_sdState.photo_sending)==enable)&&((Sound_sdState.SD_flag)==enable))
			   {
				   Sound_sdState.SD_flag=disable;// clear	   
			   }
			   else
				 return false; 
                       //------------------------------------------------------------------------
			   //  ---------------  ��д����  ---------------
			   //			read		Photo_sdState.SD_packetNum��1��ʼ����
			   //			content_startoffset 	picpage_offset				 contentpage_offset
	                  if(TF_Card_Status()==0) 
	                  {
				       Api_DFdirectory_Read(voice, Original_info + Original_info_Wr, 512,1,MediaObj.Media_currentPacketNum);  
					inadd=(Sound_sdState.SD_packetNum-1)<<9; //����512
					if(SrcFileSize>inadd)
					 {
			                           if((SrcFileSize-inadd)>512)
			                                 readsize=512;
							else
							 {	 
							     readsize=SrcFileSize-inadd; // ���һ��  
							     rt_kprintf("\r\n   ���һ�� readsize =%d \r\n",readsize);  
							 }
					  }
					  else
						  return false;    
	                  	}
				 rt_kprintf("\r\n Sound_sdState.SD_packetNum= %d   filesize=%d  readsize=%d  \r\n",Sound_sdState.SD_packetNum,SrcFileSize,SrcFileSize-inadd);  
                              Original_info_Wr+=readsize;	  


			 //-------------------------------------------------------------------------		   
			    /*
			   //  ---------------	��д����  ---------------		
			   if(TF_Card_Status()==1)
				{
				  if(mp3_sendstate==0)
				  {
						  if(Sound_sdState.SD_packetNum==1)
						  	{  // wav tou
						  	  
							  inadd=WaveFile_EncodeHeader(SrcFileSize ,Original_info + Original_info_Wr);
							  Original_info_Wr+=inadd;
							  rt_kprintf("\r\n д���ļ�ͷ��СΪ wav fileheadersize=%d  \r\n",inadd);    

						  	}				
						  //---------------------------------------------------------
						  soundpage=(Sound_sdState.SD_packetNum-1)/5;// �õ�page 
						  sounddelta=((Sound_sdState.SD_packetNum-1)%5)*SpxGet_Size; // �õ�ҳ��ƫ��
		                   rt_kprintf("\r\n inadd=%d  soundpage =%d  inpageoffset=%d \r\n",inadd,soundpage,sounddelta);
						 //  i=read_file(SpxSrcName,(soundpage<<9),512,SpxBuf); 
						 //  if(i==false)
							// {
							  // rt_kprintf("\r\n spx�ļ�: %s   ��ȡʧ��--2\r\n",SpxSrcName);  
							   //return false;
							 //} 
						 Api_Config_read(voice, Sound_sdState.SD_packetNum, SpxBuf,500);   
						 memcpy(instr,SpxBuf+sounddelta,SpxGet_Size);   		
		                 //---------  spx Decode  5  �� ---------                 
						 speachDecode(instr, Original_info + Original_info_Wr); 
						 Original_info_Wr+=160;
						 speachDecode(instr+20, Original_info + Original_info_Wr);  
						 Original_info_Wr+=160; 
						 speachDecode(instr+40, Original_info + Original_info_Wr); 
						 Original_info_Wr+=160;
						 speachDecode(instr+60, Original_info + Original_info_Wr);    
						 Original_info_Wr+=160; 
						 speachDecode(instr+80, Original_info + Original_info_Wr); 
						 Original_info_Wr+=160;
				  	}
				 else
				 if(mp3_sendstate==1)
				 	{                       
						inadd=(Sound_sdState.SD_packetNum-1)<<9; //����512
						if(mp3_fsize>inadd)
						  {
							  if((mp3_fsize-inadd)>512)
								   readsize=512;
							  else
							   {   
								  readsize=mp3_fsize-inadd; // ���һ��	
								  rt_kprintf("\r\n	 ���һ�� mp3size =%d \r\n",readsize);
							   }
						  }
						else
						   return false;
						//rt_kprintf("\r\n ��ȡ�ļ�\r\n");
						i=read_file(SpxSrcName,inadd,readsize,Original_info + Original_info_Wr); 
						//rt_kprintf("\r\n ��ȡ�ļ����\r\n");
						if(i==false)
						  {
							rt_kprintf("\r\n mp3�ļ�: %s	��ȡʧ��\r\n",SpxSrcName); 
							return false;
						  } 					 
					  Original_info_Wr+=readsize;//	
                      

				 	}
			}
              else
			  	  return false; 
       */
		          break;
		  case 2 : // ��Ƶ
                  if(TF_Card_Status()==1)
                  	{  ;
					 /*	inadd=(Video_sdState.SD_packetNum-1)<<9; //����512
						if(wmv_fsize>inadd)
						  {
							  if((wmv_fsize-inadd)>512)
								   readsize=512;
							  else
							   {   
								  readsize=wmv_fsize-inadd; // ���һ��	
								  rt_kprintf("\r\n	 ���һ�� wmvsize =%d \r\n",readsize);
							   }
						  }
						else
						   return false;
						i=read_file(SpxSrcName,inadd,readsize,Original_info + Original_info_Wr); 
						if(i==false)
						  {
							rt_kprintf("\r\n mp3�ļ�: %s	��ȡʧ��\r\n",SpxSrcName); 
							return false;
						  } 					 
					  Original_info_Wr+=readsize;	   
                        */
                  	}
				  else
				  	return false;


		          break;
		  default:
		  	      return false;
         }	  

	 
	 if(MediaObj.Media_currentPacketNum>MediaObj.Media_totalPacketNum)
	 {
	 	return false;
	 }	
	 //  5. Send 
	 Protocol_End(Packet_Divide,0);
	  if(DispContent)
	   rt_kprintf("\r\n	Send Media Data \r\n");     
	//  else
	  {
		 rt_kprintf("\r\n pic_total_num:  %d   current_num:  %d   \r\n ",MediaObj.Media_totalPacketNum,MediaObj.Media_currentPacketNum);
		 if(MediaObj.Media_currentPacketNum>=MediaObj.Media_totalPacketNum)
		 {				    
			rt_kprintf("\r\n Media ���һ��block\r\n");
			
			if(0==MediaObj.RSD_State)	// �����˳����ģʽ�£����Ϊֹͣ״̬,�ȴ��������ش�
			 {
			    MediaObj.RSD_State=2;	
			    MediaObj.RSD_Timer=0;			   
			 } 
			
		 }							 

	  }  
     //----------  �ۼӷ��ͱ��� --------------------
     if(0==MediaObj.RSD_State)
     {
		 if(MediaObj.Media_currentPacketNum<MediaObj.Media_totalPacketNum)
		  {
		    //  ͼƬ
		    if(Photo_sdState.photo_sending==enable)
		       Photo_sdState.SD_packetNum++;
			//  ��Ƶ 
			if(Sound_sdState.photo_sending==enable)
			Sound_sdState.SD_packetNum++;
			//��Ƶ
			if(Video_sdState.photo_sending==enable)
			 Video_sdState.SD_packetNum++; 
		  }
     }
	 else
	 if(1==MediaObj.RSD_State)
	 {
	   MediaObj.RSD_Reader++;
	   if(MediaObj.RSD_Reader==MediaObj.RSD_total)
	   	    MediaObj.RSD_State=2; //  ��λ�ȴ�״̬���ȴ��������ٷ��ش�ָ��
	 }	
	 //----------  ����  -------------------
	 return true; 

}
//----------------------------------------------------------------------
u8  Stuff_MultiMedia_IndexAck_0802H(void)      
{  
   u16  totalNum=0, lenregwr=0;
   u16  i=0;  
   
   
	// 1. Head
	if(!Protocol_Head(MSG_0x0802,Packet_Normal)) 
 	  return false; 
	 // 2. content     
        //   float ID  Ӧ����ˮ�� 
	    Original_info[Original_info_Wr++]=(u8)(Centre_FloatID>>8);
	    Original_info[Original_info_Wr++]=(u8)Centre_FloatID;
          
		//------- ��ý�������� ----
		lenregwr=Original_info_Wr;		
	    Original_info[Original_info_Wr++]=(u8)(totalNum>>8); // ��ʱռ��λ��
	    Original_info[Original_info_Wr++]=(u8)totalNum;

		//----- ������ЧЧλ�� ----
		totalNum=0;
		  for(i=0;i<8;i++)
		  {
			  if(SD_ACKflag.f_MediaIndexACK_0802H==1)  // ͼ��
			  {		
				 Api_RecordNum_Read(pic_index,i,(u8*)&MediaIndex, sizeof(MediaIndex));   
			  }
			  else
			  if(SD_ACKflag.f_MediaIndexACK_0802H==2) // ��Ƶ
			  {
			       Api_RecordNum_Read(voice_index,i,(u8*)&MediaIndex, sizeof(MediaIndex));   
			  }  
			 // rt_kprintf("\r\n Effective_Flag %d  f_QueryEventCode %d  EventCode %d  \r\n",MediaIndex.Effective_Flag,SD_ACKflag.f_QueryEventCode,MediaIndex.EventCode);
			  if((MediaIndex.Effective_Flag==1)&&(SD_ACKflag.f_QueryEventCode==MediaIndex.EventCode))
			  { //  ������Ч�����������Ӧ���͵�����
			    Original_info[Original_info_Wr++]=(u8)(MediaIndex.MediaID>>24); //  ��ý��ID dworrd
			    Original_info[Original_info_Wr++]=(u8)(MediaIndex.MediaID>>16);
				Original_info[Original_info_Wr++]=(u8)(MediaIndex.MediaID>>8);
				Original_info[Original_info_Wr++]=(u8)(MediaIndex.MediaID);
			    Original_info[Original_info_Wr++]=MediaIndex.Type; //  ��ý������
				Original_info[Original_info_Wr++]=MediaIndex.ID;   //  ͨ��
			    Original_info[Original_info_Wr++]=MediaIndex.EventCode;
				memcpy(Original_info+Original_info_Wr,MediaIndex.PosInfo,28);
			    Original_info_Wr+=28;    
				totalNum++;    
			  }
			  
		  }

          //---------   ����������  -------- 
          Original_info[lenregwr]=(u8)(totalNum>>8);
		  Original_info[lenregwr+1]=totalNum;     

	 //  3. Send 
	Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	Send Media Index \r\n");    
	 return true; 

}
//--------------------------------------------------------------------------------------
u8  Stuff_DriverInfoSD_0702H(void)      
{  
    u8 i=0;
	 // 1. Head
	 if(!Protocol_Head(MSG_0x0702,Packet_Normal))  
 	    return false; 
	
	 // 2. content     
        //   ��ʻԱ��������
        i=strlen((const char*) JT808Conf_struct.Driver_Info.DriveName);
	    Original_info[Original_info_Wr++]=i;
	    memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Driver_Info.DriveName,i);// name
		Original_info_Wr+=i;
        memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Driver_Info.Driver_ID,20);// ���֤����
        Original_info_Wr+=20;
		memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Driver_Info.Drv_CareerID,40);//��ҵ�ʸ�֤
        Original_info_Wr+=40;
		i=strlen((const char*)JT808Conf_struct.Driver_Info.Comfirm_agentID); // ��������
		Original_info[Original_info_Wr++]=i;
		memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Driver_Info.Comfirm_agentID,i);
		Original_info_Wr+=i;
		
	 // 3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	Send Driver Info \r\n");    
	 return true; 

}
//---------------------------------------------------------------------------------
u8  Stuff_Worklist_0701H(void)      
{  
    u32  listlen=215;
	 // 1. Head
	 if(!Protocol_Head(MSG_0x0701,Packet_Normal)) 
 	    return false; 
	
	 // 2. content     
        //   ��Ϣ����
	     listlen=207;
		 Original_info[Original_info_Wr++]=(listlen>>24);// �����¼�ID
		 Original_info[Original_info_Wr++]=(listlen>>16);
		 Original_info[Original_info_Wr++]=(listlen>>8);		  
		 Original_info[Original_info_Wr++]=listlen;
	    
        memcpy(Original_info+Original_info_Wr,"���˵�λ:�����һ��ͨ�Ź㲥���޹�˾ �绰:022-26237216  ",55);
        Original_info_Wr+=55;		
        memcpy(Original_info+Original_info_Wr,"���˵�λ:����������乫˾ �绰:022-86692666  ",45);
        Original_info_Wr+=45;
		memcpy(Original_info+Original_info_Wr,"��Ʒ����:GPS�����ն�  ��װ��ʽ:  ��ʽ   ÿ������: 20   ����: 30��  ",67);
        Original_info_Wr+=67;
		memcpy(Original_info+Original_info_Wr,"����:��ʽС���� �˴����� :  2012-1-11   ",40); 
        Original_info_Wr+=40; 
		
	 // 3. Send 
	Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	Send Worklist  \r\n");    
	 return true; 

}
//------------------------------------------------------------------------------------
u8 Stuff_DataTrans_0900_ISP_ACK(u8  AckType)
{  
  u16  TX_NUM=0,Rec_Num=0,i=0;	 
  u8   Gfcs=0;
  //---------------------------------------------
   // 1. Head
   if(!Protocol_Head(MSG_0x0900,Packet_Normal)) 
	 return false; 
	// 2. content	
	//	Ӧ����ˮ��
   Original_info[Original_info_Wr++]=1;// ����͸�����ݵ����� 1	��ʾԶ������
	// 3. ����
		//	3.1. Sub Head  
		   Rec_Num=Original_info_Wr; //reglen 
			memcpy(Original_info+Original_info_Wr,"*GB",3); // GPRS ��ʼͷ
		   Original_info_Wr+=3;
		   
		   Original_info[Original_info_Wr++]=0x00;	// SIM ����   ���������ֽ���д0x00
		   Original_info[Original_info_Wr++]=0x00;
		   memcpy(Original_info+Original_info_Wr,SIM_code,6);
		   Original_info_Wr+=6;  
  
		   Original_info_Wr+=2;   // �������������ֽڣ������������д
		   
		   Original_info[Original_info_Wr++]=0x00; //��Ϣ��� Ĭ�� 0x00
		 // 3.2 Sub Info			 
			 Original_info[Original_info_Wr-1]=0x01; //��Ϣ��� Ĭ�� 0x00
			 Original_info[Original_info_Wr++]=0x30; //���� 	����Ӧ��
			 
			 Original_info[Original_info_Wr++]=AckType; // ������	

			 switch(AckType)
			 	{
                  case  ISPACK_92:
				  case  ISPoverACK_96:    // ����Ϊ��
				  	               break;
                  case  ISPinfoACK_94:                        
								Original_info[Original_info_Wr++]=(u8)(ISP_current_packnum>>8); // ��ǰ����
								Original_info[Original_info_Wr++]=(u8)ISP_current_packnum;
								Original_info[Original_info_Wr++]=(u8)(ISP_total_packnum>>8); // �ܰ���
								Original_info[Original_info_Wr++]=(u8)ISP_total_packnum;
								Original_info[Original_info_Wr++]=ISP_ack_resualt;	// ���ص�״̬	 
					               break;
                  default:
				  	      return false;
			 	}
  
		 // 3.3 Sub end
			 TX_NUM=Original_info_Wr-Rec_Num; // ��Э��ĳ���
				//----stuff Ginfolen ---- 
			 Original_info[Rec_Num+11]=(u8)((TX_NUM-2)>>8);  // ���Ȳ���ͷ 
			 Original_info[Rec_Num+12]=(u8)(TX_NUM-2); 
			 //rt_kprintf("\r\n   ����	A=%X B=%X	reg=%d Reg=%d  REg=%d \r\n",GPRS_info[11],GPRS_info[12],reg,(u8)(reg<<8),reg&0x00ff);
			
			  Gfcs=0;				  //  ����ӵ绰���뿪ʼ��У��ǰ���ݵ�����  G Э��У�� 
			  for(i=3;i<Original_info_Wr;i++)
				   Gfcs^=Original_info[i]; 
			  Original_info[Original_info_Wr++]=Gfcs;  // ��дGУ��λ
			
			   
			  Original_info[Original_info_Wr++]=0x0D;  // G Э��β	
			  Original_info[Original_info_Wr++]=0x0A;						 
	
	//	3. Send 
    Protocol_End(Packet_Normal,1 ); 
   if(DispContent)
   	{
      		 switch(AckType)
			 	{
                  case  ISPACK_92:
				  	              rt_kprintf("\r\n	 ����͸��  ISP ID=%2X  ACK \r\n",ISPACK_92);       
				  	               break;
				  case  ISPoverACK_96:    // ����Ϊ��
				                  rt_kprintf("\r\n	 ����͸�� ISP ID=%2X  overACK \r\n",ISPoverACK_96); 
				  	               break;
                  case  ISPinfoACK_94:
					              // rt_kprintf("\r\nISP info ACK   current=%d ,totalnum=%d \r\n",ISP_current_packnum,ISP_total_packnum);		
					               break; 
                  default:
				  	      return false;
			 	}
 
   	}
   
   return true; 


}

//----------------------------------------------
u8  Update_HardSoft_Version_Judge(u8 * instr)
{
     //   ��ȡ��50 ҳ��Ϣ
     	  Device_type=((u32)instr[1]<<24)+((u32)instr[2]<<16)+((u32)instr[3]<<8)+(u32)instr[4];
	  Firmware_ver=((u32)instr[5]<<24)+((u32)instr[6]<<16)+((u32)instr[7]<<8)+(u32)instr[8];
	  rt_kprintf("	\r\n �豸����: %x  ����汾:%x \r\n",Device_type,Firmware_ver); 
	 
	 if(Device_type!=STM32F407_Recoder_32MbitDF)   
	  { 
		 rt_kprintf( "\r\n �豸���Ͳ�ƥ�䲻�����" );
		 return false;
	  }  		   
        else
		  return true;


}




 //-------------------- ISP Check  ---------------------------------------------
void  ISP_file_Check(void)
{
   
         memset(ISP_buffer,0,sizeof(ISP_buffer));
	  ISP_Read(ISP_APP_Addr,ISP_buffer,PageSIZE);
	  /*
		   ���   �ֽ���	����			  ��ע
		  1 		  1    ���±�־ 	 1 ��ʾ��Ҫ����   0 ��ʾ����Ҫ����
		  2-5			  4   �豸����				 0x0000 0001  ST712   TWA1
											0x0000 0002   STM32  103  ��A1
											0x0000 0003   STM32  101  ������
											0x0000 0004   STM32  A3  sst25
											0x0000 0005   STM32  �г���¼�� 
		  6-9		 4	   ����汾 	 ÿ���豸���ʹ�  0x0000 00001 ��ʼ���ݰ汾���ε���
		  10-29 	  20	����		' mm-dd-yyyy HH:MM:SS'
		  30-31 	  2    ��ҳ��		   ��������Ϣҳ
		  32-35 	  4    ������ڵ�ַ    
		  36-200	   165	  Ԥ��	  
		  201-		  n    �ļ���  
	 
	  */
	  //------------   Type check  ---------------------
	  Device_type=((u32)ISP_buffer[1]<<24)+((u32)ISP_buffer[2]<<16)+((u32)ISP_buffer[3]<<8)+(u32)ISP_buffer[4];
	  Firmware_ver=((u32)ISP_buffer[5]<<24)+((u32)ISP_buffer[6]<<16)+((u32)ISP_buffer[7]<<8)+(u32)ISP_buffer[8];
	  rt_kprintf("	\r\n �豸����: %x  ����汾:%x \r\n",Device_type,Firmware_ver); 
	 
	 if(Device_type!=STM32F407_Recoder_32MbitDF)   
	  { 
		 rt_kprintf( "\r\n �豸���Ͳ�ƥ�䲻�����" );
		 ISP_buffer[0]=0;	 // ���������صĳ���
		  ISP_Write(ISP_APP_Addr,ISP_buffer,PageSIZE); 
	  }  		   
	 	 
	  rt_kprintf( "\r\n �ļ�����: " );					  
	  rt_kprintf("%20s",(const char*)(ISP_buffer+10));	 
	  rt_kprintf( "\r\n");	
	  rt_kprintf( "\r\n �ļ���: " );
	  rt_kprintf("%100s",(const char*)(ISP_buffer+201));      
	  rt_kprintf( "\r\n");	
	 if(Device_type==STM32F407_Recoder_32MbitDF)         
	 {
		Systerm_Reset_counter= (Max_SystemCounter-5);	 // ׼�������������³���
		ISP_resetFlag=1;//׼������ 
		rt_kprintf( "\r\n ׼���������³���!\r\n" );	 
	 }	 
   
 }



//----------------------------------------------------------------------------------
void Stuff_O200_Info_Only( u8* Instr) 
{
   u8  Infowr=0;
   
	// 1. �澯��־  4
	memcpy( ( char * ) Instr+ Infowr, ( char * )Warn_Status,4 );    
	Infowr += 4;
	// 2. ״̬  4
	memcpy( ( char * ) Instr+ Infowr, ( char * )Car_Status,4 );   
	Infowr += 4;
	// 3.  γ��
    memcpy( ( char * ) Instr+ Infowr,( char * )  Gps_Gprs.Latitude, 4 );//γ��   modify by nathan
	Infowr += 4;
	// 4.  ����
	memcpy( ( char * ) Instr+ Infowr, ( char * )  Gps_Gprs.Longitude, 4 );	  //����    ����  Bit 7->0   ���� Bit 7 -> 1
	Infowr += 4;
	// 5.  �߳�
	Instr[Infowr++]=(u8)(GPS_Hight<<8);
	Instr[Infowr++]=(u8)GPS_Hight;
	// 6.  �ٶ�    0.1 Km/h
	Instr[Infowr++]=(u8)(Speed_gps>>8);
	Instr[Infowr++]=(u8)Speed_gps;   
	// 7. ����   ��λ 1��
	Instr[Infowr++]=(GPS_direction>>8);  //High 
	Instr[Infowr++]=GPS_direction; // Low
	// 8.  ����ʱ��	
	Instr[Infowr++]=(((Gps_Gprs.Date[0])/10)<<4)+((Gps_Gprs.Date[0])%10);		
	Instr[Infowr++]=((Gps_Gprs.Date[1]/10)<<4)+(Gps_Gprs.Date[1]%10); 
	Instr[Infowr++]=((Gps_Gprs.Date[2]/10)<<4)+(Gps_Gprs.Date[2]%10);
	Instr[Infowr++]=((Gps_Gprs.Time[0]/10)<<4)+(Gps_Gprs.Time[0]%10);
	Instr[Infowr++]=((Gps_Gprs.Time[1]/10)<<4)+(Gps_Gprs.Time[1]%10);
	Instr[Infowr++]=((Gps_Gprs.Time[2]/10)<<4)+(Gps_Gprs.Time[2]%10);	

}
//-----------------------------------------------------
u8  Save_MediaIndex( u8 type, u8* name, u8 ID,u8 Evencode)
{
  u8   i=0;

  if((type!=1)&&(type!=0))
    return false;
  
  //----- ������Чλ�� ----
  for(i=0;i<8;i++)
  {
	  if(type==0)  // ͼ��
	  {		
		   Api_RecordNum_Read(pic_index,i,(u8*)&MediaIndex, sizeof(MediaIndex));    
	  }
	  else
	  if(type==1) // ��Ƶ
	  {
	         Api_RecordNum_Read(voice_index,1,(u8*)&MediaIndex, sizeof(MediaIndex));    
	  }    
	  if(MediaIndex.Effective_Flag==0)
	  	 break;
  }
   if(i==8)  // �����������ӵ�һ����ʼ
   	   i=0;
  //----  ��д��Ϣ -------------
  memset((u8*)&MediaIndex,0,sizeof(MediaIndex));
  MediaIndex.MediaID= JT808Conf_struct.Msg_Float_ID;   
  MediaIndex.Type=type;
  MediaIndex.ID=ID;
  MediaIndex.Effective_Flag=1;
  MediaIndex.EventCode=Evencode;
  memcpy(MediaIndex.FileName,name,strlen((const char*)name));
  Stuff_O200_Info_Only(MediaIndex.PosInfo);
  
  if(type==0)  // ͼ��
  {
        Api_RecordNum_Write(pic_index,i,(u8*)&MediaIndex, sizeof(MediaIndex));    
  }
  else
  if(type==1) // ��Ƶ
  {
       Api_RecordNum_Write(voice_index,i,(u8*)&MediaIndex, sizeof(MediaIndex));    
  }
  return true;
  
}
//------------------------------------------------------------------
u8  CentreSet_subService_8103H(u32 SubID, u8 infolen, u8 *Content )
{  
 
  u8    i=0;
  u8    reg_str[80];
  u8    reg_in[20];
  u32   resualtu32=0;
  

   rt_kprintf("\r\n    �յ������������� SubID=%X \r\n",SubID);  
	   
  switch(SubID)
   {
     case 0x0001:  // �ն����������ͼ��  ��λ:s
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.Heart_Dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
                                   rt_kprintf("\r\n ���������: %d s\r\n",JT808Conf_struct.DURATION.Heart_Dur);
                    break;
	 case 0x0002:  // TCP ��ϢӦ��ʱʱ��  ��λ:s	                
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.TCP_ACK_Dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n TCP��ϢӦ����: %d s\r\n",JT808Conf_struct.DURATION.TCP_ACK_Dur); 
	                break;
	 case 0x0003:  //  TCP ��Ϣ�ش�����	                
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.TCP_ReSD_Num=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n TCP�ش�����: %d\r\n",JT808Conf_struct.DURATION.TCP_ReSD_Num); 
	                break;
	 case 0x0004:  // UDP ��ϢӦ��ʱʱ��  ��λ:s	                
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.UDP_ACK_Dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
                    rt_kprintf("\r\n UDPӦ��ʱ: %d\r\n",JT808Conf_struct.DURATION.UDP_ACK_Dur);
	                break;
	 case 0x0005:  //  UDP ��Ϣ�ش�����	                
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.UDP_ReSD_Num=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n UDP�ش�����: %d\r\n",JT808Conf_struct.DURATION.UDP_ReSD_Num); 
					break;
	 case 0x0010:  //  ��������APN 
                                    if(infolen==0)
					  	break;
					  memset(APN_String,0,sizeof(APN_String));					  
					  memcpy(APN_String,(char*)Content,infolen);  
					  memset((u8*)SysConf_struct.APN_str,0,sizeof(APN_String));	
					  memcpy((u8*)SysConf_struct.APN_str,(char*)Content,infolen);  
					 Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));


                                     DataLink_APN_Set(APN_String,1); 
									 
	                break; 
	 case 0x0013:  //  ����������ַ  IP ������
                      memset(reg_in,0,sizeof(reg_in)); 
					  memcpy(reg_in,Content,infolen);
					 //----------------------------	
					 
					 i =str2ip((char*)reg_in, RemoteIP_main);
					 if (i <= 3)
					 {
					  rt_kprintf("\r\n  ����: %s \r\n",reg_in); 
					  
					  memset(DomainNameStr,0,sizeof(DomainNameStr));					  
					  memset(SysConf_struct.DNSR,0,sizeof(DomainNameStr));
					  memcpy(DomainNameStr,(char*)Content,infolen);   
					  memcpy(SysConf_struct.DNSR,(char*)Content,infolen);   
					 Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));

                                     //----- ���� GSM ģ��------
                                    DataLink_DNSR_Set(SysConf_struct.DNSR,1); 

									 
					  SD_ACKflag.f_CentreCMDack_0001H=2 ;//DataLink_EndFlag=1; //AT_End(); �ȷ��ؽ���ٹҶ�  
					  break;
					 }
					 memset(reg_str,0,sizeof(reg_str));
					 IP_Str((char*)reg_str, *( u32 * ) RemoteIP_main); 		  
					 strcat((char*)reg_str, " :");		 
					 sprintf((char*)reg_str+strlen((const char*)reg_str), "%u\r\n", RemotePort_main);  
					 memcpy(SysConf_struct.IP_Main,RemoteIP_main,4);
					 SysConf_struct.Port_main=RemotePort_main;
					 
					 Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
					 rt_kprintf("\r\n ���������������� IP \r\n");
					 rt_kprintf("\r\n SOCKET :");  
					 rt_kprintf((char*)reg_str);   
					  //-----------  Below add by Nathan  ----------------------------				  
					  rt_kprintf("\r\n		   ����IP: %d.%d.%d.%d : %d \r\n",RemoteIP_aux[0],RemoteIP_aux[1],RemoteIP_aux[2],RemoteIP_aux[3],RemotePort_main);   
					 
					  //-----------  Below add by Nathan  ----------------------------
					  DataLink_MainSocket_set(RemoteIP_main,RemotePort_main,1);
	                              //-------------------------------------------------------------	
					   
					   SD_ACKflag.f_CentreCMDack_0001H=2 ;//DataLink_EndFlag=1; //AT_End(); �ȷ��ؽ���ٹҶ�  

	                break;
	 case 0x0014:  // ���ݷ����� APN

	                break;
	 case 0x0017:  // ���ݷ�����  IP                    
				  memset(reg_in,0,sizeof(reg_in)); 
				  memcpy(reg_in,Content,infolen);
				 //---------------------------- 
				  i =str2ip((char*)reg_in, RemoteIP_aux);
				  if (i <= 3)
				 {
					  rt_kprintf("\r\n  ����aux: %s \r\n",reg_in); 					  
					 memset(DomainNameStr_aux,0,sizeof(DomainNameStr_aux));					  
					 memset(SysConf_struct.DNSR_Aux,0,sizeof(DomainNameStr_aux));     
					  memcpy(DomainNameStr_aux,(char*)Content,infolen);   
					  memcpy(SysConf_struct.DNSR_Aux,(char*)Content,infolen);    
					 Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
                                     //----- ���� GSM ģ��------
                                    DataLink_DNSR2_Set(SysConf_struct.DNSR_Aux,1);
									 
					  SD_ACKflag.f_CentreCMDack_0001H=2 ;//DataLink_EndFlag=1; //AT_End(); �ȷ��ؽ���ٹҶ�  
					  break;
				 }  				  
				  memset(reg_str,0,sizeof(reg_str));
				  IP_Str((char*)reg_str, *( u32 * ) RemoteIP_aux);		   
				  strcat((char*)reg_str, " :"); 	  
				  sprintf((char*)reg_str+strlen((const char*)reg_str), "%u\r\n", RemotePort_aux);  
				 
				  Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
				  rt_kprintf("\r\n �������ñ��÷����� IP \r\n");
				  rt_kprintf("\r\nUDP SOCKET :");  
				  rt_kprintf((char*)reg_str);  
				  DataLink_AuxSocket_set(RemoteIP_aux,RemotePort_aux,1);
				   //-----------  Below add by Nathan  ----------------------------				   
				   rt_kprintf("\r\n 		����IP: %d.%d.%d.%d : %d \r\n",RemoteIP_aux[0],RemoteIP_aux[1],RemoteIP_aux[2],RemoteIP_aux[3],RemotePort_aux);	
	                break;
	 case 0x0018:  //  ������ TCP �˿�
                    //----------------------------	
                      if(infolen!=4)
						break;	
					  RemotePort_main=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];

					  Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
					  rt_kprintf("\r\n ���������������� PORT \r\n");
					  rt_kprintf("\r\nUDP SOCKET :");  
					  rt_kprintf((char*)reg_str);  
					   //-----------  Below add by Nathan  ----------------------------
                                     DataLink_MainSocket_set(RemoteIP_main,RemotePort_main,1);					   //-------------------------------------------------------------		   
					   SD_ACKflag.f_CentreCMDack_0001H=2 ;//DataLink_EndFlag=1; //AT_End(); �ȷ��ؽ���ٹҶ�	
	                break;
	 case 0x0019:  //  ������ UDP �˿�
                    
					if(infolen!=4)
					  break;  
					RemotePort_aux=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
					
					 Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
					rt_kprintf("\r\n ��������UDP������ PORT \r\n");
					rt_kprintf("\r\nUDP SOCKET :");   
					rt_kprintf((char*)reg_str);    
					rt_kprintf("\r\n		 ����IP: %d.%d.%d.%d : %d \r\n",RemoteIP_aux[0],RemoteIP_aux[1],RemoteIP_aux[2],RemoteIP_aux[3],RemotePort_aux);	 
	                break;
     case 0x0020:  //  �㱨����  0 ��ʱ�㱨  1 ����㱨 2 ��ʱ�Ͷ���㱨
                    if(infolen!=4)
						break;	
                    resualtu32=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
					switch(resualtu32)
						{
						   case 0:rt_kprintf("\r\n ��ʱ�㱨 \r\n");
						   	      break;
						   case 1:rt_kprintf("\r\n ����㱨 \r\n");
						   	      break;
						   case 2:rt_kprintf("\r\n ��ʱ�Ͷ���㱨\r\n"); 
						   	      break;
						   default:
						   	      break;

						}
	                break;
	 case 0x0021:  //  λ�û㱨����  0 ����ACC�ϱ�  1 ����ACC�͵�¼״̬�ϱ� 

	                break;
       //--------
					
	 case 0x0022:  //  ��ʻԱδ��¼ �㱨ʱ���� ��λ:s    >0	                  
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.NoDrvLogin_Dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
                                   rt_kprintf("\r\n ��ʻԱδ��¼�㱨���: %d\r\n",JT808Conf_struct.DURATION.NoDrvLogin_Dur);  
	                break;
	 case 0x0027:   //  ����ʱ�㱨ʱ��������λ s  >0	                
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.Sleep_Dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
                    rt_kprintf("\r\n ���߻㱨ʱ����: %d \r\n",JT808Conf_struct.DURATION.Sleep_Dur);   
	                break;
	 case 0x0028:   //  ��������ʱ�㱨ʱ����  ��λ s	                
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.Emegence_Dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                   Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct)); 
                    rt_kprintf("\r\n ��������ʱ����: %d \r\n",JT808Conf_struct.DURATION.Emegence_Dur);   
	                break;
	 case 0x0029:   //  ȱʡʱ��㱨���  ��λ s
	                if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.Default_Dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
					rt_kprintf("\r\n ȱʡ�㱨ʱ����: %d \r\n",JT808Conf_struct.DURATION.Default_Dur);   
	                break;
       //---------
					
	 case 0x002C:   //  ȱʡ����㱨���  ��λ ��
	                if(infolen!=4)
						break;					
					JT808Conf_struct.DISTANCE.Defalut_DistDelta=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n ȱʡ����㱨���: %d m\r\n",JT808Conf_struct.DISTANCE.Defalut_DistDelta); 
	                break;
	 case 0x002D:   //  ��ʻԱδ��¼�㱨������ ��λ ��
	               if(infolen!=4)
						break;					
					JT808Conf_struct.DISTANCE.NoDrvLogin_Dist=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct)); 
					rt_kprintf("\r\n ��ʻԱδ��¼�㱨����: %d m\r\n",JT808Conf_struct.DISTANCE.NoDrvLogin_Dist); 
	                break;
	 case 0x002E:   //  ����ʱ�㱨������  ��λ ��
	               if(infolen!=4)
						break;					
					JT808Conf_struct.DISTANCE.Sleep_Dist=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n ����ʱ�����ϱ����: %d m\r\n",JT808Conf_struct.DISTANCE.Sleep_Dist); 
	                break;
	 case 0x002F:   //  ��������ʱ�㱨������  ��λ ��
	               if(infolen!=4)
						break;					
					JT808Conf_struct.DISTANCE.Emergen_Dist=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n ��������ʱ�����ϱ����: %d m\r\n",JT808Conf_struct.DISTANCE.Emergen_Dist); 
	                break;
	 case 0x0030:   //  �յ㲹���Ƕ� , <180
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.SD_Delta_maxAngle=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n �յ㲹���Ƕ�: %d ��\r\n",JT808Conf_struct.DISTANCE.Emergen_Dist);  
	                break;
	 case 0x0031: 		 	            
                    if(infolen!=2)
						break;					
					JT808Conf_struct.DURATION.IllgleMovo_disttance=(Content[0]<<8)+Content[1]; 
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n ����Χ���뾶(�Ƿ��ƶ���ֵ): %d m\r\n",JT808Conf_struct.DURATION.IllgleMovo_disttance);  
        //---------
	 case 0x0040:   //   ���ƽ̨�绰����  
	                 if(infolen==0)
					 	 break;
					i=strlen((const char*)JT808Conf_struct.LISTEN_Num);
					rt_kprintf("\r\n old: %s \r\n",JT808Conf_struct.LISTEN_Num);
					 
					memset(JT808Conf_struct.LISTEN_Num,0,sizeof(JT808Conf_struct.LISTEN_Num));
					memcpy(JT808Conf_struct.LISTEN_Num,Content,infolen);											
					Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));  
 					rt_kprintf("\r\n new: %s \r\n",JT808Conf_struct.LISTEN_Num);
                     
	                //CallState=CallState_rdytoDialLis;  // ׼����ʼ����������� 
	                rt_kprintf("\r\n ���ü��ƽ̨����: %s \r\n",JT808Conf_struct.LISTEN_Num);   

	                break;
	 case 0x0041:   //   ��λ�绰���룬�ɲ��ô˵绰���벦���ն˵绰���ն˸�λ
                    if(infolen==0)
					 	 break;
					memset(reg_str,0,sizeof(reg_str)); 
					memcpy(reg_str,Content,infolen);											
 					rt_kprintf("\r\n ��λ�绰���� %s \r\n",reg_str);  
	                break;
	 case 0x0042:   //   �ָ��������õ绰���ɲ��øõ绰�������ն˻ָ���������

	                break;
	 case 0x0045:   //  �ն˵绰�������� 0 �Զ�����  1 ACC ON�Զ����� OFFʱ�ֶ�����

	                break;
	 case 0x0046:   //  ÿ��ͨ���ʱ�� ����λ  ��
                    
	                break;
	 case 0x0047:   //  �����ͨ��ʱ�䣬��λ  ��

	                break;
	 case 0x0048:   //  �����绰����                    
					if(infolen==0)
						 break;
					memset(JT808Conf_struct.LISTEN_Num,0,sizeof(JT808Conf_struct.LISTEN_Num));
					memcpy(JT808Conf_struct.LISTEN_Num,Content,infolen); 										
					Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));  
					CallState=CallState_rdytoDialLis;  // ׼����ʼ����������� 
					rt_kprintf("\r\n ���������������: %s \r\n",JT808Conf_struct.LISTEN_Num);  
	                break;  

	    //----------				
	 case 0x0050:  //  ���������֣� ��λ����Ϣ�б�����־���Ӧ����ӦλΪ1ʱ����������---
                    
                    if(infolen!=4)
						break;	
                    resualtu32=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    rt_kprintf("\r\n ����������: %x \r\n",resualtu32);    
					break;
	 case 0x0052:  //  �������տ��أ� �뱨����־��Ӧ��λ1ʱ������

	                break;
	 case 0x0053:  //  �������մ洢  	�뱨����־��Ӧ��λ1ʱ�����մ洢 ����ʵʱ�ϴ�

	                break;
	 case 0x0054:  //  �ؼ���־  		�뱨����־��Ӧ��λ1  Ϊ�ؼ�����

	                break;
        //---------  
					
	 case 0x0055:  //  ����ٶ�   ��λ   ǧ��ÿСʱ
                   if(infolen!=4)
						break;
				    JT808Conf_struct.Speed_warn_MAX=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)	+Content[3];
				   memset(reg_str,0,sizeof(reg_str));
				   memcpy(reg_str,& JT808Conf_struct.Speed_warn_MAX,4);
				   memcpy(reg_str+4,&JT808Conf_struct.Spd_Exd_LimitSeconds,4);
				    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
				   rt_kprintf("\r\n ����ٶ�: %d km/h \r\n", JT808Conf_struct.Speed_warn_MAX); 
				   Spd_ExpInit();
	                break;
	 case 0x0056:  //  ���ٳ���ʱ��    ��λ s
                   if(infolen!=4)
						break;
				   JT808Conf_struct.Spd_Exd_LimitSeconds=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)	+Content[3];
				   memset(reg_str,0,sizeof(reg_str));
				   memcpy(reg_str,& JT808Conf_struct.Speed_warn_MAX,4);
				   memcpy(reg_str+4,&JT808Conf_struct.Spd_Exd_LimitSeconds,4); 
				    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
				   rt_kprintf("\r\n ��ʱ����ʱ��: %d s \r\n",JT808Conf_struct.Spd_Exd_LimitSeconds); 
				   Spd_ExpInit();
	                break;
	 case 0x0057:  //  ������ʻʱ������ ��λ  s
					 if(infolen!=4)
						  break;
				   TiredConf_struct.TiredDoor.Door_DrvKeepingSec=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)   +Content[3];
                               Api_Config_write(tired_config,0,(u8*)&TiredConf_struct,sizeof(TiredConf_struct));				   rt_kprintf("\r\n ������ʻʱ������: %d s \r\n",TiredConf_struct.TiredDoor.Door_DrvKeepingSec);  
				   Warn_Status[3]&=~0x04;  //BIT(2)	�Ӵ�ƣ�ͼ�ʻ���� 
				   TIRED_Drive_Init();    // ���ƣ�ͼ�ʻ״̬      
	               break;
	 case 0x0058:  //  �����ۼƼ�ʻʱ������  ��λ  s
				   if(infolen!=4)
						break;
				   TiredConf_struct.TiredDoor.Door_DayAccumlateDrvSec=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)   +Content[3];
                               Api_Config_write(tired_config,0,(u8*)&TiredConf_struct,sizeof(TiredConf_struct));				   rt_kprintf("\r\n ������ʻʱ������: %d s \r\n",TiredConf_struct.TiredDoor.Door_DrvKeepingSec);  
				   TiredConf_struct.Tired_drive.ACC_ONstate_counter=0;
				   TiredConf_struct.Tired_drive.ACC_Offstate_counter=0;
                                Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct)); 
                               rt_kprintf("\r\n �����ۼƼ�ʻʱ��: %d s \r\n",TiredConf_struct.TiredDoor.Door_DayAccumlateDrvSec); 
	                break;
     case 0x0059:  //  ��С��Ϣʱ��  ��λ s
				   if(infolen!=4)
						break;
				    TiredConf_struct.TiredDoor.Door_MinSleepSec=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)   +Content[3];
                               Api_Config_write(tired_config,0,(u8*)&TiredConf_struct,sizeof(TiredConf_struct));				   rt_kprintf("\r\n ������ʻʱ������: %d s \r\n",TiredConf_struct.TiredDoor.Door_DrvKeepingSec);  
					rt_kprintf("\r\n ��С��Ϣʱ��: %d s \r\n",TiredConf_struct.TiredDoor.Door_MinSleepSec);  
	                break;
	 case 0x005A:  //  �ͣ��ʱ��   ��λ s
					 if(infolen!=4)
						  break;
					TiredConf_struct.TiredDoor.Door_MaxParkingSec=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)	 +Content[3];
                                   Api_Config_write(tired_config,0,(u8*)&TiredConf_struct,sizeof(TiredConf_struct));				   rt_kprintf("\r\n ������ʻʱ������: %d s \r\n",TiredConf_struct.TiredDoor.Door_DrvKeepingSec);  
					TiredConf_struct.TiredDoor.Parking_currentcnt=0; 
					 Warn_Status[1]&=~0x08;  // �����ʱ���� 
					rt_kprintf("\r\n �ͣ��ʱ��: %d s \r\n",TiredConf_struct.TiredDoor.Door_MaxParkingSec);   
	                break;
	     //--------- 
	 case  0x0070: //  ͼ��/��Ƶ����  1-10  1 ���

	                break;
	 case  0x0071: //  ����  0-255

	                break;
	 case  0x0072: //  �Աȶ�  0-127

	                break;
	 case  0x0073: // ���Ͷ�  0-127

	                break;
	 case  0x0074: // ɫ��   0-255

	                break;
		  //---------
	 case  0x0080: // ������̱����   1/10 km

	                break;
	 case  0x0081: // �������ڵ�ʡ��ID
	                if(infolen!=2)
						  break;
					JT808Conf_struct.Vechicle_Info.Dev_ProvinceID=(Content[0]<<8)+Content[1];
                                   Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n ��������ʡ��ID: 0x%X \r\n",JT808Conf_struct.Vechicle_Info.Dev_ProvinceID); 
	                break;
	 case  0x0082: // ������������ID
	                if(infolen!=2)
						  break;
					JT808Conf_struct.Vechicle_Info.Dev_ProvinceID=(Content[0]<<8)+Content[1];
                                   Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n ������������ID: 0x%X \r\n",JT808Conf_struct.Vechicle_Info.Dev_ProvinceID); 
	                break;
	 case  0x0083: // ������ͨ�����Ű䷢�Ļ���������
	                if(infolen<4)
						  break;					
					memset(JT808Conf_struct.Vechicle_Info.Vech_Num,0,sizeof(JT808Conf_struct.Vechicle_Info.Vech_Num));
					memcpy(JT808Conf_struct.Vechicle_Info.Vech_Num,Content,infolen);
                                   Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n ��������ʻ֤��: %s  \r\n",JT808Conf_struct.Vechicle_Info.Vech_Num);  
	                break;
	 case  0x0084: // ������ɫ  ���չ��ҹ涨
	                if(infolen!=1)
						  break;
					JT808Conf_struct.Vechicle_Info.Dev_Color=Content[0];           
                                   Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n ������ɫ: %d  \r\n",JT808Conf_struct.Vechicle_Info.Dev_Color);                 
	                break;
	 default:
				return false;
   }

    return true;
}
//--------------------------------------------------------------------
u8  CentreSet_subService_8105H(u32 Control_ID, u8 infolen, u8 *Content )
{

   switch(Control_ID) 
   {
      case 1:  //  ������������  ����֮����÷ֺŷָ�   ָ���ʽ����:
             /*
URL ��ַ���������ƣ������û������������룻��ַ��TCP�˿ڣ�UDP�˿ڣ�������ID; Ӳ���汾���̼��汾�����ӵ�ָ��������ָ���Ƿ�����ʱ�ޣ�
           ��ĳ����������ֵ����ſ�
               */
             rt_kprintf("\r\n �������� \r\n");  
			 rt_kprintf("\r\n ����: %s\r\n",Content); 
		     break;
	  case 2:  // �����ն�����ָ��������
	        /*
���ӿ��ƣ����ƽ̨��Ȩ�룻���ŵ����ƣ� �����û������������룻��ַ��TCP�˿ڣ�UDP�˿ڣ����ӵ�ָ��������ʱ��
           ��ÿ����������ֵ����ſ�
	         */  
	         rt_kprintf("\r\n �ն˿�������ָ��������\r\n");  
			 rt_kprintf("\r\n ����: %s\r\n",Content);
			 break;
	  case 3:  //  �ն˹ػ�
              SD_ACKflag.f_CentreCMDack_0001H=5; 
			  rt_kprintf("\r\n �ն˹ػ� \r\n");  
	         break;
      case 4:  //  �ն˸�λ
               SD_ACKflag.f_CentreCMDack_0001H=3;
			   rt_kprintf("\r\n �ն˸�λ \r\n");          
	         break;
	  case 5: //   �ն˻ָ���������		 		  	       
	           /* if(SysConf_struct.Version_ID==SYSID)   //  check  wether need  update  or not 
	          	{                    
					SysConf_struct.Version_ID=SYSID+1;   
					Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
					Systerm_Reset_counter=Max_SystemCounter;
					ISP_resetFlag=2;    //   ����Զ�������������Ƹ�λϵͳ 
	          	}
				rt_kprintf("\r\n �ָ��������� \r\n"); */ 
			 break;
	  case 6: //   �ر�����ͨ��
             SD_ACKflag.f_CentreCMDack_0001H=5; 
			 rt_kprintf("\r\n �ر�����ͨ�� \r\n"); 
	         break;
	  case 7: //   �ر���������ͨ��
             SD_ACKflag.f_CentreCMDack_0001H=5; 
			 rt_kprintf("\r\n �ر�����ͨ�� \r\n");   
		     break;
	  default:
	  	     return false;			 

   }
      return  true;
}

//-------------------------------------------------------------------
void CenterSet_subService_8701H(u8 cmd,  u8*Instr)
{  
 
  
  switch(cmd)
   {
     case 0x81: //	  �������� ��ʻԱ����  ��ʻ֤����
				memset(JT808Conf_struct.Driver_Info.DriverCard_ID,0,18);
                //  ��ʻԱ����û�д��� 3���ֽ� 
				memcpy(JT808Conf_struct.Driver_Info.DriveCode,Instr,3); 						   
			       memcpy(JT808Conf_struct.Driver_Info.DriverCard_ID,Instr+3,18); //ֻҪ��ʻ֤����
				 Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
				break;
   
	 case 0x82: //	  �������ó��ƺ�  
				memset((u8*)&JT808Conf_struct.Vechicle_Info,0,sizeof(JT808Conf_struct.Vechicle_Info));		
			
                memcpy(JT808Conf_struct.Vechicle_Info.Vech_VIN,Instr,17);
				memcpy(JT808Conf_struct.Vechicle_Info.Vech_Num,Instr+17,12);
				memcpy(JT808Conf_struct.Vechicle_Info.Vech_Type,Instr+29,12); 

				Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));    
				break;
	 case 0xC2: //���ü�¼��ʱ��
			    // ûɶ�ã������ظ����У�����GPSУ׼�͹��� 

		        break;

	 case 0xC3: //�����ٶ�����ϵ��������ϵ����
				 JT808Conf_struct.Vech_Character_Value=(u32)(Instr[0]<<16)+(u32)(Instr[1]<<8)+(u32)Instr[2]; // ����ϵ��  �ٶ�����ϵ��
				
				 JT808Conf_struct.DF_K_adjustState=0;		
		                ModuleStatus&=~Status_Pcheck; 
	                      Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
		        break;
	 default:
				break;
   }


}
//-----------------------------------------------
u8  CentreSet_subService_FF01H(u32 SubID, u8 infolen, u8 *Content )
{  
 
 
  u32  baud=0;
  u8    u3_Txstr[40];
  u8    Destu3_Txstr[40];
  u8    len=0;
		u8 i=0;


   rt_kprintf("\r\n    �յ���չ�ն��������� SubID=%X \r\n",SubID);  
	   
  switch(SubID)
   {
     case 0x0001:  //GNSS ��λģʽ�л�
                    if(infolen!=4)
						break;			 		
		      BD_EXT.BD_Mode=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];

		      switch(BD_EXT.BD_Mode)
		      	{
                         case 0x00:  // �� GPS ��λģʽ                        
						   gps_mode("2");
						 break;
			    case  0x01:  // 	��BD2 ��λģʽ
			                        gps_mode("1");
				               break;
			    case  0x02: //  BD2+GPS ��λģʽ
						   gps_mode("3");
				               break;							   	
		      	}
                    //BD_EXT_Write();   
                    break;
	 case 0x0002:  // GNSS ����������            
                    if(infolen!=4)
						break;					
			BD_EXT.BD_Baud=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
			switch(BD_EXT.BD_Baud)
				{ 
				    case 0x00:  //  4800
                                                  baud=4800;    
							//rt_thread_delay(5);						  
							//gps_write("$PCAS01,0*1C\r\n",14);	
							//rt_thread_delay(5);	
							// gps_baud( 4800 );
						        break;
				     case 0x01: //9600   --default 
				                     baud=9600;
							//rt_thread_delay(5);			 
                                                 //gps_write("$PCAS01,1*1D\r\n",14);	
							//rt_thread_delay(5);
							//gps_baud( 9600 );
						        break;
				     case  0x02: // 19200
				                     baud=19200;
							//rt_thread_delay(5);			 
                                                 //gps_write("$PCAS01,2*1E\r\n",14);	
							//rt_thread_delay(5);	
							//gps_baud( 19200 );
					               break;
				     case  0x03://  38400
                                                  baud=38400;
							 //rt_thread_delay(5);						  
							 //gps_write("$PCAS01,3*1F\r\n",14);	
							 //rt_thread_delay(5);	
							 //gps_baud( 38400 ); 
					               break;
				     case  0x04:// 57600
				                      baud=57600;
							 //rt_thread_delay(5);			  
                                                  //gps_write("$PCAS01,4*18\r\n",14);	
							 //rt_thread_delay(5);		
							 //gps_baud( 57600 );
					               break;
				     case   0x05:// 115200
				                      baud=115200; 
							//rt_thread_delay(5);			  
                                                 // gps_write("$PCAS01,5*19\r\n",14);	
							 //rt_thread_delay(5);	
							// gps_baud( 115200 ); 
					               break;				    				

				}
		      	 //---UART_GPS_Init(baud);  //   �޸Ĵ��ڲ����� 
		      	 rt_thread_delay(20);
                      // BD_EXT_Write();   
			rt_kprintf("\r\n ��������GNSS ������:  %d s\r\n",baud); 
	                break;
	 case 0x0003:  //  BNSS NMEA ���������               
                    if(infolen!=4)
						break;					
 		       BD_EXT.BD_OutputFreq=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                     switch(BD_EXT.BD_OutputFreq)
                     	{
                                 case 0x00: 
							 baud=500;
							 gps_write("$PCAS02,500*1A\r\n",16);	
 							 rt_thread_delay(5);	
							 break;
				     case 0x01:
					 	          baud=1000;
							  gps_write("$PCAS02,1000*2E\r\n",16);
							  rt_thread_delay(5);	
							  break;
				     case 0x02: 			  
							  baud=2000;
							 gps_write("$PCAS02,2000*2D\r\n",16);
							 rt_thread_delay(5);	
							  break;
				     case  0x03:
					 	         baud=3000;
							 gps_write("$PCAS02,3000*2C\r\n",16);
							 rt_thread_delay(5);	
							 break;
				     case  0x04:
					 	        baud=4000;
							 gps_write("$PCAS02,4000*2B\r\n",16);
							 rt_thread_delay(5);	
							 break;

                     	}
			     // BD_EXT_Write();   
			      rt_kprintf("\r\n  GNSS �������: %dms\r\n",baud);  
	                break;
	 case 0x0004:  // GNSS �ɼ�NMEA ����Ƶ��               
                    if(infolen!=4)
						break;					
			 BD_EXT.BD_SampleFrea=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                       rt_kprintf("\r\n GNSS �ɼ�Ƶ��: %d s\r\n", BD_EXT.BD_SampleFrea);   
	                break;
	 case 0x0005:  //  CAN 1  ��������               
                    if(infolen!=4)
						break;					
			BD_EXT.CAN_1_Mode=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                     CAN_App_Init();
			//BD_EXT_Write();   
			rt_kprintf("\r\n ��������CAN1 : 0x%08X\r\n",BD_EXT.CAN_1_Mode); 
			break;
	 case 0x0006:  //  CAN2   ��������    ----- Ҫ��С��ͨ��
                    if(infolen!=4)
						break;					
			BD_EXT.CAN_2_Mode=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    // BD_EXT_Write();  
			memset( u3_Txstr,0,sizeof(u3_Txstr));
			memset( Destu3_Txstr,0,sizeof(Destu3_Txstr));		 
			u3_Txstr[0]=0x7E;       // ͷ
		       u3_Txstr[1]=0x33;		//  CAN ���
		       u3_Txstr[2]=0x01;    //  �������� 
		       u3_Txstr[3]=(BD_EXT.CAN_2_Mode>>24);
			u3_Txstr[4]=(BD_EXT.CAN_2_Mode>>16);   	
	              u3_Txstr[5]=(BD_EXT.CAN_2_Mode>>8);	       
                     u3_Txstr[6]=(BD_EXT.CAN_2_Mode);	  
			u3_Txstr[7]=0x7E;	              				 

                     Destu3_Txstr[0]=0x7E;
			len=Protocol_808_Encode(Destu3_Txstr+1, u3_Txstr+1, 6);
			Destu3_Txstr[len+1]=0x7E;
		
			rt_kprintf("\r\n U3_orginal:");
			for(i=0;i<8;i++)
				rt_kprintf("% 02X",u3_Txstr[i]); 
			rt_kprintf("\r\nU3_len=%d \r\n",len);
			for(i=0;i<len+2;i++)
				rt_kprintf("% 02X",Destu3_Txstr[i]);     
			 // U3_PutData(Destu3_Txstr,len+2);  //  ���͸�����	
			rt_device_write(&Device_CAN2,0, Destu3_Txstr,len+2);       
			rt_kprintf("\r\n ��������CAN2 : 0x%08X\r\n",BD_EXT.CAN_2_Mode);    
	                break; 
        case 0x0007:  //  ��ײ��������
                    if(infolen!=4)
						break;					
			BD_EXT.Collision_Check=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                     //BD_EXT_Write();   
			memset( u3_Txstr,0,sizeof(u3_Txstr));
			memset( Destu3_Txstr,0,sizeof(Destu3_Txstr));		 
			u3_Txstr[0]=0x7E;       // ͷ
		       u3_Txstr[1]=0x32;		//  CAN ��� 
		       u3_Txstr[2]=0x01;    //  �������� 
		       u3_Txstr[3]=(BD_EXT.Collision_Check>>24);
			u3_Txstr[4]=(BD_EXT.Collision_Check>>16);   	
	              u3_Txstr[5]=(BD_EXT.Collision_Check>>8);	       
                     u3_Txstr[6]=BD_EXT.Collision_Check;	 
			u3_Txstr[7]=0x7E;	               				 

			Destu3_Txstr[0]=0x7E;
			len=Protocol_808_Encode(Destu3_Txstr+1, u3_Txstr+1, 6);
			Destu3_Txstr[len+1]=0x7E;

			//   ����������ײ����
			mma8451_config((uint16_t) (BD_EXT.Collision_Check>>16),(uint16_t) (BD_EXT.Collision_Check));
			rt_kprintf("\r\n ����������ײ����: 0x%08X\r\n",BD_EXT.Collision_Check);; 
	              break; 
		
	 default: 
				return false; 
   }
   rt_thread_delay(20);	   	  
    return true;
}
//--------------------------------------------------
u8  CentreSet_subService_FF03H(u32 SubID, u8 infolen, u8 *Content )
{  
    u8  i=0;
    u8    u3_Txstr[40];
    u8    Destu3_Txstr[40];
    u8    len=0;


   rt_kprintf("\r\n    �յ���չ�ն˲�������1 ���� SubID=%X \r\n",SubID);  
	   
  switch(SubID)
   {
     case 0x0001:  // CAN ID ����
                    if(infolen<4)
						break;			 		
		        // Content[0];  //  ��������
			  //Content[1] ;//����Ϣ�а���CAN ID ������
		     //------------------------------------	  
			//Content[2];  //  CAN ID ��ID 			
		     if(Content[3]&0x80)//  CAN ID   ������  
		     	{
			      BD_EXT.CAN_2_ID=(Content[4]<<24)+(Content[5]<<16)+(Content[6]<<8)+Content[7];// 
                            if(Content[3]&0x40)
					 BD_EXT.CAN_2_Type=1; // ��׼֡
				else 
					 BD_EXT.CAN_2_Type=0;  
			      BD_EXT.CAN_2_Duration=(Content[9]<<8)+Content[10];   // 0   ��ʾֹͣ

				 
				//     ���и���С��ͨ�ŵĴ���
				memset( Destu3_Txstr,0,sizeof(Destu3_Txstr));		 
				u3_Txstr[0]=0x7E;       // ͷ
			       u3_Txstr[1]=0x33;		//  CAN ���
			       u3_Txstr[2]=0x02;    //  �������� 
			       u3_Txstr[3]=Content[3]; // ����
			       u3_Txstr[4]=Content[4]; //  ID 
				u3_Txstr[5]=Content[5];   	
		              u3_Txstr[6]=Content[6];	       
	                     u3_Txstr[7]=Content[7];
				u3_Txstr[8]=Content[8]; // ID 		 
				u3_Txstr[9]=Content[9]; 	//
				u3_Txstr[10]=Content[10]; 	
				u3_Txstr[11]=0x7E;	              				 

				 Destu3_Txstr[0]=0x7E;
			        len=Protocol_808_Encode(Destu3_Txstr+1, u3_Txstr+1, 10);  
			        Destu3_Txstr[len+1]=0x7E;

			       rt_kprintf("\r\nU3_len=%d   :\r\n",len);  
			       for(i=0;i<len+2;i++)
				        rt_kprintf("% 02X",Destu3_Txstr[i]);     	 
				   
			       // U3_PutData(Destu3_Txstr,len+2);  //  ���͸�����	
			       rt_device_write(&Device_CAN2,0, Destu3_Txstr,len+2);       
				rt_kprintf("\r\n ��������CAN2 : 0x%08X   Dur: %d s\r\n",BD_EXT.CAN_2_ID,BD_EXT.CAN_2_Duration);     
			        //----------------------------------
			 } 
			 else
     			   {
      			       BD_EXT.CAN_1_ID=(Content[4]<<24)+(Content[5]<<16)+(Content[6]<<8)+Content[7];// 
                             if(Content[3]&0x40)   
					 BD_EXT.CAN_1_Type=1; // ��׼֡
				else 
					 BD_EXT.CAN_1_Type=0;   
				 BD_EXT.CAN_1_Duration=(Content[9]<<8)+Content[10];  // 0   ��ʾֹͣ  

				rt_kprintf("\r\n ��������CAN1 : 0x%08X   Dur: %d s\r\n",BD_EXT.CAN_1_ID,BD_EXT.CAN_1_Duration);    

			   }   
                    BD_EXT_Write();    
                    break;
	 case 0x0002:  // �ı���Ϣ��־����     
	            // Content[0]; // û��
	              //if((Content[1]==0)||(Content[1]==1))  //  �������Ͷ�����ʾ�� 
	             	{
				memset( TextInfo.TEXT_Content,0,sizeof(TextInfo.TEXT_Content));
				DwinLCD.TxInfolen=AsciiToGb(TextInfo.TEXT_Content,infolen-2,Content+2);
				//memcpy(TextInfo.TEXT_Content,Content+2,infolen-2);
				TextInfo.TEXT_SD_FLAG=1;	// �÷��͸���ʾ����־λ  // ||||||||||||||||||||||||||||||||||
                            rt_kprintf("\r\n  CAN �ı�:  "); 
				for(i=0;i<DwinLCD.TxInfolen;i++) 
							 rt_kprintf("%c",TextInfo.TEXT_Content[i]);
				  rt_kprintf("\r\n ");
			}	

		          #ifdef LCD_5inch
						           //======  ��Ϣ������Ļ����ʾ  
                                                   memset(DwinLCD.TXT_content,0,sizeof(DwinLCD.TXT_content)); 
                                                   DwinLCD.TXT_contentLen=AsciiToGb(DwinLCD.TXT_content,infolen-2,Content+2);  						  
							  DwinLCD.Type=LCD_SDTXT;  						
							   
			   #endif  		

		      break;
	 default: 
				return false;
   }
    return true;
}
//-----------------------------------------------
void  ISP_Get_RawData(u8 * Instr, u16  InLen)
{
      memcpy(ISP_Raw,Instr,InLen);
      ISP_NeedtoProcessFlag=1;	 	
}
void Nullpro_8900H(u8*Instr)
{  
      u16	i=0,DF_PageAddr=0;
	  u16  infolen=0;//,contentlen=0;
	  u8   New_CMD=0;
	//  u8   CenterAckCameraFlag=0;
	//  u8   ireg[5]; 
	  u8   FCS_RX_UDP_sub=0;

  //  2.   get infolen    CMD 
  infolen=( u16 )(Instr[11] << 8 ) + ( u16 ) Instr[12]; 
//  contentlen=infolen-14;  //  8+2+1+1+1+1
  New_CMD = Instr[15];  
 //   4 .  Classify  Process 
   // rt_kprintf("\r\n New_CMD=%X",New_CMD);
     switch(New_CMD)
   {
  		   case  0x91:     //   �����·�Զ�����ؿ�ʼ����
					       ISP_rxCMD=New_CMD; 
						f_ISP_ACK=1;						
							rt_kprintf( "\r\n ISP request! \r\n" ); 					   
						//------------- ISP state ------------------------	
						ISP_running_state=1;
						ISP_running_counter=0;
						ISP_RepeatCounter=0;
					  break;
		   case  0x93:     //   �����·�Զ������������������
		                       rt_kprintf("\r\n infolen=%d",infolen);
                                     //--------------------------------------- 
                                    ISP_current_packnum=((u16)Instr[16]<<8)+(u16)Instr[17];
					  ISP_total_packnum=((u16)Instr[18]<<8)+(u16)Instr[19]; 
					    ISP_ack_resualt=0x01;   // ok		
						ISP_RepeatCounter=0;
					  //--------------------------------------		   
				        ISP_running_state=1;
					 ISP_running_counter=0;   			                
					 f_ISP_88_ACK=1;			
					  break;
		   case  0x95:     //   �����·�Զ�����ؽ�������
					   f_ISP_23_ACK=1;					   
					   rt_kprintf( "\r\n ISP download over! \r\n" ); 
					   //------------- ISP state ------------------------  
					   ISP_running_state=1;
					   ISP_running_counter=0; 
					   ISP_RepeatCounter=0; 
			          break;
		    default:
				      break;  	
   }


}


void DataTrans_ISPService_8900H(u8*Instr)
{  
      u16	i=0,DF_PageAddr=0;
	  u16  infolen=0;//,contentlen=0;
	  u8   New_CMD=0;
	//  u8   CenterAckCameraFlag=0;
	//  u8   ireg[5]; 
	  u8   FCS_RX_UDP_sub=0;
  //  1. Sub Protocol check 
   if(strncmp((char*)Instr, "*GB",3) != 0)           //   ����ͷ 
  	   return;
 
  //  2.   get infolen    CMD 
  infolen=( u16 )(Instr[11] << 8 ) + ( u16 ) Instr[12]; 
//  contentlen=infolen-14;  //  8+2+1+1+1+1
  New_CMD = Instr[15];  

  //  3.   Check  Fcs  
	  FCS_RX_UDP_sub=0;
	  for ( i=0; i <(infolen-1); i++ ) //������յ����ݵ�����  ���Ȱ���У��������Ϣ����Ӧ��1
		  {
			  FCS_RX_UDP_sub^=Instr[3+i];	
		  } 
	  // ------- FCS filter -----------------
	  if( Instr[infolen+2]!= FCS_RX_UDP_sub ) 	   
		  {
			  rt_kprintf("\r\n ISPУ�����	  Caucate %x  ,RX  %x\r\n",FCS_RX_UDP_sub,Instr[infolen+2]); 
				//-----------------  memset  -------------------------------------
			  return; 
		  }
	  else		 
		  rt_kprintf("\r\nISPУ����ȷ	  Caucate %x  ,RX  %x\r\n",FCS_RX_UDP_sub,Instr[infolen+2]);	  
   
 //   4 .  Classify  Process 
   // rt_kprintf("\r\n New_CMD=%X",New_CMD);
     switch(New_CMD)
   {
  		   case  0x91:     //   �����·�Զ�����ؿ�ʼ����
					       ISP_rxCMD=New_CMD; 
						f_ISP_ACK=1;						
							rt_kprintf( "\r\n ISP request! \r\n" ); 					   
						//------------- ISP state ------------------------	
						ISP_running_state=1;
						ISP_running_counter=0;
						ISP_RepeatCounter=0;
					  break;
		   case  0x93:     //   �����·�Զ������������������
		                       rt_kprintf("\r\n infolen=%d",infolen);
		               #if 0
						if(532!=infolen)   
						{
							  ISP_current_packnum=((u16)Instr[16]<<8)+(u16)Instr[17]; 
							  ISP_total_packnum=((u16)Instr[18]<<8)+(u16)Instr[19]; 
							 // rt_kprintf("\r\n ISP_current_packnum=%d,ISP_total_packnum=%d",ISP_current_packnum,ISP_total_packnum);
						
							  f_ISP_88_ACK=1;
							  ISP_ack_resualt=0x02;   // no  pass
							  rt_kprintf( "\r\n Total packet len	error! \r\n" );
						}
						else
						  {
							   ISP_current_packnum=((u16)Instr[16]<<8)+(u16)Instr[17];
							   ISP_total_packnum=((u16)Instr[18]<<8)+(u16)Instr[19]; 
					         //  rt_kprintf("\r\n ISP_current_packnum=%d,ISP_total_packnum=%d",ISP_current_packnum,ISP_total_packnum);
						
								if(ISP_current_packnum<=ISP_total_packnum)
								  {
							   
									//------------------------------
									DF_PageAddr=(u16)(Instr[20]<<8)+Instr[21];  //page
									//rt_kprintf("\r\n д��Page=%d\r\n",DF_PageAddr);
						
									//-----------  protect	RAM data  ------
									if((DF_PageAddr>=DF_BL_PageNo)&&(DF_PageAddr<DF_PageAddr))
										break; 
									
									//---------------- write APP data --------- 
									ISP_Format(DF_PageAddr,0,Instr+22,PageSIZE);
					
									//rt_kprintf("\r\nд����:\r\n ");
									//for(i=0;i<PageSIZE;i++)
										//rt_kprintf("%X ",Instr[22+i]);
								    // rt_kprintf("\r\n������:\r\n ");	
								      rt_thread_delay(5); 
								       rt_kprintf("\r\n���ISP ����\r\n ");	
									ISP_Read(DF_PageAddr, ISP_buffer,PageSIZE);  
									//for(i=0;i<PageSIZE;i++)
									//	rt_kprintf("%X ",ISP_buffer[i]);
									//rt_kprintf("\r\n"); 
									for(i=0;i<PageSIZE;i++)
									{
										 if(Instr[22+i]!=ISP_buffer[i]) 
										   {
											   //------------- ISP state ------------------------
											   ISP_running_state=1;
											   ISP_running_counter=0; 
											   rt_kprintf( "\r\n i=%d  ��ȡ��д��ƥ��! \r\n",i );  
											   break; 
										   }
									}  									
									if(i<PageSIZE)
									 {
									   rt_kprintf( "\r\n DFwrite error!\r\n" );
									   ISP_ack_resualt=0x02;   // no pass
									   ISP_RepeatCounter++;
									   if(ISP_RepeatCounter>6)
									   	{
                                                                          ISP_RepeatCounter=0;
										    rt_kprintf("\r\n ����������5���������Զ����������!\r\n");
                                                                          DF_ClearUpdate_Area();  
									   	}
									 } 
									else  
										
									 {
									   ISP_ack_resualt=0x01;   // ok		
									   ISP_RepeatCounter=0;
									 } 
								   //---------------------------------
								   rt_thread_delay(10);  // ���ӻᵼ�¶�ջ���
								   
								   f_ISP_88_ACK=1;								   
						  }
				             }
						//------------- ISP state ------------------------
					#endif	
				        ISP_running_state=1;
					 ISP_running_counter=0;   			                
					 f_ISP_88_ACK=1;			
					  break;
		   case  0x95:     //   �����·�Զ�����ؽ�������
					   f_ISP_23_ACK=1;					   
					   rt_kprintf( "\r\n ISP download over! \r\n" ); 
					   //------------- ISP state ------------------------  
					   ISP_running_state=1;
					   ISP_running_counter=0; 
					   ISP_RepeatCounter=0; 
			          break;
		    default:
				      break;  	
   }


}
void  ISP_Process(void)
{
      if(1==ISP_NeedtoProcessFlag)
      	{
      	      rt_kprintf( "\r\n ISP _process \r\n" ); 
      	     //DataTrans_ISPService_8900H(ISP_Raw);
      	     f_ISP_ACK=1;	
            ISP_running_state=1;
		ISP_running_counter=0;
		ISP_RepeatCounter=0;
	     //----------------------		 
            ISP_NeedtoProcessFlag=0;
      	}	
}


u8 ISP_running_Status(void)
{ 
     return   ISP_running_state;
}

//-------------------------------------------
void  Media_Start_Init( u8  MdType , u8  MdCodeType)
{  
  MediaObj.Media_Type=MdType;//	ָ����ǰ�����ý�������   0  ��ʾͼƬ
  MediaObj.Media_CodeType=MdCodeType;  //  ��ý������ʽ   0  ��ʾJPEG	��ʽ 
  MediaObj.SD_media_Flag=1;   //  �ö�ý���¼���Ϣ���ͱ�־λ  ����ʼͼƬ����  
  MediaObj.SD_Eventstate=1;		  //  ��ʼ���ڷ���״̬ 
  MediaObj.RSD_State=0;  
  MediaObj.RSD_Timer=0;
  MediaObj.RSD_total=0;
  MediaObj.RSD_Reader=0;
  MediaObj.SD_Data_Flag=0; 
  MediaObj.Media_transmittingFlag=0;      
   //------------  add  for debuging   --------------
  // Media_Clear_State();  //  clear  
 /*  if(0==MediaObj.Media_Type)
	{
	   Photo_sdState.photo_sending=enable; 
	   Photo_sdState.SD_packetNum=1; // ��һ����ʼ  
	   rt_kprintf("\r\n ��ʼ�ϴ���Ƭ! ....\r\n");  												
    }
  */
   //---------------------------------------------------- 
}

void Media_Clear_State(void)
{
     // �����Meia Type
    MediaObj.MaxSd_counter=0;
    MediaObj.SD_Eventstate=0;
    MediaObj.SD_timer=0;
    MediaObj.SD_media_Flag=0;	
	MediaObj.SD_Data_Flag=0; 
	MediaObj.RSD_State=0;
	MediaObj.RSD_Timer=0;
	MediaObj.RSD_total=0;
	MediaObj.RSD_Reader=0;
	MediaObj.Media_transmittingFlag=0;
}
void  Media_Timer(void)
{
  if(1==MediaObj.SD_Eventstate) 
  	{
  	  MediaObj.SD_timer++;
      if(MediaObj.SD_timer>6) 
      	{
            MediaObj.SD_timer=0;			
            MediaObj.SD_media_Flag=1; 
			MediaObj.MaxSd_counter++;
			if(MediaObj.MaxSd_counter>5)
                   Media_Clear_State();  
      	}

  	}
}
void Media_RSdMode_Timer(void)
{
    if((1==MediaObj.RSD_State))
	   	{ 
	        MediaObj.RSD_Timer++;
			if(MediaObj.RSD_Timer>10)
				{
	              MediaObj.RSD_Timer=0;
				  MediaObj.SD_Data_Flag=1; // ���ش����Ͷ�ý����Ϣ��־λ
				  switch(MediaObj.Media_Type)    //   ͼƬ�ش�����
				  	{
				  	 case 0://  ͼ��
				  	         Photo_sdState.SD_packetNum=MediaObj.Media_ReSdList[MediaObj.RSD_Reader];
	                         Photo_sdState.SD_flag=1;
							 break;
					 case 1:// ��Ƶ                             
				  	         Sound_sdState.SD_packetNum=MediaObj.Media_ReSdList[MediaObj.RSD_Reader];
	                         Sound_sdState.SD_flag=1;  
					         break;
					 case 2:// ��Ƶ
							 Video_sdState.SD_packetNum=MediaObj.Media_ReSdList[MediaObj.RSD_Reader];
							 Video_sdState.SD_flag=1;  					          
					         break;
				     default:
					 	     break;

				  	}	
				}
	   	} 
      else
	  if(2==MediaObj.RSD_State)
	  {
          MediaObj.RSD_Timer++;		  
		  if(MediaObj.RSD_Timer>120)   //   ���״̬һֱ�ڵȴ��ҳ���30s�����״̬ 
		  { 
		     switch (MediaObj.Media_Type)
	         {
	          case 0 : // ͼ��
	                  Photo_send_end();  // �����ϴ�����
	                  
			          break;
			  case 1 : // ��Ƶ
			         Sound_send_end();  
			          break;
			  case 2 : // ��Ƶ
                               Video_send_end();
			          break;
			  default:
			  	      break;
	         } 
            Media_Clear_State();
			rt_kprintf("\r\n ��Ϣ�ش���ʱ����! \r\n");
			
			Check_MultiTakeResult_b4Trans();  // ��·����ͷ����״̬���
			
		  }

	  }
   


	
}
#ifdef   MEDIA
//-------- photo send -------------------------------------
void Photo_send_start(u16 Numpic)  
{
  
//  UINT ByteRead;  
 // FIL FileCurrent; 

        rt_kprintf("   \r\n  Photo_send_start =%d \r\n",Numpic); 
        Photo_sdState.photo_sending=disable;  
	 Photo_sdState.SD_flag=0;  
	 Photo_sdState.SD_packetNum=1; // ��1 ��ʼ
        Photo_sdState.Exeption_timer=0;
	 
	if(Camera_Number==1)
	             PicFileSize=Api_DFdirectory_Query(camera_1, 1);
	else
	if(Camera_Number==2)
		     PicFileSize=Api_DFdirectory_Query(camera_2, 1);	   
	else
	if(Camera_Number==3)
		     PicFileSize=Api_DFdirectory_Query(camera_3, 1);   
	else
	if(Camera_Number==4)
		     PicFileSize=Api_DFdirectory_Query(camera_4, 1);    		
		
		
      //	 DF_ReadFlash(pic_current_page, 0,PictureName, 23);
     //      Camera_Number=PictureName[18]; 
     //      memcpy((u8*)&PicFileSize,PictureName+19,4); 
	 //pic_current_page++; //ָ���ļ����� 
	 //pic_PageIn_offset=0;// ͼƬ��ȡҳ��ƫ�Ƶ�ַ 
	// rt_kprintf("\r\n    open Pic =%s",PictureName); 
	  
	  if(PicFileSize%512)
	     Photo_sdState.Total_packetNum=PicFileSize/512+1;
	  else
	  	 Photo_sdState.Total_packetNum=PicFileSize/512;

	 rt_kprintf("\r\n    Camera %d  ReadpicStart total :%d ��Pagesize: %d Bytes\r\n\r\n",Camera_Number,Photo_sdState.Total_packetNum,PicFileSize);    
	 if((Camera_Number==0)||(Photo_sdState.Total_packetNum==0))
	 {
	   Photo_send_end(); // clear  state 
       rt_kprintf("\r\n  ͼƬ�ܰ���Ϊ�� ������ͷ���Ϊ0 ����������ʧ�ܵ����� \r\n");    
	 }


      // -------  MultiMedia Related --------
      Media_Start_Init(0,0);

}

u8  Sound_send_start(void)
{
  u8 sound_name[20]; 
  u16  i;
//  u8  oldstate=0;
//  u16 i2,j; 
 // u8  WrieEnd=0,LeftLen=0; 
  

  Sound_sdState.photo_sending=disable;  
  Sound_sdState.SD_flag=0;  
  Sound_sdState.SD_packetNum=1; // ��1 ��ʼ
  Sound_sdState.Exeption_timer=0;
   
 //---  Speex_Init();	 // speachX ��ʼ��	 
  // 1. �������µ��ļ�
/*  memset((u8*)&MediaIndex,0,sizeof(MediaIndex)); 
  for(i=0;i<8;i++)
  {
	 Api_RecordNum_Read(voice_index, i+1, (u8*)&MediaIndex, sizeof(MediaIndex));
        if(MediaIndex.Effective_Flag==1)
        {  
            break;
	  }       
  }
  if(MediaIndex.Effective_Flag)  	
  { 
	rt_kprintf("\r\n ����filename:%s\r\n",MediaIndex.FileName);
  }	
  else
  	{
  	   rt_kprintf("\r\n û���Ѵ洢����Ƶ�ļ� \r\n");
  	   return false	;
  	}

*/
    
	//	2.	����wav �ļ�   

	//	3. �ļ���С
	  // file name
	memset(sound_name,0,sizeof(sound_name));
	DF_ReadFlash(SoundStart_offdet, 4, sound_name,20);
       SrcFileSize=Api_DFdirectory_Query(voice, 1);
     //  Sound_sdState.Total_packetNum=(SrcFileSize/512); // ÿ��100���ֽ�
        if(SrcFileSize%512)
	     Sound_sdState.Total_packetNum=SrcFileSize/512+1;
	  else
	  	 Sound_sdState.Total_packetNum=SrcFileSize/512; 
	rt_kprintf("\r\n	�ļ���: %s��С: %d Bytes  totalpacketnum=%d \r\n",sound_name,SrcFileSize,Sound_sdState.Total_packetNum);
  
    // -------  MultiMedia Related --------
      Media_Start_Init(1,3); // ��Ƶ  wav ��ʽ   0:JPEG ;   1: TIF ;   2:MP3;  3:WAV  4: WMV  ��������   
                                           //    5   amr
     return true;  
}

u8  MP3_send_start(void) 
{
   u8   mp3_name[13];
  mp3_sendstate=1;
  Sound_sdState.photo_sending=disable;  
  Sound_sdState.SD_flag=0;  
  Sound_sdState.SD_packetNum=1; // ��1 ��ʼ
    Sound_sdState.Exeption_timer=0;

    memset(mp3_name,0,sizeof(mp3_name));
	memcpy(mp3_name,"ch12.mp3",8);
	
	if(mp3_fsize%512)
	   Sound_sdState.Total_packetNum=mp3_fsize/512+1;
	else
	   Sound_sdState.Total_packetNum=mp3_fsize/512;

	rt_kprintf("\r\n  mp3�ļ�����:%s    �ļ���С  %d Bytes  \r\n",mp3_name,mp3_fsize);
  
    // -------  MultiMedia Related --------
    Media_Start_Init(1,2); // ��Ƶ  wav ��ʽ       
	return true;
}

u8  Video_send_start(void)
{
   u8 video_name[13];
   	
  wmv_sendstate=1;
  Video_sdState.photo_sending=disable;  
  Video_sdState.SD_flag=0;  
  Video_sdState.SD_packetNum=1; // ��1 ��ʼ
  Video_sdState.Exeption_timer=0;

    memset(video_name,0,sizeof(video_name));
	memcpy(video_name,"ch1.wmv",7);
	
    if(wmv_fsize%512)
	   Video_sdState.Total_packetNum=wmv_fsize/512+1;
	else
	   Video_sdState.Total_packetNum=wmv_fsize/512;  

	rt_kprintf("\r\n  wmv�ļ�����:%s    �ļ���С  %d Bytes  \r\n",video_name,wmv_fsize);
  
    // -------  MultiMedia Related --------
    Media_Start_Init(2,4); // ��Ƶ  wav ��ʽ       
	return true; 
}



void Photo_send_end(void)
{
        Photo_sdState.photo_sending=0;
	 Photo_sdState.SD_flag=0;
	 Photo_sdState.SD_packetNum=0;
	 Photo_sdState.Total_packetNum=0;
	 Photo_sdState.Exeption_timer=0;
	 MediaObj.Media_transmittingFlag=0;  // clear  
	 Media_Clear_State(); 
} 
void Sound_send_end(void) 
{
       Sound_sdState.photo_sending=0;
	 Sound_sdState.SD_flag=0;
	 Sound_sdState.SD_packetNum=0;
	 Sound_sdState.Total_packetNum=0;
	  Sound_sdState.Exeption_timer=0;
	 MediaObj.Media_transmittingFlag=0;  // clear  
	 mp3_sendstate=0;
	 VocREC.running=0;   // clear
	 Media_Clear_State(); 
} 

void Video_send_end(void)  
{
       Video_sdState.photo_sending=0;
	 Video_sdState.SD_flag=0;
	 Video_sdState.SD_packetNum=0;
	 Video_sdState.Total_packetNum=0;
	 Video_sdState.Exeption_timer=0;
	 MediaObj.Media_transmittingFlag=0;  // clear  
	 wmv_sendstate=0;
	 Media_Clear_State(); 
} 

void Video_Timer(void)
{
  
  if((Video_sdState.photo_sending==enable)&&(2==MediaObj.Media_Type)) // ��Ƶ
  {
	 if((Video_sdState.SD_packetNum<=Video_sdState.Total_packetNum+1)&&(2!=MediaObj.RSD_State))  
	  {  //  һ�¶�ʱ����	��˳���͹�������	 ��   �յ��ش���ʼ����Ч
		  Video_sdState.Data_SD_counter++;
		  if( Video_sdState.Data_SD_counter>40)   
			  {
				 Video_sdState.Data_SD_counter=0;
				 Video_sdState.SD_flag=1;
				 MediaObj.SD_Data_Flag=1; 
				 Video_sdState.Exeption_timer=0;
				 //rt_kprintf("\r\n Video send Flag \r\n"); 
				 
			  }
	  }   
  }
}

void Sound_Timer(void)
{
  
  if((Sound_sdState.photo_sending==enable)&&(1==MediaObj.Media_Type)) // ��Ƶ
  {
	 if((Sound_sdState.SD_packetNum<=Sound_sdState.Total_packetNum+1)&&(2!=MediaObj.RSD_State))  
	  {  //  һ�¶�ʱ����	��˳���͹�������	 ��   �յ��ش���ʼ����Ч
		  Sound_sdState.Data_SD_counter++;
		  if( Sound_sdState.Data_SD_counter>14)         
			  {
				 Sound_sdState.Data_SD_counter=0;
				  Sound_sdState.Exeption_timer=0;
				 Sound_sdState.SD_flag=1;
				 MediaObj.SD_Data_Flag=1; 
				 
				 //rt_kprintf("\r\n Sound  Transmit set Flag \r\n"); 				
			  }
	  }   
  }
}
void Photo_Timer(void)  
{
	if((Photo_sdState.photo_sending==enable)&&(0==MediaObj.Media_Type))
	{
	   if((Photo_sdState.SD_packetNum<=Photo_sdState.Total_packetNum+1)&&(2!=MediaObj.RSD_State))  
	   	{  //  һ�¶�ʱ����   ��˳���͹�������   ��   �յ��ش���ʼ����Ч 
	        Photo_sdState.Data_SD_counter++;
			if( Photo_sdState.Data_SD_counter>12)  //40   12     
				{
	                        Photo_sdState.Data_SD_counter=0;
				   Photo_sdState.Exeption_timer=0;			
	                         Photo_sdState.SD_flag=1;
				   MediaObj.SD_Data_Flag=1; 
				  
				}
	   	} 	
	}
}

void Meida_Trans_Exception(void) 
{
    u8  resualt=0;
	
        if(Photo_sdState.photo_sending==enable)
	 {
	     if( Photo_sdState.Exeption_timer++>50)
	     	{
                  Photo_send_end();
		    resualt=1;		  
	     	}
        }		
	 else 
	 if(Sound_sdState.photo_sending==enable)	 
	 { 
	    if( Sound_sdState.Exeption_timer++>50)
	     	{
                  Sound_send_end();
		    resualt=2;		  
	     	}
	}
	 else
        if(Video_sdState.photo_sending==enable)	 
        {
              if( Video_sdState.Exeption_timer++>50)
	     	{
                  Video_send_end();
		    resualt=2;		  
	     	}
        }

	if(resualt)
		rt_kprintf("\r\n   Media  Trans  Timeout  resualt: %d\r\n", resualt);
		
}

 void Media_Timer_Service(void)
{
       //----------------------------------
   if(DataLink_Status())
   {
       if(Photo_sdState.photo_sending==enable)
	  Photo_Timer();  
	 else 
	 if(Sound_sdState.photo_sending==enable)	 
	  Sound_Timer(); 
	 else
	  Video_Timer(); 	
	 
	  Media_RSdMode_Timer();
   }   
}

#endif
//------------------------------------------------------------
void DataTrans_Init(void)
{
   DataTrans.Data_RxLen=0; 
   DataTrans.Data_TxLen=0;
   DataTrans.Tx_Wr=0; 
   memset(DataTrans.DataRx,0,sizeof((const char*)DataTrans.DataRx));   
   memset(DataTrans.Data_Tx,0,sizeof((const char*)DataTrans.Data_Tx)); 
}
//------------------------------------------------------------
void DoorCameraInit(void)
{
   DoorOpen.currentState=0;
   DoorOpen.BakState=0;
}
//-----------------------------------------------------------
void Spd_ExpInit(void)
{
   speed_Exd.current_maxSpd=0;
   speed_Exd.dur_seconds=0;
   speed_Exd.excd_status=0;   
   memset((char*)(speed_Exd.ex_startTime),0,5);
   speed_Exd.speed_flag=0;
}

//-----------------------------------------------------------
void TCP_RX_Process( u8  LinkNum)  //  ---- 808  ��׼Э�� 
{
  	  u16	i=0;//,DF_PageAddr;
	  u16  infolen=0,contentlen=0;
	 // u8   ireg[5]; 
	  u8   Ack_Resualt=1;
	  u16  Ack_CMDid_8001=0;
	  u8   Total_ParaNum=0;        // �������ò������� 
	  u8   Process_Resualt=0;  //  bit ��ʾ   bit0 ��ʾ 1  bit 1 ��ʾ2
	  u8   ContentRdAdd=0;   // ��ǰ��ȡ���ĵ�ַ
	  u8   SubInfolen=0;     // ����Ϣ����
	  u8   Reg_buf[22];
	  //u8   CheckResualt=0;
      u32  reg_u32=0;
  //----------------      �г���¼��808 Э�� ���մ���   -------------------------- 

  //  0.  Decode     
  Protocol_808_Decode();
  //  1.  fliter head
  if(UDP_HEX_Rx[0]!=0x7e)           //   ����ͷ 
  	   return; 
  //  2.  check Centre Ack
  Centre_CmdID=(UDP_HEX_Rx[1]<<8)+UDP_HEX_Rx[2];  // ���յ�������ϢID   
  Centre_FloatID=(UDP_HEX_Rx[11]<<8)+UDP_HEX_Rx[12];  // ���յ�������Ϣ��ˮ��        

  //  �ְ��ж�
  if(UDP_HEX_Rx[3]&0x20) 
  	{   //  �ְ��ж�
        ;  
  	} 

  //  3.   get infolen    ( ����Ϊ��Ϣ��ĳ���)    ���ְ��Ļ�  ��Ϣͷ����Ϊ12 ��������У��ĳ��� =infolen+12
  //infolen =( u16 )((UDP_HEX_Rx[3]&0x3f) << 8 ) + ( u16 ) UDP_HEX_Rx[4];  
  infolen =( u16 )((UDP_HEX_Rx[3]&0x03) << 8 ) + ( u16 ) UDP_HEX_Rx[4];  
  contentlen=infolen+12;    //  ����У���ֽڵĳ���   

  //  4.   Check  Fcs 
	  FCS_RX_UDP=0; 
  	  //nop;nop;
	  for ( i=0; i<(UDP_DecodeHex_Len-3); i++ ) //������յ����ݵ�����  
		  {
			  FCS_RX_UDP^=UDP_HEX_Rx[1+i];	  
		  } 
	  //nop;
	  // ------- FCS filter -----------------
	/*  if( UDP_HEX_Rx[UDP_DecodeHex_Len-2]!=FCS_RX_UDP ) //�ж�У����	    
		  {
			  rt_kprintf("\r\n  infolen=%d ",infolen);  
			  rt_kprintf("\r\n808Э��У�����	  Caucate %x  ,RX  %x\r\n",FCS_RX_UDP,UDP_HEX_Rx[UDP_DecodeHex_Len-2]); 
				//-----------------  memset  -------------------------------------
			  memset(UDP_HEX_Rx, 0, sizeof(UDP_HEX_Rx));    
			  UDP_hexRx_len= 0; 
			  return; 
		  }	
	  */
	//  else		  
		 // rt_kprintf("\r\n 808Э��У����ȷ	  Caucate %x  ,RX  %x\r\n",FCS_RX_UDP,UDP_HEX_Rx[UDP_DecodeHex_Len-2]);	   
   
 //   5 .  Classify  Process 
         rt_kprintf("\r\n           CentreCMD = 0x%X  \r\n",Centre_CmdID);  // add for  debug  
         
     switch(Centre_CmdID)
     	{
     	   case  0x8001:  //ƽ̨ͨ��Ӧ��
     	             // ��û�зְ�����Ļ�  ��Ϣͷ��12  ��0��ʼ�����12���ֽ�����Ϣ�������

					  //  13 14  ��Ӧ���ն���Ϣ��ˮ��  
					  //  15 16  ��Ӧ�ն˵���Ϣ
                       Ack_CMDid_8001=(UDP_HEX_Rx[15]<<8)+UDP_HEX_Rx[16];

					   switch(Ack_CMDid_8001)   // �ж϶�Ӧ�ն���Ϣ��ID�����ִ���
					   	{ 
					   	   case 0x0200:    //  ��Ӧλ����Ϣ��Ӧ��
                                      //----- �ж�ȷ���Ƿ�ɹ�
						              if(0x00!=UDP_HEX_Rx[17])
									  	 break;
									  //--------------------------------
									  if (Warn_Status[3]&0x01) 
									  {
											  StatusReg_WARN_Clear();			  
											  f_Exigent_warning = 0;
											  warn_flag=0;
											  Send_warn_times=0; 
											   StatusReg_WARN_Clear();	
											   rt_kprintf( "\r\n���������յ�Ӧ�𣬵����!\r\n");   
									  }
									  //------------------------------------
                                     if(Warn_Status[1]&0x10)// �������򱨾�
                                     {
						                  InOut_Object.TYPE=0;//Բ������
						                  InOut_Object.ID=0; //  ID
						                  InOut_Object.InOutState=0;//  ������ 
						                  Warn_Status[1]&=~0x10;
                                     } 
									   if(Warn_Status[1]&0x20)// ����·�߱���
                                     {
						                  InOut_Object.TYPE=0;//Բ������
						                  InOut_Object.ID=0; //  ID
						                  InOut_Object.InOutState=0;//  ������ 
						                  Warn_Status[1]&=~0x20;  
                                     } 
					 if(Warn_Status[0]&0x20)
					 {
                                                  Warn_Status[0]&=~0x20; 
					 }
									  //------------------------------------  
										 rt_kprintf( "\r\nCentre ACK!\r\n");  	
									    Api_cycle_Update();   
									   //-------------------------------------------------------------------
									  /* cycle_read++;   //  �յ�Ӧ��ŵ���
									   if(cycle_read>=Max_CycleNum)
											   cycle_read=0;
									   ReadCycle_status=RdCycle_Idle;
                                                                 */
                                        //--------------  ��ý���ϴ����  --------------                                       
									   if(MediaObj.Media_transmittingFlag==1)  // clear									      
									      {
									         MediaObj.Media_transmittingFlag=2;   
											 if(Duomeiti_sdFlag==1)
											  { 
											      Duomeiti_sdFlag=0; 
											      Media_Clear_State();
												Photo_send_end();
												Sound_send_end();
												Video_send_end();
                                                                                     rt_kprintf("\r\n  �ֶ��ϱ���ý���ϴ�����\r\n");
											  }	
											 rt_kprintf("\r\n  ��ý����Ϣǰ�Ķ�ý�巢����� \r\n");  
									   	  }	
									   
								       break;
				  case 0x0002:  //  ��������Ӧ��
                                       //  �����н����  ---     
                                        JT808Conf_struct.DURATION.TCP_ACK_DurCnter=0;//clear
                                        JT808Conf_struct.DURATION.TCP_SD_state=0; //clear
                                         rt_kprintf( "\r\n  Centre  Heart ACK!\r\n");  
							           break;
                            case 0x0101:  //  �ն�ע��Ӧ��
                                        if(0==UDP_HEX_Rx[17])
                                        {  // ע���ɹ�
                                          
										 memset(Reg_buf,0,sizeof(Reg_buf));
 										 memcpy(Reg_buf,JT808Conf_struct.ConfirmCode,20);
 										 JT808Conf_struct.Regsiter_Status=0; 
 										 Reg_buf[20]=JT808Conf_struct.Regsiter_Status; 
 										Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
										 rt_kprintf("\r\n  �ն�ע���ɹ�!  \r\n"); 
                                        }  

							           break;
						    case 0x0102:  //  �ն˼�Ȩ
						                
 						                rt_kprintf("\r\n �յ���Ȩ���: %x \r\n",UDP_HEX_Rx[17]); 
						                if(0==UDP_HEX_Rx[17])
                                         {  // ��Ȩ�ɹ� 
                                                          DEV_Login.Operate_enable=2; // ��Ȩ���
					                        if(DataLink_Status())   
							                   DataLinkOK_Process();
											 rt_kprintf("\r\n  �ն˼�Ȩ�ɹ�!  \r\n");   
						                }
									   break;
                            case 0x0800:  // ��ý���¼���Ϣ�ϴ�
                                             rt_kprintf("\r\n ��ý���¼���Ϣ�ϴ���Ӧ! \r\n");											 
											 Media_Clear_State();  //  clear 
											 
                                            if(0==MediaObj.Media_Type)
											{
											   MediaObj.Media_transmittingFlag=1; 
											   PositionSD_Enable(); 
											   Current_UDP_sd=1;
											   
											   Photo_sdState.photo_sending=enable; 
											   Photo_sdState.SD_packetNum=1; // ��һ����ʼ 
											   PositionSD_Enable();  //   ʹ���ϱ�
											   rt_kprintf("\r\n ��ʼ�ϴ���Ƭ! ....\r\n");   												
                            	            }
											else
											if(1==MediaObj.Media_Type)
											{
											   MediaObj.Media_transmittingFlag=1;  
											   
											   Sound_sdState.photo_sending=enable; 
											   Sound_sdState.SD_packetNum=1; // ��һ����ʼ 
											   PositionSD_Enable();  //   ʹ���ϱ� 
											   Current_UDP_sd=1;
											   rt_kprintf("\r\n ��ʼ�ϴ���Ƶ! ....\r\n");    												
                            	            }	
											else
											if(2==MediaObj.Media_Type)
											{
											   MediaObj.Media_transmittingFlag=1;   
											   PositionSD_Enable();  //   ʹ���ϱ�
											   Current_UDP_sd=1;
											   Video_sdState.photo_sending=enable; 
											   Video_sdState.SD_packetNum=1; // ��һ����ʼ   
											   rt_kprintf("\r\n ��ʼ�ϴ���Ƶ! ....\r\n");    												
                            	            }	
											break;
						    case 0x0702: 
                                          rt_kprintf("\r\n  ��ʻԱ��Ϣ�ϱ�---����Ӧ��!  \r\n"); 							       
											 
							           break;
							case 0x0701: 
                                           rt_kprintf("\r\n  �����˵��ϱ�---����Ӧ��!  \r\n"); 							       
											 
							           break;		   
									   
							default	 :
								       break;
					   	}


					    //---------------------
						 ACKFromCenterCounter=0; 
						 fCentre_ACK=0;
						 ACK_timer=0;
				        break;
           case  0x8100:    //  ������Ķ��ն�ע����Ϣ��Ӧ��		
                       //-----------------------------------------------------------
					   switch(UDP_HEX_Rx[15])
                        	{
                               case 0: rt_kprintf("\r\n   ----ע��ɹ�\r\n");
							           memset(JT808Conf_struct.ConfirmCode,0,sizeof(JT808Conf_struct.ConfirmCode));
                                       memcpy(JT808Conf_struct.ConfirmCode,UDP_HEX_Rx+16,infolen-3);

									   memset(Reg_buf,0,sizeof(Reg_buf));
							           memcpy(Reg_buf,JT808Conf_struct.ConfirmCode,20);
									   JT808Conf_struct.Regsiter_Status=1; 
							           Reg_buf[20]=JT808Conf_struct.Regsiter_Status;
                                Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
									   rt_kprintf("��Ȩ��: %s\r\n		   ��Ȩ�볤��: %d\r\n",JT808Conf_struct.ConfirmCode,strlen((const char*)JT808Conf_struct.ConfirmCode)); 
                                       //-------- ��ʼ��Ȩ ------
									   DEV_Login.Operate_enable=1;  
									   break;
							   case 1: rt_kprintf("\r\n   ----�����ѱ�ע��\r\n");
									   break;	
							   case 2: rt_kprintf("\r\n   ----���ݿ����޸ó���\r\n");
									   break;
							   case 3: rt_kprintf("\r\n   ----�ն��ѱ�ע��\r\n");  
							           if(0==JT808Conf_struct.Regsiter_Status)  
                                        {
                                          ;//JT808Conf_struct.Regsiter_Status=2;  // not  1
                                          //DEV_regist.DeRegst_sd=1;
							           	}  
									   else
									   if(1==JT808Conf_struct.Regsiter_Status) 
							               DEV_Login.Operate_enable=1; //��ʼ��Ȩ 
							          
									   break;		    
                               case 4: rt_kprintf("\r\n   ----���ݿ����޸��ն�\r\n");  
									   break;
                        	}					    
		              break;
		   case  0x8103:    //  �����ն˲���   
		               //  Ack_Resualt=0;
                      if(contentlen)
                       {                       // �������������ÿ��ֻ�·�����һ������
                         Total_ParaNum=UDP_HEX_Rx[13]; // �������ò�������
                         rt_kprintf("\r\n Set ParametersNum =%d  \r\n",Total_ParaNum);
						 //------------------------------------------------------------------- 
						 ContentRdAdd=14;
						 Process_Resualt=0;  // clear resualt
						 for(i=0;i<Total_ParaNum;i++)
						 {
						   //  ��������DWORD 4 ���ֽ�
						   SubCMD_8103H=(UDP_HEX_Rx[ContentRdAdd]<<24)+(UDP_HEX_Rx[ContentRdAdd+1]<<16)+(UDP_HEX_Rx[ContentRdAdd+2]<<8)+UDP_HEX_Rx[ContentRdAdd+3]; 
                           //  ����Ϣ����
						   SubInfolen=UDP_HEX_Rx[ContentRdAdd+4];
						   //  ��������Ϣ ������óɹ�����ӦBit λ��Ϊ 1 ���򱣳� 0
						   if(CentreSet_subService_8103H(SubCMD_8103H,SubInfolen,UDP_HEX_Rx+ContentRdAdd+5))
                                Process_Resualt|=(0x01<<i);						   
						   //  �ƶ�ƫ�Ƶ�ַ
						   ContentRdAdd+=5+UDP_HEX_Rx[ContentRdAdd+4]; // ƫ���±�
						 } 						   

                         //--------------�ж����е����ý��  ---------------
                        /* for(i=0;i<Total_ParaNum;i++)
                         {
                             if(!((Process_Resualt>>0)&0x01))
						     	{
                                  Ack_Resualt=1; //  1  ��ʾʧ��
                                  break; 
						     	}
							 if(i==(Total_ParaNum-1))  //  ���õ����һ��ȷ�ϳɹ�
							 	Ack_Resualt=0;  //  �ɹ�/ȷ��
                         }*/
						 Ack_Resualt=0;
                      }	
					  
						 //-------------------------------------------------------------------
						 if(SD_ACKflag.f_CentreCMDack_0001H==2)
						 {                  
						    Ack_Resualt=0;
							SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
						 }
						 else
                         if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {
						     SD_ACKflag.f_CentreCMDack_0001H=1;
							 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
                         } 	 
						  rt_kprintf("\r\n  Set Device !\r\n");   
						  
		              break;  
		   case  0x8104:    //  ��ѯ�ն˲���
                      SD_ACKflag.f_SettingPram_0104H=1;  // ����ʲô���ݻظ�ͳһ���
                      rt_kprintf("\r\n  ���Ĳ�ѯ�ն˲��� !\r\n");   
			          break;
           case  0x8105:     // �ն˿���
                    // Ack_Resualt=0; 
                     if(contentlen)
                      {                       // �������������ÿ��ֻ�·�����һ������
                         Total_ParaNum=UDP_HEX_Rx[13]; //  �ն˿��������� 
                         rt_kprintf("\r\n Set ParametersNum =%d  \r\n",Total_ParaNum); 
						 //------------------------------------------------------------------- 
						 if(CentreSet_subService_8105H(Total_ParaNum,contentlen-1,UDP_HEX_Rx+14)) 
                                Ack_Resualt=0;   // ���سɹ�						   
                       }	
					 
						 //-------------------------------------------------------------------
						 Ack_Resualt=0;
					   if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {
						     SD_ACKflag.f_CentreCMDack_0001H=1;
							 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
                         }	 
						  rt_kprintf("\r\ny  �ն˿��� !\r\n"); 
						  
		              break;
		   case  0x8201:     // λ����Ϣ��ѯ    λ����Ϣ��ѯ��Ϣ��Ϊ��
		               SD_ACKflag.f_CurrentPosition_0201H=1;		
					   rt_kprintf("\r\n  λ����Ϣ��ѯ !\r\n");  
		              break;
		   case  0x8202:     // ��ʱλ�ø��ٿ���
		                 Ack_Resualt=0;

						 //  13 14  ʱ���� 
					     JT808Conf_struct.RT_LOCK.Lock_Dur=(UDP_HEX_Rx[13]<<8)+UDP_HEX_Rx[14];
					    //  15 16  17 18 ��Ӧ�ն˵���Ϣ
                         JT808Conf_struct.RT_LOCK.Lock_KeepDur=(UDP_HEX_Rx[15]<<24)+(UDP_HEX_Rx[16]<<16)+(UDP_HEX_Rx[17]<<8)+UDP_HEX_Rx[18];

                         JT808Conf_struct.RT_LOCK.Lock_state=1;    // Enable Flag
						 JT808Conf_struct.RT_LOCK.Lock_KeepCnter=0;  //  ���ּ�����
						 Current_SD_Duration=JT808Conf_struct.RT_LOCK.Lock_KeepDur;  //���ķ��ͼ��						 

						 JT808Conf_struct.SD_MODE.DUR_TOTALMODE=1;   // ���¶�ʱ���״̬λ
						 JT808Conf_struct.SD_MODE.Dur_DefaultMode=1;
						 //  ��������
						 Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
						 
		                // if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {
						     Ack_Resualt=0;
						     SD_ACKflag.f_CentreCMDack_0001H=1;
							 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;      
                         }
                      rt_kprintf("\r\n  ��ʱλ�ø��ٿ���!\r\n"); 
		              break;
		   case  0x8300:    //  �ı���Ϣ�·�
		                  Ack_Resualt=0;
                          TextInfo.TEXT_FLAG=UDP_HEX_Rx[13];
						  if(TextInfo.TEXT_FLAG&0x09)  // ����Ƿ��TTS�ն�  ������Ҳ��TTS����
						  {
						      
							  Dev_Voice.CMD_Type='2';
							  memset(Dev_Voice.Play_info,0,sizeof(Dev_Voice.Play_info));
                                                   memcpy(Dev_Voice.Play_info,UDP_HEX_Rx+14,infolen-1);
							  Dev_Voice.info_sdFlag=1;

							//#ifdef LCD_5inch
							   //DwinLCD.Type=LCD_SDTXT;
							 // memset(DwinLCD.TXT_content,0,sizeof(DwinLCD.TXT_content)); 
                                                   //DwinLCD.TXT_contentLen=AsciiToGb(DwinLCD.TXT_content,infolen-1,UDP_HEX_Rx+14);  						  
							//#endif   
                                                   //  TTS  
							   TTS_Get_Data(UDP_HEX_Rx+14,infolen-1);

						  }
						 if((TextInfo.TEXT_FLAG&0x04)||(TextInfo.TEXT_FLAG&0x01))  // ����Ƿ���ն���ʾ��
						 {                            
							memset( TextInfo.TEXT_Content,0,sizeof(TextInfo.TEXT_Content));
							memcpy(TextInfo.TEXT_Content,UDP_HEX_Rx+14,infolen-1);
							TextInfo.TEXT_SD_FLAG=1;	// �÷��͸���ʾ����־λ  // ||||||||||||||||||||||||||||||||||

							//========================================
							TextInforCounter++;
							rt_kprintf("\r\nд���յ��ĵ� %d ����Ϣ,��Ϣ����=%d,��Ϣ:%s",TextInforCounter,infolen-1,TextInfo.TEXT_Content);
							TEXTMSG_Write(TextInforCounter,1,infolen-1,TextInfo.TEXT_Content);				
							//========================================
						 } 

						  #ifdef LCD_5inch
						           //======  ��Ϣ������Ļ����ʾ  
                                                	  DwinLCD.Type=LCD_SDTXT;
							  memset(DwinLCD.TXT_content,0,sizeof(DwinLCD.TXT_content)); 
                                                   DwinLCD.TXT_contentLen=AsciiToGb(DwinLCD.TXT_content,infolen-1,UDP_HEX_Rx+14);  						  
   						#endif    

                          //------- ���� ----
						 //  if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {   
						      Ack_Resualt=0;
						     SD_ACKflag.f_CentreCMDack_0001H=1;
							 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
                         }
						   rt_kprintf("\r\n �ı���Ϣ: %s\r\n",TextInfo.TEXT_Content);
		              break; 
		   case  0x8301:    //  �¼����� 
					   if(contentlen)
                      {                       
                         //--- ��������--
                         switch(UDP_HEX_Rx[13])
                         {
                            case 0 :  //  ɾ���ն����������¼���������󲻴�����ַ�
                                      Event_Init(1); 	
                                      rt_kprintf("\r\n ɾ�������¼�\r\n");
                                      break; 
							case 1:  // �����¼�
									 if(UDP_HEX_Rx[13]==1)
									 	rt_kprintf("\r\n �����¼�\r\n");
							         //break;
							case 2:  // ׷���¼�
                                     if(UDP_HEX_Rx[13]==2)
									 	rt_kprintf("\r\n ׷���¼�\r\n");
							          //break;
							case 3:  // �޸��¼�
                                     if(UDP_HEX_Rx[13]==3)
									 	 rt_kprintf("\r\n �޸��¼�\r\n");  
							          //break;
							case 4:  // ɾ���ض��¼�       
							         if(UDP_HEX_Rx[13]==4)
									 	 rt_kprintf("\r\n ɾ���ص��¼�\r\n"); 
									  Total_ParaNum=UDP_HEX_Rx[14]; // �������ò�������
									  rt_kprintf("\r\n Set ParametersNum =%d  \r\n",Total_ParaNum);
									  if(Total_ParaNum!=1)
										 break;
									  //-------------------------------
									  if((UDP_HEX_Rx[15]>8)&&(UDP_HEX_Rx[15]==0))
									     EventObj.Event_ID=0;
									  else
									  	 EventObj.Event_ID=UDP_HEX_Rx[15];
									  
									  EventObj.Event_Len=UDP_HEX_Rx[16];
									  memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str));
									  memcpy(EventObj.Event_Str,UDP_HEX_Rx+17,EventObj.Event_Len);
									  EventObj.Event_Effective=1; 
                                                                 Api_RecordNum_Write(event_808, EventObj.Event_ID, (u8*)&EventObj,sizeof(EventObj));									  rt_kprintf("\r\n �¼�����:%s\r\n",EventObj.Event_Str);   
									  rt_kprintf("\r\n �¼�����:%s\r\n",EventObj.Event_Str);   
							          break;
							default:
								      break;

                         }

                          //---------���� -------
                         // if(SD_ACKflag.f_CentreCMDack_0001H==0) // һ��ظ�
 					      {
		  						 SD_ACKflag.f_CentreCMDack_0001H=1;
		  						 Ack_Resualt=0;
		  						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					       }
						  /*
						  if(SD_ACKflag.f_CurrentEventACK_0301H==0)
						  {
							  SD_ACKflag.f_CurrentEventACK_0301H=1; 
						  }
                              */
					   }
                          
		              break;			  
		   case  0x8302:    // �����·�
						 // if(UDP_HEX_Rx[13]&0x08)  // ����־�Ƿ����ʾ�ն� 
						   rt_kprintf("\r\n  �����·����� \r\n"); 
						  {
						     ASK_Centre.ASK_infolen=UDP_HEX_Rx[14];
							 memset(ASK_Centre.ASK_info,0,sizeof(ASK_Centre.ASK_info));
							 memcpy(ASK_Centre.ASK_info,UDP_HEX_Rx+15,ASK_Centre.ASK_infolen);
							 rt_kprintf("\r\n  ����: %s \r\n",ASK_Centre.ASK_info); 
							 memset(ASK_Centre.ASK_answer,0,sizeof(ASK_Centre.ASK_answer));
							 memcpy(ASK_Centre.ASK_answer,UDP_HEX_Rx+15+ASK_Centre.ASK_infolen,infolen-2-ASK_Centre.ASK_infolen);	 

							 ASK_Centre.ASK_SdFlag=1;   // ||||||||||||||||||||||||||||||||||
							 ASK_Centre.ASK_floatID=Centre_FloatID; // ���� FloatID	 
                                                  ASK_Centre.ASK_disp_Enable=1;
							 rt_kprintf("\r\n ����Answer:%s\r\n",ASK_Centre.ASK_answer+3);
       
                                                  Api_RecordNum_Write(ask_quesstion,1, (u8*)&ASK_Centre,sizeof(ASK_Centre)); 	
                                                  
						  }
						   
						// if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {
						     Ack_Resualt=0; 
						     SD_ACKflag.f_CentreCMDack_0001H=1;
					          SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;  
                                           } 

					
		              break;
           case  0x8303:    //  ��Ϣ�㲥�˵�����
                              //--- ��������--
                         switch(UDP_HEX_Rx[13])
                         {
                            case 0 :  //  ɾ���ն�����������Ϣ
                                      MSG_BroadCast_Init(1); 
                                      rt_kprintf("\r\n ɾ����Ϣ\r\n");
                                      break;
							case 1:  // ���²˵�
									  if(UDP_HEX_Rx[13]==1)
									 	  rt_kprintf("\r\n ���²˵�\r\n");
							         //break;
							case 2:  // ׷�Ӳ˵�
                                       if(UDP_HEX_Rx[13]==2)
									 	  rt_kprintf("\r\n ׷�Ӳ˵�\r\n");
							          //break;
							case 3:  // �޸Ĳ˵�
							          if(UDP_HEX_Rx[13]==3)
									 	  rt_kprintf("\r\n �޸Ĳ˵�\r\n");
									  Total_ParaNum=UDP_HEX_Rx[14];           // ��Ϣ������
									  rt_kprintf("\r\n Set ParametersNum =%d  \r\n",Total_ParaNum);
									  if(Total_ParaNum!=1)
										 break;
									  //-------------------------------
									  if((UDP_HEX_Rx[15]>8)&&(UDP_HEX_Rx[15]==0))
									     MSG_BroadCast_Obj.INFO_TYPE=0;
									  else
									  	  MSG_BroadCast_Obj.INFO_TYPE=UDP_HEX_Rx[15];
									  
									  MSG_BroadCast_Obj.INFO_LEN=(UDP_HEX_Rx[16]<<8)+UDP_HEX_Rx[17];
									  memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR));
									  memcpy(MSG_BroadCast_Obj.INFO_STR,UDP_HEX_Rx+18,MSG_BroadCast_Obj.INFO_LEN);  
									  MSG_BroadCast_Obj.INFO_Effective=1;
									  MSG_BroadCast_Obj.INFO_PlyCancel=1;  
	                                                          Api_RecordNum_Write(msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 
									  rt_kprintf("\r\n ��Ϣ�㲥����:%s\r\n",MSG_BroadCast_Obj.INFO_STR); 
							          break;
							default:
								      break;

                         }

                          //---------���� -------
                          // if(SD_ACKflag.f_CentreCMDack_0001H==0) // һ��ظ�
 					      {
		  						 SD_ACKflag.f_CentreCMDack_0001H=1;
		  						 Ack_Resualt=0;
		  						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					       }
						   
                        /* 
                             if(SD_ACKflag.f_MsgBroadCast_0303H==0)
						 {
						     SD_ACKflag.f_MsgBroadCast_0303H=1;  
                             }
                             */ 
		              break;
		   case  0x8304:    //  ��Ϣ����
                          Ack_Resualt=0;
                          MSG_BroadCast_Obj.INFO_TYPE=UDP_HEX_Rx[13];  //  ��Ϣ����
                          MSG_BroadCast_Obj.INFO_LEN=(UDP_HEX_Rx[14]<<8)+UDP_HEX_Rx[15];
						  memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR));
						  memcpy(MSG_BroadCast_Obj.INFO_STR,UDP_HEX_Rx+16,infolen-3); 

						  MSG_BroadCast_Obj.INFO_SDFlag=1;    // ||||||||||||||||||||||||||||||||||
                           //------------------------------
						 /*  Dev_Voice.CMD_Type='2';
						   memset(Dev_Voice.Play_info,0,sizeof(Dev_Voice.Play_info));
                               memcpy(Dev_Voice.Play_info,UDP_HEX_Rx+16,infolen-3);
						   Dev_Voice.info_sdFlag=1;
						   */
						   rt_kprintf("\r\n ��Ϣ��������:%s\r\n",Dev_Voice.Play_info); 

                           // --------  ���͸��ı���Ϣ  --------------    
						   memset( TextInfo.TEXT_Content,0,sizeof(TextInfo.TEXT_Content));
						   memcpy(TextInfo.TEXT_Content,UDP_HEX_Rx+16,infolen-3);
						   TextInfo.TEXT_SD_FLAG=1;    // �÷��͸���ʾ����־λ	// ||||||||||||||||||||||||||||||||||
						   
 
                          //------- ���� ----
						 //  if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {
						     Ack_Resualt=0;
						     SD_ACKflag.f_CentreCMDack_0001H=1;
							 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
                         }
		              break;	
		   case  0x8400:    //  �绰�ز�
                       
					   if(infolen==0)
							 break;
					   if(0==UDP_HEX_Rx[13])   // ��ͨͨ��
					   	   rt_kprintf("\r\n   �绰�ز�-->��ͨͨ��\r\n");
					   else
					   if(1==UDP_HEX_Rx[13])  //  ����
					       rt_kprintf("\r\n   �绰�ز�-->����");
					   else
					       break;
	 					  memset(JT808Conf_struct.LISTEN_Num,0,sizeof(JT808Conf_struct.LISTEN_Num));
	 					  memcpy(JT808Conf_struct.LISTEN_Num,UDP_HEX_Rx+14,infolen-1);  										  
                                            Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
                                            CallState=CallState_rdytoDialLis;  // ׼����ʼ����������� 

						 // if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {
						     SD_ACKflag.f_CentreCMDack_0001H=1;
							 SD_ACKflag.f_CentreCMDack_resualt=0;      
                         }
		              break;
		   case  0x8401:  //   ���õ绰��
                      
					  switch(UDP_HEX_Rx[13])
					  {
						 case 0 :  //  ɾ���ն�����������Ϣ
								  // PhoneBook_Init(1); 
								   rt_kprintf("\r\n ɾ���绰��\r\n"); 
								   break;
						 case 1:  // ���²˵�
						          if(UDP_HEX_Rx[13]==1)
									 	 rt_kprintf("\r\n ���µ绰��\r\n"); 
						 case 3:  // �޸Ĳ˵�
						           if(UDP_HEX_Rx[13]==3)
									 	 rt_kprintf("\r\n �޸ĵ绰��\r\n"); 
						           Rx_PhoneBOOK.CALL_TYPE=UDP_HEX_Rx[15]; // ��־ ���������
                                   Rx_PhoneBOOK.NumLen=UDP_HEX_Rx[16];								   
								   memset(Rx_PhoneBOOK.NumberStr,0,sizeof(Rx_PhoneBOOK.NumberStr)); 
								   memcpy(Rx_PhoneBOOK.NumberStr,UDP_HEX_Rx+17,Rx_PhoneBOOK.NumLen);
								   Rx_PhoneBOOK.UserLen=UDP_HEX_Rx[17+Rx_PhoneBOOK.NumLen]; 								   
								   memset(Rx_PhoneBOOK.UserStr,0,sizeof(Rx_PhoneBOOK.UserStr));								   
								   memcpy(Rx_PhoneBOOK.UserStr,UDP_HEX_Rx+18+Rx_PhoneBOOK.NumLen,Rx_PhoneBOOK.UserLen);
								   
								   for(i=0;i<8;i++)
								   {
                                    PhoneBook.CALL_TYPE=2; //���Ͷ���Ϊ��� 
								    PhoneBook.NumLen=0;    // ���볤��
								    memset(PhoneBook.NumberStr,0,sizeof(PhoneBook.NumberStr));  
								    PhoneBook.UserLen=0;		
								    memset(PhoneBook.UserStr,0,sizeof(PhoneBook.UserStr)); 	 
									   Api_RecordNum_Read(phonebook, i+1, (u8*)&PhoneBook,sizeof(PhoneBook)); 
									if(strncmp((char*)PhoneBook.UserStr,(const char*)Rx_PhoneBOOK.UserStr,Rx_PhoneBOOK.UserLen) == 0) 
									{ // �ҵ���ͬ���ֵİ���ǰ��ɾ�����µĴ���
									   Api_RecordNum_Write(phonebook, i+1, (u8*)&Rx_PhoneBOOK,sizeof(Rx_PhoneBOOK)); 
									   break;  // ����for 
									}

								   }
								   break;
						 case 2:  // ׷�Ӳ˵�	
						          if(UDP_HEX_Rx[13]==2)
									 	 rt_kprintf("\r\n ׷�ӵ绰��\r\n"); 
						           Rx_PhoneBOOK.CALL_TYPE=UDP_HEX_Rx[15]; // ��־ ���������
                                   Rx_PhoneBOOK.NumLen=UDP_HEX_Rx[16];								   
								   memset(Rx_PhoneBOOK.NumberStr,0,sizeof(Rx_PhoneBOOK.NumberStr)); 
								   memcpy(Rx_PhoneBOOK.NumberStr,UDP_HEX_Rx+17,Rx_PhoneBOOK.NumLen);
								   Rx_PhoneBOOK.UserLen=UDP_HEX_Rx[17+Rx_PhoneBOOK.NumLen]; 								   
								   memset(Rx_PhoneBOOK.UserStr,0,sizeof(Rx_PhoneBOOK.UserStr));		
								   Rx_PhoneBOOK.Effective_Flag=1; // ��Ч��־λ  
								   memcpy(Rx_PhoneBOOK.UserStr,UDP_HEX_Rx+18+Rx_PhoneBOOK.NumLen,Rx_PhoneBOOK.UserLen);							   
								   Api_RecordNum_Read(phonebook, UDP_HEX_Rx[14], (u8*)&Rx_PhoneBOOK, sizeof(Rx_PhoneBOOK)); 
                                                           rt_kprintf("\r\n Name:%s\r\n",Rx_PhoneBOOK.UserStr); 
								   rt_kprintf("\r\n Number:%s\r\n",Rx_PhoneBOOK.NumberStr);
								   break;
						 default:
								   break;
					  
					  }

					   
					// if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
					 
			          break;
           case  0x8500:    //  ��������
                      Vech_Control.Control_Flag=UDP_HEX_Rx[13];
					  if(UDP_HEX_Rx[13]&0x01)
					  	{ // ���ż���       bit 12
                            Car_Status[2]|=0x10;     // ��Ҫ���Ƽ̵���
							rt_kprintf("\r\n  �������� \r\n"); 
					  	}
					  else
					  	{ // ���Ž���
                            Car_Status[2]&=~0x10;    // ��Ҫ���Ƽ̵���
                            rt_kprintf("\r\n  �������� \r\n"); 
					  	}					  
					  Vech_Control.ACK_SD_Flag=1;    
		              break;
		   case  0x8600:    //  ����Բ������
                       rt_kprintf("\r\n  ����Բ������ \r\n"); 
		              if(UDP_HEX_Rx[14]==1)  //  ����֧������һ������
		              { 
	                       switch(UDP_HEX_Rx[13])
	                       	{	                       	      
		                          case 1:  // ׷������
                                           for(i=0;i<8;i++)
                                           	{                                                
												memset((u8*)&Rail_Cycle,0,sizeof(Rail_Cycle));
												Api_RecordNum_Write(Rail_cycle,Rail_Cycle.Area_ID, (u8*)&Rail_Cycle,sizeof(Rail_Cycle)); 	 
												if(Rail_Cycle.Area_attribute) // �ҳ���δʹ�õ�
													break;
                                           	}
										   if(8==i)  //  ��������ˣ���ô�� 0
										   	{
										   	  i=0;
										   	}
								         
		                          case 0:  // ��������
		                          case 2:  // �޸�����                                      
										   memset((u8*)&Rail_Cycle,0,sizeof(Rail_Cycle));
                                           Rail_Cycle.Area_ID=(UDP_HEX_Rx[15]<<24)+(UDP_HEX_Rx[16]<<16)+(UDP_HEX_Rx[17]<<8)+UDP_HEX_Rx[18];
										   Rail_Cycle.Area_attribute=(UDP_HEX_Rx[19]<<8)+UDP_HEX_Rx[20];
										   Rail_Cycle.Center_Latitude=(UDP_HEX_Rx[21]<<24)+(UDP_HEX_Rx[22]<<16)+(UDP_HEX_Rx[23]<<8)+UDP_HEX_Rx[24];
										   Rail_Cycle.Center_Longitude=(UDP_HEX_Rx[25]<<24)+(UDP_HEX_Rx[26]<<16)+(UDP_HEX_Rx[27]<<8)+UDP_HEX_Rx[28]; 
										   Rail_Cycle.Radius=(UDP_HEX_Rx[29]<<24)+(UDP_HEX_Rx[30]<<16)+(UDP_HEX_Rx[31]<<8)+UDP_HEX_Rx[32]; 
										   memcpy(Rail_Cycle.StartTimeBCD,UDP_HEX_Rx+33,6);
										   memcpy(Rail_Cycle.EndTimeBCD,UDP_HEX_Rx+39,6);   
										   Rail_Cycle.MaxSpd=(UDP_HEX_Rx[45]<<8)+UDP_HEX_Rx[46]; 
										   Rail_Cycle.KeepDur=UDP_HEX_Rx[47];  
										   Rail_Cycle.Effective_flag=1; 

										   if((Rail_Cycle.Area_ID>8)||(Rail_Cycle.Area_ID==0))
										   	Rail_Cycle.Area_ID=1;										   
										    Api_RecordNum_Write(Rail_cycle,Rail_Cycle.Area_ID, (u8*)&Rail_Cycle,sizeof(Rail_Cycle)); 	 
									   
								           break;
									default:
										    break;

	                       	}    
					   
		              	} 
                		 //------- ���� ----
						//   if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {
						     SD_ACKflag.f_CentreCMDack_0001H=1;
							 Ack_Resualt=0;
							 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
                         }
				      break;
           case  0x8601:    //  ɾ��Բ������ 
                         rt_kprintf("\r\n  ɾ��Բ������ \r\n"); 
						 if(0==UDP_HEX_Rx[13])   // ������
                                                    RailCycle_Init();  // ɾ����������  
						 else
						 	{						 	  
							  memset((u8*)&Rail_Cycle,0,sizeof(Rail_Cycle));  //  clear all  first
						 	  for(i=0;i<UDP_HEX_Rx[13];i++)
						 	  {
						 	    Rail_Cycle.Area_ID=(UDP_HEX_Rx[14+i]<<24)+(UDP_HEX_Rx[15+i]<<16)+(UDP_HEX_Rx[16+i]<<8)+UDP_HEX_Rx[17+i];
                                if((Rail_Cycle.Area_ID>8)||(Rail_Cycle.Area_ID==0))
										   	Rail_Cycle.Area_ID=1;	
								Rail_Cycle.Effective_flag=0; // clear
								 Api_RecordNum_Write(Rail_cycle,Rail_Cycle.Area_ID, (u8*)&Rail_Cycle,sizeof(Rail_Cycle)); // ɾ����Ӧ��Χ��
						 	  }

						 	}

                        //----------------
                    //   if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
		              break;
		   case  0x8602:    //  ���þ�������
		               rt_kprintf("\r\n  ���þ������� \r\n"); 
                        if(UDP_HEX_Rx[14]==1)  //  ����֧������һ������
		              { 
	                       switch(UDP_HEX_Rx[13])
	                       	{	                       	      
		                          case 1:  // ׷������
                                  case 0:  // ��������
		                          case 2:  // �޸�����                                      
										   memset((u8*)&Rail_Rectangle,0,sizeof(Rail_Rectangle));
                                           Rail_Rectangle.Area_ID=(UDP_HEX_Rx[15]<<24)+(UDP_HEX_Rx[16]<<16)+(UDP_HEX_Rx[17]<<8)+UDP_HEX_Rx[18];
										   Rail_Rectangle.Area_attribute=(UDP_HEX_Rx[19]<<8)+UDP_HEX_Rx[20];
										   Rail_Rectangle.LeftUp_Latitude=(UDP_HEX_Rx[21]<<24)+(UDP_HEX_Rx[22]<<16)+(UDP_HEX_Rx[23]<<8)+UDP_HEX_Rx[24];
										   Rail_Rectangle.LeftUp_Longitude=(UDP_HEX_Rx[25]<<24)+(UDP_HEX_Rx[26]<<16)+(UDP_HEX_Rx[27]<<8)+UDP_HEX_Rx[28]; 
										   Rail_Rectangle.RightDown_Latitude=(UDP_HEX_Rx[29]<<24)+(UDP_HEX_Rx[30]<<16)+(UDP_HEX_Rx[31]<<8)+UDP_HEX_Rx[32];
										   Rail_Rectangle.RightDown_Longitude=(UDP_HEX_Rx[33]<<24)+(UDP_HEX_Rx[34]<<16)+(UDP_HEX_Rx[35]<<8)+UDP_HEX_Rx[36]; 
										   memcpy(Rail_Rectangle.StartTimeBCD,UDP_HEX_Rx+37,6);
										   memcpy(Rail_Rectangle.EndTimeBCD,UDP_HEX_Rx+43,6);   
										   Rail_Rectangle.MaxSpd=(UDP_HEX_Rx[49]<<8)+UDP_HEX_Rx[50]; 
										   Rail_Rectangle.KeepDur=UDP_HEX_Rx[51];   
										   Rail_Rectangle.Effective_flag=1;

										   if((Rail_Rectangle.Area_ID>8)||(Rail_Rectangle.Area_ID==0))
										   	Rail_Rectangle.Area_ID=1;										   
										   Api_RecordNum_Write(Rail_rect,Rail_Rectangle.Area_ID, (u8*)&Rail_Rectangle,sizeof(Rail_Rectangle)); 	 

										   rt_kprintf("\r\n   ��������  ����Χ�� leftLati=%d leftlongi=%d\r\n",Rail_Rectangle.LeftUp_Latitude,Rail_Rectangle.LeftUp_Longitude);  
										   break;
									default:
										    break;

	                       	}    
					   
		              	} 
                        //----------------
                     //if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
		              break;
		   case  0x8603:    //  ɾ����������
                           rt_kprintf("\r\n  ɾ���������� \r\n");    
					   	  if(0==UDP_HEX_Rx[13])   // ������
                             RailRect_Init();  // ɾ����������
						   else
						 	{						 	  
							  memset((u8*)&Rail_Rectangle,0,sizeof(Rail_Rectangle));  //  clear all  first
						 	  for(i=0;i<UDP_HEX_Rx[13];i++)
						 	  {
						 	    Rail_Rectangle.Area_ID=(UDP_HEX_Rx[14+i]<<24)+(UDP_HEX_Rx[15+i]<<16)+(UDP_HEX_Rx[16+i]<<8)+UDP_HEX_Rx[17+i];
                                if((Rail_Rectangle.Area_ID>8)||(Rail_Rectangle.Area_ID==0))
										   	Rail_Rectangle.Area_ID=1;	
								Rail_Rectangle.Effective_flag=0;
								Api_RecordNum_Write(Rail_rect,Rail_Rectangle.Area_ID, (u8*)&Rail_Rectangle,sizeof(Rail_Rectangle)); // ɾ����Ӧ��Χ��
						 	  }  
						 	}


                         //----------------
                     //  if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
			          break;
		   case  0x8604:	//  ���������
		                     rt_kprintf("\r\n  ���ö�������� \r\n");
			                 if(UDP_HEX_Rx[14]==1)  //  ����֧������һ������
							 { 
								  switch(UDP_HEX_Rx[13])
								   {								 
										 case 1:  // ׷������
										 case 0:  // ��������
										 case 2:  // �޸�����									   
												  memset((u8*)&Rail_Polygen,0,sizeof(Rail_Polygen));   
												  Rail_Polygen.Area_ID=(UDP_HEX_Rx[15]<<24)+(UDP_HEX_Rx[16]<<16)+(UDP_HEX_Rx[17]<<8)+UDP_HEX_Rx[18];
												  Rail_Polygen.Area_attribute=(UDP_HEX_Rx[19]<<8)+UDP_HEX_Rx[20];
												  memcpy(Rail_Polygen.StartTimeBCD,UDP_HEX_Rx+20,6);
												  memcpy(Rail_Polygen.EndTimeBCD,UDP_HEX_Rx+26,6);   
												  Rail_Polygen.MaxSpd=(UDP_HEX_Rx[32]<<8)+UDP_HEX_Rx[33]; 
												  Rail_Polygen.KeepDur=UDP_HEX_Rx[34];   
                                                                                      Rail_Polygen.Acme_Num=UDP_HEX_Rx[35];												  
												  Rail_Polygen.Acme1_Latitude=(UDP_HEX_Rx[36]<<24)+(UDP_HEX_Rx[37]<<16)+(UDP_HEX_Rx[38]<<8)+UDP_HEX_Rx[39];
												  Rail_Polygen.Acme1_Longitude=(UDP_HEX_Rx[40]<<24)+(UDP_HEX_Rx[41]<<16)+(UDP_HEX_Rx[42]<<8)+UDP_HEX_Rx[43]; 
												  Rail_Polygen.Acme2_Latitude=(UDP_HEX_Rx[44]<<24)+(UDP_HEX_Rx[45]<<16)+(UDP_HEX_Rx[46]<<8)+UDP_HEX_Rx[47];
												  Rail_Polygen.Acme2_Longitude=(UDP_HEX_Rx[48]<<24)+(UDP_HEX_Rx[49]<<16)+(UDP_HEX_Rx[50]<<8)+UDP_HEX_Rx[51];
												  Rail_Polygen.Acme3_Latitude=(UDP_HEX_Rx[52]<<24)+(UDP_HEX_Rx[53]<<16)+(UDP_HEX_Rx[54]<<8)+UDP_HEX_Rx[55];
												  Rail_Polygen.Acme3_Longitude=(UDP_HEX_Rx[56]<<24)+(UDP_HEX_Rx[57]<<16)+(UDP_HEX_Rx[58]<<8)+UDP_HEX_Rx[59];  
			   
												  if((Rail_Polygen.Area_ID>8)||(Rail_Polygen.Area_ID==0))
												   Rail_Polygen.Area_ID=1;		
												  Rail_Polygen.Effective_flag=1;
                                                                                       Api_RecordNum_Write(Rail_polygen,Rail_Polygen.Area_ID, (u8*)&Rail_Polygen,sizeof(Rail_Polygen)); 	 
												  break;
										   default:
												   break;
			   
								   }	
							  
							   } 

                          //----------------
                      // if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					     {
	 						 SD_ACKflag.f_CentreCMDack_0001H=1;
	 						 Ack_Resualt=0;
	 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					      }
		              break;
		   case  0x8605:    //  ɾ���������
                         rt_kprintf("\r\n  ɾ����������� \r\n");
                        if(0==UDP_HEX_Rx[13])   // ������
                             RailPolygen_Init();  // ɾ����������
						 else
						 	{						 	  
							  memset((u8*)&Rail_Polygen,0,sizeof(Rail_Polygen));  //  clear all  first
						 	  for(i=0;i<UDP_HEX_Rx[13];i++)
						 	  {
						 	    Rail_Polygen.Area_ID=(UDP_HEX_Rx[14+i]<<24)+(UDP_HEX_Rx[15+i]<<16)+(UDP_HEX_Rx[16+i]<<8)+UDP_HEX_Rx[17+i];
                                if((Rail_Polygen.Area_ID>8)||(Rail_Polygen.Area_ID==0))
										   	Rail_Polygen.Area_ID=1;	
								Rail_Polygen.Effective_flag=0;   
                                                         Api_RecordNum_Write(Rail_polygen,Rail_Polygen.Area_ID, (u8*)&Rail_Polygen,sizeof(Rail_Polygen)); 	 
						 	  }   
						 	}

                          //----------------
                      // if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 } 
		              break;
		   case  0x8606:    //  ����·��
		                 rt_kprintf("\r\n  ����·�� \r\n");
						memset((u8*)&ROUTE_Obj,0,sizeof(ROUTE_Obj));  //  clear all  first			
                                          ROUTE_Obj.Route_ID=(UDP_HEX_Rx[13]<<24)+(UDP_HEX_Rx[14]<<16)+(UDP_HEX_Rx[15]<<8)+UDP_HEX_Rx[16];  
 						ROUTE_Obj.Route_attribute=(UDP_HEX_Rx[17]<<8)+UDP_HEX_Rx[18];  						
						memcpy(ROUTE_Obj.StartTimeBCD,UDP_HEX_Rx+19,6); 
						memcpy(ROUTE_Obj.EndTimeBCD,UDP_HEX_Rx+25,6); 
						ROUTE_Obj.Points_Num=(UDP_HEX_Rx[31]<<8)+UDP_HEX_Rx[32];  
						rt_kprintf("\r\n ROUTE_Obj.ID:  %d  \r\n ",ROUTE_Obj.Route_ID);    
						rt_kprintf("\r\n ROUTE_Obj.ID:  %04X  \r\n ",ROUTE_Obj.Route_attribute); 
						rt_kprintf("\r\n ROUTE_Obj.Points_Num:  %d  \r\n ",ROUTE_Obj.Points_Num);   
						if(ROUTE_Obj.Points_Num<3) 
						{ 
 						   SD_ACKflag.f_CentreCMDack_0001H=1;
 						   Ack_Resualt=0;
 						    SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
						  break;
						}  
						reg_u32=33;
						for(i=0;i<6;i++)  // �յ���Ŀ
						{						  
						   // if((infolen+32)<reg_u32)   
							//	 break;   
							   
						  ROUTE_Obj.RoutePoints[i].POINT_ID=(UDP_HEX_Rx[reg_u32]<<24)+(UDP_HEX_Rx[reg_u32+1]<<16)+(UDP_HEX_Rx[reg_u32+2]<<8)+UDP_HEX_Rx[reg_u32+3]; 
						  reg_u32+=4;
						  rt_kprintf("\r\n PointID=%08x\r\n",ROUTE_Obj.RoutePoints[i].POINT_ID);
						  ROUTE_Obj.RoutePoints[i].Line_ID=(UDP_HEX_Rx[reg_u32]<<24)+(UDP_HEX_Rx[reg_u32+1]<<16)+(UDP_HEX_Rx[reg_u32+2]<<8)+UDP_HEX_Rx[reg_u32+3]; 
						  reg_u32+=4;
                                            rt_kprintf("\r\n LineID=%08x\r\n",ROUTE_Obj.RoutePoints[i].Line_ID);
						  ROUTE_Obj.RoutePoints[i].POINT_Latitude=(UDP_HEX_Rx[reg_u32]<<24)+(UDP_HEX_Rx[reg_u32+1]<<16)+(UDP_HEX_Rx[reg_u32+2]<<8)+UDP_HEX_Rx[reg_u32+3]; 
						  reg_u32+=4;
                                             rt_kprintf("\r\n LatiID=%08x\r\n",ROUTE_Obj.RoutePoints[i].POINT_Latitude); 
						  ROUTE_Obj.RoutePoints[i].POINT_Longitude=(UDP_HEX_Rx[reg_u32]<<24)+(UDP_HEX_Rx[reg_u32+1]<<16)+(UDP_HEX_Rx[reg_u32+2]<<8)+UDP_HEX_Rx[reg_u32+3]; 
						  reg_u32+=4;
                                                rt_kprintf("\r\n LongID=%08x\r\n",ROUTE_Obj.RoutePoints[i].POINT_Longitude); 
						  ROUTE_Obj.RoutePoints[i].Width=UDP_HEX_Rx[reg_u32++];  
						     rt_kprintf("\r\n Width=%02x\r\n",ROUTE_Obj.RoutePoints[i].Width); 
						  ROUTE_Obj.RoutePoints[i].Atribute=UDP_HEX_Rx[reg_u32++]; 
						   rt_kprintf("\r\n atrit=%02x\r\n\r\n",ROUTE_Obj.RoutePoints[i].Atribute);         
						   if(ROUTE_Obj.RoutePoints[i].Atribute==0)
                                                            ;
						   else
						    if(ROUTE_Obj.RoutePoints[i].Atribute==1) 
                                                      ROUTE_Obj.RoutePoints[i].MaxSpd=(UDP_HEX_Rx[reg_u32++]<<8)+UDP_HEX_Rx[reg_u32++]; 
                                             else
							 { 
									  ROUTE_Obj.RoutePoints[i].TooLongValue=(UDP_HEX_Rx[reg_u32++]<<8)+UDP_HEX_Rx[reg_u32++]; 
									  ROUTE_Obj.RoutePoints[i].TooLessValue=(UDP_HEX_Rx[reg_u32++]<<8)+UDP_HEX_Rx[reg_u32++];   
									  ROUTE_Obj.RoutePoints[i].MaxSpd=(UDP_HEX_Rx[reg_u32++]<<8)+UDP_HEX_Rx[reg_u32++]; 
									  ROUTE_Obj.RoutePoints[i].KeepDur=(UDP_HEX_Rx[reg_u32++]<<8)+UDP_HEX_Rx[reg_u32++];    
						     	}

						}	
                           
						  if((ROUTE_Obj.Route_ID>Route_Mum)||(ROUTE_Obj.Route_ID==0))
									  ROUTE_Obj.Route_ID=1;		
						  ROUTE_Obj.Effective_flag=1;
						  Api_RecordNum_Write(route_line,ROUTE_Obj.Route_ID, (u8*)&ROUTE_Obj,sizeof(ROUTE_Obj));// ɾ����Ӧ��Χ�� 



                           //----------------
                    // if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
		              break;
		   case  0x8607:    //  ɾ��·��
                         rt_kprintf("\r\n  ɾ��·�� \r\n");  
                         if(0==UDP_HEX_Rx[13])   // ������
                             RouteLine_Init();  // ɾ����������
						 else
						 	{						 	  
							  memset((u8*)&ROUTE_Obj,0,sizeof(ROUTE_Obj));  //  clear all  first
						 	  for(i=0;i<UDP_HEX_Rx[13];i++)
						 	  {
						 	    ROUTE_Obj.Route_ID=(UDP_HEX_Rx[14+i]<<24)+(UDP_HEX_Rx[15+i]<<16)+(UDP_HEX_Rx[16+i]<<8)+UDP_HEX_Rx[17+i];
                                if((ROUTE_Obj.Route_ID>Route_Mum)||(ROUTE_Obj.Route_ID==0))
										   	ROUTE_Obj.Route_ID=1;	
								ROUTE_Obj.Effective_flag=0;       
								     Api_RecordNum_Write(route_line,ROUTE_Obj.Route_ID, (u8*)&ROUTE_Obj,sizeof(ROUTE_Obj)); 	  // ɾ����Ӧ��Χ��
						 	  }   
						 	}  

                           //----------------
                    // if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;        
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
		              break;
           case  0x8700:    //  �г���¼�����ݲɼ�����
                       rt_kprintf("\r\n  ��¼�ǲɼ����� \r\n");
                       Recode_Obj.Float_ID=Centre_FloatID;
					   Recode_Obj.CMD=UDP_HEX_Rx[13];
					   Recode_Obj.SD_Data_Flag=1;
					   
		              break;
		   case  0x8701:    //  ��ʻ��¼�ǲ����´�����
		               rt_kprintf("\r\n  ��¼�ǲ����´� \r\n");
		               Recode_Obj.Float_ID=Centre_FloatID;
					   Recode_Obj.CMD=UDP_HEX_Rx[13];    
					   Recode_Obj.SD_Data_Flag=1;  
					   CenterSet_subService_8701H(Recode_Obj.CMD,UDP_HEX_Rx+14); //����2B���Ⱥ�1 ������
			          break; 
		   case  0x8800:    //   ��ý�������ϴ�Ӧ��
		              if(infolen==5)
		              	{  //  �ж��Ƿ����ش�ID�б����û�����ʾ���Ľ������!						
                                switch (MediaObj.Media_Type)
						         {
						          case 0 : // ͼ��
						                  Photo_send_end();  // �����ϴ�����
						                  rt_kprintf("\r\n ͼ�������! \r\n");
                                          //------------��·���մ���  -------
                                         // CheckResualt=Check_MultiTakeResult_b4Trans();  
						                 				   
								          break;
								  case 1 : // ��Ƶ
								          Sound_send_end();
										  rt_kprintf("\r\n ��Ƶ�������! \r\n");
								          break;
								  case 2 : // ��Ƶ
									      Video_send_end();
  								          rt_kprintf("\r\n ��Ƶ�������! \r\n");  
								          break;
								  default:
								  	      break;
						         }
						// if(CheckResualt==0)	
						   	   Media_Clear_State(); 
		              	}
					  else
					  	{  //  �ش���ID �б� 
					  	  if(UDP_HEX_Rx[17]!=0)
					  	  {
						  	   MediaObj.RSD_State=1;   //   �����ش�״̬
						  	   MediaObj.RSD_Timer=0;   //   ����ش���ʱ��   
						  	   MediaObj.RSD_Reader=0;
							   MediaObj.RSD_total=UDP_HEX_Rx[17];    // �ش�������
						  	   memset(MediaObj.Media_ReSdList,0,125);
							   memcpy(MediaObj.Media_ReSdList,UDP_HEX_Rx+18,UDP_HEX_Rx[17]); 
	                            rt_kprintf("\r\n  �ش��б�: ");
							   for(i=0;i<MediaObj.RSD_total;i++)
							     rt_kprintf(" %d",UDP_HEX_Rx[18+i]); 
							   rt_kprintf("\r\n"); 
					  	   } 
						}
                        
		              break;
		   case  0x8801:   //    ����ͷ��������		  

				     Camera_Obj.Channel_ID=UDP_HEX_Rx[13];     //   ͨ��
					 Camera_Obj.Operate_state=UDP_HEX_Rx[18];  //   �Ƿ񱣴��־λ  
		            //----------------------------------
				   
					if((Camera_Take_Enable())&&(Photo_sdState.photo_sending==0)) //ͼƬ�����в�����
					{
					   Camera_Number=UDP_HEX_Rx[13];
					   if((Camera_Number>Max_CameraNum)&&(Camera_Number<1)) 
						  break;
					 
					   Start_Camera(Camera_Number);   //��ʼ����  
					   SingleCamera_TakeRetry=0;   // clear 
					}	   
					rt_kprintf("\r\n   ���ļ�ʱ����  Camera: %d    \r\n",Camera_Number);	
		              
                     // if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
                      break;
		   case  0x8802:   //    �洢��ý�����ݼ���
		                 SD_ACKflag.f_QueryEventCode=UDP_HEX_Rx[15];    
                         switch(UDP_HEX_Rx[13])
                         {
                            case 0:  // ͼ��
                                     SD_ACKflag.f_MediaIndexACK_0802H=1;
									 rt_kprintf("\r\n  ���Ĳ�ѯͼ������ \r\n");   
							       break;
							case 1:  //  ��Ƶ
                                     SD_ACKflag.f_MediaIndexACK_0802H=2;
									  rt_kprintf("\r\n  ���Ĳ�ѯ��Ƶ���� \r\n"); 	 
							case 2:  //  ��Ƶ
							         SD_ACKflag.f_MediaIndexACK_0802H=3;
						    default:
								     break; 
						}						 
						
		              break;
		   case  0x8803:   //    �洢��ý�������ϴ�����
                      rt_kprintf("\r\n ��ý�������ϴ�\r\n"); 
                       switch(UDP_HEX_Rx[13])
                         {
                            case 0:  // ͼ��
									 rt_kprintf("\r\n   �ϴ�����ͼƬ\r\n");    
							       break;
							case 1:  //  ��Ƶ
                                     MP3_send_start();
									  rt_kprintf("\r\n  �ϴ�������Ƶ \r\n"); 
								   break;	  
							case 2:  //  ��Ƶ
							        // Video_send_start();
							         // MP3_send_start();
									  rt_kprintf("\r\n  �ϴ�������Ƶ  �������� ����Ƶ\r\n"); 
								   break;	  
						    default:
								     break; 
						}				
					  
                     //----------------------------------------------------------
                     // if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
		              break;
		   case  0x8804:   //    ¼����ʼ����
		   
		            //#if  1			     
                       switch(UDP_HEX_Rx[13])
                       	{ 
                       	  case 0x00:  // ͣ¼��
                       	                      // VOC_REC_STOP(void);
						       VOC_REC_Stop();
							/* 					  
	                                     Dev_Voice.CMD_Type='0';
										 Dev_Voice.info_sdFlag=0;    
										 Dev_Voice.Voice_FileOperateFlag=0;
										 Dev_Voice.Centre_RecordFlag=0; // �������¼����־λ
										 if(TF_Card_Status()==1)	
												{
												  //memset(Dev_Voice.Voice_Reg,0,512);//clear left
												  //edit_file(Dev_Voice.FileName,Dev_Voice.Voice_Reg,512); //д��Ϣ��TF
												  //Dev_Voice.Voice_FileSize+=500; //add  											  
												  rt_kprintf("\r\n ---------   �����ļ�������ַ	  VoiceFileSize %d Bytes  \r\n",Dev_Voice.Voice_FileSize);
												}   
										Api_DFdirectory_Write(voice, Dev_Voice.Voice_Reg,500);  
		                                 Dev_Voice.Voice_FileSize+=500;
		                                 Sound_SaveEnd();  */
						            break;
						  case 0x01:  // ��ʼ¼��
                                                       
								     VOC_REC_Start();
						 					   
					 /*
						          if(MMedia2_Flag==0)
						          	{
                                       MP3_send_start();
									   rt_kprintf("\r\n  �ϴ�������Ƶ \r\n"); 
									   MMedia2_Flag=1;
									   break;
						          	}
                                    Dev_Voice.Rec_Dur=(UDP_HEX_Rx[14]<<8)+UDP_HEX_Rx[15];
									Dev_Voice.SaveOrNotFlag=UDP_HEX_Rx[16];

                                    // ------   ¼���ļ��� -----------
                                    if(TF_Card_Status()==1)
	                                { 
	                                    memset(Dev_Voice.FileName,0,sizeof(Dev_Voice.FileName)); 	    									
										sprintf((char*)Dev_Voice.FileName,"%d%d%d%d.spx",time_now.day,time_now.hour,time_now.min,time_now.sec);    
	                                   // creat_file(Dev_Voice.FileName); //�����ļ����� 
	                                    Sound_SaveStart(); 
										rt_kprintf("\r\n			  ����¼���ļ�����: %s \r\n",Dev_Voice.FileName);	
										Save_MediaIndex(1,Dev_Voice.FileName,0,0);     
                                     }
                                    Dev_Voice.Centre_RecordFlag=1;//���Ŀ�ʼ¼����־λ
									              
									Dev_Voice.CMD_Type='1';
									Dev_Voice.info_sdFlag=2;									
									Dev_Voice.Voice_FileSize=0; //clear size
									Dev_Voice.Voice_FileOperateFlag=1; 
									MMedia2_Flag=0;  
					   */				
							        break;

                       	}
					  
					      
					  //------------------------------
					 //if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
		              break;
		   case  0x8805:   //    �����洢��ý�����ݼ����ϴ�����	---- ����Э��Ҫ��

                      reg_u32=(UDP_HEX_Rx[13]<<24)+(UDP_HEX_Rx[14]<<16)+(UDP_HEX_Rx[15]<<8)+UDP_HEX_Rx[16];
					  rt_kprintf("\r\n�����洢��ý�����ݼ����ϴ� MeidiaID=%d ɾ����־: %d ",reg_u32,UDP_HEX_Rx[17]);

					  	  //------------------------------
					 // if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
		              break;
		 case  0x8900:   //    ��������͸��                      
		                      if(LinkNum==0)
		                     {
                                     DataTrans.TYPE=UDP_HEX_Rx[13];
					  memset(DataTrans.DataRx,0,sizeof(DataTrans.DataRx)); 
                                     memcpy(DataTrans.DataRx,UDP_HEX_Rx+14,infolen-1);
					  DataTrans.Data_RxLen=infolen-1;   

					  //--------- �͸�С��Ļ---------- 	
					  	memset( TextInfo.TEXT_Content,0,sizeof(TextInfo.TEXT_Content));
						AsciiToGb(TextInfo.TEXT_Content,infolen-1,UDP_HEX_Rx+14); 
						TextInfo.TEXT_SD_FLAG=1;	// �÷��͸���ʾ����־λ  // ||||||||||||||||||||||||||||||||||

						//========================================
						   

						     #ifdef LCD_5inch
						 
							 DwinLCD.Type=LCD_SDTXT;
							  memset(DwinLCD.TXT_content,0,sizeof(DwinLCD.TXT_content)); 
                                                   DwinLCD.TXT_contentLen=AsciiToGb(DwinLCD.TXT_content,infolen-1,UDP_HEX_Rx+14);  						  
				   
							#endif  
					 // if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
		   	        }
				 else
				 {
                                   if(UDP_HEX_Rx[13]==0x1F) 
					  	{
					  	 
						    //ISP_Get_RawData(UDP_HEX_Rx+14,UDP_hexRx_len-14); 	
						    rt_kprintf("\r\n ISP_raw\r\n"); 
					           Nullpro_8900H (UDP_HEX_Rx+14);
						    return;	
					  	  //  DataTrans_ISPService_8900H(UDP_HEX_Rx+14); //14 ƫ������Ϣ����
					  	}
					  else
					  	{
					  	rt_kprintf("\r\n͸���������ʹ���UDP_HEX_Rx[13]=%d",UDP_HEX_Rx[13]);
					  	}
				 }
		              break;
		   case  0x8A00:   //    ƽ̨RSA��Կ

		   
                    // if(SD_ACKflag.f_CentreCMDack_0001H==0) 
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 } 
				      break;	
		  //----------     ���������չЭ��   ----------------------			  
		  case   0xFF01:   // ��չ�ն˲�������
                                       if(contentlen)
                                      {                       // �������������ÿ��ֻ�·�����һ������
                                          Total_ParaNum=UDP_HEX_Rx[13]; // �������ò�������
                                          rt_kprintf("\r\n Set ParametersNum =%d  \r\n",Total_ParaNum);
						 //------------------------------------------------------------------- 
						 ContentRdAdd=14;
						 Process_Resualt=0;  // clear resualt
						 for(i=0;i<Total_ParaNum;i++)
						 {
						   //  ��������DWORD 4 ���ֽ�
						   SubCMD_FF01H=(UDP_HEX_Rx[ContentRdAdd]<<24)+(UDP_HEX_Rx[ContentRdAdd+1]<<16)+(UDP_HEX_Rx[ContentRdAdd+2]<<8)+UDP_HEX_Rx[ContentRdAdd+3]; 
                                            //  ����Ϣ����
						   SubInfolen=UDP_HEX_Rx[ContentRdAdd+4];
						   //  ��������Ϣ ������óɹ�����ӦBit λ��Ϊ 1 ���򱣳� 0
						   if(CentreSet_subService_FF01H(SubCMD_FF01H,SubInfolen,UDP_HEX_Rx+ContentRdAdd+5))
                                              Process_Resualt|=(0x01<<i);						   
						   //  �ƶ�ƫ�Ƶ�ַ
						   ContentRdAdd+=5+UDP_HEX_Rx[ContentRdAdd+4]; // ƫ���±� 
						 } 						   
						 Ack_Resualt=0;
                                        }	
                                          BD_EXT_Write();   
					  
                                   {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }    
		                   break;
		  case   0xFF02: // ��չ�ն˲�����ѯ
                                  SD_ACKflag.f_BD_Extend_7F02H=1;
				      rt_kprintf("\r\n  ���Ĳ�ѯ�������������Ϣ\r\n");			  
		                   break;
		  case  0xFF03:   //  ��չ�ն˲�������1 ����
                                   
                                    if(contentlen)
                                      {                       // �������������ÿ��ֻ�·�����һ������
                                          Total_ParaNum=UDP_HEX_Rx[13]; // �������ò�������
                                          rt_kprintf("\r\n Set ParametersNum =%d  \r\n",Total_ParaNum);
						 //------------------------------------------------------------------- 
						 ContentRdAdd=14;
						 Process_Resualt=0;  // clear resualt
						 for(i=0;i<Total_ParaNum;i++)
						 {
						   //  ��������DWORD 4 ���ֽ�
						   SubCMD_FF03H=(UDP_HEX_Rx[ContentRdAdd]<<24)+(UDP_HEX_Rx[ContentRdAdd+1]<<16)+(UDP_HEX_Rx[ContentRdAdd+2]<<8)+UDP_HEX_Rx[ContentRdAdd+3]; 
                                            //  ����Ϣ����
						   SubInfolen=UDP_HEX_Rx[ContentRdAdd+4];
						   //  ��������Ϣ ������óɹ�����ӦBit λ��Ϊ 1 ���򱣳� 0
						   if(CentreSet_subService_FF03H(SubCMD_FF03H,SubInfolen,UDP_HEX_Rx+ContentRdAdd+5))
                                              Process_Resualt|=(0x01<<i);						   
						   //  �ƶ�ƫ�Ƶ�ַ
						   ContentRdAdd+=5+UDP_HEX_Rx[ContentRdAdd+4]; // ƫ���±� 
						 } 						   
						 Ack_Resualt=0;
                                        }	


                                  {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 } 
			            break;
		    default:
				      break;  	    
	    } 

		//-----------------  memset  ------------------------------------- 
		//memset(UDP_HEX_Rx, 0, sizeof(UDP_HEX_Rx));	
		//UDP_hexRx_len= 0; 
        return;  

}


void Time2BCD(u8 *dest)
 {
	dest[0]=((time_now.year/10)<<4)+(time_now.year%10);  	   	
	dest[1]=((time_now.month/10)<<4)+(time_now.month%10); 
	dest[2]=((time_now.day/10)<<4)+(time_now.day%10);
	dest[3]=((time_now.hour/10)<<4)+(time_now.hour%10);
	dest[4]=((time_now.min/10)<<4)+(time_now.min%10);
	dest[5]=((time_now.sec/10)<<4)+(time_now.sec%10);     
	
 }

void AvrgSpd_MintProcess(u8 hour,u8 min, u8 sec)  
{
	   //------- ϵͳ����ʱ����   �� ��־λΪ0 �������ն�Ϊ 0��������ϵͳ�ո���������ʱ��ȡ��ǰʱ��ѵ�ǰʱ���BAK------
	if((0==Avrgspd_Mint.saveFlag)&&(Avrgspd_Mint.datetime_Bak[0]==0)&&(Avrgspd_Mint.datetime_Bak[1]==0)&&(Avrgspd_Mint.datetime_Bak[2]==0))
		{
		Time2BCD(Avrgspd_Mint.datetime);  
		memcpy(Avrgspd_Mint.datetime_Bak,Avrgspd_Mint.datetime,6);//����ǰʱ���BAK
		avgspd_Mint_Wr=Temp_Gps_Gprs.Time[1];//time_now.min;
		//rt_kprintf("\r\n-------------()avgspd_Mint_Wr=%d",avgspd_Mint_Wr);
		}	
	else
		{     // ������ϵͳ������ʼ   
		if(min==0) //  ������дΪ��0 ���Ա�����ʱ��ȥ�洢��1Сʱ������ 
			{
			if(sec==5)  //�洢��һСʱÿ���ӵ�ƽ���ٶ� ==2����ΪҪ��֤��59�����ٶ��Ѿ����洢����Ӧ�ļĴ�����
				{
				memcpy(Avrgspd_Mint.datetime_Bak,Avrgspd_Mint.datetime,6);//����ǰʱ���BAK
				Avrgspd_Mint.datetime_Bak[4]=0;// ����Ϊ0
				Avrgspd_Mint.datetime_Bak[5]=0;// ��ҲΪ0 
				Avrgspd_Mint.saveFlag=1;//ʹ�ܴ洢      ��NandFlag  ��־λ
				Time2BCD(Avrgspd_Mint.datetime);//��ȡ�µ�ʱ��
				//rt_kprintf("\r\n�洢ǰһСʱƽ���ٶ�,min=%d",min);
				} 
			if(sec==8)    //�洢���  ���ʱ���±�  
				avgspd_Mint_Wr=0;
			}
		}
	
	if(sec==0)  // ��Ϊ0ʱ�洢��1���ӵ�����
		{
		//rt_kprintf("\r\n��һ���ӿ�ʼ,PerMinSpdTotal=%d,AspdCounter=%d",PerMinSpdTotal,AspdCounter);
		if(AspdCounter)
			{			
			//rt_kprintf("\r\n--------------PerMinSpdTotal=%d,AspdCounter=%d", PerMinSpdTotal,AspdCounter);  
			Avrgspd_Mint.avgrspd[avgspd_Mint_Wr]=PerMinSpdTotal/AspdCounter;//�ڼ����ӵ�ƽ���ٶ�
			//=============================================================
			//Ϊ�˺ò�����ʱ��1�������ٶ��ܺ͵���ƽ���ٶ���ͳ��
			//Avrgspd_Mint.avgrspd[avgspd_Mint_Wr]=PerMinSpdTotal;
			//=============================================================
			}
		else
			{
			Avrgspd_Mint.avgrspd[avgspd_Mint_Wr]=0; 
			//rt_kprintf("\r\n-------------(�ٶ���Ч)");
			}
		//rt_kprintf("\r\n--------------%d��,ƽ���ٶ�=%d \r\n", avgspd_Mint_Wr,Avrgspd_Mint.avgrspd[avgspd_Mint_Wr]);  
		//avgspd_Mint_Wr++;//=(Avrgspd_Mint.datetime[4]>>4)*10+(0x0f&Avrgspd_Mint.datetime[4]);  
		
		PerMinSpdTotal=0;  // clear   
		AspdCounter=0;	   // clear  
		
		//rt_kprintf("\r\n��0���ٶ��ܺ�=%d,����=%d\r\n",PerMinSpdTotal,AspdCounter);
		
		}
	else
		{
		if(UDP_dataPacket_flag==0x02 ) 
			{
			avgspd_Mint_Wr=Temp_Gps_Gprs.Time[1];;//++;//=time_now.min+1;
			
			AspdCounter++;
			PerMinSpdTotal+=GPS_speed/10;   // ֻҪ��ȷ�� km/h   ����Ҫ�� 10
			//if(AspdCounter%10==0)
				//rt_kprintf("\r\nAspdCounter=%d,GPS_speed=%d,PerMinSpdTotal=%d \r\n",AspdCounter,GPS_speed/10,PerMinSpdTotal);  
		
			} 
		}  
}

//==================================================================================================
// ���Ĳ��� :   �������г���¼�����Э�� �� ��¼A   
//==================================================================================================

//  1.  �����������������  
/*
u8 In_Type  :   ��������
u8* InStr   :   �����ַ���
u8 TransType:   ���䷽ʽ   ����  ��  GPRS

*/
void Device_Data(u8 In_Type,u8 TransType) 
{
  u8   UpReg[500];
  u16  Up_wr=0,SregLen=0,Greglen=0,Swr=0;//,Gwr=0; // S:serial  G: GPRS
  u8   Sfcs=0, Gfcs=0;  
  u16  i=0;
//  u16  packet_len=0;
  u8   Reg[70];  
  u32  regdis=0,reg2=0;
  u8    QueryRecNum=0;
  
  memset(UpReg,0,500);
  Up_wr=0;

  
  if(TransType)  //-------  GPRS  ����� ����Э��ͷ
  {
    memcpy(UpReg+Up_wr,"*GB",3); // GPRS ��ʼͷ
    Up_wr+=3;
	
    UpReg[Up_wr++]=0x00;  // SIM ����   ���������ֽ���д0x00
	UpReg[Up_wr++]=0x00;
	memcpy(UpReg+Up_wr,SIM_code,6);
	Up_wr+=6;
	
	Greglen=Up_wr;                      // ����
	Up_wr+=2;
	
	UpReg[Up_wr++]=0x00; //��Ϣ��� Ĭ�� 0x00

	UpReg[Up_wr++]=0x20; //����  bit5 bit4 10 ѡ��Ӧ������Э�� 

    UpReg[Up_wr++]=0xF0; // ������  ���ߴ���RS232����

	//   ��������������
  }  
  //---------------��д A Э��ͷ ------------------------
  Swr=Up_wr;  // reg save
  UpReg[Up_wr++]=0xAA;  // ��ʼͷ
  UpReg[Up_wr++]=0x75;
  //---------------�������ͷ�����д����------------------
  switch(In_Type)  
  	{
  	    //---------------- �ϴ�������  -------------------------------------
		case  A_Up_DrvInfo  :      //  ��ǰ��ʻ����Ϣ
		                     UpReg[Up_wr++]=In_Type; //������

							 SregLen=0x00;           // ��Ϣ����
                             UpReg[Up_wr++]=0x00;    // Hi
                             UpReg[Up_wr++]=39;      // Lo
                             
                             UpReg[Up_wr++]=0x00;    // ������ 
                             
                             
                             memcpy(UpReg+Up_wr,JT808Conf_struct.Driver_Info.DriverCard_ID,18); //��Ϣ����
                             Up_wr+=18;
							 memcpy(UpReg+Up_wr,JT808Conf_struct.Driver_Info.DriveName,21);
							 Up_wr+=21;
							 
			                 break;
		case  A_Up_RTC      :      //  �ɼ���¼�ǵ�ʵʱʱ��
                             UpReg[Up_wr++]=In_Type; //������

							 SregLen=0x00;           // ��Ϣ����
                             UpReg[Up_wr++]=0x00;    // Hi
                             UpReg[Up_wr++]=6;      // Lo
                             
                             UpReg[Up_wr++]=0x00;    // ������
                             
                             Time2BCD(UpReg+Up_wr);   //��Ϣ����  
                             Up_wr+=6;
                             break;
		case  A_Up_Dist     :    //  �ɼ����
                             UpReg[Up_wr++]=In_Type; //������

							 SregLen=0x00;           // ��Ϣ����
                             UpReg[Up_wr++]=0x00;    // Hi
                             UpReg[Up_wr++]=16;      // Lo
                             
                             UpReg[Up_wr++]=0x00;    // ������
                             //   ��Ϣ����
                             Time2BCD(UpReg+Up_wr);
							 Up_wr+=6;
                             memcpy(UpReg+Up_wr,(u8*)JT808Conf_struct.FirstSetupDate,6);
							 Up_wr+=6;
							 regdis=JT808Conf_struct.Distance_m_u32/100;  //��λ0.1km 
							 reg2=regdis/10000000;
							 UpReg[Up_wr++]=(reg2<<4)+((regdis%10000000)/1000000);
							 UpReg[Up_wr++]=((regdis%1000000/100000)<<4)+(regdis%100000/10000);
							 UpReg[Up_wr++]=((regdis%10000/1000)<<4)+(regdis%1000/100);
							 UpReg[Up_wr++]=((regdis%100/10)<<4)+(regdis%10); 
							 

		case  A_Up_PLUS     :    //  �ɼ���¼���ٶ�����ϵ��
                             UpReg[Up_wr++]=In_Type; //������

							 SregLen=0x00;           // ��Ϣ����
                             UpReg[Up_wr++]=0x00;    // Hi
                             UpReg[Up_wr++]=10;      // Lo
                             
                             UpReg[Up_wr++]=0x00;    // ������

							 //  ��Ϣ����
							 Time2BCD(UpReg+Up_wr);
							 Up_wr+=6;
                             UpReg[Up_wr++]=(u8)(JT808Conf_struct.Vech_Character_Value<<24);
							 UpReg[Up_wr++]=(u8)(JT808Conf_struct.Vech_Character_Value<<16);
							 UpReg[Up_wr++]=(u8)(JT808Conf_struct.Vech_Character_Value<<8);
							 UpReg[Up_wr++]=(u8)(JT808Conf_struct.Vech_Character_Value); 
							 
                             break;
		case  A_Up_VechInfo :    //  ������Ϣ
		                     UpReg[Up_wr++]=In_Type; //������

							 SregLen=0x00;           // ��Ϣ����
                             UpReg[Up_wr++]=0x00;    // Hi
                             UpReg[Up_wr++]=41;      // Lo
                             
                             UpReg[Up_wr++]=0x00;    // ������                              
                             
                             memcpy(UpReg+Up_wr,JT808Conf_struct.Vechicle_Info.Vech_VIN,17); //��Ϣ����
                             Up_wr+=17;
							 memcpy(UpReg+Up_wr,JT808Conf_struct.Vechicle_Info.Vech_Num,12);
                             Up_wr+=12;
							 memcpy(UpReg+Up_wr,JT808Conf_struct.Vechicle_Info.Vech_Type,12);  
                             Up_wr+=12;

		                     break;
		case  A_Up_AvrgMin  :      //  ÿ����ƽ���ٶȼ�¼      // Ĭ����д����7���Ӽ�¼
                                       UpReg[Up_wr++]=In_Type; //������
			
									   SregLen=455;		   // ��Ϣ����
									   UpReg[Up_wr++]=(u8)(SregLen>>8);    // Hi
									   UpReg[Up_wr++]=(u8)SregLen;		   // Lo	65x7    
									   
									   UpReg[Up_wr++]=0x00;    // ������	 
									   //-----------  ��Ϣ����  --------------		
									       //----------------------
										  QueryRecNum=Api_DFdirectory_Query(spdpermin,0);   //��ѯ��ǰƣ�ͼ�ʻ��¼��Ŀ
										 if(QueryRecNum>7)								   
										        QueryRecNum=7;			
								
										    SregLen=QueryRecNum*65;     // ��д��Ϣ����
			                                                      UpReg[Up_wr-3]=(u8)(SregLen>>8);	  // Hi
			                                                      UpReg[Up_wr-2]=(u8)SregLen;	  // Lo    65x7    

																  
										 for(i=0;i<QueryRecNum;i++)			   // �����´���ȡ�洢��д
										  {
											 Api_DFdirectory_Read(spdpermin,Reg,70,0,i); // ��new-->old  ��ȡ
											  memcpy(UpReg+Up_wr,Reg+5,60);	// ֻ��д�ٶ�
											Up_wr+=65;	    
										  }
									       //------------------------------
									   
		                     break; 
		case  A_Up_Tired    :     //  ƣ�ͼ�ʻ��¼
		                               UpReg[Up_wr++]=In_Type; //������
			
									   SregLen=180;		   // ��Ϣ����
									   UpReg[Up_wr++]=(u8)(SregLen>>8);    // Hi
									   UpReg[Up_wr++]=(u8)SregLen;		   // Lo	30x6   
									   
									   UpReg[Up_wr++]=0x00;    // ������	   


									   //----------------------------------									    
                                                                   	QueryRecNum=Api_DFdirectory_Query(tired_warn,0);   //��ѯ��ǰƣ�ͼ�ʻ��¼��Ŀ
										 if(QueryRecNum>6)								   
										        QueryRecNum=6;		
										 
                                                                  SregLen=30*QueryRecNum;		   // ��Ϣ����
									   UpReg[Up_wr-3]=(u8)(SregLen>>8);    // Hi
									   UpReg[Up_wr-2]=(u8)SregLen;		   // Lo	65x7  
										 
										 for(i=0;i<QueryRecNum;i++)			   // �����´���ȡ�洢��д
										  {
											 Api_DFdirectory_Read(tired_warn,Reg,31,0,i); // ��new-->old  ��ȡ
                                                                              memcpy(UpReg+Up_wr,Reg,30); 
											 Up_wr+=30; 	    
										  }
									  //----------------------------------	 
                                                                    

		                     break;
		//------------------------ �´������������ ---------------------
	    case  A_Dn_DrvInfo  :       //  ���ó�����Ϣ
                              
							  //-----------------------------------------------------------------------
							/*  memset(JT808Conf_struct.Vechicle_Info.Vech_VIN,0,18);
							  memset(JT808Conf_struct.Vechicle_Info.Vech_Num,0,13);
							  memset(JT808Conf_struct.Vechicle_Info.Vech_Type,0,13); 
							  
							  memcpy(JT808Conf_struct.Vechicle_Info.Vech_VIN,InStr,17);
							  memcpy(JT808Conf_struct.Vechicle_Info.Vech_Num,InStr+17,12);
							  memcpy(JT808Conf_struct.Vechicle_Info.Vech_Type,InStr+29,12);  
							  
							  Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));  
							  */
		                    // break;
                           
		case  A_Dn_RTC      :        //  ���ü�¼��ʱ��
		                    
		                     UpReg[Up_wr++]=In_Type; //������

							                    // ��Ϣ����
                             UpReg[Up_wr++]=0x00;    // Hi
                             UpReg[Up_wr++]=0;      // Lo   20x8
                             
                             UpReg[Up_wr++]=0x00;    // ������  

		                     break;
		case  A_Dn_Plus     :        //  �����ٶ�����ϵ��
                             
						//	 JT808Conf_struct.Vech_Character_Value=((u32)(*InStr)<<24)+((u32)(*InStr+1)<<16)+((u32)(*InStr+2)<<8)+(u32)(*InStr+3); // ����ϵ��	�ٶ�����ϵ��
						//	 Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));  
                             //-------------------------------------------------------------  
							 UpReg[Up_wr++]=In_Type; //������
							                    // ��Ϣ����
                             UpReg[Up_wr++]=0x00;    // Hi
                             UpReg[Up_wr++]=0;      // Lo   20x8                             
                             UpReg[Up_wr++]=0x00;    // ������   
		                     break;
		default :
                             rt_kprintf("Error:   Device Type Error! \r\n");
			                 return; 
							 
  	}
  //---------------  ��д���� A Э��  Serial Data   У��λ  -------------------------------------
  Sfcs=0;						//	����SУ�� ��OxAA ��ʼ 
  for(i=Swr;i<Up_wr;i++)
	 Sfcs^=UpReg[i];
  UpReg[Up_wr++]=Sfcs;			 // ��дFCS  
  //-----------------------------------------------------------------------------
  if(TransType)  //-------ͨ��  GPRS  
  {
    
    //----stuff Ginfolen ----
    UpReg[Greglen]=(u8)((Up_wr-2)<<8);  //���Ȳ���ͷ 
	UpReg[Greglen+1]=(u8)(Up_wr-2);   
	
    Gfcs=0;                 //  ����ӵ绰���뿪ʼ��У��ǰ���ݵ�����  G Э��У�� 
	for(i=3;i<Up_wr;i++)
		Gfcs^=UpReg[i];
	
    UpReg[Up_wr++]=Gfcs;  // ��дGУ��λ

	
    UpReg[Up_wr++]=0x0D;  // G Э��β
	UpReg[Up_wr++]=0x0A;
    //=================================================================================================
    //---------ͨ�� GPRS ����  --------------    
	GPRS_infoWr_Tx=0;	//--------------  clear  --------- 
	//-----------------------------------------------------------------------
	memcpy(GPRS_info+GPRS_infoWr_Tx,UpReg,Up_wr); // �������ݵ����ͻ�����
	GPRS_infoWr_Tx+=Up_wr; 	
	//-----------  Add for Debug ---------------------------------
#if  0   	
	 memset(UDP_AsciiTx,0,sizeof(UDP_AsciiTx)); 	 
	 strcat((char*)UDP_AsciiTx,"AT%IPSENDX=1,\"");
	 packet_len=14;//strlen((const char*)UDP_AsciiTx);	
	 UDP_AsciiTx_len=HextoAscii(GPRS_info, GPRS_infoWr_Tx,UDP_AsciiTx+packet_len);
	
	 packet_len+=UDP_AsciiTx_len;	 
	 strcat((char*)UDP_AsciiTx,"\"\r\n");  
 
	  �Ժ���˵��
	 if(DispContent==2)
		Uart1_PutData((uint8_t *) UDP_AsciiTx, packet_len+3);		
	 GSM_PutData((uint8_t*)UDP_AsciiTx,packet_len+3);  
	 rt_kprintf("\r\n	SEND DriveRecord -UP-Data! \r\n");  
#endif	
  }  
  else	 
  {                 //-----  ͨ���������
    for(i=0;i<Up_wr;i++)
		rt_kprintf("%c",UpReg[i]); 
  }


} 

//------------------------------------------------------------------
void  Process_GPRSIN_DeviceData(u8 *instr, u16  infolen)
{
   u8  fcs=0;
   u16 i=0;



   //   caculate  and   check fcs
   for(i=0;i<infolen-1;i++)
   	  fcs^=instr[i];
   
   if(fcs!=instr[infolen-1])
   	  return;
   //  classify  cmd             
   switch(instr[2])   // AAH 75H CMD
   	{
   	  //  ����
      case 0x00:  // �ɼ��г���¼��ִ�б�׼�汾��
                 Adata_ACKflag.A_Flag__Up_Ver_00H=0xff;
				 break;
	  case 0x01:  // �ɼ���ǰ��ʻ����Ϣ
	             Adata_ACKflag.A_Flag_Up_DrvInfo_01H=instr[2];
				 break;
	  case 0x02:  // �ɼ���¼�ǵ�ʵʱʱ��
	             Adata_ACKflag.A_Flag_Up_RTC_02H=instr[2];
				 break;
	  case 0x03:  // �ɼ���ʻ���
	             Adata_ACKflag.A_Flag_Up_Dist_03H=instr[2];
				 break;
	  case 0x04:  // �ɼ���¼���ٶ�����ϵ��
	             Adata_ACKflag.A_Flag_Up_PLUS_04H=instr[2];
				 break;
	  case 0x06:  // �ɼ�������Ϣ
	             Adata_ACKflag.A_Flag_Up_VechInfo_06H=instr[2];
				 break;
	  case 0x08:  // �ɼ���¼��״̬�ź�������Ϣ
	             Adata_ACKflag.A_Flag_Up_SetInfo_08H=instr[2];
				 break;
	  case 0x16:  // �ɼ���¼��Ψһ���
	             Adata_ACKflag.A_Flag_Up_DevID_16H=instr[2];
				 break;
	  case 0x09:  // �ɼ�ָ����ÿ����ƽ���ٶȼ�¼            
	             Adata_ACKflag.A_Flag_Up_AvrgSec_09H=instr[2];  // ����ʼ����ʱ��
	  	         break;
	  case 0x05: // �ɼ�ָ����ÿ����ƽ���ٶȼ�¼
	             Adata_ACKflag.A_Flag_Up_AvrgMin_05H=instr[2]; // ����ʼ����ʱ��
	             break;
	  case 0x13: // �ɼ�ָ����λ����Ϣ��¼
	             Adata_ACKflag.A_Flag_Up_Posit_13H=instr[2];
				 break;
	  case 0x07: // �ɼ��¹��ɵ��¼
	             Adata_ACKflag.A_Flag_Up_Doubt_07H=instr[2];  // ����ʼ����ʱ��
	             break;
	  case 0x11: // �ɼ�ָ����ƣ�ͼ�ʻ��¼
	             Adata_ACKflag.A_Flag_Up_Tired_11H=instr[2]; // ����ʼ����ʱ��
				 break;
	  case 0x10: // �ɼ�ָ���ĵ�¼�˳���¼
	             Adata_ACKflag.A_Flag_Up_LogIn_10H=instr[2]; // ����ʼ����ʱ��
	             break;
	  case 0x14: // �ɼ�ָ���ļ�¼���ⲿ�����¼
	            Adata_ACKflag.A_Flag_Up_Powercut_14H=instr[2]; // ����ʼ����ʱ��
				break;
	  case 0x15: // �ɼ�ָ���ļ�¼�ǲ����޸ļ�¼
	            Adata_ACKflag.A_Flag_Up_SetMdfy_15H=instr[2];
				break;
	  //  ����
	  case 0x82: // ���ó�����Ϣ                 
				    memset(JT808Conf_struct.Vechicle_Info.Vech_VIN,0,sizeof(JT808Conf_struct.Vechicle_Info.Vech_VIN));
				    memset(JT808Conf_struct.Vechicle_Info.Vech_Num,0,sizeof(JT808Conf_struct.Vechicle_Info.Vech_Num));
				    memset(JT808Conf_struct.Vechicle_Info.Vech_Type,0,sizeof(JT808Conf_struct.Vechicle_Info.Vech_Type)); 	
                 
					 //-----------------------------------------------------------------------
					 memcpy(JT808Conf_struct.Vechicle_Info.Vech_VIN,instr,17);
					 memcpy(JT808Conf_struct.Vechicle_Info.Vech_Num,instr+17,12);
					 memcpy(JT808Conf_struct.Vechicle_Info.Vech_Type,instr+29,12); 
					 
			
				 	Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   

                 Adata_ACKflag.A_Flag_Dn_DrvInfo_82H=instr[2];
				    
					Settingchg_Status=0x82;//���ó�����Ϣ 					
					NandsaveFlg.Setting_SaveFlag=1;  //�洢�����޸ļ�¼
	  case 0x83:  // ���ó��ΰ�װ����
                             memcpy(JT808Conf_struct.FirstSetupDate,instr,6); 
				  Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
                 Adata_ACKflag.A_Flag_Dn_SetupDate_83H=instr[2];
				 Settingchg_Status=0x83;//���ó�����Ϣ 				 
				 NandsaveFlg.Setting_SaveFlag=1;  //�洢�����޸ļ�¼
				 break;
	  case 0x84:  // ����״̬����Ϣ	
	             Settingchg_Status=0x84;//���ó�����Ϣ 	             
				 NandsaveFlg.Setting_SaveFlag=1;  //�洢�����޸ļ�¼
	             break;
	  case 0xc2: // ���ü�¼��ʱ��
                 Adata_ACKflag.A_Flag_Dn_RTC_C2H=instr[2];
				 Settingchg_Status=0xc2;//���ó�����Ϣ 				 
				 NandsaveFlg.Setting_SaveFlag=1;  //�洢�����޸ļ�¼
				 break;
	  case 0xc3: // ���ü�¼���ٶ�����ϵ��                 
				  JT808Conf_struct.Vech_Character_Value=(u32)(instr[0]<<24)+(u32)(instr[1]<<16)+(u32)(instr[2]<<8)+(u32)(instr[3]); // ����ϵ��	�ٶ�����ϵ��
				  Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
		         Adata_ACKflag.A_Flag_Dn_Plus_C3H=instr[2];   
				 Settingchg_Status=0xc3;//���ó�����Ϣ  				 
				 NandsaveFlg.Setting_SaveFlag=1;  //�洢�����޸ļ�¼
				 break;
	  default:

		         break;
				 
				 	


   	}
}

u8 Send_Device_Data(void)  //  ���г���¼�Ƿ�������
{
  u8  j=0,i=0,reg=0;;

   j=sizeof(Adata_ACKflag);
  for(i=0;i<j;i++)
  	{
  	    reg=(*((u8*)&Adata_ACKflag+i)); 
        if(reg)
        {
		    if(reg!=0xff)
			{
			  Device_Data(reg,1);
		    }
			else
			   Device_Data(0x00,1);	   // �ɼ��г���¼��ִ�б�׼�汾�� ����Ϊ0x00 ��λʱ��д0xFF,���Ե�������
            (*((u8*)&Adata_ACKflag+i))=0;  // clear flag 
            return true;
		}  
		
  	}
   return false; 

}


#if 0

u8 RecordSerial_output_Str(const char *fmt,...) 
{
    u8 regstr[100],fcs=0;
	u16 reglen=0,i=0;
	
	va_list args;
	va_start(args, fmt);
	  memset(regstr,0,sizeof(regstr));  
	regstr[0]=0x55; // Э��ͷ    
    regstr[1]=0x7A;
	regstr[2]=0xFE; // ������ ����Ԥ�������ֱ�ʾ�������	
	//  3,4 Ϊ�����ֽ������д
	regstr[5]=0x00; // ������  0x00

	reglen= vsprintf((char*)regstr+6, fmt, args);
	va_end(args); 
	regstr[3]=(u8)(reglen>>8);  // ��д����  ������Ϊ��Ϣ���ݵĳ��� 
	regstr[4]=(u8)reglen;   
	
	reglen+=6;
	fcs=0;	
	for(i=0;i<reglen;i++)
		fcs^=regstr[i];
	regstr[reglen]=fcs;
	reglen++; 
	for(i=0;i<reglen;i++)
	     rt_kprintf("%c",regstr[i]);  
	
	return 1;
}

#endif

void SpeedWarnJudge(void)  //  �ٶȱ����ж� 
{
  					 //--------  �ٶȱ���  -------	 
					 if(  JT808Conf_struct.Speed_warn_MAX >0 )   //> 0 
					 {
			            
						 //----- GPS  ��ʱ�ٶ�	0.1 km/h  ---------
							 if  ( GPS_speed>( JT808Conf_struct.Speed_warn_MAX*10) )							 	
							// if( DebugSpd > ( JT808Conf_struct.Speed_warn_MAX*10) ) 
							 {
											 speed_Exd.dur_seconds++;
											 if ( speed_Exd.dur_seconds >JT808Conf_struct.Spd_Exd_LimitSeconds)   
											 {
													 speed_Exd.dur_seconds = 0;  
													 if ( speed_Exd.speed_flag!= 1 )
													 {				   
														 speed_Exd.speed_flag = 1;			 
														 //PositionsSD_Enable();	  //  �ر�GPS ��Ϣ	 

														 StatusReg_SPD_WARN(); //  ���ٱ���״̬
														 rt_kprintf("\r\n  ���ٱ���\r\n"); 
													 } 
													 //---------------------------------------------
													 Time2BCD(speed_Exd.ex_startTime); //��¼���ٱ�����ʼʱ��
													 if(speed_Exd.current_maxSpd<GPS_speed) //������ٶ�
														  speed_Exd.current_maxSpd=GPS_speed;
													 speed_Exd.excd_status=1;
													 speed_Exd.dur_seconds++;		
			 
												   //---------------------------------------------- 
											 }
											 
											 if(speed_Exd.excd_status==1) // ʹ��flag ��ʼ��ʱ 
											  {
												 speed_Exd.dur_seconds++; 
												 if(speed_Exd.current_maxSpd<GPS_speed) //������ٶ�
														  speed_Exd.current_maxSpd=GPS_speed;
											   } 
									 
							 }
							 else
							 { 
							              StatusReg_SPD_NORMAL(); //  ����ٶȱ���״̬�Ĵ���  
										  
										  if(speed_Exd.excd_status!=2)
										  {
										    StatusReg_SPD_NORMAL(); //  ����ٶȱ���״̬�Ĵ���
											speed_Exd.dur_seconds = 0;
											speed_Exd.speed_flag = 0; 	
										  } 											   
											//----------------------------------------------
											if(speed_Exd.excd_status==1)  
		                                    {		
											    Time2BCD(speed_Exd.ex_endTime); //��¼���ٱ�������ʱ�� 
												speed_Exd.excd_status=2;
							 	            }
											else
											if(speed_Exd.excd_status==0)
												Spd_ExpInit();
											//---------------------------------------------- 
							 }
					  }//------- �ٶȱ��� over	 ---


}


u16  Protocol_808_Encode(u8 *Dest,u8 *Src, u16 srclen)
{
  u16  lencnt=0,destcnt=0;

  for(lencnt=0;lencnt<srclen;lencnt++)
  {
     if(Src[lencnt]==0x7e)  // 7e ת��
     {
       Dest[destcnt++]=0x7d;
	   Dest[destcnt++]=0x02;
     }
     else
	 if(Src[lencnt]==0x7d) //  7d  ת�� 	
	 {
       Dest[destcnt++]=0x7d;
	   Dest[destcnt++]=0x01; 
	 }
	 else
        Dest[destcnt++]=Src[lencnt]; // ԭʼ��Ϣ
  }

  return destcnt; //����ת���ĳ���

}
//-------------------------------------------------------------------------------
void Protocol_808_Decode(void)  // ����ָ��buffer :  UDP_HEX_Rx  
{
	//-----------------------------------
	  u16 i=0;

    // 1.  clear  write_counter
	  UDP_DecodeHex_Len=0;//clear DecodeLen

	// 2   decode process   
	for(i=0;i<UDP_hexRx_len;i++)
	 {
		if((UDP_HEX_Rx[i]==0x7d)&&(UDP_HEX_Rx[i+1]==0x02))
		{
		   UDP_HEX_Rx[UDP_DecodeHex_Len]=0x7e;
		   i++;
		}
		else
		if((UDP_HEX_Rx[i]==0x7d)&&(UDP_HEX_Rx[i+1]==0x01))
		{
		   UDP_HEX_Rx[UDP_DecodeHex_Len]=0x7d;
		   i++;
		}
		else  
		{
		  UDP_HEX_Rx[UDP_DecodeHex_Len]=UDP_HEX_Rx[i];  
		}
	    UDP_DecodeHex_Len++;
	 }	
    //  3.  The  End
}
//---------  �յ㲹�����Գ���  ---------------------------
//#if 0
/*
void Inflexion_Process(void)
{            //
  u16  once_delta=0;


	Inflexion_Current=GPS_direction;  //  update new
   //----------------------------------------------------------------------- 
    if(Inflexion_Current>Inflexion_Bak)   // �����жϴ�С  
    	{  // ����
            if((Inflexion_Current-Inflexion_Bak)>300)  // �ж��Ƿ��ü�С
			{   //  �����ֵ����300 ��˵����С��
			     once_delta=Inflexion_Bak+360-Inflexion_Current;  //�жϲ�ֵ����ֵ
			     InflexDelta_Accumulate+=once_delta;
				 if((once_delta>=15)&&(once_delta<=60)) // �Ƕ���С�仯�ʲ���С��15��  �յ㲹���ǶȲ�����180 Ҫ������3s  ����ÿ�벻����60
				 {
					    if((Inflexion_chgcnter>0)&&(InflexLarge_or_Small==2))  //�ж�֮ǰ�Ƿ�һֱ��С��
					    	{
		                        Inflexion_chgcnter++;
								if((InflexDelta_Accumulate>=3)&&(InflexDelta_Accumulate>JT808Conf_struct.DURATION.SD_Delta_maxAngle)&&(InflexDelta_Accumulate<=180))
									{     //Ҫ�����ٳ���3s	�ۼƹսǲ����ǶȲ��ô���180
                                           InflexLarge_or_Small=0;
										   Inflexion_chgcnter=0;
										   InflexDelta_Accumulate=0;
										   PositionSD_Enable=1; // ���͹յ��־λ
										   rt_kprintf("\r\n �յ��ϱ� --1\r\n");
									}
								else
								  InflexLarge_or_Small=2; // �����С��
					    	}
						else
							{
							   InflexLarge_or_Small=2;  // ���ǵ�һ��С�� 
							   Inflexion_chgcnter=1;
							   InflexDelta_Accumulate=once_delta;
							}
				 }
				 else
				 {    //  С�� 15 �������
				   InflexLarge_or_Small=0;
				   Inflexion_chgcnter=0;
				   InflexDelta_Accumulate=0;
				 }
				 	
            } 
			else		// current���������ı�Bak ��
			{  
			   once_delta=Inflexion_Current-Inflexion_Bak;  //�жϲ�ֵ����ֵ
			   InflexDelta_Accumulate+=once_delta;
			   if((once_delta>=15)&&(once_delta<=60)) // �Ƕ���С�仯�ʲ���С��15��  �յ㲹���ǶȲ�����180 Ҫ������3s  ����ÿ�벻����60
			   {
	               if((Inflexion_chgcnter>0)&&(InflexLarge_or_Small==1)) //�ж�֮ǰ�Ƿ�һֱ����
				   {
				       Inflexion_chgcnter++;					   
					   if((InflexDelta_Accumulate>=3)&&(InflexDelta_Accumulate>JT808Conf_struct.DURATION.SD_Delta_maxAngle)&&(InflexDelta_Accumulate<=180))
						   {	 //Ҫ�����ٳ���3s	�ۼƹսǲ����ǶȲ��ô���180
								  InflexLarge_or_Small=0;
								  Inflexion_chgcnter=0;
								  InflexDelta_Accumulate=0;
								  PositionSD_Enable(); // ���͹յ��־λ
								  rt_kprintf("\r\n �յ��ϱ� --2\r\n");
						   }
					   else
					     InflexLarge_or_Small=1; // ����Ǵ���
	               }	
				   else
				   	{  
                       InflexLarge_or_Small=1;  // ���ǵ�һ�δ���
					   Inflexion_chgcnter=1;
					   InflexDelta_Accumulate=once_delta; 
				   	}
			   }
			    else
				 {     // С��15�Ⱦ������
				   InflexLarge_or_Small=0;
				   Inflexion_chgcnter=0;
				   InflexDelta_Accumulate=0;
				 }

			}  
    	}
	else
	 if(Inflexion_Current<Inflexion_Bak)	
	 	{  // ��С
               if((Inflexion_Bak-Inflexion_Current)>300)  // �ж��Ƿ�������
               { //  �����ֵ����300 ��˵���Ǵ���
                  once_delta=Inflexion_Current+360-Inflexion_Bak;  //�жϲ�ֵ����ֵ
			      InflexDelta_Accumulate+=once_delta;
				  if((once_delta>=15)&&(once_delta<=60)) // �Ƕ���С�仯�ʲ���С��15��	�յ㲹���ǶȲ�����180 Ҫ������3s  ����ÿ�벻����60
                  {   // ��С�仯�� ��С�� 15
                     if((Inflexion_chgcnter>0)&&(InflexLarge_or_Small==1))  //�ж�֮ǰ�Ƿ�һֱ�Ǵ���
					    	{
		                        Inflexion_chgcnter++;
								if((InflexDelta_Accumulate>=3)&&(InflexDelta_Accumulate>JT808Conf_struct.DURATION.SD_Delta_maxAngle)&&(InflexDelta_Accumulate<=180))
									{     //Ҫ�����ٳ���3s	�ۼƹսǲ����ǶȲ��ô���180
                                           InflexLarge_or_Small=0;
										   Inflexion_chgcnter=0;
										   InflexDelta_Accumulate=0;
										   PositionSD_Enable(); // ���͹յ��־λ
										   rt_kprintf("\r\n �յ��ϱ� --3\r\n");
									}
								else
								  InflexLarge_or_Small=1; // ����Ǵ���
					    	}
						else
							{
							   InflexLarge_or_Small=1;  // ���ǵ�һ�δ��� 
							   Inflexion_chgcnter=1;
							   InflexDelta_Accumulate=once_delta;
							}

                  }
				  else
				  {    //  С�� 15 �������
				   InflexLarge_or_Small=0;
				   Inflexion_chgcnter=0;
				   InflexDelta_Accumulate=0;
				  }
			  }//---------------------------
			   else 	   // current ���������ı�Bak С
			   {
				  once_delta=Inflexion_Bak-Inflexion_Current;  //�жϲ�ֵ����ֵ
				  InflexDelta_Accumulate+=once_delta;
				  if((once_delta>=15)&&(once_delta<=60)) // �Ƕ���С�仯�ʲ���С��15��	�յ㲹���ǶȲ�����180 Ҫ������3s  ����ÿ�벻����60
				  {
					  if((Inflexion_chgcnter>0)&&(InflexLarge_or_Small==2)) //�ж�֮ǰ�Ƿ�һֱС��
					  {
						  Inflexion_chgcnter++; 					  
						  if((InflexDelta_Accumulate>=3)&&(InflexDelta_Accumulate>JT808Conf_struct.DURATION.SD_Delta_maxAngle)&&(InflexDelta_Accumulate<=180))
							  { 	//Ҫ�����ٳ���3s	�ۼƹսǲ����ǶȲ��ô���180
									 InflexLarge_or_Small=0;
									 Inflexion_chgcnter=0;
									 InflexDelta_Accumulate=0;
									 PositionSD_Enable(); // ���͹յ��־λ
									 rt_kprintf("\r\n �յ��ϱ� --4\r\n"); 
							  }
						  else
						   InflexLarge_or_Small=2; // �����С��
					  }    
					  else
					   {  
						  InflexLarge_or_Small=2;  // ���ǵ�һ��С��
						  Inflexion_chgcnter=1;
						  InflexDelta_Accumulate=once_delta; 
					   }
				  }
				   else
					{	  // С��15�Ⱦ������
					  InflexLarge_or_Small=0;
					  Inflexion_chgcnter=0;
					  InflexDelta_Accumulate=0;
					}
			   
			   }
			  	
	

	 	}
	else
		{
		  InflexLarge_or_Small=0;
		  Inflexion_chgcnter=0;
		  InflexDelta_Accumulate=0;
		}  

    //--------------------------------------------------------
    Inflexion_Bak=Inflexion_Current; //  throw  old	 to  Bak

}*/
//#endif 

void  Sleep_Mode_ConfigEnter(void)
{
   if(SleepState==0)
		{
		   	  SleepCounter++;
			  if(SleepCounter>15)   // ��������
			 {
			      SleepCounter=0; 
			   	   SleepState=1;		
				   if(JT808Conf_struct.RT_LOCK.Lock_state!=1)
				   Current_SD_Duration=JT808Conf_struct.DURATION.Sleep_Dur; // 5���� 
				   //JT808Conf_struct.DURATION.Heart_SDCnter=25; 
				   //JT808Conf_struct.DURATION.Heart_Dur=320; 
				  /*   ���ͨҪ�����߲����¼�ʱ  ����  
				    memcpy(BakTime,CurrentTime,3); // update  //gps��ʼ��ʱ 
				    systemTick_trigGPS_counter=0; // ϵͳ��ʱ ���
				  */  
				   if(DataLink_Status())
	                                PositionSD_Enable();  //  ���߾ͷ���    �ӱ����ͨҪ���������� 
				    rt_kprintf("\r\n ��������״̬! \r\n");          
			 }  
		 }
}

void  Sleep_Mode_ConfigExit(void)
{
    if(SleepState==1)
		   	{
		   	   SleepState=0;
			   rt_kprintf("\r\n ��̨����! \r\n");      
		   	}
   if(JT808Conf_struct.RT_LOCK.Lock_state!=1)
       Current_SD_Duration=JT808Conf_struct.DURATION.Default_Dur; 
   JT808Conf_struct.DURATION.Heart_Dur=300;  
   SleepState=0;
   SleepCounter=0;
}

#if 0
u16 WaveFile_EncodeHeader(u32 inFilesize ,u8* DestStr)
{
   u32 Filesize=0,i=0;     // Header Len  =44Bytes  

    
	//  1. RIFF
    memcpy(DestStr,"RIFF",4); 
	i+=4;
	//  2. Wave �ļ� ��С  С��ģʽ 
	Filesize=0x24+(inFilesize<<3); // ����16 �� 36 wave �ļ���С 
	rt_kprintf("\r\n .wav �ļ���С: %d Rawdata: %d \r\n ",Filesize,(inFilesize<<3)); 
	DestStr[i++]=Filesize; // LL
	DestStr[i++]=(Filesize>>8);//LH
	DestStr[i++]=(Filesize>>16);//HL
	DestStr[i++]=(Filesize>>24);//HH
	//  3. WAVE string	
    memcpy(DestStr+i,"WAVE",4);
	i+=4;
	//  4. fmt string	
    memcpy(DestStr+i,"fmt ",4);
	i+=4;
    //  5. PCM Code    
	DestStr[i++]=0x10; // LL
	DestStr[i++]=0x00;//LH
	DestStr[i++]=0x00;//HL
	DestStr[i++]=0x00;//HH   
	//  6. Audio Format  PCM=1	
	DestStr[i++]=0x01; // L
	DestStr[i++]=0x00;//H
	//  7. NumChannels  ͨ����
	DestStr[i++]=0x01; // L
	DestStr[i++]=0x00;//H
	//  8. SampleRate     8000<=>0x00001F40    16000<=>0x00003E80      
	DestStr[i++]=0x40;//0x40; // LL
	DestStr[i++]=0x1F;//0x1F;//LH
	DestStr[i++]=0x00;//HL
	DestStr[i++]=0x00;//HH    
    //  9.ByteRate       == SampleRate * NumChannels * BitsPerSample/8  ==8000x1x8/8 
	DestStr[i++]=0x40;//0x40; // LL
	DestStr[i++]=0x1F;//0x1F;//LH
	DestStr[i++]=0x00;//HL
	DestStr[i++]=0x00;//HH   

	// 10.BlockAlign   	== NumChannels * BitsPerSample/8
	DestStr[i++]=0x01;//0x02;//0x01; // L 
	DestStr[i++]=0x00;//H
	// 11.BitsPerSample
	DestStr[i++]=0x08;//0x10;//0x08; // L
	DestStr[i++]=0x00;//H
    // 12.data string	
    memcpy(DestStr+i,"data",4);
	i+=4;
	// 13 .datasize
	Filesize=(inFilesize<<3); // ����16 �� 36 wave �ļ���С 
	DestStr[i++]=Filesize; // LL
	DestStr[i++]=(Filesize>>8);//LH
	DestStr[i++]=(Filesize>>16);//HL
	DestStr[i++]=(Filesize>>24);//HH

    return i;
}
#endif

//-----------  starttime[6]
u8 CurrentTime_Judge( u8*startTime ,u8* endTime)
{
   u32 daycaculate_current=0,daycaculate_start=0,daycaculate_end=0;
   u32 secondcaculate_current=0,secondcaculate_start=0,secondcaculate_end=0;
  

   daycaculate_start=((startTime[0]>>4)*10+(startTime[0]&0x0f))*365+((startTime[1]>>4)*10+(startTime[1]&0x0f))*30+((startTime[2]>>4)*10+(startTime[2]&0x0f));
   secondcaculate_start=((startTime[3]>>4)*10+(startTime[3]&0x0f))*60+((startTime[4]>>4)*10+(startTime[4]&0x0f))*60+((startTime[5]>>4)*10+(startTime[5]&0x0f)); 

   
   daycaculate_end=((endTime[0]>>4)*10+(endTime[0]&0x0f))*365+((endTime[1]>>4)*10+(endTime[1]&0x0f))*30+((endTime[2]>>4)*10+(endTime[2]&0x0f));
   secondcaculate_end=((endTime[3]>>4)*10+(endTime[3]&0x0f))*60+((endTime[4]>>4)*10+(endTime[4]&0x0f))*60+((endTime[5]>>4)*10+(endTime[5]&0x0f)); 

   daycaculate_current=(time_now.year)*365+time_now.month*30+time_now.day;
   secondcaculate_current=time_now.hour*60+time_now.min*60+time_now.sec;

   if((daycaculate_current>daycaculate_start)&&(daycaculate_current<daycaculate_end))
   {
        return  true;
   }
   else
   if((secondcaculate_current>=secondcaculate_start)&&(secondcaculate_current<=secondcaculate_end))
   	{     
        return true;
   	}
   else   
   	    return false;

   

}

void CycleRail_Judge(u8* LatiStr,u8* LongiStr) 
{
   /*
       γ��û�в�ֵ    1γ��  111km
       40��γ���� 1����Ϊ  85.3km   (��������)
   */
   u8 i=0;
   u32 Latitude=0,Longitude=0;
   u32 DeltaLatiDis=0,DeltaLongiDis=0,CacuDist=0;
   u8  InOutState=0;   //   0 ��ʾ in   1  ��ʾOut

   //  1. get value
      Latitude=(LatiStr[0]<<24)+(LatiStr[1]<<16)+(LatiStr[2]<<8)+LatiStr[3];
      Longitude=(LongiStr[0]<<24)+(LongiStr[1]<<16)+(LongiStr[2]<<8)+LongiStr[3]; 


 
  for(i=0;i<8;i++)
  {
         InOutState=0; 
		 memset((u8*)&Rail_Cycle,0,sizeof(Rail_Cycle));
		Api_RecordNum_Read(Rail_cycle,i+1, (u8*)&Rail_Cycle,sizeof(Rail_Cycle)); 	 
		// rt_kprintf("\r\n\r\n Բ��Χ�� ��Ч״̬:%d  TYPE: %d    atrri=%d  lati: %d  longiti:%d  radicus:%d  maxspd: %d  keepdur:%d \r\n",Rail_Cycle.Effective_flag,Rail_Cycle.Area_ID,Rail_Cycle.Area_attribute,Rail_Cycle.Center_Latitude,Rail_Cycle.Center_Longitude,Rail_Cycle.Radius,Rail_Cycle.MaxSpd,Rail_Cycle.KeepDur);  
         

      if(Rail_Cycle.Effective_flag==1) 
      {
         
			 DeltaLatiDis=abs(Latitude-Rail_Cycle.Center_Latitude)/9; //  a/1000000*111000=a/9.009	����һ����õ����� �ٳ��� 111000 �׵õ�ʵ�ʾ��� 
             
			 DeltaLongiDis=abs(Longitude-Rail_Cycle.Center_Longitude)*853/10000; // a/1000000*85300=a 853/10000 m 	  
			 
			 CacuDist=sqrt((DeltaLatiDis*DeltaLatiDis)+(DeltaLongiDis*DeltaLongiDis));  
             
			 rt_kprintf("\r\n  TemperLati  %d  TemperLongi	%d	  Centerlati %d  center longi %d\r\n",Latitude,Longitude,Rail_Cycle.Center_Latitude,Rail_Cycle.Center_Longitude); 
			 rt_kprintf("\r\n  he=%d heng=%d   shu=%d   juli=%d\r\n",abs(Longitude-Rail_Cycle.Center_Longitude),DeltaLongiDis,DeltaLatiDis,CacuDist);   
			 
		   if(DeltaLatiDis>Rail_Cycle.Radius ) 
		   {  // ���γ�Ⱦ������ �뾶�϶���
		     InOutState=1;
		   }	 
		    else
		   {
			   DeltaLongiDis=abs(Longitude-Rail_Cycle.Center_Longitude)*853/10000; // a/1000000*85300=a 853/10000 m		
			   if(DeltaLongiDis>Rail_Cycle.Radius )	 
			   	{  // ������Ⱦ�����ڰ뾶�϶���
                   InOutState=1;
			   	}
			   else  //  �������������
			    CacuDist=sqrt((DeltaLatiDis*DeltaLatiDis)+(DeltaLongiDis*DeltaLongiDis));
		    } 


		     // 1. �ж�����
		     if(Rail_Cycle.Area_attribute &0x0001) //Bit 0 ����ʱ��
		     {
                if(CurrentTime_Judge(Rail_Cycle.StartTimeBCD,Rail_Cycle.EndTimeBCD)==false)
				{
				  rt_kprintf("\r\n ʱ��û�������� \r\n");
				  return;
                }  
			   //continue;
		     }
			 if(Rail_Cycle.Area_attribute &0x0002) //Bit 1 ����
			 {
                if(GPS_speed>Rail_Cycle.MaxSpd) 
                	{
                          StatusReg_SPD_WARN(); //  ���ٱ���״̬
						  rt_kprintf("\r\n  �趨Χ�����ٱ���\r\n"); 
                	}
				else
					 StatusReg_SPD_NORMAL();
	           //continue;
			 }
	         if(Rail_Cycle.Area_attribute &0x0004) //Bit 2 �����򱨾�����ʻԱ
	         {


	           //continue;
	         }
			 if(Rail_Cycle.Area_attribute &0x0008) //Bit 3 �����򱨾���ƽ̨ 
			 {
               if((InOutState==0)&&(CacuDist<Rail_Cycle.Radius )&&(Rail_Cycle.MaxSpd>(Speed_gps/10)))
               {
                  Warn_Status[1]|=0x10;// �������򱨾�
                  InOut_Object.TYPE=1;//Բ������
                  InOut_Object.ID=i; //  ID
                  InOut_Object.InOutState=0;//  ������	
                  rt_kprintf("\r\n -----Բ�ε���Χ��--�뱨��");   
			      break;
               }			 
	           //continue;
			 }
			  if(Rail_Cycle.Area_attribute &0x0010) //Bit 4 �����򱨾���˾��
			 {
	           ;   
	           //continue;
			 }
			 if((Rail_Cycle.Area_attribute &0x0020)&&(Rail_Cycle.MaxSpd>(Speed_gps/10))) //Bit 5 �����򱨾���ƽ̨ 
			 {
			   if((InOutState==1)||(CacuDist>Rail_Cycle.Radius )) 
			   	{
			   	  Warn_Status[1]|=0x10;// �������򱨾�
                  InOut_Object.TYPE=1;//Բ������
                  InOut_Object.ID=i; //  ID
                  InOut_Object.InOutState=1;//  ������ 
                  rt_kprintf("\r\n -----Բ�ε���Χ��--������");        
			      break;
			   	}
						 
	           //continue;
			 } 
      	}

  }



}

void RectangleRail_Judge(u8* LatiStr,u8* LongiStr)
{  
	u8 i=0;
	u32 Latitude=0,Longitude=0;
//	u32 DeltaLatiDis=0,DeltaLongiDis=0,CacuDist=0;
	u8	InOutState=1;	//	 0 ��ʾ in	 1	��ʾOut
	
  
  //  1. get value
	 Latitude=(LatiStr[0]<<24)+(LatiStr[1]<<16)+(LatiStr[2]<<8)+LatiStr[3];
	 Longitude=(LongiStr[0]<<24)+(LongiStr[1]<<16)+(LongiStr[2]<<8)+LongiStr[3]; 

	 
	// rt_kprintf("\r\n  1---TemperLati  %d  TemperLongi	%d	 res %d\r\n",Latitude,Longitude,InOutState);  

	for(i=0;i<8;i++)
	{
	    InOutState=1;
		 Api_RecordNum_Read(Rail_rect,i+1, (u8*)&Rail_Rectangle, sizeof(Rail_Rectangle));		  
        
		if(Rail_Rectangle.Effective_flag==1) 
		{
		 
		//  rt_kprintf("\r\n\r\n �жϾ�����Χ�� ��Ч:%d ID: %d  atrri=%X  leftlati: %d  leftlongiti:%d    rightLati:%d   rightLongitu: %d	\r\n",Rail_Rectangle.Effective_flag,i+1,Rail_Rectangle.Area_attribute,Rail_Rectangle.LeftUp_Latitude,Rail_Rectangle.LeftUp_Longitude,Rail_Rectangle.RightDown_Latitude,Rail_Rectangle.RightDown_Longitude);	
            if((Latitude>Rail_Rectangle.RightDown_Latitude)&&(Latitude<Rail_Rectangle.LeftUp_Latitude)&&(Longitude>Rail_Rectangle.LeftUp_Longitude)&&(Longitude<Rail_Rectangle.RightDown_Longitude))
		       InOutState=0;
            
			//rt_kprintf("\r\n  TemperLati  %d  TemperLongi  %d   res %d\r\n",Latitude,Longitude,InOutState); 
		     
			// 1. �ж�����
			if(Rail_Rectangle.Area_attribute &0x0001) //Bit 0 ����ʱ��
			{
			
			  //continue;
			}
			if(Rail_Rectangle.Area_attribute &0x0002) //Bit 1 ����
			{
			
			  //continue;
			}
			if(Rail_Rectangle.Area_attribute &0x0004) //Bit 2 �����򱨾�����ʻԱ
			{
			
			
			  //continue;
			}
			if(Rail_Rectangle.Area_attribute &0x0008) //Bit 3 �����򱨾���ƽ̨ 
			{
			  if(InOutState==0)
			  {
				 Warn_Status[1]|=0x10;// �������򱨾�
				 InOut_Object.TYPE=2;//��������
				 InOut_Object.ID=i; //	ID
				 InOut_Object.InOutState=0;//  ������		
				 rt_kprintf("\r\n -----���ε���Χ��--�뱨��"); 
				 break;
			  } 			
			  //continue;
			}
			 if(Rail_Rectangle.Area_attribute &0x0010) //Bit 4 �����򱨾���˾��
			{
			
			
			 // continue;
			}
			if(Rail_Rectangle.Area_attribute &0x0020) //Bit 5 �����򱨾���ƽ̨ 
			{
			  if(InOutState==1)
			   {
				 Warn_Status[1]|=0x10;// �������򱨾�
				 InOut_Object.TYPE=2;//��������
				 InOut_Object.ID=i; //	ID
				 InOut_Object.InOutState=1;//  ������		
				 rt_kprintf("\r\n -----���ε���Χ��--������"); 
				 break;
			   }
						
			 // continue;  
			} 

 
		}
		
	} 




}

void RouteRail_Judge(u8* LatiStr,u8* LongiStr)  
{
   /*
       γ��û�в�ֵ    1γ��  111km
       40��γ���� 1����Ϊ  85.3km   (��������)
   */
     u8 i=0;
      u8 route_cout=0, seg_count=0,seg_num=0; 
      u32 Latitude=0,Longitude=0;
    // u32 DeltaLatiDis=0,DeltaLongiDis=0,CacuDist=0;
    // u8  InOutState=0;   //   0 ��ʾ in   1  ��ʾOut
     u32  Route_Status=0;   // ÿ��bit ��ʾ һ��·�� ƫ��״̬Ĭ��Ϊ0
     u32  Segment_Status=0;  //  ��ǰ��·�У���Ӧ�˵�ƫ������� Ĭ��Ϊ0
     u32  Distance=0;
//     u8    InAreaJudge=0; //  �ж��Ƿ����ж����� bit 0 ���ȷ�Χ bit  1 γ�ȷ�Χ
     u32  Distance_Array[6]; //�洢������·����С���룬Ĭ���Ǹ�����ֵ

       //  1. get value
	 Latitude=(LatiStr[0]<<24)+(LatiStr[1]<<16)+(LatiStr[2]<<8)+LatiStr[3];
	 Longitude=(LongiStr[0]<<24)+(LongiStr[1]<<16)+(LongiStr[2]<<8)+LongiStr[3]; 

	 // rt_kprintf("\r\n ��ǰ---->  Latitude:   %d     Longitude: %d\r\n",Latitude,Longitude); 

      //  2.  Judge 
      for(route_cout=0;route_cout<Route_Mum;route_cout++)      // ��ȡ·��
     { 
                 
	        // 2.1  --------   ��ȡ·��-----------	        
	        memset((u8*)&ROUTE_Obj,0,sizeof(ROUTE_Obj));  //  clear all  first        
	        DF_ReadFlash(DF_Route_Page+route_cout, 0,(u8*)&ROUTE_Obj, sizeof(ROUTE_Obj));	  
		 DF_delay_us(20);  
		//rt_kprintf("\r\n -----> ROUTE_Obj.RouteID:   %d \r\n",ROUTE_Obj.Route_ID);   
              // 2.2  -----  �ж��Ƿ���Ч  -------
	       if((ROUTE_Obj.Effective_flag==1) &&(ROUTE_Obj.Points_Num>1)) //  �ж��Ƿ���Ч���йյ㣬����Ч������
	      {     
	              // 2.2.0    ��ǰ�ξ��븶��һ�������ֵ
	              for(i=0;i<6;i++)
				  	  Distance_Array[i]=ROUTE_DIS_Default;  
		      // 2.2.1      �������
                     seg_num=ROUTE_Obj.Points_Num-1; // ��·����Ŀ
                    //  2.2.2    �ж�·����ÿһ�ε�״̬
                     Segment_Status=0;  // ������ж�״̬��ÿ����·���¿�ʼһ��
			for(seg_count=0;seg_count<seg_num;seg_count++)
			{      
			          if((ROUTE_Obj.RoutePoints[seg_count+1].POINT_Latitude==0)&&(ROUTE_Obj.RoutePoints[seg_count+1].POINT_Longitude==0))
				      	{
				      	     rt_kprintf("\r\n  �õ�Ϊ0 ��jump\r\n"); 
					     continue;
				      	}
			      //----- ��ʼ���������, ��û�������ں�����������ж�
                           Distance_Array[seg_count]=Distance_Point2Line(Latitude, Longitude,ROUTE_Obj.RoutePoints[seg_count].POINT_Latitude,ROUTE_Obj.RoutePoints[seg_count].POINT_Longitude,ROUTE_Obj.RoutePoints[seg_count+1].POINT_Latitude,ROUTE_Obj.RoutePoints[seg_count+1].POINT_Longitude);   

			}
		//=========================================================	
              //  2.4 ------  ��ӡ��ʾ���룬�ҳ���С��ֵ----
              Distance=Distance_Array[0];  // ��С����  
               for(i=0;i<6;i++)
		   {
		       if(Distance>=Distance_Array[i])  
                                    Distance=Distance_Array[i];			   
		      // rt_kprintf("\r\n  Distance[%d]=%d",i,Distance_Array[i]);
                 }	 
		  rt_kprintf("\r\n MinDistance =%d  Width=%d \r\n",Distance,(ROUTE_Obj.RoutePoints[seg_num].Width>>1));	  // 

		if(Distance<ROUTE_DIS_Default)
	      {
		       //  ---- ��·�ο�����Ա�	   
	                 if(Distance>(ROUTE_Obj.RoutePoints[seg_num].Width>>1))
	                {
	                       rt_kprintf("\r\n ·��ƫ��\r\n");   
	                        Segment_Status|=(1<<seg_num);   //  ����Ӧ��bit  ��λ    
	                }  
		}

                     //                    
	      }  
		// 2.4  ���� 2.2 ����жϵ���·��״̬
		if(Segment_Status)
			Route_Status|=(1<<route_cout);   //  ����Ӧ��bit  ��λ 
     }
     // 3.  Result
     		if(Route_Status) 
		 {
		    if( (Warn_Status[1]&0x80)==0)   //  �����ǰû��������ô��ʱ�ϱ�
			{	  
                          PositionSD_Enable();  
				 Current_UDP_sd=1;   
		    	}
			
		    Warn_Status[1]|=0x80;// ·��ƫ������ 
		    rt_kprintf("\r\n    ·��ƫ������ !\r\n");           
		   
		}
		else
		{
		     if (Warn_Status[1]&0x80)   //  �����ǰû��������ô��ʱ�ϱ�
			{	  
                             PositionSD_Enable();  
				 Current_UDP_sd=1;   
		    	}

		    Warn_Status[1]&=~0x80;// ·��ƫ������      
		}
	  

}

//--------  D�㵽ֱ�߾������-------
/*
     P1(x1,y1)   P2(x2,y2)  ,�ѵ�P(x1,y2)��Ϊ����ԭ�㣬��x1=0��y2=0��

     ��ô����P1��P2 ȷ����ֱ�߷���(����ʽ)Ϊ:
             (x-x1)/(x2-x1) =(y-y1)/(y2-y1)                          (1)
             
    ע:  ��׼ʽֱ�߷���Ϊ AX+BY+C=0;
             ��ôƽ��������һ��P(x0,y0) ��ֱ�ߵľ����ʾΪ
             d=abs(Ax0+By0+C)/sqrt(A^2+B^2)

    ���аѷ���ʽ(1) ת���ɱ�׼ʽΪ:
            (y2-y1)x+(x1-x2)y+x1(y1-y2)+y1(x2-x1)=0; 

   ���ڵ�(x1,y2)Ϊԭ��  ��x1=0��y2=0��  P1(0,y1) , P2(x2,0)
    ����   A=-y1 ,  B=-x2, C=y1x2
    ��ô ֱ�ߵķ���:
                  -y1x-x2y+y1x2=0;  (2)  
 
  =>     d=abs(-y1x0-x2y0+y1x2)/sqrt(y1^2+x2^2)       (3)

         ���� (3)  Ϊ����Ӧ�õĹ�ʽ 

        ע:  �ȸ��ݾ�γ���ۺϼ���� x0��y0��x1,y1,x2,y2  ����ֵ��λΪ: ��     
=>  �����ж�:
           ����(2) �������  �� P1(0,y1) , P2(x2,0) ������ֱ֪�ߴ�ֱ������ֱ�߷���
              P1(0,y1) :      x2x-y1y+y1^2=0  (4)
              P2(x2,0) :      x2x-y1y-x2^2=0  (5)

          ��� y1 >=0      ֱ��(4)    ��ֱ��(5)  ���ϱ�
          ��ô �����߶������ڵ��жϷ�����
                       (4) <=  0    ��  (5)  >=0 
       ��                
           ��� y1 <=0      ֱ��(5)    ��ֱ��(4)  ���ϱ�
          ��ô �����߶������ڵ��жϷ�����
                       (4) >=  0    ��  (5)  <=0 
   //------------------------------------------------------------------------------------------     
     
       γ��û�в�ֵ    1γ��  111km
       40��γ���� 1����Ϊ  85.3km   (��������)   

       X ��Ϊ ����(longitude) ��ֵ
       Y ��Ϊγ�� (latitude)  ��ֵ
                
     
   //------------------------------------------------------------------------------------------
*/

u32   Distance_Point2Line(u32 Cur_Lat, u32  Cur_Longi,u32 P1_Lat,u32 P1_Longi,u32 P2_Lat,u32 P2_Longi)
{   //   ���뵱ǰ�� �����ص㵽����ֱ�ߵľ���
      long  x0=0,y0=0,Line4_Resualt=0,Line5_Resualt=0;  // ��λ: ��
      long  y1=0; 
      long  x2=0;	
      long   distance=0;	 
     // long  Rabs=0;
//      long  Rsqrt=0;  
     long  DeltaA1=0,DeltaA2=0,DeltaO1=0,DeltaO2=0; //  DeltaA : Latitude     DeltaO:  Longitude 	   
     // u32   Line4_Resualt2=0,Line5_Resualt2=0; 
      double   fx0=0,fy0=0,fy1=0,fx2=0;	  
      double   FLine4_Resualt2=0,FLine5_Resualt2=0,fRabs=0,fRsqrt=0; 	  

	  // 0.   �ȴ��Ե��ж� 
	       DeltaA1=abs(Cur_Lat-P1_Lat); 
	       DeltaA2=abs(Cur_Lat-P2_Lat); 
		DeltaO1=abs(Cur_Lat-P1_Longi); 
	       DeltaO2=abs(Cur_Lat-P2_Longi); 
	   /* if((DeltaA1>1000000) &&(DeltaA2>1000000))	    
            {  
                rt_kprintf("\r\n  Latitude ��̫��\r\n");
                return   ROUTE_DIS_Default; 
	    }	
	     if((DeltaO1>1000000) &&(DeltaO2>1000000))	    
            {  
                rt_kprintf("\r\n  Longitude ��̫��\r\n"); 
                return   ROUTE_DIS_Default; 
	    }	  	
       */
	 // 1.  ��ȡ  P1(0,y1)   P2(x2,0) ,��P(x0,y0)    P(x1,y2)Ϊԭ��  ��x1=0��y2=0��  P1(0,y1) , P2(x2,0)
	  x2=abs(P2_Longi-P1_Longi); // a/1000000*85300=a 853/10000 m =a x 0.0853
         if(P2_Longi<P1_Longi)
		 	x2=0-x2;
	  fx2=(double)((double)x2/1000);
	 //rt_kprintf("\r\n P2_L=%d,P1_L=%d   delta=%d \r\n",P2_Longi,P1_Longi,(P2_Longi-P1_Longi));
	 // if(P2_Longi
         y1=abs(P2_Lat-P1_Lat); //  a/1000000*111000=a/9.009	����һ����õ����� �ٳ��� 111000 �׵õ�ʵ�ʾ��� 
         if(P2_Lat<P1_Lat) 
		 	y1=0-y1;
	   fy1=(double)((double)y1/1000);	  
	  //rt_kprintf("\r\n P2_LA=%d,P1_LA=%d   delta=%d \r\n",P2_Lat,P1_Lat,(P2_Lat-P1_Lat));

      //   rt_kprintf("\r\n ��֪��������: P1(0,%d)   P2(%d,0) \r\n", y1,x2); 
       //    ��ǰ��
	  x0=abs(Cur_Longi-P1_Longi);     
	   if(Cur_Longi<P1_Longi)
	   	x0=0-x0;
	   fx0=(double)((double)x0/1000);  
         //rt_kprintf("\r\n Cur_L=%d,P1_L=%d   delta=%d \r\n",Cur_Longi,P1_Longi,(Cur_Longi-P1_Longi));
 
         y0=abs(Cur_Lat-P2_Lat); //  a/1000000*111000=a/9.009	    
         if(Cur_Lat<P2_Lat)
		   y0=0-y0;         
	 fy0=(double)((double)y0/1000);	 
          // rt_kprintf("\r\n Cur_La=%d,P2_La=%d   delta=%d \r\n",Cur_Lat,P2_Lat,(Cur_Lat-P2_Lat)); 
        //   rt_kprintf("\r\n��ǰ������: P0(%d,%d)    \r\n", x0,y0);  
	  // 2. �ж�y1  �Ĵ�С�� ����� P1(0,y1)   P2(x2,0) ,����ֱ֪�ߵķ��̣����ж�
	  //     ��ǰ���Ƿ���·�δ�ֱ��Χ��
	  
             //  2.1   ����ǰ����룬 �� P1(0,y1)   �� ֱ�߷���(4)  ������
                                  Line4_Resualt=(x2*x0)-(y1*y0)+(y1*y1);
	                            FLine4_Resualt2=fx2*fx0-fy1*fy0+fy1*fy1;
	   //     rt_kprintf("\r\n Line4=x2*x0-y1*y0+y1*y1=(%d)*(%d)-(%d)*(%d)+(%d)*(%d)=%ld     x2*x0=%d    y1*y0=%d   y1*y1=%d  \r\n",x2,x0,y1,y0,y1,y1,Line4_Resualt,x2*x0,y1*y0,y1*y1); 
          //     rt_kprintf("\r\n FLine4=fx2*fx0-fy1*fy0+fy1*fy1=(%f)*(%f)-(%f)*(%f)+(%f)*(%f)=%f      fx2*fx0=%f    fy1*fy0=%f   fy1*fy1=%f  \r\n",fx2,fx0,fy1,fy0,fy1,fy1,FLine4_Resualt2,fx2*fx0,fy1*fy0,fy1*fy1); 
  
	      //   2.2   ����ǰ����룬 ��P2(x2,0) �� ֱ�߷���(5)  ������ 
                                  Line5_Resualt=(x2*x0)-y1*y0-x2*x2;
		                    FLine5_Resualt2=fx2*fx0-fy1*fy0-fx2*fx2;    
		//rt_kprintf("\r\n Line5=x2*x0-y1*y0-x2*x2=(%d)*(%d)-(%d)*(%d)-(%d)*(%d)=%ld     Se : %ld   \r\n",x2,x0,y1,y0,x2,x2,Line5_Resualt,Line5_Resualt2); 
          //    rt_kprintf("\r\n FLine5=fx2*fx0-fy1*fy0-fx2*fx2=(%f)*(%f)-(%f)*(%f)-(%f)*(%f)=%f      fx2*fx0=%f    fy1*fy0=%f   fx2*fx2=%f  \r\n",fx2,fx0,fy1,fy0,fx2,fx2,FLine5_Resualt2,fx2*fx0,fy1*fy0,fx2*fx2);   
 	      // rt_kprintf("\r\n  Line4_Resualt=%d     Line5_Resualt=%d  \r\n",Line4_Resualt,Line5_Resualt);    

	     if(fy1>=0)      //  ֱ��(4) ���Ϸ�
	  	{
               
			//   2.3   �ж�����    (4) <=  0    ��  (5)  >=0     // �ж�����ȡ��
			     if((FLine4_Resualt2>0) ||(FLine5_Resualt2<0))
				 	 return   ROUTE_DIS_Default;      //  �������������������ֵ
	  	}
	     else
	     	{      //  ֱ��(5)

                   	//   2.4   �ж�����     (4) >=  0    ��  (5)  <=0     // �ж�����ȡ�� 
			     if((FLine4_Resualt2<0) ||(FLine5_Resualt2>0)) 
				 	 return   ROUTE_DIS_Default;      //  �������������������ֵ 

	     	}	

              rt_kprintf("\r\n In judge area \r\n");  
		//rt_kprintf("\r\n   Current== Latitude:   %d     Longitude: %d     Point1== Latitude:   %d     Longitude: %d     Point2== Latitude:   %d     Longitude: %d\r\n",Cur_Lat,Cur_Longi,P1_Lat,P1_Longi,P2_Lat,P2_Longi);   

        //  3. ����ֵ�����ʵ�ʾ���
             #if 0
		    x2=x2*0.0853; // a/1000000*85300=a 853/10000 m =a x 0.0853
                 y1=y1/9; //  a/1000000*111000=a/9.009	����һ����õ����� �ٳ��� 111000 �׵õ�ʵ�ʾ��� 
                 x0=x0*0.0853;     
		   y0=y0/9; //  a/1000000*111000=a/9.009	    
	      #else
		   fx2=fx2*0.0853; // a/1000000*85300=a 853/10000 m =a x 0.0853
                 fy1=fy1/9; //  a/1000000*111000=a/9.009	����һ����õ����� �ٳ��� 111000 �׵õ�ʵ�ʾ��� 
                 fx0=fx0*0.0853;     
		   fy0=fy0/9; //  a/1000000*111000=a/9.009	      
             #endif

	 //  4. ������� 
	       //Rabs=0-y1*x0-x2*y0+y1*x2;  
              // rt_kprintf("\r\n Test -y1*x0=%d -y0*x2=%d  y1*x2=%d   Rabs=%d  \r\n",0-y1*x0,0-y0*x2,0-y1*x2,Rabs);          
	  #if 0
	       Rabs=abs(-y1*x0-x2*y0+y1*x2);   
	       Rsqrt=sqrt(y1*y1+x2*x2); 
	        // distance=abs(-y1*x0-x2*y0-y1*x2)/sqrt(y1*y1+x2*x2); 
	       distance=Rabs/Rsqrt;
	  // rt_kprintf("\r\n Rabs=%d    Rsqrt=%d   d=%d",Rabs,Rsqrt,distance);        
        #else
             	fRabs=abs(-fy1*fx0-fx2*fy0+fy1*fx2);   
	       fRsqrt=sqrt(fy1*fy1+fx2*fx2); 
	        // distance=abs(-y1*x0-x2*y0-y1*x2)/sqrt(y1*y1+x2*x2); 
	       distance=(long) ((fRabs/fRsqrt)*1000);
	       // rt_kprintf("\r\n Rabs=%d    Rsqrt=%d   d=%d",Rabs,Rsqrt,distance);      
	  #endif
	  
	     
     return   distance;
}




 unsigned short int CRC16_file(unsigned short int num)  
{
#if  0
 unsigned short int i=0,j=0;
 unsigned char buffer_temp[514];
   memset(buffer_temp,0,sizeof(buffer_temp));
   for(i=0;i<num;i++)
	{
		if(i==0)   //��һ��
			{
			Last_crc=0;   // clear first  
			crc_fcs=0;
			buffer_temp[0]=0;
	        buffer_temp[1]=51;
	        DF_ReadFlash(51,0,&buffer_temp[2],PageSIZE);
			 WatchDog_Feed(); 
			
			Last_crc=CRC16_1(buffer_temp,514,0xffff);
			rt_kprintf("\r\ni=%d,j=%d,Last_crc=%x",i,j,Last_crc);
			}
		else if(i==(num-1))  //���һ��
			{
            buffer_temp[0]=0;
	        buffer_temp[1]=50;
	        DF_ReadFlash(50,0,&buffer_temp[2],PageSIZE);
			FileTCB_CRC16=((unsigned short int)buffer_temp[512]<<8)+(unsigned short int)buffer_temp[513];
			crc_fcs=CRC16_1(buffer_temp,512,Last_crc);
			rt_kprintf("\r\ni=%d,j=%d,Last_crc=%x ReadCrc=%x ",i,j,crc_fcs,FileTCB_CRC16);
			}
		else  
			{             // �м�İ�
			 j=i+51;
			 buffer_temp[0]=(char)(j>>8);
			 buffer_temp[1]=(char)j;
			 DF_ReadFlash(j,0,&buffer_temp[2],PageSIZE);
			 WatchDog_Feed(); 
			Last_crc=CRC16_1(buffer_temp,514,Last_crc);
			//rt_kprintf("\r\ni=%d,j=%d,Last_crc=%d",i,j,Last_crc);
			}
	   }
rt_kprintf("\r\n  У���� %x",crc_fcs);
return crc_fcs;
#endif
		  return 1;
}



//-------  JT808  Related   Save  Process---------
void  Save_Status(u8 year,u8 mon,u8 day,u8 hour,u8 min,u8 sec)
{   // �洢�¹��ɵ� ���20s������״̬�֣��͵�ǰ�������Чλ����Ϣ  

       
		u16 wr_add=3,j=0,cur=0;   
		u8	FCS;	 
		u8  regDateTime[6];		
		static uint8_t Statustmp[500]; 
//		u32  latitude=0,longitude=0;    
 
                       Time2BCD(regDateTime);  
                       //----------------------------------------------------------------------------
					   wr_add=0; 
					   memcpy(Statustmp,regDateTime,6);
					   wr_add+=6;		   
					   //-----------------------  Status Register   --------------------------------
					 cur=save_sensorCounter;  //20s���¹��ɵ�
					 for(j=0;j<100;j++)
					 {
                         Statustmp[wr_add++]=Sensor_buf[cur].DOUBTspeed;    //�ٶ�
						 Statustmp[wr_add++]=Sensor_buf[cur].DOUBTstatus;   //״̬    
                         cur++;
						 if(cur>100)
						 	cur=0;                            
					 }           
					 //---------------------------------------------------------------------------------------
					 FCS = 0;  
					 for( j=0;j<wr_add;j++)    
					 {
						FCS ^=Statustmp[j];	            
					 }				//���ϱ����ݵ�����      ����У�� ��Ϣ����Ϊ   206
					 //------------------------------------------------------------------------------
					 Statustmp[wr_add++]=FCS; 
					   //----------------------------------------------------------------------------- 	 
					   rt_kprintf("\r\n �洢�г���¼  Record  write: %d    SaveLen: %d    \r\n",Recorder_write,wr_add); 
                                      Api_DFdirectory_Write(doubt_data,Statustmp, wr_add);

					   
}
//-----------------------------------------------------------------------------------
void Save_AvrgSpdPerMin(void) 
{ /*
     Note: �洢��λСʱÿ����ƽ���ٶ� 
  */
  u8   content[70];
  u8   saveLen=0,FCS=0,i;   
    
	 saveLen=0; 
	 memset(content,0,70);
	 memcpy(content+saveLen,Avrgspd_Mint.datetime_Bak,5);    //��BAKtime
	 saveLen+=5;
	 memcpy(content+saveLen,Avrgspd_Mint.avgrspd,60);    //��BAKtime
	 saveLen+=60;  
  //---------------------------------------------------------------------------------------
  FCS = 0;
  for ( i = 0; i < 69; i++ ) 
  {
	 FCS ^=content[i];    
  } 			 //���ϱ����ݵ�����   
  content[69] = FCS;	 // ��8�ֽ�	       
  //----------------------------------------------------------------------------------------  
  Api_DFdirectory_Write(spdpermin, content, 70); 
  //-----------------------------------------------------------------
  memset(Avrgspd_Mint.avgrspd,0,60); // �洢�������¼     
   if(DispContent)
        rt_kprintf("\r\n   �洢��λСʱÿ����ƽ���ٶ�   Save Time %X-%X-%X %x:%x     \r\n",\
  	Avrgspd_Mint.datetime_Bak[0],Avrgspd_Mint.datetime_Bak[1],\
  	Avrgspd_Mint.datetime_Bak[2],Avrgspd_Mint.datetime_Bak[3],Avrgspd_Mint.datetime_Bak[4]);   
  
}

  void Spd_Exp_Wr(void)
  {	
     u8  content[40];
     u8  wr_add=0,i=0,FCS=0; 	 

	 memset(content,0,sizeof(content));
	memcpy(content+wr_add,JT808Conf_struct.Driver_Info.DriverCard_ID,18);
	wr_add+=18;
	memcpy(content+wr_add,speed_Exd.ex_startTime,6);
	wr_add+=6;
	memcpy(content+wr_add,speed_Exd.ex_endTime,6);  
	wr_add+=6; 
	content[wr_add++]=speed_Exd.current_maxSpd/10; 

       FCS = 0;
	   for ( i = 0; i < 32; i++ )  
	   {
			 FCS ^=content[i];
	   }			  //���ϱ����ݵ�����
	   content[wr_add++] = FCS;	  // ��31�ֽ� 
	   
       Api_DFdirectory_Write(spd_warn, (u8 *)content, 32); 
	  //----------- debug -----------------------
	  rt_kprintf("\r\n ���ٱ���  %X-%X-%X %X:%X:%X,MaxSpd=%d\r\n",speed_Exd.ex_endTime[0],speed_Exd.ex_endTime[1],speed_Exd.ex_endTime[2],speed_Exd.ex_endTime[3],speed_Exd.ex_endTime[4],speed_Exd.ex_endTime[5],speed_Exd.current_maxSpd); 
     //--------- clear status ----------------------------
     Spd_ExpInit(); 

  }

void Save_TiredDrive_Record(void)
{
  u8   content[40];
  u8   wr_add=0,FCS=0,i; 

        wr_add=0; 
	 memset(content,0,70);

	  Time2BCD(TiredConf_struct.Tired_drive.end_time); 
	 
         memcpy(content+wr_add,JT808Conf_struct.Driver_Info.DriverCard_ID,18);
	  wr_add+=18;
	  memcpy(content+wr_add,TiredConf_struct.Tired_drive.start_time,6);
	  wr_add+=6;
	  memcpy(content+wr_add,TiredConf_struct.Tired_drive.end_time,6);
	  wr_add+=6; 

	      FCS = 0;
	   for ( i = 0; i < 32; i++ )  
	   {
			 FCS ^=content[i];
	   }			  //���ϱ����ݵ�����
	   content[wr_add++] = FCS;	  // ��31�ֽ� 
	   
       Api_DFdirectory_Write(tired_warn, (u8 *)content, 32); 
       //-----  clear status ---------------  
	  TIRED_Drive_Init();      
	  Status_TiredwhRst=0;
	 // DF_WriteFlashSector(DF_TiredStartTime_Page,0,(u8*)&Status_TiredwhRst,1); //���O��Ҫд��		
	   TIRED_Drive_Init(); //	��Ϣ�� 
	   Warn_Status[3]&=~0x04;  //BIT(2)	ƣ�ͼ�ʻ  

}


void  JT808_Related_Save_Process(void)
{
       if(DF_LOCK)
	   	 return ;

	      if(Dev_Voice.CMD_Type!='1')  // ¼��ʱ�������´���  
      {           

		//-------------------------
		//    �洢�ɵ�����
		if(1==NandsaveFlg.Doubt_SaveFlag) 
		{
			if(sensor_writeOverFlag==1)	
			{ 
			  time_now=Get_RTC();     //  RTC  ��� 
			  Save_Status(time_now.year,time_now.month,time_now.day,time_now.hour,time_now.min,time_now.sec);
			  NandsaveFlg.Doubt_SaveFlag=0;
			  return;
			} 
		} 							 
		//  �洢ÿ�����ٶ���Ϣ  
		if(1==Avrgspd_Mint.saveFlag) 
		{
			Save_AvrgSpdPerMin();
			Avrgspd_Mint.saveFlag=0; 
			return;
		}							  	  

		//-----------------  �洢ƣ�ͼ�ʻ��¼ ---------------			
		if( TiredConf_struct.Tired_drive.Tireddrv_status==2)   
		{
	             Save_TiredDrive_Record();
			return;	 
		}			
		//-----------------  ���ٱ��� ----------------------
		if(speed_Exd.excd_status==2)
		{
		   Spd_Exp_Wr();   
		   return;
		}   
		 /*
		if(NandsaveFlg.Setting_SaveFlag==1)   // ���� �޸ļ�¼
		{
		  Save_Common(Settingchg_write,TYPE_SettingChgAdd); 
		  Settingchg_write++;
		  if(Settingchg_write>=Max_CommonNum)  
		  	Settingchg_write=0; 
		  DF_Write_RecordAdd(Settingchg_write,Settingchg_read,TYPE_SettingChgAdd);   
		  NandsaveFlg.Setting_SaveFlag=0; // clear flag
		} 
		*/
			
      	}           
	//	 ��ʱ�洢���
	if((Vehicle_RunStatus)&&((Systerm_Reset_counter&0xff)==0xff))
	{   //  �����������ʻ�����У�ÿ255 ��洢һ���������    
                Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct)); 
		  return;		
	} 
			
	 //--------------------------------------------------------------

}

/*
    ��ӡ��� HEX ��Ϣ��Descrip : ������Ϣ ��instr :��ӡ��Ϣ�� inlen: ��ӡ����
*/
void OutPrint_HEX(u8 * Descrip, u8 *instr, u16 inlen )
{
    u32  i=0;
     rt_kprintf("\r\n %s:",Descrip);
     for( i=0;i<inlen;i++)
          rt_kprintf("%2X ",instr[i]); 
     rt_kprintf("\r\n");
}


void  dur(u8 *content)
{
  sscanf(content, "%d", (u32*)&Current_SD_Duration);
  rt_kprintf("\r\n �ֶ������ϱ�ʱ���� %d s\r\n",Current_SD_Duration);
  
       JT808Conf_struct.DURATION.Default_Dur=Current_SD_Duration;
        Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
}
FINSH_FUNCTION_EXPORT(dur, dur);  



void handsms(u8 *instr)
{    
    memset(SMS_Service.SMS_sd_Content,0,sizeof(SMS_Service.SMS_sd_Content)); 
    memcpy(SMS_Service.SMS_sd_Content,instr,strlen(instr));  
    SMS_Service.SMS_sendFlag=1;	
	SMS_Service.SMS_come=1;
    rt_kprintf("�ֶ�����:%s",SMS_Service.SMS_sd_Content);    
}
FINSH_FUNCTION_EXPORT(handsms, handsms);  


void driver_name(u8 *instr)
{   
    memset(JT808Conf_struct.Driver_Info.DriveName,0,sizeof(JT808Conf_struct.Driver_Info.DriveName)); 
	memcpy(JT808Conf_struct.Driver_Info.DriveName,instr,strlen(instr)); 
    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));		
}
FINSH_FUNCTION_EXPORT(driver_name, set_driver_name );   



void chepai(u8 *instr)
{    
    memset(JT808Conf_struct.Vechicle_Info.Vech_Num,0,sizeof(JT808Conf_struct.Vechicle_Info.Vech_Num));
    memcpy(JT808Conf_struct.Vechicle_Info.Vech_Num,instr,8); 
    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));  
}
FINSH_FUNCTION_EXPORT(chepai, set_chepai);   

void  vin_set(u8 *instr)
{
	 //����VIN
	memset(JT808Conf_struct.Vechicle_Info.Vech_VIN,0,sizeof(JT808Conf_struct.Vechicle_Info.Vech_VIN));
	memcpy(JT808Conf_struct.Vechicle_Info.Vech_VIN,instr,strlen(instr)); 
	Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
       rt_kprintf("\r\n �ֶ�����vin:%s \r\n",instr); 

}
FINSH_FUNCTION_EXPORT(vin_set, vin_set );    

void  current(void)
{
    PositionSD_Enable();
    Current_UDP_sd=1;

}
FINSH_FUNCTION_EXPORT(current, current );    


void  link_mode(u8 *instr)
{
   if(instr[0]=='1')
   {
      JT808Conf_struct.Link_Frist_Mode=1;
	   rt_kprintf("\r\n Mainlink:%s \r\n",instr); 
   }	  
   else
   if(instr[0]=='0')
   {
   	   JT808Conf_struct.Link_Frist_Mode=0;
       rt_kprintf("\r\n DNSR :%s \r\n",instr);  
   }

  
   Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));	 

}
FINSH_FUNCTION_EXPORT(link_mode, link_mode );    

void  redial(void)
{
      DataLink_EndFlag=1; //AT_End();   
        rt_kprintf("\r\n Redial\r\n");      
}
FINSH_FUNCTION_EXPORT(redial, redial);


// C.  Module
