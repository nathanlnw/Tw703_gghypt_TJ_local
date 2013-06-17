#include <rtthread.h> 
#include <rthw.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>

#include  <stdlib.h>
#include  <stdio.h>
#include  <string.h>
#include "App_moduleConfig.h"
#include "Device_808.h"

ALIGN(RT_ALIGN_SIZE) 
SYS_CONF          SysConf_struct;   //  ϵͳ����  

ALIGN(RT_ALIGN_SIZE) 
JT808_CONF       JT808Conf_struct;   //  JT 808   ������� 

ALIGN(RT_ALIGN_SIZE) 
TIRED_CONF      TiredConf_struct;    //  ƣ�ͼ�ʻ�������





//----------  Basic  Config---------------------------
u8      DeviceNumberID[13];//="800130100001";    // ����DeviceID    ---- �ӱ����ͨ��

u8          RemoteIP_Dnsr[4]={255,255,255,255}; 
u8		RemoteIP_main[4]={60,28,50,210};//{125,38,185,88};//{113,31,28,101 };//{113,31,92,200};//���{60,28,50,210}; �ӱ����ͨ 113,31,28,100                        
u16		RemotePort_main= 9131;//���9131;   �ӱ����ͨ 8201             //test tianjin    
u8		RemoteIP_aux[4]={60,28,50,210};    //{60,28,50,210}
u16		RemotePort_aux=4000; 
//      Link2  Related 
u8      Remote_Link2_IP[4]={60,28,50,210};
u16     Remote_Link2_Port=9131;     



u8          APN_String[30]="UNINET"; //"CMNET";   //  �ӱ����ͨ  �ƶ��Ŀ�
u8           DomainNameStr[50]="jt1.gghypt.net"; ;  // ����  ���ͨup.gps960.com //jt1.gghypt.net
u8           DomainNameStr_aux[50]="jt2.gghypt.net";     //"www.sina.com";//jt2.gghypt.net
u16         ACC_on_sd_Duration=30;    //  ACC ������ʱ�� �ϱ���ʱ����  
u16         ACC_off_sd_Duration=60;    //  ACC �ر�ʱ���ϱ���ʱ����  
u8          TriggerSDsatus=0x80;   // �����������ϱ�״̬λ






u32	     Current_SD_Duration=20;  //GPS ��Ϣ�����ʱ����   
u32      Current_SD_Distance=100; // GPS ��Ϣ�����ϱ�����
u32      DistanceAccumulate=0;    // �����ϱ��ۼ���
u8		 Current_State=0; //��ʽΪ0 �� ��ʾΪ1		 // �ϱ�ʵʱ��־λ��Ϣ	 Ԥ��DF �� 

u16      StopLongDuration=15300; //255minutes 15300s   //����ͣ�������ʱ��    
u16      Stop_counter=0;               //����ͣ������ʱ������
   
u8      EmergentWarn=0;               // ��������

//-------------- Vehicle Recorder Setting --------------------

//---------------------------   ��ʻԱ��Ϣ  ------------------------------
//u8     DriverCard_ID[18]="000000000000000000";  // ��ʻԱ��ʻ֤���� 18λ
//u8     DriveName[21]="����";                    // ��ʻԱ ����

//-----------------  ������Ϣ ------------------------------------------
//u8     Vech_VIN[17]="00000000000000000";        // ����VIN��
//u8     Vech_Num[12]="��A00001";	                // ���ƺ�
//u8     Vech_Type[12]="000000000000";            // �������� 


//-----------------  ������Ϣ ------------------------------------------
//VechINFO  Vechicle_Info; 



u8     Vechicle_TYPE=1;                 //   ��������    1:���ͻ���  2: С�ͻ���  3:���Ϳͳ�  4: ���Ϳͳ�   5:С�Ϳͳ�
u8     OnFire_Status=0;                      //   1 : ACC ���������     0 :  ACC  �ػ������� 
u8     Login_Status=0x02;                    //   01H:��¼��02H���˳���03H��������ʻԱ
u8     Powercut_Status=0x01;                 //01H:�ϵ磬02H���ϵ�
u8     Settingchg_Status=0x00;                 /*
												82H:���ó�����Ϣ��84H������״̬��
												C2H:���ü�¼��ʱ�� 
												C3H:���ü�¼���ٶ�����ϵ��
											*/
               
//u16    DaySpdMax=0;                                //  ��������ٶ�
//u16    DayDistance=0;                              //  ������ʻ����

//------ ����ش� ------

//--------------  �����ϱ�  --------------------
u32  former_distance_meter=0;     //   ��һ�ξ���    ����ش�ʱ������
u32  current_distance_meter=0;    //   ��ǰ����

//---------  SytemCounter ------------------
u32  Systerm_Reset_counter=0;
u8   SYSTEM_Reset_FLAG=0;        // ϵͳ��λ��־λ 
u32  Device_type=0x00000001; //Ӳ������   STM32103  ��A1 
u32  Firmware_ver=0x0000001; // ����汾
u8   ISP_resetFlag=0;        //Զ��������λ��־λ



/*
          ����ϵͳĿ¼
*/
void    Create_Sys_Directory(void)
{
         //  ���� 
         Api_DFdirectory_Create(config,config_size);      //   sysconfig
         Api_DFdirectory_Create(jt808,jt808_size);       
         Api_DFdirectory_Create(tired_config,tired_config_size);        //   ƣ�ͼ�ʻ����

        //   ѭ���洢
         Api_DFdirectory_Create(cyc_gps,cyc_gps_size);    //  gps  ����ѭ���洢      

	//  �̶����	 
	  Api_DFdirectory_Create(event_808, event_size);   // �¼�
         Api_DFdirectory_Create(msg_broadcast,msg_broadcast_size);   //  ������Ϣ
         Api_DFdirectory_Create(phonebook,phonebook_size);         //   �绰��
         Api_DFdirectory_Create(Rail_cycle,Rail_cycle_size);  
         Api_DFdirectory_Create(Rail_rect,Rail_rect_size);  
         Api_DFdirectory_Create(Rail_polygen,Rail_polygen_size);  
         Api_DFdirectory_Create(turn_point,turn_point_size);        // �յ�
	  Api_DFdirectory_Create(route_line,route_line_size);    // ·��
	  Api_DFdirectory_Create(ask_quesstion,ask_quesstion_size);    // ����
	  Api_DFdirectory_Create(text_msg,text_msg_size);    // �ı���Ϣ



	 //  ��¼ 
         Api_DFdirectory_Create(spd_warn,spd_warn_size);      // ���ٱ���   
	  Api_DFdirectory_Create(tired_warn,tired_warn_size);    // ƣ�ͼ�ʻ
	  Api_DFdirectory_Create(doubt_data,doubt_data_size);   // �¹��ɵ�
	  Api_DFdirectory_Create(spdpermin,spdpermin_size);    // ÿ����ƽ���ٶ�
	  Api_DFdirectory_Create(pospermin,pospermin_size);    //  ÿ����λ����Ϣ  
	  Api_MediaIndex_Init();    //  ��ý������   ͼƬ+  ����
	 // Api_DFdirectory_Create(pic_index,pic_index_size);    //  ͼƬ����
	 // Api_DFdirectory_Create(voice_index,voice_index_size);    //  ��������


	  // ����
	   Api_DFdirectory_Create(camera_1,camera_1_size);   //    4 ·���գ�ÿ· 32K  
	   Api_DFdirectory_Create(camera_2,camera_2_size); 
          Api_DFdirectory_Create(camera_3,camera_3_size); 
	   Api_DFdirectory_Create(camera_4,camera_4_size); 	    

	  //  ¼�� 
         Api_DFdirectory_Create(voice,voice_size);    //  ������Ϣ  

}

/*
        ɾ������Ŀ¼����
*/
void  Delete_exist_Directory(void)
{
            //  ���� 
         Api_DFdirectory_Delete(config);      //   sysconfig
         Api_DFdirectory_Delete(jt808);       
         Api_DFdirectory_Delete(tired_config);        //   ƣ�ͼ�ʻ����

        //   ѭ���洢
         Api_DFdirectory_Delete(cyc_gps);      

	//  �̶����	 
         Api_DFdirectory_Delete(msg_broadcast);   //  ������Ϣ
         Api_DFdirectory_Delete(phonebook);         //   �绰��
         Api_DFdirectory_Delete(Rail_cycle);  
         Api_DFdirectory_Delete(Rail_rect);  
         Api_DFdirectory_Delete(Rail_polygen);  
         Api_DFdirectory_Delete(turn_point);        // �յ� 
	  Api_DFdirectory_Delete(route_line);    // ·��
	  Api_DFdirectory_Delete(ask_quesstion);    // ����
	  Api_DFdirectory_Delete(text_msg);   //  �ı���Ϣ


	 //  ��¼ 
         Api_DFdirectory_Delete(spd_warn);      // ���ٱ���   
	  Api_DFdirectory_Delete(tired_warn);    // ƣ�ͼ�ʻ
	  Api_DFdirectory_Delete(doubt_data);   // �¹��ɵ�
	  Api_DFdirectory_Delete(spdpermin);    // ÿ����ƽ���ٶ�
	  Api_DFdirectory_Delete(pospermin);    //  ÿ����λ����Ϣ    
	  Api_DFdirectory_Delete(pic_index);    //  ͼƬ����
	  Api_DFdirectory_Delete(voice_index); //  ��������


}  


/*
       ϵͳ������Ϣд��
*/
u8  SysConfig_init(void)
{
	  
       //  1. Stuff
	                 //   ϵͳ�汾
	        SysConf_struct.Version_ID=SYSID; 
	                         //   APN  
		memset((u8*)SysConf_struct.APN_str,0 ,sizeof(SysConf_struct.APN_str));
		memcpy(SysConf_struct.APN_str,(u8*)APN_String,strlen((const char*)APN_String));  
	                        //   ����main 
		memset((u8*)SysConf_struct.DNSR,0 ,sizeof(SysConf_struct.DNSR));
		memcpy(SysConf_struct.DNSR,(u8*)DomainNameStr,strlen((const char*)DomainNameStr)); 
                            //   ����aux
		memset((u8*)SysConf_struct.DNSR_Aux,0 ,sizeof(SysConf_struct.DNSR_Aux));
		memcpy(SysConf_struct.DNSR_Aux,(u8*)DomainNameStr_aux,strlen((const char*)DomainNameStr_aux));   
		

			 
	        //   �� IP   +  �˿�
	        memcpy(SysConf_struct.IP_Main,(u8*)RemoteIP_main,4); 
	        SysConf_struct.Port_main=RemotePort_main;
	       //   ���� IP   +  �˿�
	        memcpy(SysConf_struct.IP_Aux,(u8*)RemoteIP_aux,4); 
	        SysConf_struct.Port_Aux=RemotePort_aux;	 				 

           //   LINK2 +      �˿�           
		   memcpy(SysConf_struct.Link2_IP,(u8*)Remote_Link2_IP,4); 
		   SysConf_struct.Link2_Port=Remote_Link2_Port;  					



	       //  �����������ϱ�״̬
	       SysConf_struct.TriggerSDsatus=TriggerSDsatus;
		//   ACC  on   off  ����
		SysConf_struct.AccOn_Dur=ACC_on_sd_Duration;
		SysConf_struct.AccOff_Dur=ACC_off_sd_Duration; 
   //    2. Operate
            return(Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct)));       

}

void SysConfig_Read(void)
{
            if( Api_Config_read(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct)==false))   //��ȡϵͳ������Ϣ
                      rt_kprintf("\r\nConfig_ Read Error\r\n");   

			
 		memset((u8*)APN_String,0 ,sizeof(APN_String)); 
		memcpy((u8*)APN_String,SysConf_struct.APN_str,strlen(SysConf_struct.APN_str));  
	                        //   ����
		memset((u8*)DomainNameStr,0 ,sizeof(DomainNameStr));
		memcpy((u8*)DomainNameStr,SysConf_struct.DNSR,strlen(SysConf_struct.DNSR)); 
	                        //   ����aux
		memset((u8*)DomainNameStr_aux,0 ,sizeof(DomainNameStr_aux));
		memcpy((u8*)DomainNameStr_aux,SysConf_struct.DNSR_Aux,strlen(SysConf_struct.DNSR_Aux)); 

		
			 
	        //   �� IP   +  �˿�
	        memcpy((u8*)RemoteIP_main,SysConf_struct.IP_Main,4); 
	        RemotePort_main=SysConf_struct.Port_main;
	       //   ���� IP   +  �˿�
	        memcpy((u8*)RemoteIP_aux,SysConf_struct.IP_Aux,4); 
	        RemotePort_aux=SysConf_struct.Port_Aux;	 		
		   //  Link2  
	        memcpy((u8*)Remote_Link2_IP,SysConf_struct.Link2_IP,4); 
	        Remote_Link2_Port=SysConf_struct.Link2_Port;	 				 

	       //  �����������ϱ�״̬
	        TriggerSDsatus=SysConf_struct.TriggerSDsatus;
		//   ACC  on   off  ����
		ACC_on_sd_Duration=SysConf_struct.AccOn_Dur;
		ACC_off_sd_Duration=SysConf_struct.AccOff_Dur;              


}


/*
       JT808    Related 
*/
void JT808_DURATION_Init(void)
{
       JT808Conf_struct.DURATION.Heart_Dur=300;       // ���������ͼ��
	JT808Conf_struct.DURATION.TCP_ACK_Dur=20;     //  TCP Ӧ��ʱ
	JT808Conf_struct.DURATION.TCP_ReSD_Num=3;     //  TCP �ط�����
	JT808Conf_struct.DURATION.TCP_ACK_Dur=20;     //  UDP Ӧ��ʱ
	JT808Conf_struct.DURATION.UDP_ReSD_Num=5;     //  UDP �ط�����
 	JT808Conf_struct.DURATION.NoDrvLogin_Dur=40;  //  ��ʻԱû��½ʱ�ķ��ͼ��
 	JT808Conf_struct.DURATION.Sleep_Dur=30;       //  ����ʱ�ϱ���ʱ����    
	JT808Conf_struct.DURATION.Emegence_Dur=20;    //  ��������ʱ�ϱ�ʱ����
	JT808Conf_struct.DURATION.Default_Dur=30;     //  ȱʡ������ϱ���ʱ����
	JT808Conf_struct.DURATION.SD_Delta_maxAngle=60; // �յ㲹�������Ƕ�
	JT808Conf_struct.DURATION.IllgleMovo_disttance=300; // �Ƿ��ƶ���ֵ  
}

void JT808_SendDistances_Init(void)
{
   JT808Conf_struct.DISTANCE.Defalut_DistDelta=200;    // Ĭ�϶���ش�����
   JT808Conf_struct.DISTANCE.NoDrvLogin_Dist=300;      // ��ʻԱδ��¼ʱ�ش�����
   JT808Conf_struct.DISTANCE.Sleep_Dist=500;           // ����������ϱ��Ķ���ش�
   JT808Conf_struct.DISTANCE.Emergen_Dist=100;         // ��������������ϱ��Ķ���ش� 
}

//------------------------- �ն����ݷ��ͷ�ʽ -----------------------
void JT808_SendMode_Init(void)
{
  JT808Conf_struct.SD_MODE.DUR_TOTALMODE=1;  // ʹ�ܶ�ʱ����
  JT808Conf_struct.SD_MODE.Dur_DefaultMode=1; //  ȱʡ��ʽ�ϱ�
  JT808Conf_struct.SD_MODE.Dur_EmegencMode=0;
  JT808Conf_struct.SD_MODE.Dur_NologinMode=0;
  JT808Conf_struct.SD_MODE.Dur_SleepMode=0;

  JT808Conf_struct.SD_MODE.DIST_TOTALMODE=0;
  JT808Conf_struct.SD_MODE.Dist_DefaultMode=0;
  JT808Conf_struct.SD_MODE.Dist_EmgenceMode=0;
  JT808Conf_struct.SD_MODE.Dist_NoLoginMode=0;
  JT808Conf_struct.SD_MODE.Dist_SleepMode=0;
}

//------------------------------------------------------------------
void  JT808_RealTimeLock_Init(void)
{                         // Ԥ��ֵĬ��ֵ     
   JT808Conf_struct.RT_LOCK.Lock_state=0;
   JT808Conf_struct.RT_LOCK.Lock_Dur=20;   
   JT808Conf_struct.RT_LOCK.Lock_KeepDur=300;
   JT808Conf_struct.RT_LOCK.Lock_KeepCnter=0;
}

void  JT808_Vehicleinfo_Init(void) 
{
	memset((u8*)&JT808Conf_struct.Vechicle_Info,0,sizeof(JT808Conf_struct.Vechicle_Info));
	//-----------------------------------------------------------------------
	memcpy(JT808Conf_struct.Vechicle_Info.Vech_VIN,"00000000000000000",17);
	memcpy(JT808Conf_struct.Vechicle_Info.Vech_Num,"��TST002",8);        
	memcpy(JT808Conf_struct.Vechicle_Info.Vech_Type,"δ֪��",6);       
	JT808Conf_struct.Vechicle_Info.Dev_ProvinceID=0;  // Ĭ��ʡID   0
	JT808Conf_struct.Vechicle_Info.Dev_CityID=0;      // Ĭ����ID   0		
	JT808Conf_struct.Vechicle_Info.Dev_Color=1;       // Ĭ����ɫ    // JT415    1  �� 2 �� 3 �� 4 �� 9����     
}

u8     JT808_Conf_init( void ) 
  {
         u8  FirstUseDate[6]={0,0,0,0,0,0};
            //  1.  clear
                    memset((u8*)&(JT808Conf_struct),0,sizeof(JT808Conf_struct)); 	  // ��ʻԱ��Ϣ						

		  
        //   2.  Stuff 
  		   JT808_DURATION_Init();
                         //  JT808Conf_struct.
                 JT808_SendDistances_Init();
		   JT808_SendMode_Init();
		   JT808Conf_struct.LOAD_STATE=1; //  ����״̬
		   
		   memset((u8*)JT808Conf_struct.ConfirmCode,0,sizeof(JT808Conf_struct.ConfirmCode));
		   memcpy((u8*)JT808Conf_struct.ConfirmCode,"012345\x00",7); //  ��Ȩ��

		   JT808Conf_struct.Regsiter_Status=0;   //  ע��״̬

		   memset((u8*)JT808Conf_struct.LISTEN_Num,0,sizeof(JT808Conf_struct.LISTEN_Num));
		   memcpy((u8*)JT808Conf_struct.LISTEN_Num,"10086",5); //  ��������

                 JT808Conf_struct.Vech_Character_Value=6240; // ����ϵ��  �ٶ�����ϵ�� 



		   memset((u8*)JT808Conf_struct.FirstSetupDate,0,sizeof(JT808Conf_struct.FirstSetupDate));
		   memcpy((u8*)JT808Conf_struct.FirstSetupDate,FirstUseDate,6); // �״ΰ�װʱ��
  

                 memset((u8*)JT808Conf_struct.DeviceOnlyID,0,sizeof(JT808Conf_struct.DeviceOnlyID));
		   memcpy((u8*)JT808Conf_struct.DeviceOnlyID,"00000010000000000000001",23);   //   �г���¼�ǵ�ΨһID

		   JT808Conf_struct.Msg_Float_ID=0;   // ��Ϣ��ˮ��

  
		

                JT808Conf_struct.Distance_m_u32=0;            //  ��ʻ��¼����ʻ���  ��λ: ��
                JT808Conf_struct.DayStartDistance_32=0;     //  ÿ�����ʼ�����Ŀ

                JT808Conf_struct.Speed_warn_MAX=200;           //  �ٶȱ�������
                JT808Conf_struct.Spd_Exd_LimitSeconds=10;  //  ���ٱ�������ʱ������ s
                JT808Conf_struct.Speed_GetType=0;             //  ��¼�ǻ�ȡ�ٶȵķ�ʽ  00  gpsȡ�ٶ�  01 ��ʾ�Ӵ�����ȥ�ٶ� 
                JT808Conf_struct.DF_K_adjustState=0; // ����ϵ���Զ�У׼״̬˵��  1:�Զ�У׼��    0:��δ�Զ�У׼   

   	
                JT808Conf_struct.OutGPS_Flag=1;     //  0  Ĭ��  1  ���ⲿ��Դ���� 
                JT808Conf_struct.concuss_step=40;
				JT808Conf_struct.password_flag=0;//����Ϊ0�����úú�Ϊ1
		   JT808_RealTimeLock_Init();   //  ʵʱ��������	

		    		 
                 memset((u8*)&(JT808Conf_struct.StdVersion),0,sizeof(JT808Conf_struct.StdVersion));  // ��׼���Ұ汾 
  		   memcpy((u8*)(JT808Conf_struct.StdVersion.stdverStr),"GB/T19056-2011",14); // ��׼�汾 
  	          JT808Conf_struct.StdVersion.MdfyID=0x02; //�޸ĵ���



		  memset((u8*)&(JT808Conf_struct.Driver_Info),0,sizeof(JT808Conf_struct.Driver_Info)); 	  // ��ʻԱ��Ϣ						
		//--------------------------------------------------------------------------
		
		  memcpy(JT808Conf_struct.Driver_Info.DriveCode,"000",3);  
		  memcpy(JT808Conf_struct.Driver_Info.DriverCard_ID,"000000000000000000",18);  
		  memcpy(JT808Conf_struct.Driver_Info.DriveName,"δ֪",4);   
		  memcpy(JT808Conf_struct.Driver_Info.Driver_ID,"000000000000000000",18);  
		  memcpy(JT808Conf_struct.Driver_Info.Drv_CareerID,"0000000000000000000000000000000000000000",40); 
		  memcpy(JT808Conf_struct.Driver_Info.Comfirm_agentID,"000000000000000",16);
							
  	          JT808_Vehicleinfo_Init();
       

       //    3. Operate
            return(Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct)));       

  }

void TIRED_DoorValue_Init(void)
{
    TiredConf_struct.TiredDoor.Door_DrvKeepingSec=14400;  // ���ұ�׼�� 3Сʱ        
    TiredConf_struct.TiredDoor.Door_MinSleepSec=1200;     // 
    TiredConf_struct.TiredDoor.Door_DayAccumlateDrvSec=28800; //8Сʱ
    TiredConf_struct.TiredDoor.Door_MaxParkingSec=7200;  // 2 Сʱ  
    TiredConf_struct.TiredDoor.Parking_currentcnt=0;  // ͣ��״̬������      
}

void  TIRED_Drive_Init(void)
{
          //--------- ��λʱ ƣ�ͼ�ʻACC on off �������  ---      
	 TiredConf_struct.Tired_drive.Status_TiredwhRst=0;;
	 TiredConf_struct.Tired_drive.Tireddrv_status=0;;
	 TiredConf_struct.Tired_drive.Flag=0;
	 TiredConf_struct.Tired_drive.ACC_ONstate_counter=0;
	 TiredConf_struct.Tired_drive.ACC_Offstate_counter=0;

        TiredConf_struct.Tired_drive.Tgvoice_play=0;
	 TiredConf_struct.Tired_drive.voicePly_counter=0;
	 TiredConf_struct.Tired_drive.voicePly_timer=0;

         //--- below again ---
	   TiredConf_struct.Tired_drive.ACC_ONstate_counter=0;
	   TiredConf_struct.Tired_drive.ACC_Offstate_counter=0;
	   TiredConf_struct.Tired_drive.Tgvoice_play=0;
	   TiredConf_struct.Tired_drive.Tireddrv_status=0;
	   TiredConf_struct.Tired_drive.voicePly_timer=0;
	   TiredConf_struct.Tired_drive.voicePly_counter=0; 
	   memset((char*)(TiredConf_struct.Tired_drive.start_time),0,5);
	   memset((char*)(TiredConf_struct.Tired_drive.end_time),0,5);
	   TiredConf_struct.Tired_drive.Flag=0;
}

 void TIRED_DoorValue_Read(void)   //��ʱ��Ҫע�� ��3���ط�Ҫ��� 0
 {
 	  Api_Config_read(tired_config,0,(u8*)&TiredConf_struct,sizeof(TiredConf_struct));       

	 TiredConf_struct.Tired_drive.Tgvoice_play=0;
	 TiredConf_struct.Tired_drive.voicePly_counter=0;
	 TiredConf_struct.Tired_drive.voicePly_timer=0;
 }


u8   TIRED_CONF_Init(void)
{
    //  1.  clear
     memset((u8*)&(TiredConf_struct),0,sizeof(TiredConf_struct)); 	  // ��ʻԱ��Ϣ						

    // 2. stuff 
    TIRED_DoorValue_Init();
    TIRED_Drive_Init(); 	
 
    //    2. Operate
     return(Api_Config_write(tired_config,0,(u8*)&TiredConf_struct,sizeof(TiredConf_struct)));       

}

/*
         �¼� 
*/
//-----------------------------------------------------------------
void Event_Write_Init(void)
{
u8 len_write=8;
    //�¼�д��
    len_write=8;
       EventObj.Event_ID=1;//�¼�ID
	EventObj.Event_Len=len_write;//���� 4*2
	EventObj.Event_Effective=1;//�¼���Ч
	memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str));
	memcpy(EventObj.Event_Str,"�������",EventObj.Event_Len);  
       Api_RecordNum_Write(event_808, EventObj.Event_ID, (u8*)&EventObj,sizeof(EventObj)); 

	len_write=16;
    EventObj.Event_ID=2;//�¼�ID
	EventObj.Event_Len=len_write;//���� 8*2
	memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str));
	memcpy(EventObj.Event_Str,"����װ��׼������",EventObj.Event_Len);  
          Api_RecordNum_Write(event_808, EventObj.Event_ID, (u8*)&EventObj,sizeof(EventObj)); 

	EventObj.Event_ID=3;//�¼�ID
	EventObj.Event_Len=len_write;//���� 8*2
	memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str));
	memcpy(EventObj.Event_Str,"ƽ������һ��˳��",EventObj.Event_Len);  
          Api_RecordNum_Write(event_808, EventObj.Event_ID, (u8*)&EventObj,sizeof(EventObj)); 
	
    EventObj.Event_ID=4;//�¼�ID
	EventObj.Event_Len=len_write;//���� 8*2
	memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str));
	memcpy(EventObj.Event_Str,"ָ���ص��δ����",EventObj.Event_Len);  
          Api_RecordNum_Write(event_808, EventObj.Event_ID, (u8*)&EventObj,sizeof(EventObj)); 

	EventObj.Event_ID=5;//�¼�ID
	EventObj.Event_Len=len_write;//���� 8*2
	memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str));
	memcpy(EventObj.Event_Str,"ָ���ص����˽Ӵ�",EventObj.Event_Len);  
          Api_RecordNum_Write(event_808, EventObj.Event_ID, (u8*)&EventObj,sizeof(EventObj)); 

	EventObj.Event_ID=6;//�¼�ID
	EventObj.Event_Len=len_write;//���� 8*2
	EventObj.Event_Effective=0;//�¼���Ч
	memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str));
	memcpy(EventObj.Event_Str,"�����޷���ϵ����",EventObj.Event_Len);  
          Api_RecordNum_Write(event_808, EventObj.Event_ID, (u8*)&EventObj,sizeof(EventObj)); 

       len_write=14;
	EventObj.Event_ID=7;//�¼�ID
	EventObj.Event_Len=len_write;//���� 7*2
	memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str));
	memcpy(EventObj.Event_Str,"������������",EventObj.Event_Len);  
          Api_RecordNum_Write(event_808, EventObj.Event_ID, (u8*)&EventObj,sizeof(EventObj)); 

	EventObj.Event_ID=8;//�¼�ID
	EventObj.Event_Len=len_write;//���� 7*2
	memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str));
	memcpy(EventObj.Event_Str,"�м������ٻػ�",EventObj.Event_Len);  
          Api_RecordNum_Write(event_808, EventObj.Event_ID, (u8*)&EventObj,sizeof(EventObj)); 
}

void Event_Read(void)
{
  u8 i=0;
  for(i=0;i<8;i++)
    {
	memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str)); 
       Api_RecordNum_Read(event_808, i+1, (u8*)&EventObj,sizeof(EventObj)); 
	
	EventObj_8[i].Event_ID=EventObj.Event_ID;
	EventObj_8[i].Event_Len=EventObj.Event_Len;
	EventObj_8[i].Event_Effective=EventObj.Event_Effective;
	memcpy(EventObj_8[i].Event_Str,EventObj.Event_Str,sizeof(EventObj.Event_Str));
//	memcpy(DisInfor_Affair[i],EventObj.Event_Str,20);
	//rt_kprintf("\r\n�¼�ID:%d  ����:%d  �Ƿ���Ч:%d(1��ʾ0����ʾ) Info: %s",EventObj.Event_ID,EventObj.Event_Len,EventObj.Event_Effective,EventObj.Event_Str); 
	}
} 


void Event_Init(u8  Intype) 
{
  u8 i=0;

	  if(Intype==0)
	  {
	    EventObj.Event_Len=8; 
	    memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str));
	    memcpy(EventObj.Event_Str,"����·��",8);
		EventObj.Event_Effective=1;
	  } 
	  else
	  if(Intype==1)
		{
		  EventObj.Event_Len=0;  
		  memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str));
		  EventObj.Event_Effective=0;
		}
	  
   for(i=0;i<8;i++)
   {
          EventObj.Event_ID=i+1;
          Api_RecordNum_Write(event_808, EventObj.Event_ID, (u8*)&EventObj,sizeof(EventObj)); 
   }
}

/*
          ��Ϣ 
*/
//----------------------------------------------------------------
void MSG_BroadCast_Write_Init(void)
{
u8 len_write=8;
	MSG_BroadCast_Obj.INFO_TYPE=1;//����
	MSG_BroadCast_Obj.INFO_LEN=len_write;//���� 4*2
	MSG_BroadCast_Obj.INFO_PlyCancel=1;//�㲥
	MSG_BroadCast_Obj.INFO_SDFlag=1;//���ͱ�־λ
	MSG_BroadCast_Obj.INFO_Effective=1;//��ʾ��Ч
	memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR));
	memcpy(MSG_BroadCast_Obj.INFO_STR,"����Ԥ��",MSG_BroadCast_Obj.INFO_LEN);  
	Api_RecordNum_Write(msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 

	MSG_BroadCast_Obj.INFO_TYPE=2;//����
	memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR));
	memcpy(MSG_BroadCast_Obj.INFO_STR,"������Ϣ",MSG_BroadCast_Obj.INFO_LEN);  
	Api_RecordNum_Write(msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 
	
	MSG_BroadCast_Obj.INFO_TYPE=3;//����
	memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR));
	memcpy(MSG_BroadCast_Obj.INFO_STR,"��ͨ��Ϣ",MSG_BroadCast_Obj.INFO_LEN);  
	Api_RecordNum_Write(msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 
	
	MSG_BroadCast_Obj.INFO_TYPE=4;//����
	memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR));
	memcpy(MSG_BroadCast_Obj.INFO_STR,"��ʳ��Ϣ",MSG_BroadCast_Obj.INFO_LEN);  
	Api_RecordNum_Write(msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 

	MSG_BroadCast_Obj.INFO_Effective=0;//��ʾ��Ч
	MSG_BroadCast_Obj.INFO_TYPE=5;//����
	memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR));
	memcpy(MSG_BroadCast_Obj.INFO_STR,"��¼��Ϣ",MSG_BroadCast_Obj.INFO_LEN);  
	Api_RecordNum_Write(msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 
	
	MSG_BroadCast_Obj.INFO_TYPE=6;//����
	memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR));
	memcpy(MSG_BroadCast_Obj.INFO_STR,"�¼���Ϣ",MSG_BroadCast_Obj.INFO_LEN);  
	Api_RecordNum_Write(msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 
	
	MSG_BroadCast_Obj.INFO_TYPE=7;//����
	memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR));
	memcpy(MSG_BroadCast_Obj.INFO_STR,"ʱ����Ϣ",MSG_BroadCast_Obj.INFO_LEN);  
	Api_RecordNum_Write(msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 
	
	MSG_BroadCast_Obj.INFO_TYPE=8;//����
	memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR));
	memcpy(MSG_BroadCast_Obj.INFO_STR,"������Ϣ",MSG_BroadCast_Obj.INFO_LEN);  
	Api_RecordNum_Write(msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 
}

void MSG_BroadCast_Read(void) 
{
  u8 i=0;
  
  for(i=0;i<8;i++)
    {
	memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR)); 
	Api_RecordNum_Read(msg_broadcast, i+1, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 

	MSG_Obj_8[i].INFO_TYPE=MSG_BroadCast_Obj.INFO_TYPE;
	MSG_Obj_8[i].INFO_LEN=MSG_BroadCast_Obj.INFO_LEN;
	MSG_Obj_8[i].INFO_PlyCancel=MSG_BroadCast_Obj.INFO_PlyCancel;
	MSG_Obj_8[i].INFO_SDFlag=MSG_BroadCast_Obj.INFO_SDFlag;
	MSG_Obj_8[i].INFO_Effective=MSG_BroadCast_Obj.INFO_Effective;
       memcpy(MSG_Obj_8[i].INFO_STR,MSG_BroadCast_Obj.INFO_STR,sizeof(MSG_BroadCast_Obj.INFO_STR));
	
	//memcpy(DisInfor_Menu[i],MSG_BroadCast_Obj.INFO_STR,20);    
	//rt_kprintf("\r\n ��ϢTYPE:%d  ����:%d  �Ƿ�㲥:%d �Ƿ���ʾ��Ч:%d(1��ʾ0����ʾ) Info: %s",MSG_BroadCast_Obj.INFO_TYPE,MSG_BroadCast_Obj.INFO_LEN,MSG_BroadCast_Obj.INFO_PlyCancel,MSG_BroadCast_Obj.INFO_Effective,MSG_BroadCast_Obj.INFO_STR); 
    }
}

void MSG_BroadCast_Init(u8  Intype)
{
  u8 i=0;

	  if(Intype==0)
	  {
	    MSG_BroadCast_Obj.INFO_LEN=8; 
		MSG_BroadCast_Obj.INFO_Effective=1;
	    memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR)); 
	    memcpy(MSG_BroadCast_Obj.INFO_STR,"�������",8);
	  } 
	  else
	  if(Intype==1)
		{
		  MSG_BroadCast_Obj.INFO_LEN=0;  		  
		  MSG_BroadCast_Obj.INFO_Effective=0;   
		  memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR));    
		}
	  
   for(i=0;i<8;i++)
   {
       MSG_BroadCast_Obj.INFO_TYPE=i+1;
	Api_RecordNum_Write(msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 
   }
}

/*
      �绰��
*/
void PhoneBook_Read(void)
{
  u8 i=0;

  for(i=0;i<8;i++)
  {    
      	Api_RecordNum_Read(phonebook, i+1, (u8*)&PhoneBook_8[i],sizeof(PhoneBook)); 
	//rt_kprintf("\r\n\r\n �绰�� TYPE: %d   Numlen=%d  Num: %s   UserLen: %d  UserName:%s \r\n",PhoneBook.CALL_TYPE,PhoneBook.NumLen,PhoneBook.NumberStr,PhoneBook.UserLen,PhoneBook.UserStr);  
  }
} 
void PhoneBook_Init(u8  Intype)
{
  u8 i=0;

	  if(Intype==0)
	  {
	    PhoneBook.Effective_Flag=1;  //��Ч��־λ
           PhoneBook.CALL_TYPE=2; //���Ͷ���Ϊ��� 
	    PhoneBook.NumLen=5;    // ���볤��
	    memset(PhoneBook.NumberStr,0,sizeof(PhoneBook.NumberStr)); // ��������
	    memcpy(PhoneBook.NumberStr,"10086",5);
		PhoneBook.UserLen=8;		// �û�������
	    memset(PhoneBook.UserStr,0,sizeof(PhoneBook.UserStr)); // �û�������
	    memcpy(PhoneBook.UserStr,"�й��ƶ�",8); 
		
	  } 
	  else
	  if(Intype==1)
		{
		   PhoneBook.Effective_Flag=0;  //��Ч��־λ 
		   PhoneBook.CALL_TYPE=2; //���Ͷ���Ϊ���  
	       PhoneBook.NumLen=0;    // ���볤��
	       memset(PhoneBook.NumberStr,0,sizeof(PhoneBook.NumberStr));  
		   PhoneBook.UserLen=0;		
	       memset(PhoneBook.UserStr,0,sizeof(PhoneBook.UserStr)); 	
		}
	  
   for(i=0;i<8;i++)
   {
       	Api_RecordNum_Write(phonebook, i+1, (u8*)&PhoneBook,sizeof(PhoneBook)); 
   }
}

/*
       Բ��Χ��
*/
//---------------------------------------------------------- 
void  RailCycle_Init(void)
{
  u8 i=0;
  
   Rail_Cycle.Area_ID=0;
   Rail_Cycle.Area_attribute=0; // Bit 0  ��ʾ����ʱ��  ���ֶ�Ϊ0 ��ʾΧ��û������
   Rail_Cycle.Center_Latitude=0;
   Rail_Cycle.Center_Longitude=0;   
   Rail_Cycle.Radius=100; // �뾶   
   Time2BCD(Rail_Cycle.StartTimeBCD);
   Time2BCD(Rail_Cycle.EndTimeBCD);
   Rail_Cycle.MaxSpd=100; // ����ٶ�
   Rail_Cycle.KeepDur=30; // ���ٳ���ʱ��
   Rail_Cycle.Rail_Num=8;
   Rail_Cycle.Effective_flag=0; 

   for(i=0;i<8;i++)
   {
     Rail_Cycle.Area_ID=i+1;
     Api_RecordNum_Write(Rail_cycle,Rail_Cycle.Area_ID, (u8*)&Rail_Cycle,sizeof(Rail_Cycle)); 	 
   }
}
void  RailCycle_Read(void)   
{
  u8 i=0;

   for(i=0;i<8;i++)
   {
          Api_RecordNum_Read(Rail_cycle,i+1, (u8*)&Rail_Cycle,sizeof(Rail_Cycle)); 		  
		//  rt_kprintf("\r\n\r\n Բ��Χ�� TYPE: %d    atrri=%d  lati: %d  longiti:%d  radicus:%d	maxspd: %d  keepdur:%d \r\n",Rail_Cycle.Area_ID,Rail_Cycle.Area_attribute,Rail_Cycle.Center_Latitude,Rail_Cycle.Center_Longitude,Rail_Cycle.Radius,Rail_Cycle.MaxSpd,Rail_Cycle.KeepDur);  
   }
}

/*
       ����Χ��
*/
void  RailRect_Init(void)
{
  u8 i=0;
  
   Rail_Rectangle.Area_ID=0;
   Rail_Rectangle.Area_attribute=0; // Bit 0  ��ʾ����ʱ��  ���ֶ�Ϊ0 ��ʾΧ��û������
   Rail_Rectangle.LeftUp_Latitude=0; // ����
   Rail_Rectangle.LeftUp_Longitude=0;   
   Rail_Rectangle.RightDown_Latitude=0; //  ���� 
   Rail_Rectangle.RightDown_Longitude=0;  
   Time2BCD(Rail_Rectangle.StartTimeBCD);
   Time2BCD(Rail_Rectangle.EndTimeBCD);
   Rail_Rectangle.MaxSpd=100; // ����ٶ�
   Rail_Rectangle.KeepDur=30; // ���ٳ���ʱ��
   Rail_Rectangle.Rail_Num=8;  
   Rail_Rectangle.Effective_flag=0;

   for(i=0;i<8;i++)
   {
     Rail_Rectangle.Area_ID=i+1;
     Api_RecordNum_Write(Rail_rect,Rail_Rectangle.Area_ID, (u8*)&Rail_Rectangle,sizeof(Rail_Rectangle)); 	 
   }      
   
}

void  RailRect_Read(void)
{
  u8 i=0;

   for(i=0;i<8;i++)
   {
	 Api_RecordNum_Read(Rail_rect,i+1, (u8*)&Rail_Rectangle,sizeof(Rail_Rectangle)); 	 	  
        //  rt_kprintf("\r\n\r\n ������Χ�� TYPE: %d    atrri=%d  leftlati: %d  leftlongiti:%d    rightLati:%d   rightLongitu: %d  \r\n",Rail_Rectangle.Area_ID,Rail_Rectangle.Area_attribute,Rail_Rectangle.LeftUp_Latitude,Rail_Rectangle.LeftUp_Longitude,Rail_Rectangle.RightDown_Latitude,Rail_Rectangle.RightDown_Longitude);  
   }  
}  

/*
       �����Χ��
*/
void  RailPolygen_Init(void)
{
  u8 i=0;
  
   Rail_Polygen.Area_ID=0;
   Rail_Polygen.Area_attribute=0; // Bit 0  ��ʾ����ʱ��  ���ֶ�Ϊ0 ��ʾΧ��û������
   Time2BCD(Rail_Polygen.StartTimeBCD);
   Time2BCD(Rail_Polygen.EndTimeBCD);
   Rail_Polygen.MaxSpd=100; // ����ٶ�
   Rail_Polygen.KeepDur=30; // ���ٳ���ʱ��
   Rail_Polygen.Acme_Num=3;   
   Rail_Polygen.Acme1_Latitude=10; //����1 
   Rail_Polygen.Acme1_Longitude=10;   
   Rail_Polygen.Acme2_Latitude=20; //����2
   Rail_Polygen.Acme2_Longitude=20;     
   Rail_Polygen.Acme3_Latitude=30; //����3
   Rail_Polygen.Acme3_Longitude=30;  
   Rail_Polygen.Effective_flag=0;

   for(i=0;i<8;i++)
   {
     Rail_Polygen.Area_ID=i+1;
     Api_RecordNum_Write(Rail_polygen,Rail_Polygen.Area_ID, (u8*)&Rail_Polygen,sizeof(Rail_Polygen)); 	 
   }
}

void  RailPolygen_Read(void)
{
  u8 i=0;

   for(i=0;i<8;i++)
   {
       Api_RecordNum_Read(Rail_polygen,i+1, (u8*)&Rail_Polygen,sizeof(Rail_Polygen)); 	 		  
       //  rt_kprintf("\r\n\r\n �����Χ�� TYPE: %d   1lat: %d  1long:%d    2lat:%d   2long: %d  3lat:%d 3long:%d \r\n",Rail_Polygen.Area_ID,Rail_Polygen.Acme1_Latitude,Rail_Polygen.Acme1_Longitude,Rail_Polygen.Acme2_Latitude,Rail_Polygen.Acme2_Longitude,Rail_Polygen.Acme3_Latitude,Rail_Polygen.Acme3_Longitude);   
   }
} 

/*
        �յ�����    (Maybe Null)
*/
      




/*
        ·������Χ�� 
*/
void  RouteLine_Obj_init(void)
{
  u8 i=0;
  
   ROUTE_Obj.Route_ID=0;
   ROUTE_Obj.Route_attribute=0; // Bit 0  ��ʾ����ʱ��  ���ֶ�Ϊ0 ��ʾΧ��û������
   Time2BCD(ROUTE_Obj.StartTimeBCD);
   Time2BCD(ROUTE_Obj.EndTimeBCD);
   ROUTE_Obj.Points_Num=3;   
   for(i=0;i<3;i++)
   	{
   	  ROUTE_Obj.RoutePoints[i].POINT_ID=i+1;
	  ROUTE_Obj.RoutePoints[i].Line_ID=i;
	  ROUTE_Obj.RoutePoints[i].POINT_Latitude=i+300;
	  ROUTE_Obj.RoutePoints[i].POINT_Longitude=i+500;
	  ROUTE_Obj.RoutePoints[i].Width=20;
	  ROUTE_Obj.RoutePoints[i].Atribute=0; // 0 ��ʾδ����
	  ROUTE_Obj.RoutePoints[i].TooLongValue=100;
	  ROUTE_Obj.RoutePoints[i].TooLessValue=50;
	  ROUTE_Obj.RoutePoints[i].MaxSpd=60;
	  ROUTE_Obj.RoutePoints[i].KeepDur=3;
	  ROUTE_Obj.Effective_flag=0; 
   	}

}
 
 void  RouteLine_Init(void)
{
  u8 i=0;
  
    RouteLine_Obj_init();

   for(i=0;i<8;i++)
   {
     ROUTE_Obj.Route_ID=i+1;
     Api_RecordNum_Write(route_line,ROUTE_Obj.Route_ID, (u8*)&ROUTE_Obj,sizeof(ROUTE_Obj)); 	 
   }
}

void  RouteLine_Read(void)
{
  u8 i=0;

   for(i=0;i<8;i++)
   {
       Api_RecordNum_Read(route_line,i+1, (u8*)&ROUTE_Obj,sizeof(ROUTE_Obj)); 	 
	//  rt_kprintf("\r\n\r\n ·�� TYPE: %d   pointsNum:%d   p1lat: %d  p1long:%d    p2lat:%d   p2long: %d  p3lat:%d p3long:%d \r\n",ROUTE_Obj.Route_ID,ROUTE_Obj.Points_Num,ROUTE_Obj.RoutePoints[0].POINT_Latitude,ROUTE_Obj.RoutePoints[0].POINT_Longitude,ROUTE_Obj.RoutePoints[1].POINT_Latitude,ROUTE_Obj.RoutePoints[1].POINT_Longitude,ROUTE_Obj.RoutePoints[2].POINT_Latitude,ROUTE_Obj.RoutePoints[2].POINT_Longitude);   
   }
}    

/*
       ����
*/

void Question_Read(void)
{
	//������Ϣ	 ����
	  Api_RecordNum_Read(ask_quesstion,1, (u8*)&ASK_Centre,sizeof(ASK_Centre)); 	
	  rt_kprintf("\r\n��־λ:%d  ��ˮ��:%d  ��Ϣ����:%d �ظ�ID:%d",ASK_Centre.ASK_SdFlag,ASK_Centre.ASK_floatID,ASK_Centre.ASK_infolen,ASK_Centre.ASK_answerID); 
	if(ASK_Centre.ASK_SdFlag==1)
	  {
	  ASK_Centre.ASK_SdFlag=0;
	  rt_kprintf("\r\n��Ϣ����: %s",ASK_Centre.ASK_info); 
	  rt_kprintf("\r\n��ѡ��: %s",ASK_Centre.ASK_answer); 
	  }
}
 
 

/*
        �ı���Ϣ
*/

void TEXTMsg_Read (void)  
{
  u8 i=0,min=0,max=0;
  
		  for(i=0;i<=7;i++)
		    {
                      Api_RecordNum_Read(text_msg,i+1, (u8*)&TEXT_Obj,sizeof(TEXT_Obj)); 			
			memcpy((u8*)&TEXT_Obj_8bak[i],(u8*)&TEXT_Obj,sizeof(TEXT_Obj));     
		    //rt_kprintf("\r\n�ı���Ϣ ����:%d  ��ϢTYPE:%d  ����:%d",TEXT_Obj_8bak[i].TEXT_mOld,TEXT_Obj_8bak[i].TEXT_TYPE,TEXT_Obj_8bak[i].TEXT_LEN);  
		    }

		  //����һ������
		  max=TEXT_Obj_8bak[0].TEXT_TYPE;    
		  for(i=0;i<=7;i++)
		  	{
		  	if(TEXT_Obj_8bak[i].TEXT_TYPE>max)
				max=TEXT_Obj_8bak[i].TEXT_TYPE;
		  	}
		  TextInforCounter=max;
		  //rt_kprintf("\r\n  �����������  max=%d,TextInforCounter=%d",max,TextInforCounter); 
		  

		  //�ҳ������һ����Ϣ
		  min=TEXT_Obj_8bak[0].TEXT_TYPE;
		  //rt_kprintf("\r\n  ����ǰ  ����һ����Ϣ���  min=%d",min); 
		  for(i=0;i<=7;i++)
		  	{
		  	if((TEXT_Obj_8bak[i].TEXT_TYPE<min)&&(TEXT_Obj_8bak[i].TEXT_TYPE>0))
				min=TEXT_Obj_8bak[i].TEXT_TYPE;
		  	}
		  //rt_kprintf("\r\n	�����	����һ����Ϣ���  min=%d",min); 
  
  if(max<1)return;
  
  if(max<=8)
  	{
  	for(i=1;i<=max;i++)
		{
		Api_RecordNum_Read(text_msg,max-i+1, (u8*)&TEXT_Obj,sizeof(TEXT_Obj)); 	
		memcpy((u8*)&TEXT_Obj_8[i-1],(u8*)&TEXT_Obj,sizeof(TEXT_Obj));
		//rt_kprintf("\r\n(<8)��ϢTYPE:%d  ����:%d",TEXT_Obj_8[i-min].TEXT_TYPE,TEXT_Obj_8[i-min].TEXT_LEN);   
		}
  	}
  else
  	{
  	  max=max%8;
	  //rt_kprintf("\r\n max%8=%d",max);   

	  if(max==0)
	  	{
  		  for(i=0;i<8;i++)
			{
			Api_RecordNum_Read(text_msg,(7-i), (u8*)&TEXT_Obj,sizeof(TEXT_Obj)); 	
			memcpy((u8*)&TEXT_Obj_8[i],(u8*)&TEXT_Obj,sizeof(TEXT_Obj));
			//rt_kprintf("\r\n(8*n)i=%d ��ϢTYPE:%d  ����:%d",i,TEXT_Obj_8[i].TEXT_TYPE,TEXT_Obj_8[i].TEXT_LEN);   
			}
	  	}
	  else
	  	{
		  for(i=0;i<max;i++)
			{
			Api_RecordNum_Read(text_msg,(max-i), (u8*)&TEXT_Obj,sizeof(TEXT_Obj)); 
			memcpy((u8*)&TEXT_Obj_8[i],(u8*)&TEXT_Obj,sizeof(TEXT_Obj));
			//rt_kprintf("\r\n(1)��ϢTYPE:%d  ����:%d",TEXT_Obj_8[i].TEXT_TYPE,TEXT_Obj_8[i].TEXT_LEN);   
			}
		  for(i=7;i>=max;i--)
			{
			Api_RecordNum_Read(text_msg,i+1, (u8*)&TEXT_Obj,sizeof(TEXT_Obj)); 
			memcpy((u8*)&TEXT_Obj_8[max+7-i],(u8*)&TEXT_Obj,sizeof(TEXT_Obj));
			//rt_kprintf("\r\n(2)��ϢTYPE:%d  ����:%d",TEXT_Obj_8[max+7-i].TEXT_TYPE,TEXT_Obj_8[max+7-i].TEXT_LEN);  
			}
	  	}
	  }
}

void TEXTMSG_Write(u8 num,u8 new_state,u8 len,u8 *str)   
{     //  д������Ϣ
u8 pos_1_8=0;//,i=0;
    //�¼�д��
       TEXT_Obj.TEXT_mOld=new_state;//�Ƿ���������Ϣ
	TEXT_Obj.TEXT_TYPE=num;// 1
	TEXT_Obj.TEXT_LEN=len; //��Ϣ����
	memset(TEXT_Obj.TEXT_STR,0,sizeof(TEXT_Obj.TEXT_STR));
	memcpy(TEXT_Obj.TEXT_STR,str,TEXT_Obj.TEXT_LEN); 
	
	if(num%8)
		pos_1_8=TEXT_Obj.TEXT_TYPE%8;
	else
		pos_1_8=8; 
        Api_RecordNum_Write(text_msg,pos_1_8, (u8*)&TEXT_Obj,sizeof(TEXT_Obj)); 
}

void TEXTMSG_Write_Init(void)
{
    u8  i=0;

       TEXT_Obj.TEXT_mOld=1;//�Ƿ���������Ϣ
	TEXT_Obj.TEXT_TYPE=1;// 1
	TEXT_Obj.TEXT_LEN=0; //��Ϣ����
	memset(TEXT_Obj.TEXT_STR,0,sizeof(TEXT_Obj.TEXT_STR));   

	for(i=0;i<8;i++)
	{
	     //-------------------------------------------------------
             if(i==0)
			  TEXT_Obj.TEXT_mOld=1;//�Ƿ���������Ϣ 	
	      else
		  	    TEXT_Obj.TEXT_mOld=0;//�Ƿ���������Ϣ
           //-------------------------------------------------------
           TEXT_Obj.TEXT_TYPE=i+1;// 1
           Api_RecordNum_Write(text_msg,TEXT_Obj.TEXT_TYPE, (u8*)&TEXT_Obj,sizeof(TEXT_Obj)); 

	}
}


/*
     ��ý���������õ�ʱ���ٲ��� write / read   ��ʼ�������ô���
*/




void  BD_EXT_initial(void)
{
      //    ��������
    BD_EXT.BD_Mode=0x02;     //   ˫ģ
    BD_EXT.BD_Baud=0x01;     //   9600
    BD_EXT.BD_OutputFreq=0x01;  // 1000ms
    BD_EXT.BD_SampleFrea=1; // 
    BD_EXT.BD_Baud=0x01;  //  9600
    //-----  ��̨��� ----------------------
    BD_EXT.Termi_Type=0x0001;   //  �ն�����
    BD_EXT.Software_Ver=0x0100; //  Ver  1.00
    BD_EXT.GNSS_Attribute=0x54444244;// TDBD
    BD_EXT.GSMmodule_Attribute=0x48554157;// HUAW
    BD_EXT.Device_Attribute=0x00000001; //  �ն�����

     //   CAN   �������
     BD_EXT.CAN_1_Mode=0xC0000014;    //  CAN  ģʽ  01:����ģʽ  10  : ��ͨģʽ   11:  ��Ĭģʽ  
          /*  bit31  ����  bit 30 -29   ģʽ   bit 15-0   ������0x14   <=> 20k*/
     BD_EXT.CAN_1_ID=0x01;
     BD_EXT.CAN_1_Type=0;// ��չ֡
     BD_EXT.CAN_1_Duration=1;  // 0   ��ʾֹͣ
     BD_EXT.CAN_2_Mode=0xC0000014;   //   CAN  ģʽ 
     BD_EXT.CAN_2_ID=0x02;
     BD_EXT.CAN_2_Type=0;// ��չ֡	 
     BD_EXT.CAN_2_Duration=1;  // 0   ��ʾֹͣ
     BD_EXT.Collision_Check=0x0101;     //     �ر���  0.1g    4ms

  //   λ�ø�����Ϣ
       // 1. �ź�ǿ��
      BD_EXT.FJ_SignalValue=0x0000;  //  �ź�ǿ��   ���ֽ� 0 �����ֽ�  ��4Ϊ X2 gprsǿ�� ����4λ ���ǿ���
      //  2. �Զ���״̬��ģ�����ϴ�  
       BD_EXT.FJ_IO_1=0x00;
	BD_EXT.FJ_IO_2=0x00;
	BD_EXT.AD_0=0x00;  //  2 Byte
	BD_EXT.AD_1=0x00; //   2 Byte
 
       BD_EXT.Close_CommunicateFlag=disable; 
	BD_EXT.Trans_GNSS_Flag=disable;

       Api_Config_Recwrite_Large(BD_ext_config,0,(u8*)&BD_EXT,sizeof(BD_EXT));   
       DF_delay_ms(5);
 
	
}

void BD_EXT_Write(void)
{
    Api_Config_Recwrite_Large(BD_ext_config,0,(u8*)&BD_EXT,sizeof(BD_EXT));   
    DF_delay_ms(15);
}

void BD_EXT_Read(void)
{
    Api_Config_read(BD_ext_config,0,(u8*)&BD_EXT,sizeof(BD_EXT));  	 
}

void  BD_list(void)
{ 
     u8  BD_str[20];
	 
     //-----list -----
     rt_kprintf("\r\n -------������չ��Ϣ���------\r\n ");
     rt_kprintf("\r\n\r\n	 �ն�����:    0x%08X      \r\n       ����汾:   0x%08X  \r\n",BD_EXT.Termi_Type,BD_EXT.Software_Ver);  
     rt_kprintf("\r\n\r\n	 GNNS����:   %04s      \r\n       GSM����:  %04s    \r\n   �ն�����:  0x%08X \r\n\r\n    ",(char*)&BD_EXT.GNSS_Attribute,(char*)&BD_EXT.GSMmodule_Attribute,BD_EXT.Device_Attribute);   

     memset(BD_str,0,sizeof(BD_str));
     switch(BD_EXT.CAN_1_Mode&0x60000000)
     	{
     	    case 0x20000000: 
				memcpy(BD_str,"����ģʽ",8);
			        break;
	    case 0x40000000:
                           	memcpy(BD_str,"��ͨģʽ",8);
			        break;
	    case 0x6000000:
                           	memcpy(BD_str,"��Ĭģʽ",8);
				break; 

     	}
     rt_kprintf("\r\n   CAN1 :\r\n            CAN1  Mode:    %s      \r\n             CAN1  ID:    0x%08X      \r\n\r\n ",BD_str,BD_EXT.CAN_1_ID);  

      memset(BD_str,0,sizeof(BD_str));
     switch(BD_EXT.CAN_2_Mode&0x60000000)
     	{
     	    case 0x20000000: 
				memcpy(BD_str,"����ģʽ",8);
			        break;
	    case 0x40000000:
                           	memcpy(BD_str,"��ͨģʽ",8);
			        break;
	    case 0x6000000:
                           	memcpy(BD_str,"��Ĭģʽ",8);
				break; 

     	}
     rt_kprintf("\r\n   CAN2 :\r\n            CAN2  Mode:    %s      \r\n             CAN2  ID:    0x%08X      \r\n\r\n ",BD_str,BD_EXT.CAN_2_ID);

     

}

FINSH_FUNCTION_EXPORT(BD_list, BD status); 


void SendMode_Config(void)     //  ���ͷ�ʽ���� 
{
     if(JT808Conf_struct.RT_LOCK.Lock_state==1)
     {
       JT808Conf_struct.SD_MODE.DUR_TOTALMODE=1;
       Current_SD_Duration=JT808Conf_struct.RT_LOCK.Lock_Dur;
	   if(JT808Conf_struct.RT_LOCK.Lock_KeepCnter>JT808Conf_struct.RT_LOCK.Lock_KeepDur)
	   	{
		   Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
	   	} 
	   else	   
	      return;
     }
    if(1==JT808Conf_struct.SD_MODE.DUR_TOTALMODE)
    {
        
		if(1==JT808Conf_struct.SD_MODE.Dur_EmegencMode)
        {
            Current_SD_Duration=JT808Conf_struct.DURATION.Emegence_Dur;
			return;
        }
        if(1==JT808Conf_struct.SD_MODE.Dur_DefaultMode)
        {
            Current_SD_Duration=JT808Conf_struct.DURATION.Default_Dur;
			return;
        }
        if(1==JT808Conf_struct.SD_MODE.Dur_NologinMode)
        {
            Current_SD_Duration=JT808Conf_struct.DURATION.NoDrvLogin_Dur;
			return;
        }
		if(1==JT808Conf_struct.SD_MODE.Dur_SleepMode)
        {
            Current_SD_Duration=JT808Conf_struct.DURATION.Sleep_Dur;
			return;
        }

    }
	else
	if(1==JT808Conf_struct.SD_MODE.DIST_TOTALMODE)
	{
        if(1==JT808Conf_struct.SD_MODE.Dist_EmgenceMode) 
        {
            Current_State=JT808Conf_struct.DISTANCE.Emergen_Dist;
			return;
        }
		if(1==JT808Conf_struct.SD_MODE.Dist_DefaultMode) 
        {
            Current_State=JT808Conf_struct.DISTANCE.Defalut_DistDelta;
			return;
        }
        if(1==JT808Conf_struct.SD_MODE.Dist_NoLoginMode) 
        {
            Current_State=JT808Conf_struct.DISTANCE.NoDrvLogin_Dist;
			return;
        }
	    if(1==JT808Conf_struct.SD_MODE.Dist_SleepMode) 
        {
            Current_State=JT808Conf_struct.DISTANCE.Sleep_Dist; 
			return;
        }
	   
	}

}

void  SendMode_ConterProcess(void)         //  ��ʱ���ʹ������
{   //   ���ͷ�ʽ����������
   //  1. ������������
    JT808Conf_struct.DURATION.Heart_SDCnter++;       
    if(JT808Conf_struct.DURATION.Heart_SDCnter>JT808Conf_struct.DURATION.Heart_Dur)  //�������������õļ��
      	{
            JT808Conf_struct.DURATION.Heart_SDCnter=0;     
            JT808Conf_struct.DURATION.Heart_SDFlag=1; 
    	}
   //  2. ���ͳ�ʱ�ж�
    if(1==JT808Conf_struct.DURATION.TCP_SD_state)
    {
      JT808Conf_struct.DURATION.TCP_ACK_DurCnter++;
	  if(JT808Conf_struct.DURATION.TCP_ACK_DurCnter>JT808Conf_struct.DURATION.TCP_ACK_Dur) //����Ӧ��ʱ
	  	{
          JT808Conf_struct.DURATION.TCP_ACK_DurCnter=0;
		  JT808Conf_struct.DURATION.Heart_SDFlag=1;         //���·���
		  JT808Conf_struct.DURATION.TCP_ReSD_cnter++;
		  if(JT808Conf_struct.DURATION.TCP_ReSD_cnter>JT808Conf_struct.DURATION.TCP_ReSD_Num)  //���·��ʹ����ж�
		  	{
               JT808Conf_struct.DURATION.TCP_ReSD_cnter=0;
			   Close_DataLink();   // AT_End();	   //�Ҷ�GPRS����    

		  	}
	  	}
 
    }
}





//-----------------------------------------------------------------

void  FirstRun_Config_Write(void)
{     //   �����״θ�����д���ò��� 

       //  rt_kprintf("\r\n  sizeof(sysconfig): %d   sizeof(jt808): %d    sizeof(tiredconfig): %d   \r\n",sizeof(SysConf_struct),sizeof(JT808Conf_struct),sizeof(TiredConf_struct)); 
                  SysConfig_init();   //  д��ϵͳ������Ϣ
                  TIRED_CONF_Init(); //  д��ƣ�ͼ�ʻ���������Ϣ
                 JT808_Conf_init();   //  д�� JT808   ������Ϣ
                  Api_WriteInit_var_rd_wr();	
		    BD_EXT_initial(); 		  
                 Event_Write_Init();
		   MSG_BroadCast_Write_Init();
		   PhoneBook_Init(0);  
		   RailCycle_Init();		   
		   RailRect_Init();
		   RailPolygen_Init();	
		   RouteLine_Init(); 
                 TEXTMSG_Write_Init();	   
 		 

}
//-----------------------------------------------------------------
void SetConfig(void)
{
       u8  res=0;
	u8 Reg_buf[22];	 
	u8 i=0;//,len_write=0;
//	u32 j=0;
	
       rt_kprintf("\r\nSave Config\r\n");
	// 1.  ��ȡconfig ����      0 :�ɹ�    1 :  ʧ��
	res=Api_Config_read(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));  
       //rt_kprintf("\r\nRead Save SYSID\r\n");
       //  2. ��ȡ�ɹ�  ���ж�  �汾ID 
	if(SysConf_struct.Version_ID!=SYSID)//SYSID)   //  check  wether need  update  or not 
	{
	       rt_kprintf("\r\n ID not Equal   Saved==0x%X ,  Read==0x%X !\r\n",SYSID,SysConf_struct.Version_ID);	
	        SysConf_struct.Version_ID=SYSID;  // update  ID 
             //  2.1   ��ɾ�� 
               // Delete_exist_Directory();
	     //  2.2  ����д��		 
                 FirstRun_Config_Write();   // ��߸����� SYSID                 
	  	   rt_kprintf("\r\nSave Over!\r\n");
	}
	else			
		   rt_kprintf("\r\n Config Already Exist!\r\n"); 
}

 void ReadConfig(void) 
{

                 Api_Config_read(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   //  ��ȡJT808   ������Ϣ
                 SysConfig_Read();  //��ȡϵͳ������Ϣ	                 
                 TIRED_DoorValue_Read();          		   
                 Event_Read();
		   MSG_BroadCast_Read();
		   PhoneBook_Read();  
		  // RailCycle_Read();
 		   //RailPolygen_Read();
 		   //RailRect_Read();
 		   //RouteLine_Read();   
                 TEXTMsg_Read();	 
                 BD_EXT_Read();   
		   Api_Read_var_rd_wr();	  	  	   

		   
		   if(JT808Conf_struct.DF_K_adjustState)  
		   {
                             ModuleStatus|=Status_Pcheck; 
		   }
                 else 
		   {
		              ModuleStatus&=~Status_Pcheck;
                  } 
                 
       
    rt_kprintf("\r\n Read Config Over \r\n");   
}
void DefaultConfig(void)
{
   u32 DriveCode32=0;
   u8  reg_str[30];

       rt_kprintf("\r\n         SYSTEM ID=0x%X ",SysConf_struct.Version_ID);   
	rt_kprintf("\r\n		   ���õļ�Ȩ��Ϊ: ");
       rt_kprintf(" %s\r\n		   ��Ȩ�볤��: %d\r\n",JT808Conf_struct.ConfirmCode,strlen((const char*)JT808Conf_struct.ConfirmCode));   
					   
  if(JT808Conf_struct.Regsiter_Status)
  	   rt_kprintf("\r\n		   ���ն��Ѿ�ע���!    %d \r\n",JT808Conf_struct.Regsiter_Status);  
  else
  	   rt_kprintf("\r\n		   ���ն˱���δ��ע��!\r\n");   
        // APN ����
	  rt_kprintf("\r\n		   APN ���� :%s	 \r\n",APN_String); 
         DataLink_APN_Set(APN_String,1); 
            //     ����
          memset(reg_str,0,sizeof(reg_str));
          memcpy(reg_str,DomainNameStr,strlen((char const*)DomainNameStr));
          rt_kprintf("\r\n		  �������� :	 %s\r\n",reg_str); 
	      //    ����aux
          memset(reg_str,0,sizeof(reg_str));
          memcpy(reg_str,DomainNameStr_aux,strlen((char const*)DomainNameStr_aux));
          rt_kprintf("\r\n		aux  �������� :	 %s\r\n",reg_str); 	    
					  
         // ��������IP ��ַ(4Bytes)  UDP�˿ں���(2Bytes) TCP�˿ں���(2Bytes)
         rt_kprintf("\r\n		  ��IP: %d.%d.%d.%d : %d \r\n",RemoteIP_main[0],RemoteIP_main[1],RemoteIP_main[2],RemoteIP_main[3],RemotePort_main);   
	  DataLink_MainSocket_set(RemoteIP_main, RemotePort_main,0);
	  rt_kprintf("\r\n		   ����IP: %d.%d.%d.%d : %d \r\n",RemoteIP_aux[0],RemoteIP_aux[1],RemoteIP_aux[2],RemoteIP_aux[3],RemotePort_aux);   
         DataLink_AuxSocket_set(RemoteIP_aux, RemotePort_main,0);	 
	  //  ACC On �ϱ����(2Bytes)  ACC Off �ϱ����(2Bytes)
	  rt_kprintf("\r\n		   ACC on ���ͼ��Ϊ: %d S\r\n		   ACC Off ���ͼ��Ϊ: %d S\r\n",ACC_on_sd_Duration,ACC_off_sd_Duration);
					  
          //  ����ͣ������(2Bytes)	
	  rt_kprintf("\r\n		   ����ͣ������: %d S\r\n",StopLongDuration);
	   //  ������ش�ʹ�ܱ�־λ(1Bytes) �� �������ϴ�����(4Bytes)  ��λ: m ��  
         rt_kprintf("\r\n         ��������: %s \r\n",JT808Conf_struct.LISTEN_Num);  


	   
          //   ��¼��ΨһID
         rt_kprintf("\r\n		   ����Ψһ�Ա��: %35s \r\n",JT808Conf_struct.DeviceOnlyID);    
         rt_kprintf("\r\n		   �ϱ��ٶȻ�ȡ��ʽ: %d :",JT808Conf_struct.Speed_GetType);	  
         if(JT808Conf_struct.Speed_GetType)							
                rt_kprintf("����ͨ�������ٶȴ�������ȡ!\r\n");
         else
                rt_kprintf("����ͨ��GPSģ���ȡ!\r\n"); 


         rt_kprintf(" \r\n  ------------------------GPS ����ź�Դ״̬ JT808Conf_struct.OutGPS_Flag= %d",JT808Conf_struct.OutGPS_Flag);  

         rt_kprintf("\r\n		   ����ϵ��У׼״̬: %d :",JT808Conf_struct.DF_K_adjustState);	    
         if(JT808Conf_struct.DF_K_adjustState)					  	 
                  rt_kprintf(" ����ϵ��--У׼�ɹ�!\r\n");
         else
                  rt_kprintf("����ϵ��--��δУ׼!\r\n");   

	  //   ���
         rt_kprintf("\r\n		   �ۼ����: %d  ��   ,  �������:   %d��\r\n",JT808Conf_struct.Distance_m_u32,JT808Conf_struct.Distance_m_u32-JT808Conf_struct.DayStartDistance_32);  	
         //  �ٶ�����
         rt_kprintf("		   ��������ٶ�: %d  Km/h    ���ٱ�������ʱ������: %d  s \r\n", JT808Conf_struct.Speed_warn_MAX,JT808Conf_struct.Spd_Exd_LimitSeconds);  		 

         rt_kprintf("\r\n		  ���ұ�׼�汾: %14s \r\n",JT808Conf_struct.StdVersion.stdverStr);					  
         rt_kprintf("\r\n		  ���ұ�׼�汾�޸ĵ���: %d \r\n",JT808Conf_struct.StdVersion.MdfyID); 


         rt_kprintf("\r\n		  ����ϵ��(�ٶ�����ϵ��): %d \r\n",JT808Conf_struct.Vech_Character_Value); 					  
         rt_kprintf("\r\n		  ���ΰ�װ����: %X-%X-%X %X:%X:%X \r\n",JT808Conf_struct.FirstSetupDate[0],JT808Conf_struct.FirstSetupDate[1],JT808Conf_struct.FirstSetupDate[2],JT808Conf_struct.FirstSetupDate[3],JT808Conf_struct.FirstSetupDate[4],JT808Conf_struct.FirstSetupDate[5]);  


         DriveCode32=(JT808Conf_struct.Driver_Info.DriveCode[0]<<16)+(JT808Conf_struct.Driver_Info.DriveCode[1]<<8)+JT808Conf_struct.Driver_Info.DriveCode[2];
         rt_kprintf("\r\n		  ��ʻԱ����: %d \r\n",DriveCode32);  					  
         rt_kprintf("\r\n		  ��������ʻ֤��: %18s \r\n",JT808Conf_struct.Driver_Info.DriverCard_ID);  					  
         rt_kprintf("\r\n		  ��ʻԱ����: %s \r\n",JT808Conf_struct.Driver_Info.DriveName); 					  
         rt_kprintf("\r\n		  ��ʻԱ���֤: %20s \r\n",JT808Conf_struct.Driver_Info.Driver_ID); 					  
         rt_kprintf("\r\n		  ��ʻԱ��ҵ�ʸ�֤: %40s \r\n",JT808Conf_struct.Driver_Info.Drv_CareerID); 
         rt_kprintf("\r\n		  ��֤����: %s \r\n",JT808Conf_struct.Driver_Info.Comfirm_agentID);   



         rt_kprintf("\r\n		  ����VIN��: %17s \r\n",JT808Conf_struct.Vechicle_Info.Vech_VIN);   
         rt_kprintf("\r\n		  ���ƺ���: %12s \r\n",JT808Conf_struct.Vechicle_Info.Vech_Num);  
         rt_kprintf("\r\n		  ���Ʒ���: %12s \r\n",JT808Conf_struct.Vechicle_Info.Vech_Type);  
         rt_kprintf("\r\n        ��������ʡID: %d \r\n",JT808Conf_struct.Vechicle_Info.Dev_ProvinceID);
         rt_kprintf("\r\n        ����������ID: %d \r\n",JT808Conf_struct.Vechicle_Info.Dev_CityID); 
         rt_kprintf("\r\n        ������ɫ:   JT415    1  �� 2 �� 3 �� 4 �� 9����----��ǰ��ɫ %d \r\n",JT808Conf_struct.Vechicle_Info.Dev_Color);  


         rt_kprintf("\r\n        �����ϱ�������Ϊ  TriggerSDsatus=%X    \r\n",TriggerSDsatus);   
         rt_kprintf("\r\n        Max_picNum =  %d   Max_CycleNum = %d   Max_DrvRcdNum=%d \r\n",Max_PicNum,Max_CycleNum,Max_RecoderNum); 

         rt_kprintf("\r\n        ƣ�ͼ�ʻ���� ������ʻ���� =  %d s   �����ۼƼ�ʻ���� = %d s   ��С��Ϣ����=%d s  �ͣ������= %d s\r\n",TiredConf_struct.TiredDoor.Door_DrvKeepingSec,TiredConf_struct.TiredDoor.Door_DayAccumlateDrvSec,TiredConf_struct.TiredDoor.Door_MinSleepSec,TiredConf_struct.TiredDoor.Door_MaxParkingSec); 
         rt_kprintf("\r\n        ƣ�ͼ�ʻ״̬ Status_TiredwhRst =  %d   ACCON_timer = %d   ACCOFF_timer=%d \r\n",TiredConf_struct.Tired_drive.Status_TiredwhRst,TiredConf_struct.Tired_drive.ACC_ONstate_counter,TiredConf_struct.Tired_drive.ACC_Offstate_counter); 
         if(TiredConf_struct.Tired_drive.Status_TiredwhRst==2)
                  rt_kprintf("\r\n		  ƣ�ͼ�ʻ��ʼʱ��: %X-%X-%X %X:%X:%X \r\n",TiredConf_struct.Tired_drive.start_time[0],TiredConf_struct.Tired_drive.start_time[1],TiredConf_struct.Tired_drive.start_time[2],TiredConf_struct.Tired_drive.start_time[3],TiredConf_struct.Tired_drive.start_time[4],TiredConf_struct.Tired_drive.start_time[5]);  


         rt_kprintf("\r\n\r\n        ���������ͼ�� =	%d s   TCPӦ��ʱDur = %d s	 TCP�ش�����=%d 	��ʻԱδ��¼ʱ�ϱ����= %d s\r\n",JT808Conf_struct.DURATION.Heart_Dur,JT808Conf_struct.DURATION.TCP_ACK_Dur,JT808Conf_struct.DURATION.TCP_ReSD_Num,JT808Conf_struct.DURATION.NoDrvLogin_Dur); 
         rt_kprintf("\r\n\r\n                                 UDPӦ��ʱDur = %d s	 UDP�ش�����=%d 	                             \r\n",JT808Conf_struct.DURATION.UDP_ACK_Dur,JT808Conf_struct.DURATION.UDP_ReSD_Num);  
         rt_kprintf("\r\n        ����ʱ�ϱ���� =	%d s   ��������ʱ�ϱ���� = %d s	ȱʡ�ϱ����=%d s	�յ��ϴνǶ�= %d �� �Ƿ��ƶ���ֵ= %d m\r\n",JT808Conf_struct.DURATION.Sleep_Dur,JT808Conf_struct.DURATION.Emegence_Dur,JT808Conf_struct.DURATION.Default_Dur,JT808Conf_struct.DURATION.SD_Delta_maxAngle,JT808Conf_struct.DURATION.IllgleMovo_disttance); 

         rt_kprintf("\r\n\r\n		  ȱʡ�����ϱ� =	 %d m	��ʻԱδ��¼���� = %d m	 ���߶���=%d m	 ������������= %d m \r\n",JT808Conf_struct.DISTANCE.Defalut_DistDelta,JT808Conf_struct.DISTANCE.NoDrvLogin_Dist,JT808Conf_struct.DISTANCE.Sleep_Dist,JT808Conf_struct.DISTANCE.Emergen_Dist); 

         rt_kprintf("\r\n\r\n		��ʱ�ϱ���ʽ = %d    ��ʱȱʡ = %d   ��ʱδ��¼=%d  ��ʱ����=%d m   ��ʱ����= %d m \r\n",JT808Conf_struct.SD_MODE.DUR_TOTALMODE,JT808Conf_struct.SD_MODE.Dur_DefaultMode,JT808Conf_struct.SD_MODE.Dur_NologinMode,JT808Conf_struct.SD_MODE.Dur_SleepMode,JT808Conf_struct.SD_MODE.Dur_EmegencMode); 
         rt_kprintf("\r\n\r\n		�����ϱ���ʽ = %d    ����ȱʡ = %d   ����δ��¼=%d  ��������=%d m   �������= %d m \r\n",JT808Conf_struct.SD_MODE.DIST_TOTALMODE,JT808Conf_struct.SD_MODE.Dist_DefaultMode,JT808Conf_struct.SD_MODE.Dist_NoLoginMode,JT808Conf_struct.SD_MODE.Dist_SleepMode,JT808Conf_struct.SD_MODE.Dist_EmgenceMode);  


         rt_kprintf("\r\n\r\n	   ��ʱλ�ø���״̬= %d    ���ټ�� = %d  	����ʱ��=%d  ������ǰ������=%d  \r\n",JT808Conf_struct.RT_LOCK.Lock_state,JT808Conf_struct.RT_LOCK.Lock_Dur,JT808Conf_struct.RT_LOCK.Lock_KeepDur,JT808Conf_struct.RT_LOCK.Lock_KeepCnter);  
         rt_kprintf("\r\n\r\n	  ��������״̬: ");
         switch(JT808Conf_struct.LOAD_STATE)
		{  
		case 1:
		   rt_kprintf("�ճ�\r\n"); 
		  break;
		case 2:
		   rt_kprintf("����\r\n"); 
		  break;		  
		case 3:
		   rt_kprintf("����\r\n"); 
		  break;
		default:
		       JT808Conf_struct.LOAD_STATE=1;
		       Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
		   rt_kprintf("�ճ�2\r\n");  
		  break;
		}
            rt_kprintf("\r\n\r\n  ��ʼ��ˮ��: %d \r\n", JT808Conf_struct.Msg_Float_ID); 
	     rt_kprintf("\r\n\r\n             cyc_read:   %d ,     cyc_write :%d\r\n  \r\n",cycle_read,cycle_write);     		

         //=====================================================================
         //API_List_Directories();
         //-----------  ����ģ�����  ---------
	  BD_list(); 
 
}

/*
�����������ļ�
*/
	void SysConfiguration(void)
	{
			SetConfig();  
			ReadConfig();
			DefaultConfig();      
	}


	 


void RstWrite_ACConoff_counter(void) 
{
 
  if(TiredConf_struct.Tired_drive.Status_TiredwhRst==0)
  	return;
    Api_Config_write(tired_config,0,(u8*)&TiredConf_struct,sizeof(TiredConf_struct));
    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));  
}
//-------------------------------------------------------------------------------



void  idip(u8 *str)
{
   u8 Reg_buf[25];
   
    if (strlen((const char*)str)==0){
	  	rt_kprintf("\r\n		   ��ǰ��Ȩ��Ϊ: ");
              rt_kprintf(" %s\r\n		   ��Ȩ�볤��: %d\r\n",JT808Conf_struct.ConfirmCode,strlen((const char*)JT808Conf_struct.ConfirmCode));   

		return ;
	}
   else
   	{	
   	    if(strncmp((const char*)str,"clear",5)==0)
   	    	{
                   JT808Conf_struct.Regsiter_Status=0; 
		     rt_kprintf("     �ֶ���� ��Ȩ�� !\r\n");   		   
   	    	}
		else
	      {
                  memset(JT808Conf_struct.ConfirmCode,0,sizeof(JT808Conf_struct.ConfirmCode));
                  memcpy(JT808Conf_struct.ConfirmCode,str,strlen((const char*)str));
		     JT808Conf_struct.Regsiter_Status=1; 
		    rt_kprintf("     �ֶ�����  ��Ȩ��: %s\r\n",JT808Conf_struct.ConfirmCode);   
	 
		}  
		    memset(Reg_buf,0,sizeof(Reg_buf));
		    memcpy(Reg_buf,JT808Conf_struct.ConfirmCode,20);
		    Reg_buf[20]=JT808Conf_struct.Regsiter_Status;			
                  Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
      }
}
FINSH_FUNCTION_EXPORT(idip, id code set);














