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


//----   多媒体发送状态 -------
_Media_SD_state Photo_sdState;   //  图片发送状态
_Media_SD_state Sound_sdState;	//声音发送  
_Media_SD_state Video_sdState;	//视频发送


//------ Photo -----
 u32 PicFileSize=0; // 图片文件大小  
 u8  PictureName[40];      



//------  voice -----



//------  video  --------


/*
             杂
*/
//------ phone
u8       CallState=CallState_Idle; // 通话状态

//   ASCII  to   GB    ---- start  
//0-9        10
u8  arr_A3B0[20]={0xA3,0xB0,0xA3,0xB1,0xA3,0xB2,0xA3,0xB3,0xA3,0xB4,0xA3,0xB5,0xA3,0xB6,0xA3,0xB7,0xA3,0xB8,0xA3,0xB9};

//@ A-O      16
u8  arr_A3C0[32]={0xA3,0xC0,0xA3,0xC1,0xA3,0xC2,0xA3,0xC3,0xA3,0xC4,0xA3,0xC5,0xA3,0xC6,0xA3,0xC7,0xA3,0xC8,0xA3,0xC9,0xA3,0xCA,0xA3,0xCB,0xA3,0xCC0,0xA3,0xCD,0xA3,0xCE,0xA3,0xCF};

//P-Z         11个
u8  arr_A3D0[22]={0xA3,0xD0,0xA3,0xD1,0xA3,0xD2,0xA3,0xD3,0xA3,0xD4,0xA3,0xD5,0xA3,0xD6,0xA3,0xD7,0xA3,0xD8,0xA3,0xD9,0xA3,0xDA};

//.  a-0       16
u8  arr_A3E0[32]={0xA3,0xE0,0xA3,0xE1,0xA3,0xE2,0xA3,0xE3,0xA3,0xE4,0xA3,0xE5,0xA3,0xE6,0xA3,0xE7,0xA3,0xE8,0xA3,0xE9,0xA3,0xEA,0xA3,0xEB,0xA3,0xEC,0xA3,0xED,0xA3,0xEE,0xA3,0xEF};

//p-z          11
u8  arr_A3F0[22]={0xA3,0xF0,0xA3,0xF1,0xA3,0xF2,0xA3,0xF3,0xA3,0xF4,0xA3,0xF5,0xA3,0xF6,0xA3,0xF7,0xA3,0xF8,0xA3,0xF9,0xA3,0xFA};
//-------  ASCII to GB ------



//----------- 行车记录仪相关  -----------------
Avrg_MintSpeed  Avrgspd_Mint; 
u32         PerMinSpdTotal=0; //记录每分钟速度总数  
u8          avgspd_Mint_Wr=0;       // 填写每分钟平均速度记录下标
u8          avgspd_Sec_Wr=0;       // 填写每秒钟平均速度记录下标
u8          avgWriteOver=0;   // 写溢出标志位
u8          AspdCounter=0;    // 每分钟速度有效报数计数器 
u8          Vehicle_sensor=0; // 车辆传感器状态   0.2s  查询一次                                    
							  /*	   
							  D7  刹车
							  D6  左转灯
							  D5  右转灯
							  D4  喇叭
							  D3  远光灯
							  D2  雨刷
							  D1  预留
							  D0  预留
							  */
u8		  Vehicle_sensor_BAK=0; // 车辆传感器状态	0.2s  查询一次	
                              
DOUBT_TYPE  Sensor_buf[100];// 20s 状态记录
u8          save_sensorCounter=0,sensor_writeOverFlag=0;;
u32     total_plus=0; 






u8  Camera_Number=1;
u8       DispContent=1;   // 发送时是否显示数据内容  
                             /*                          
                                         1 <->  正常显示 
                                         2 <->  显示发送信息的 
                                         3 <->  显示 任务的运行情况  
                                         0<-> 不显示调试输出，只显示协议数据
                                  */ 

u8         TextInforCounter=0;//文本信息条数

u8 		   FCS_GPS_UDP=0;						//UDP 数据异或和
u8         FCS_RX_UDP=0;                       // UDP 数据接收校验

u8          Centre_IP_modify=0;               //  中修改IP了 
u8          IP_change_counter=0;             //   中心修改IP 计数器
u8          Down_Elec_Flag=0;                //   断油断电使能标志位 



//------------ 超速报警---------------------  
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
u8	        GPS_getfirst=0; 		 //  首次有经纬度 
u8          HDOP_value=99;         //  Hdop 数值    
u8          Satelite_num=0;   // 卫星颗数
u8 CurrentTime[3];
u8 BakTime[3];
u8 Sdgps_Time[3];  // GPS 发送 时间记录   BCD 方式

//static u8      UDP_AsciiTx[1800];     
 ALIGN(RT_ALIGN_SIZE)
u8      GPRS_info[900];  
u16     GPRS_infoWr_Tx=0; 

 ALIGN(RT_ALIGN_SIZE)
u8  UDP_HEX_Rx[1024];    // EM310 接收内容hex 
u16 UDP_hexRx_len=0;    // hex 内容 长度
u16 UDP_DecodeHex_Len=0;// UDP接收后808 解码后的数据长度


GPS_RMC GPRMC; // GPMC格式

        /*                         pGpsRmc->status,\
								pGpsRmc->latitude_value,\
								pGpsRmc->latitude,\
								pGpsRmc->longtitude_value,\
								pGpsRmc->longtitude,\
								pGpsRmc->speed,\
								pGpsRmc->azimuth_angle);
								*/


		
//----------808 协议 -------------------------------------------------------------------------------------
u16	   GPS_Hight=0;               //   808协议-> 高程   m    
u16	   GPS_speed=0;			   //   808协议-> 速度   0.1km/h    
u16     GPS_direction=0;           //   808协议-> 方向   度           
u16     Centre_FloatID=0; //  中心消息流水号
u16     Centre_CmdID=0;   //  中心命令ID

u8      Original_info[850]; // 没有转义处理前的原始信息  
u16     Original_info_Wr=0; // 原始信息写地址
//---------- 用GPS校准特征系数相关 ----------------------------
u8      Speed_area=60; // 校验K值范围
u16     Speed_gps=0;  // 通过GPS计算出来的速度 0.1km/h
u8      Speed_Rec=0;  // 速度传感器 校验K用的存储器
u16     Speed_cacu=0; // 通过K值计算出来的速度
u16     Spd_adjust_counter=0; // 确保匀速状态计数器
u16     Former_DeltaPlus[K_adjust_Duration]; // 前几秒的脉冲数 
u8      Former_gpsSpd[K_adjust_Duration];// 前几秒的速度      
u8      DF_K_adjustState=0; // 特征系数自动校准状态说明  1:自动校准过    0:尚未自动校准   
//-----  车台注册定时器  ----------
DevRegst   DEV_regist;  // 注册
DevLOGIN   DEV_Login;   //  鉴权  




 
//------- 文本信息下发 -------
TEXT_INFO      TextInfo;    // 文本信息下发 
//------- 事件 ----
EVENT          EventObj;    // 事件   
EVENT          EventObj_8[8]; // 事件  
//-------文本信息-------
MSG_TEXT       TEXT_Obj;
MSG_TEXT       TEXT_Obj_8[8],TEXT_Obj_8bak[8];

//------ 提问  --------
CENTRE_ASK     ASK_Centre;  // 中心提问
//------  信息点播  ---
MSG_BRODCAST   MSG_BroadCast_Obj;    // 信息点播         
MSG_BRODCAST   MSG_Obj_8[8];  // 信息点播    
//------  电话本  -----
PHONE_BOOK    PhoneBook,Rx_PhoneBOOK;   //  电话本
PHONE_BOOK    PhoneBook_8[8];

//-----  车辆控制 ------
VEHICLE_CONTROL Vech_Control; //  车辆控制     
//-----  电子围栏  -----
POLYGEN_RAIL Rail_Polygen;   // 多边形围栏
RECT_RAIL    Rail_Rectangle; // 矩形围栏
CIRCLE_RAIL  Rail_Cycle;     // 圆形围栏
//------- 线路设置 -----
POINT        POINT_Obj;      // 路线的拐点
ROUTE        ROUTE_Obj;      // 路线相关 
//-------    行车记录仪  -----
RECODER      Recode_Obj;     // 行车记录仪  
//-------  拍照  ----  
CAMERA        Camera_Obj;     //  中心拍照相关      
//-----   录音  ----
VOICE_RECODE  VoiceRec_Obj;   //  录音功能     
//------ 多媒体  --------
MULTIMEDIA    MediaObj;       // 多媒体信息  
//-------  数据信息透传  -------
DATATRANS     DataTrans;      // 数据信息透传   
//-------  进出围栏状态 --------
INOUT        InOut_Object;    // 进出围栏状态    
//-------- 多媒体检索  ------------
MEDIA_INDEX  MediaIndex;  // 多媒体信息    
//------- 车辆负载状态 ---------------
u8  CarLoadState_Flag=1;//选中车辆状态的标志   1:空车   2:半空   3:重车

//------- 多媒体信息类型---------------
u8  Multimedia_Flag=1;//需要上传的多媒体信息类型   1:视频   2:音频   3:图像
u8  SpxBuf[SpxBuf_Size];  
u16 Spx_Wr=0,Spx_Rd=0;
u8  Duomeiti_sdFlag=0; 

//------- 录音开始或者结束---------------
u8  Recor_Flag=1; //  1:录音开始   2:录音结束


//----------808协议 -------------------------------------------------------------------------------------	
u8		SIM_code[6];							   // 要发送的IMSI	号码
u8		IMSI_CODE[15]="000000000000000";							//SIM 卡的IMSI 号码
u8		Warn_Status[4]		=
{
		0x00, 0x00,0x00,0x00
}; //  报警标志位状态信息
u8		Car_Status[4]		=
{
		0x00, 0x00,0x00,0x00 
}; //  车辆状态信息	
T_GPS_Info_GPRS 	Gps_Gprs,Bak_GPS_gprs;	 
T_GPS_Info_GPRS	Temp_Gps_Gprs; 
u8   A_time[6]; // 定位时刻的时间

u8      ReadPhotoPageTotal=0;
u8      SendPHPacketFlag=0; ////收到中心启动接收下一个block时置位


//-------- 紧急报警 -------- 
u8		warn_flag= 0;		  
u8		f_Exigent_warning= 0;//0;     //脚动 紧急报警装置 (INT0 PD0)
u8		Send_warn_times= 0;     //   设备向中心上报报警次数，最大3 次
u32  	fTimer3s_warncount=0;


//------  车门开关拍照 -------
DOORCamera   DoorOpen;    //  开关车门拍照

//------- 北斗扩展协议  ------------
BD_EXTEND     BD_EXT;     //  北斗扩展协议  

// ---- 拐点 -----
u16  Inflexion_Current=0;
u16  Inflexion_Bak=0;      
u16  Inflexion_chgcnter=0; //变化计数器
u16  InflexLarge_or_Small=0;     // 判断curent 和 Bak 大小    0 equql  1 large  2 small  
u16  InflexDelta_Accumulate=0;  //  差值累计 

// ----休眠状态  ------------
u8  SleepState=0;  //   0  不休眠ACC on            1  休眠Acc Off 
u8  SleepConfigFlag=0; //  休眠时发送鉴权标志位

//---- 固定文件大小 --- 
u32 mp3_fsize=5616;
u8  mp3_sendstate=0;
u32 wmv_fsize=25964;
u8  wmv_sendstate=0;

//-------------------   公共 ---------------------------------------		
static u8 GPSsaveBuf[40];     // 存储GPS buffer
static u8	ISP_buffer[520];
static u16 GPSsaveBuf_Wr=0;


POSIT Posit[60];           // 每分钟位置信息存储
u8    PosSaveFlag=0;      // 存储Pos 状态位 

NANDSVFlag   NandsaveFlg;
A_AckFlag    Adata_ACKflag;  // 无线GPRS协议 接收相关 RS232 协议返回状态寄存器
TCP_ACKFlag  SD_ACKflag;     // 无线GPRS协议返回状态标志 
u8  SubCMD_8103H=0;            //  02 H命令 设置记录仪安装参数回复 子命令
u32  SubCMD_FF01H=0;            //  FF02 北斗信息扩展
u32  SubCMD_FF03H=0;     //  FF03  设置扩展终端参数设置1

u8  SubCMD_10H=0;            //  10H   设置记录仪定位告警参数
u8  OutGPS_Flag=0;     //  0  默认  1  接外部有源天线
u8   Spd_senor_Null=0;  // 手动传感器速度为0
u32  Centre_DoubtRead=0;     //  中心读取事故疑点数据的读字段 
u32  Centre_DoubtTotal=0;    //  中心读取事故疑点的总字段
u8   Vehicle_RunStatus=0;    //  bit 0: ACC 开 关             1 开  0关
                             //  bit 1: 通过速度传感器感知    1 表示行驶  0 表示停止   
                             //  bit 2: 通过gps速度感知       1 表示行驶  0 表示停止  
u8   Status_TiredwhRst=0;    //  当复位时 疲劳驾驶的状态   0 :停车  1:停车但没触发 2:触发了还没结束                              



u32   SrcFileSize=0,DestFilesize=0,SrcFile_read=0;
u8    SleepCounter=0; 

u16   DebugSpd=0;   //调试用GPS速度  
u8    MMedia2_Flag=0;  // 上传固有音频 和实时视频  的标志位    0 传固有 1 传实时






//-----  ISP    远程下载相关 -------
u8       f_ISP_ACK=0;   // 远程升级应答	
u8       ISP_FCS[2];    //  下发的校验
u16      ISP_total_packnum=0;  // ISP  总包数
u16      ISP_current_packnum=0;// ISP  当前包数
u32      ISP_content_fcs=0;    // ISP  的内容校验
u8       ISP_ack_resualt=0;    // ISP 响应
u8       ISP_rxCMD=0;          // ISP 收到的命令
u8       f_ISP_88_ACK=0;       // Isp  内容应答
u8       ISP_running_state=0;  // Isp  程序运行状态
u8       f_ISP_23_ACK=0;    //  Isp  返回 文件完成标识
u16      ISP_running_counter=0;// Isp  运行状态寄存器
u8       ISP_RepeatCounter=0; //   ISP 单包发送重复次数 超过5次校验失败擦除区域


 u8     ISP_NeedtoProcessFlag=0;   //   需要处理ISP 程序
 u8     ISP_Raw[600];                        //  属于ISP 但还未处理的字符


ISP_RSD  Isp_Resend;
unsigned short int FileTCB_CRC16=0;
unsigned short int Last_crc=0,crc_fcs=0;



//---------  中心应答  -----------
u8		 fCentre_ACK=0; 			  // ---------判断中心应答标志位－－
u8		 ACK_timer=0;				   //---------	ACK timer 定时器--------------------- 
u16         ACKFromCenterCounter=0;//十包无应答重新拨号  
u8           Send_Rdy4ok=0;


//---------------  速度脉冲相关--------------
u16  Delta_1s_Plus=0;
u16  Delta_1s_Plus2=0; 
u16  Sec_counter=0;


void K_AdjustUseGPS(u32 sp, u32  sp_DISP);  // 通过GPS 校准  K 值  (车辆行驶1KM 的脉冲数目) 
u16  Protocol_808_Encode(u8 *Dest,u8 *Src, u16 srclen);
void Protocol_808_Decode(void);  // 解析指定buffer :  UDP_HEX_Rx  
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
						Stuff_DevLogin_0102H();   //  鉴权   ==2 时鉴权完毕
						DEV_Login.Enable_sd=0;
						//------ 发送鉴权不判断 ------------------
						//DEV_Login.Operate_enable=2;  //  不用判断鉴权了   
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
			   return true; // 按照808 协议要求 ，传输多媒体过程中不允许发送别的信息包
            } 
            if(1==DEV_regist.Enable_sd)
               {									
                  Stuff_RegisterPacket_0100H(0);   // 注册
                   JT808Conf_struct.Msg_Float_ID=0;
                  Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
		    DEV_regist.Enable_sd=0;
		    JT808Conf_struct.Regsiter_Status=1; //标注注册，但不存储		  
                return true;
               }
            if(1==DEV_regist.DeRegst_sd)
            	{
            	  Stuff_DeviceDeregister_0101H(); 
                  DEV_regist.DeRegst_sd=0;		     		  
				  return true;
            	}
            if((1==JT808Conf_struct.DURATION.Heart_SDFlag)&&(DataLink_Status())&&(SleepState==0)) //  心跳
            	{
                  Stuff_DeviceHeartPacket_0002H();
                  JT808Conf_struct.DURATION.Heart_SDFlag=0;
				  JT808Conf_struct.DURATION.TCP_SD_state=1;  //发送完后置 1   
				  return true;  
            	} 			
            if((1==SleepConfigFlag)&&(DataLink_Status())&&(SleepState==1)) //  休眠时心跳
            	{
                 // Stuff_DevLogin_0102H();   //  鉴权   ==2 时鉴权完毕
                 // rt_kprintf("\r\n	 休眠时用鉴权做心跳包 ! \r\n");  
                  SleepConfigFlag=0;  
				  return true;   
            	} 
			
            if(1==SD_ACKflag.f_CurrentPosition_0201H)    // 位置信息查询
            	{
                  Stuff_Current_Data_0201H();
				  SD_ACKflag.f_CurrentPosition_0201H=0;
				  return true;
            	} 
            if(1==SD_ACKflag.f_CurrentEventACK_0301H)   //  事件报告
            	{
                  Stuff_EventACK_0301H();
				  SD_ACKflag.f_CurrentEventACK_0301H=0;
				  return true;                  
            	}
				            
	         if(2==ASK_Centre.ASK_SdFlag)      //  提问应答 
	             	{
	                  Stuff_ASKACK_0302H();
	                  ASK_Centre.ASK_SdFlag=0; 
	                  return true;  
	             	}             	
             	
             if(1==Vech_Control.ACK_SD_Flag)   //  车辆应答控制
             	{
                  Stuff_ControlACK_0500H(); 
				  Vech_Control.ACK_SD_Flag=0;
				  return true;                  
             	}
			 
			 if(1==Recode_Obj.SD_Data_Flag)
			 	{
                   Stuff_RecoderACK_0700H();   //   行车记录仪数据上传  
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
                  Stuff_DataTransTx_0900H();  // 数据透传 做远程下载
                  DataTrans_Init();     //clear 
                  return true;
             	} 		
             if(SD_ACKflag.f_MediaIndexACK_0802H)
             	{
                   Stuff_MultiMedia_IndexAck_0802H();   // 多媒体索引上报 
                   SD_ACKflag.f_MediaIndexACK_0802H=0;
				   return true;
             	}			 
			 if(SD_ACKflag.f_DriverInfoSD_0702H)
			 	{
			 	   Stuff_DriverInfoSD_0702H();  //  驾驶员信息上报
                   SD_ACKflag.f_DriverInfoSD_0702H=0;
                   return true;
			 	}
			  if(SD_ACKflag.f_Worklist_SD_0701H)
			 	{
			 	   Stuff_Worklist_0701H();  //   电子运单 
                   SD_ACKflag.f_Worklist_SD_0701H=0;
                   return true;
			 	}
	        if(SD_ACKflag.f_CentreCMDack_0001H)
	         {
                               Stuff_DevCommmonACK_0001H();        
				   if(SD_ACKflag.f_CentreCMDack_0001H==2)  //  修改IP设置了需要重拨
				   	   Close_DataLink();    //  AT_END    
				   else	   
				   if(SD_ACKflag.f_CentreCMDack_0001H==3) //   远程复位
				   	{
                                       Systerm_Reset_counter=Max_SystemCounter;
					  ISP_resetFlag=2;    //   借助远程下载重启机制复位系统
				   	}
				   else
				   if(SD_ACKflag.f_CentreCMDack_0001H==5) //   关闭数据通信
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

	     //----------   远程下载相关 ------------		
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
						ISP_file_Check();//准备更新程序
				   else
						{    // 清除远程升级区域 
						     
							 rt_kprintf("\r\n检验不对清除  Caculate_crc=%x ReadCrc=%x ",crc_file,FileTCB_CRC16); 
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
		   	      
                          Stuff_RegisterPacket_0100H(1);  //    ISP 心跳包  
                          rt_kprintf("\r\nISP--heart\r\n"); 
                          TCP2_sdFlag=0;
                           return true;
		   	}
	         //----------------------远程下载相关完毕------------------ 		
			
			// if((Current_SD_Duration<=10)||(Current_State==1))   // 调试时30  实际是10 Current_SD_Duration
			// {	
			 if(PositionSD_Status()&&(DEV_Login.Operate_enable==2)&&((enable==BD_EXT.Trans_GNSS_Flag)||(DispContent==6))||(Current_UDP_sd&&PositionSD_Status()&&(DEV_Login.Operate_enable==2))||((DF_LOCK==enable)&&PositionSD_Status()))	  //首次定位再发 
			   //  if((PositionSD_Status())&&(DataLink_Status())&&(DEV_Login.Operate_enable==2))	                                                                                                                     // DF  锁定发送当前位置信息  
		      {
		                  PositionSD_Disable();
				   Current_UDP_sd=0; 
                              //1.   时间超前判断
                              //if(Time_FastJudge()==false)
							//return false;		
				  // 2. 			  
			 	   Stuff_Current_Data_0200H();  // 上报即时数据  				   
				   //----应答次数 ----		   
				  // ACKFromCenterCounter++; // 只关注应答报数，不关心应答时间 
				   //---------------------------------------------------------------------------------
				   if(DispContent)	
					    rt_kprintf("\r\n 发送 GPS -current !\r\n");     
			    }   
			 //}
			 else 
             if((RdCycle_RdytoSD==ReadCycle_status)&&(0==ISP_running_state)&&(DataLink_Status())&&(DEV_Login.Operate_enable==2))    // 读取发送--------- 正常GPS 
             {                                       /* 远程下载时不允许上报GPS ，因为有可能发送定位数据时
                                                          正在接收大数据量得下载数据包,导致GSM模块处理不过来，而不是单片机处理不过来*/
                  if (false==Stuff_Normal_Data_0200H())        
				 	return false;
				  fCentre_ACK=1;// 判断应答
				 //-------- change status  Ready  ACK  ------ 
			         ReadCycle_status=RdCycle_SdOver;
				  Send_Rdy4ok=1;  // enable
				  //----应答次数 ----		  
				  ACKFromCenterCounter++; 
				   if(DispContent)	
					  rt_kprintf("\r\n 发送 GPS --saved  OK!\r\n");    
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
// 第一部分 :   以下是GPS 解析转换相关函数 
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
                
				//-----------  天地通协议 ------------- 	 
				Temp_Gps_Gprs.Time[0] = hour;
				Temp_Gps_Gprs.Time[1] = ( tmpinfo[2] - 0x30 ) * 10 + tmpinfo[3] - 0x30;
				Temp_Gps_Gprs.Time[2] = ( tmpinfo[4] - 0x30 ) * 10 + tmpinfo[5] - 0x30;  
            
}

void Status_pro(u8 *tmpinfo) 
{
 	                GPRMC.status=tmpinfo[0];
  
					//-------------------------天地通协议-----------------------------
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
  
	//--------	808 协议 --------------------
  if(UDP_dataPacket_flag==0X02)    //精确到百万分之一度
  {  
    
    //------------  dd part   -------- 
	latitude = ( u32 ) ( ( tmpinfo[0] - 0x30 ) * 10 + ( u32 ) ( tmpinfo[1] - 0x30 ) ) * 1000000;
	//------------  mm  part  -----------
	/*    转换成百万分之一度
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
 //--------  808协议  ---------
 if(UDP_dataPacket_flag==0X02)  //精确到百万分之一度
 {
     //------  ddd part -------------------
	 longtitude = ( u32 )( ( tmpinfo[0] - 0x30 ) * 100 + ( tmpinfo[1] - 0x30 ) * 10 + ( tmpinfo[2] - 0x30 ) ) * 1000000;  
	 //------  mm.mmmm --------------------	 
	 /*    转换成百万分之一度
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
			  //-----808 协议 -------------- 
			 //两个字节单位0.1 km/h  
			 if ( Point == 1 )	//0.0-9.9=>
			 {
												  //++++++++  Nathan Modify on 2008-12-1   ++++++++++
					if((tmpinfo[0]>=0x30)&&(tmpinfo[0]<=0x39)&&(tmpinfo[2]>=0x30)&&(tmpinfo[2]<=0x39))
					 {
					   sp = ( tmpinfo[0] - 0x30 ) * 10 + ( tmpinfo[2] - 0x30 );  //扩大10倍
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
	 
		 // --------  sp 当前是0.1 knot------------------	 
		  sp= (u32)(sp * 185.6) ;  //  1 海里=1.856 千米  现在是m/h
								 
		  if(sp>220000)   //时速大于220km/h则剔除
			  return;  
	 
		  sp_DISP=sp/100;   //  sp_Disp 单位是 0.1km/h 
							  
	         //------------------------------ 通过GPS模块数据获取到的速度 --------------------------------
                   Speed_gps=(u16)sp_DISP;
	         //---------------------------------------------------------------------------
       if(JT808Conf_struct.Speed_GetType)  // 通过速度传感器 获取速度
       	{ 
              K_AdjustUseGPS(sp,sp_DISP);  //  调整K值    
              if(JT808Conf_struct.DF_K_adjustState==0)
			  {
			     // ---  在未校准前，获得到的速度是通过GPS计算得到的
			     GPS_speed=Speed_gps;      
			       //------- GPS	里程计算-------- 
				 if(sp>=5000)		//	过滤零点漂移  速度大于
				 {
					 reg=sp/3600;  // 除以3600 是m/s 
					 JT808Conf_struct.Distance_m_u32+=reg;
					 if(JT808Conf_struct.Distance_m_u32>0xFFFFFF)
							  JT808Conf_struct.Distance_m_u32=0; 	  //里程最长这么多米	

					 //----- 定距回传处理---  
                     if(1==JT808Conf_struct.SD_MODE.DIST_TOTALMODE)
                     {
                        DistanceAccumulate+=reg;
                        if(DistanceAccumulate>=Current_SD_Distance)
                        	{
                               DistanceAccumulate=0;
				   PositionSD_Enable();        //发送
				   Current_UDP_sd=1;
                        	}
                     }
					 //------- 定距处理结束 -----
							  
				 }	
				
              }  
       	}  
	   else
	   	{  // 从GPS 取速度
			     //------- GPS	里程计算-------- 
				 if(sp>=5000)		//	过滤零点漂移  速度大于
				 {
					 JT808Conf_struct.Distance_m_u32+=sp/3600;  // 除以3600 是m/s 
					 if(JT808Conf_struct.Distance_m_u32>0xFFFFFF)
							  JT808Conf_struct.Distance_m_u32=0; 	  //里程最长这么多米	  

					 //----- 定距回传处理---  
                     if(1==JT808Conf_struct.SD_MODE.DIST_TOTALMODE)
                     {
                        DistanceAccumulate+=reg;
                        if(DistanceAccumulate>=Current_SD_Distance)
                        	{
                               DistanceAccumulate=0;
					PositionSD_Enable();          //发送
					Current_UDP_sd=1;
                        	}
                     }
					 //------- 定距处理结束 -----		   
				 }			 
					
					GPS_speed=Speed_gps;    // 用GPS数据计算得的速度 单位0.1km/h
					
		            //-----------------------------------------------
					
	   	     }
			 // if(DispContent==2) 
			  //  rt_kprintf("\r\n				  速度: %d Km/h\r\n",GPS_speed/10);     
	 }
	 else if ( UDP_dataPacket_flag == 0x03 )
	 { 
       if(0==JT808Conf_struct.Speed_GetType)  
		{
			 //----- GPS 临时速度	km/h  ---------
		     GPS_speed=0;	 
       	} 
        if(JT808Conf_struct.Speed_GetType)  // 通过速度传感器 获取速度
       	{ 
              K_AdjustUseGPS(sp,sp_DISP);  //  调整K值  
              GPS_speed=Speed_cacu;      
       	}      
	   Speed_gps=0;
	   if(DispContent==2)
		   rt_kprintf("\r\n 2 GPS没定位\r\n"); 
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
	
   
   //--------------808 协议  1 度------------------------- 	 
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
		   GPS_direction=sp;  //  单位 1度

	   //----------  拐点补传相关   ----------
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
 if(GPRMC.status=='A')     //  记录定位时间
{
  Time2BCD(A_time);  
  //------- Debug 存储 每秒的经纬度  || 实际应该是 存储每分钟的位置  -----   
  //  内容持续55秒每秒更新，这寄存器中记录的是在每分钟内最后一包定位的经纬度 ,预留5秒用于存储上一小时的位置 
  if(sec<55)
  	{
      memcpy(Posit[min].latitude_BgEnd,Gps_Gprs.Latitude,4); //北纬
      memcpy(Posit[min].longitude_BgEnd,Gps_Gprs.Longitude,4); //经度	  
      Posit[min].longitude_BgEnd[0]|=0x80;//  东经
  	}
  if((min==59)&&(sec==55))
    { // 每个小时的位置信息               
       NandsaveFlg.MintPosit_SaveFlag=1; 
	}  
 }   
  //---- 存储当前的起始里程  跨天时------------
   if((hour==0)&&(min==0)&&(sec<3))   // 存储3次确保存储成功 
  	{ 
	  JT808Conf_struct.DayStartDistance_32=JT808Conf_struct.Distance_m_u32;
          Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct)); 
       } 
 
 //-------------------------------------------------
      //---------  天地通协议  -------
                       
						//if(systemTick_TriggerGPS==0)
 {
   Temp_Gps_Gprs.Date[0] = year;
   Temp_Gps_Gprs.Date[1] = mon;
   Temp_Gps_Gprs.Date[2] = day;   						
  }

  
 //-------------- 计算算平均速度 ----------------
 AvrgSpd_MintProcess(hour,min,sec); 
}


//---------  GGA --------------------------
void HDop_pro(u8 *tmpinfo)
{ 
 float dop;
 
  dop=atof((char *)tmpinfo);				        
  HDOP_value=dop;		 //  Hdop 数值

}

void  GPS_Delta_DurPro(void)    //告GPS 触发上报处理函数
{
  if(1==JT808Conf_struct.SD_MODE.DUR_TOTALMODE)   // 定时上报模式
  {
	 	//----- 上一包数据记录的时间
		fomer_time_seconds = ( u32 ) ( BakTime[0] * 60 * 60 ) + ( u32 ) ( BakTime[1] * 60 ) + ( u32 ) BakTime[2];  
		
		//-----  当前数据记录的时间
		tmp_time_secnonds = ( u32 ) ( CurrentTime[0] * 60 * 60 ) + ( u32 ) ( CurrentTime[1] * 60 ) + ( u32 )  CurrentTime[2];
		
		//一天86400秒
		
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
		
		if((SleepState==1)&&(delta_time_seconds==(Current_SD_Duration-5)))  //  --  休眠时 先发鉴权
		   {
			  SleepConfigFlag=1;  //发送前5 发送一包鉴权
		   }            
		
		if((delta_time_seconds >= Current_SD_Duration))//limitSend_idle
		  {
			  PositionSD_Enable();    
			  memcpy(BakTime,CurrentTime,3); // update   
		  }
  	}  
  
    //------------------------------ do this every  second-----------------------------------------    
	memcpy((char*)&Gps_Gprs,(char*)&Temp_Gps_Gprs,sizeof(Temp_Gps_Gprs));  

   //------  电子围栏 判断  ----------
  /*  if((Temp_Gps_Gprs.Time[2]%20)==0) //   认证时不检测圆形电子围栏
    {
        CycleRail_Judge(Temp_Gps_Gprs.Latitude,Temp_Gps_Gprs.Longitude);
		//rt_kprintf("\r\n --- 判断圆形电子围栏");
    }	*/
    // if((Temp_Gps_Gprs.Time[2]==5)||(Temp_Gps_Gprs.Time[2]==25)||(Temp_Gps_Gprs.Time[2]==45)) //  
    if(Temp_Gps_Gprs.Time[2]%2==0)//    认证时要求2 秒
   {
        RectangleRail_Judge(Temp_Gps_Gprs.Latitude,Temp_Gps_Gprs.Longitude);
		//rt_kprintf("\r\n -----判断矩形电子围栏"); 
    }
      if((Temp_Gps_Gprs.Time[2]%5)==0) //    
    {
          // printf("\r\n --- 判断圆形电子围栏");
           RouteRail_Judge(Temp_Gps_Gprs.Latitude,Temp_Gps_Gprs.Longitude);
    }
	//rt_kprintf("\r\n Delta_seconds %d \r\n",delta_time_seconds);    
}

//----------------------------------------------------------------------------------------------------
void  SpeedSensorProcess(void)   // 通过汽车的速度传感器获得 速度 并计算里程
{
  u32  Distance_1s_m=0;  // 一秒钟运行 多少米
//  u32 sp=0;
  //  1. 用K值计算速度   -----------------------------------------------------------
     /*
           K 表示 每公里 脉冲数 
           1米/脉冲数 :   K/1000 
           Delta_1s_Plus: 每秒钟采集到的脉冲数    
           每秒行驶米数:  Delta_1s_Plus *1000/K
           => Delta_1s_Plus *1000/K*3.6*10 单位:0.1  km/h  =>Delta_1s_Plus *36000/K  单位:0.1 km/h
      */
    Speed_cacu=(Delta_1s_Plus*36000)/JT808Conf_struct.Vech_Character_Value;  // 通过计算得到的速度 
    GPS_speed=Speed_cacu;  //把计算得到的传感器速度给 协议 寄存器
    Distance_1s_m=(Delta_1s_Plus*1000)/JT808Conf_struct.Vech_Character_Value;  // 每秒运行多少米
  // 2. 计算里程相关  -------------------------------------------------------------
          //------------------------------------
		ModuleStatus|=Status_Pcheck;	
		 
   
		  //------- GPS  里程计算  -------- 
		  JT808Conf_struct.Distance_m_u32+=Distance_1s_m;	// 除以3600 是m/s 
		  if(JT808Conf_struct.Distance_m_u32>0xFFFFFF)
				   JT808Conf_struct.Distance_m_u32=0;	   //里程最长这么多米	   

  // ------------------------------------------------------------------------------ 
}
//---------------------------------------------------------------------------------------------------
void K_AdjustUseGPS(u32 sp, u32  sp_DISP)  // 通过GPS 校准  K 值  (车辆行驶1KM 的脉冲数目) 
{
      
  u32 Reg_distance=0;
  u32 Reg_plusNum=0;
  u16 i=0;
  
  if(JT808Conf_struct.DF_K_adjustState)   // 只有没校准时才有效
			return;
		
   Speed_Rec=(u8)(sp_DISP/10);	// GPS速度    单位:km/h
   // -------	要求速度在60到65km/h  -------------
   if(((Speed_Rec>=Speed_area)&&(Speed_Rec<=(Speed_area+8)))||((Speed_Rec>=40)&&(Speed_Rec<=(40+8)))||((Speed_Rec>=70)&&(Speed_Rec<=(70+8))))   // Speed_area=60   
 // if(Speed_Rec>=Speed_area) 
//   if((Speed_Rec>=40)&&(Speed_Rec<=48))   // Speed_area=60    
   {
	   Spd_adjust_counter++;    
	   if(Spd_adjust_counter>K_adjust_Duration)  //持续在速度在60~65下认为已经是匀速了
	   {
		   // 用获取到的匀速GPS速度作为基准，和根据传感器计算出来的速度，做K值得校准
		   Reg_distance=0;	// clear
		   Reg_plusNum=0;	// clear 
		   for(i=0;i<K_adjust_Duration;i++)
		   {
			  Reg_distance+=Former_gpsSpd[i];  // 除以3.6km/h 表示该秒内走了多少米
			  Reg_plusNum+=Former_DeltaPlus[i]; 						   
		   }
           /*
                做一个判断  ， 如果速度传感器不管用， 那么返回， 
             */
           if(Reg_plusNum<20)
		   	 {
		   	   Spd_adjust_counter=0; 
			   rt_kprintf("\r\n    速度传感器 没有脉冲!\r\n");
		   	   return; 
           	 }  		   	
		   //===================================================================
		   // 转换成根据GPS速度计算行驶了多少米，(总距离) 先求和在除以3.6 ，为了计算方便 先x10  再除以36 
		   Reg_distance=(u32)(Reg_distance*10/36); // 转换成根据GPS速度计算行驶了多少米，(总距离)
		   // (Reg_plusNum/Reg_distance) 表示用脉冲总数除以距离(米)= 每米产生多少个脉冲 ，因为K值是1000米脉冲数，所以应该乘以1000
		   JT808Conf_struct.Vech_Character_Value=1000*Reg_plusNum/Reg_distance;
		   //-------  存储新的特征系数 -------------------------------- 						
		   JT808Conf_struct.DF_K_adjustState=1;					// clear  Flag
		   ModuleStatus|=Status_Pcheck;
		   Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
		   
		  Spd_adjust_counter=0; // clear  counter
	   }
	   else
	   {   //-------- 记录规定时间内的脉冲数和GPS速度----------
		   Former_gpsSpd[Spd_adjust_counter]=Speed_Rec; 
		   Former_DeltaPlus[Spd_adjust_counter]=Delta_1s_Plus;
	   }
  
   }  
   else
	  Spd_adjust_counter=0;  // 只要速度超出预设范围 计数器清0
}
//==================================================================================================
// 第二部分 :   以下是外部传感器状态监测
//==================================================================================================
/*    
     -----------------------------
     2.1   和协议相关的功能函数
     ----------------------------- 
*/


/*    
     -----------------------------
    2.4  不同协议状态寄存器变化
     ----------------------------- 
*/

void StatusReg_WARN_Enable(void)
{
  //     紧急报警状态下 寄存器的变化
    Warn_Status[3] |= 0x01;//BIT( 0 );		   
}

void StatusReg_WARN_Clear(void)
{
  //     清除报警寄存器  
    Warn_Status[3] &= ~0x01;//BIT( 0 );  	  	
}

void StatusReg_ACC_ON(void)
{  //    ACC 开
  Car_Status[3]|=0x01; //  Bit(0)     Set  1  表示 ACC开

}

void StatusReg_ACC_OFF(void)
{  //    ACC 关  
  
  Car_Status[3]&=~0x01; //  Bit(0)     Set  01  表示 ACC关
}

void StatusReg_POWER_CUT(void)
{  //  主电源断开
   Warn_Status[2]	|= 0x01;//BIT( 0 );
   ModuleStatus|=Status_Battery;
}

void StatusReg_POWER_NORMAL(void)
{  // 主电源正常
   Warn_Status[2] &= ~0x01;//BIT( 0 );     
   ModuleStatus&=~Status_Battery;
}

void StatusReg_GPS_A(void)
{     // GPS 定位 
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
{    //  GPS 不定位  
  Car_Status[3]&=~0x02;   //Bit(1)
  ModuleStatus &= ~Status_GPS;
}
	
void StatusReg_SPD_WARN(void)
{    //  超速报警
   Warn_Status[3] |= 0x02;//BIT( 1 );
}

void StatusReg_SPD_NORMAL(void)
{    //  速度正常
  Warn_Status[3] &=~ 0x02;//BIT( 1 );
}

void StatusReg_Relay_Cut(void)
{  // 断油断电状态 

}

 void StatusReg_Relay_Normal(void)
{ //  断油电状态正常
 
}


void StatusReg_Default(void)
{    //   状态寄存器还原默认设置
  
   Warn_Status[0]=0x00; //HH
   Warn_Status[1]=0x00; //HL  
   Warn_Status[2]=0x00; //LH 
   Warn_Status[3]=0x00; //LL	      
}

//==================================================================================================
// 第三部分 :   以下是GPRS无线传输相关协议
//==================================================================================================
void  Save_GPS(void)     
{
   u16 counter_mainguffer,i;
   u8  lati_reg[4];//,regstatus;
   
		  if (PositionSD_Status())	
		  {
		         if(DF_LOCK==enable)    // 清除文件区域时 ，禁止操作DF
				 	return ;
		     //-------------------------------------------------------	 
		     //1.   时间超前判断
                              //if(Time_FastJudge()==false)    
							//return ;		 
				  //----------------------- Save GPS --------------------------------------
				  memset(GPSsaveBuf,0,40);
				  GPSsaveBuf_Wr=0;				  
				   //------------------------------- Stuff ----------------------------------------
					counter_mainguffer = GPSsaveBuf_Wr;
				   // 1. 告警状态   4 Bytes
				   memcpy( ( char * ) GPSsaveBuf + GPSsaveBuf_Wr, ( char * )Warn_Status,4 );   
				   GPSsaveBuf_Wr += 4;  
                   // 2. 车辆状态   4 Bytes
				   memcpy( ( char * ) GPSsaveBuf + GPSsaveBuf_Wr, ( char * )Car_Status,4 );   
				   GPSsaveBuf_Wr += 4; 
				   // 3.   纬度     4 Bytes
				   memcpy(lati_reg,Gps_Gprs.Latitude,4);
				   memcpy( ( char * ) GPSsaveBuf + GPSsaveBuf_Wr, ( char * )Gps_Gprs.Latitude, 4 );//纬度   modify by nathan
				   GPSsaveBuf_Wr += 4;
				   // 4.   经度     4 Bytes
				   memcpy( ( char * ) GPSsaveBuf + GPSsaveBuf_Wr, ( char * )  Gps_Gprs.Longitude, 4 );	 //经度    东经  Bit 7->0	西经 Bit 7 -> 1
				   GPSsaveBuf_Wr += 4;
				   // 5.  高度	  2 Bytes    m
				   GPSsaveBuf[GPSsaveBuf_Wr++]=(GPS_Hight>>8);	// High 			   
				   GPSsaveBuf[GPSsaveBuf_Wr++]=(u8)GPS_Hight;  // Low  
				   // 6.  速度	  2 Bytes     0.1Km/h
				   GPSsaveBuf[GPSsaveBuf_Wr++]=(Speed_gps>>8);	// High 			   
				   GPSsaveBuf[GPSsaveBuf_Wr++]=(u8)Speed_gps;  // Low 
				   // 7.  方向	  2 Bytes	    1度 				   
				   GPSsaveBuf[GPSsaveBuf_Wr++]=(GPS_direction>>8);	//High
				   GPSsaveBuf[GPSsaveBuf_Wr++]=GPS_direction; // Low				   
				   // 8.  日期时间	  6 Bytes 
				    GPSsaveBuf[GPSsaveBuf_Wr++]=(((Gps_Gprs.Date[0])/10)<<4)+((Gps_Gprs.Date[0])%10);  	   	
					GPSsaveBuf[GPSsaveBuf_Wr++]=((Gps_Gprs.Date[1]/10)<<4)+(Gps_Gprs.Date[1]%10); 
					GPSsaveBuf[GPSsaveBuf_Wr++]=((Gps_Gprs.Date[2]/10)<<4)+(Gps_Gprs.Date[2]%10);
					GPSsaveBuf[GPSsaveBuf_Wr++]=((Gps_Gprs.Time[0]/10)<<4)+(Gps_Gprs.Time[0]%10);
					GPSsaveBuf[GPSsaveBuf_Wr++]=((Gps_Gprs.Time[1]/10)<<4)+(Gps_Gprs.Time[1]%10);
					GPSsaveBuf[GPSsaveBuf_Wr++]=((Gps_Gprs.Time[2]/10)<<4)+(Gps_Gprs.Time[2]%10);    
                   //------------------------------------------------------------------ 
                   //   注:   因为长度有限所以存储时只存储传感器速度，不填写 类型和长度 
                  //----------- 附加信息  ------------
				    //  附加信息 1  -----------------------------    
					//	附加信息 ID
					//Original_info[Original_info_Wr++]=0x03; // 行驶记录仪的速度
					//	附加信息长度
					//Original_info[Original_info_Wr++]=2;
					//	类型
					//GPSsaveBuf[GPSsaveBuf_Wr++]=(u8)(Speed_cacu>>8);  
					//GPSsaveBuf[GPSsaveBuf_Wr++]=(u8)(Speed_cacu);	           
				   //--------------------------------------------------------------------
				if(strncmp(( char * )GPSsaveBuf + GPSsaveBuf_Wr-3,(const char*)Sdgps_Time,3)==0)						
					{
					/*if(strncmp((const char*)GPSsaveBuf + GPSsaveBuf_Wr-3,(const char*)Sdgps_Time,3)==0)
						{
						PositionSD_Disable();
						rt_kprintf("\r\n  -->不存储上报时间相同的车\r\n");
						return;
						}
					else*/
						//{    //-------- 用RTC 时钟 -----     
				        time_now=Get_RTC();  
						Time2BCD(GPSsaveBuf + GPSsaveBuf_Wr-6);  
						rt_kprintf("\r\n    启用RTC时间了! \r\n");
						//}
					}
					memcpy(Sdgps_Time,GPSsaveBuf + GPSsaveBuf_Wr-3,3); //更新最新一次存储时间

                   //-------------  Caculate  FCS  -----------------------------------
					FCS_GPS_UDP=0;  
					for ( i = counter_mainguffer; i < 30; i++ )  
					{
							FCS_GPS_UDP ^= *( GPSsaveBuf + i ); 
					}			   //求上边数据的异或和
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
					 rt_kprintf("\r\n休眠时存储时间 %d-%d-%d %02d:%02d:%02d\r\n", time_now.year+2000, time_now.month, time_now.day, \
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
	   Original_info[Original_info_Wr++]=(MSG_ID>>8); // 消息ID 
	   Original_info[Original_info_Wr++]=(u8)MSG_ID;
	   
	   Original_info[Original_info_Wr++]=0x00; // 分包、加密方式、状态位 
	   Original_info[Original_info_Wr++]=28; // 消息体长度   位置信息长度为28个字节
	   
	   memcpy(Original_info+Original_info_Wr,SIM_code,6); // 终端手机号 ，设备标识ID	BCD
	   Original_info_Wr+=6;  
	   
	   Original_info[Original_info_Wr++]=( JT808Conf_struct.Msg_Float_ID>>8); //消息流水号
	   Original_info[Original_info_Wr++]= JT808Conf_struct.Msg_Float_ID;

       if(Packet_Type==Packet_Divide) 
       	{
            switch (MediaObj.Media_Type)
	         {
	          case 0 : // 图像
	                  MediaObj.Media_totalPacketNum=Photo_sdState.Total_packetNum;  // 图片总包数
					  MediaObj.Media_currentPacketNum=Photo_sdState.SD_packetNum;  // 图片当前报数
					  MediaObj.Media_ID=1;   //  多媒体ID
					  MediaObj.Media_Channel=Camera_Number;  // 图片摄像头通道号
			          break;
			  case 1 : // 音频
					  MediaObj.Media_totalPacketNum=Sound_sdState.Total_packetNum;	// 音频总包数
					  MediaObj.Media_currentPacketNum=Sound_sdState.SD_packetNum;  // 音频当前报数
					  MediaObj.Media_ID=1;	 //  多媒体ID
					  MediaObj.Media_Channel=1;  // 音频通道号   

			          break;
			  case 2 : // 视频
					  MediaObj.Media_totalPacketNum=Video_sdState.Total_packetNum;	// 视频总包数
					  MediaObj.Media_currentPacketNum=Video_sdState.SD_packetNum;  // 视频当前报数
					  MediaObj.Media_ID=1;	 //  多媒体ID
					  MediaObj.Media_Channel=1;  // 视频通道号 
			          break;
			  default:
			  	      return false;
	         }
			 
			 Original_info[Original_info_Wr++]=(MediaObj.Media_totalPacketNum&0xff00)>>8;//总block
			 Original_info[Original_info_Wr++]=(u8)MediaObj.Media_totalPacketNum;//总block
			 
			 
			 Original_info[Original_info_Wr++]=((MediaObj.Media_currentPacketNum)&0xff00)>>8;//当前block
			 Original_info[Original_info_Wr++]=(u8)((MediaObj.Media_currentPacketNum)&0x00ff);//当前block

       	}  
	    return true;

}  

void Protocol_End(u8 Packet_Type,u8 LinkNum)   
{                   
 u16 packet_len=0;  
 u16  i=0;		//要发送的UDP 数据内容的长度
 u8   Gfcs=0;
 u16   Msg_bodyLen=0; //  协议里的消息只表示消息体     不包含消息头 消息头默认长度是12 , 分包消息头长度 20   
 
   Gfcs=0;				   //  计算从消息头开始到校验前数据的异或和  808协议校验  1Byte
   //---  填写信息长度 ---
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
     Original_info[2]=(Msg_bodyLen>>8)|0x20 ;  // Bit 13  0x20 就是Bit 13
     Original_info[3]=Msg_bodyLen;  
   }
   //---- 计算校验  -----
   for(i=0;i<Original_info_Wr;i++)  
		Gfcs^=Original_info[i]; 
   Original_info[Original_info_Wr++]=Gfcs;  // 填写G校验位     


  // 1.stuff start
   GPRS_infoWr_Tx=0;	
   GPRS_info[GPRS_infoWr_Tx++]=0x7e;   // Start 标识位
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
   GPRS_info[GPRS_infoWr_Tx++]=0x7e;  //  End  标识
  //  4. Send   

    // 4.1 发送信息内容1
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
   //--------消息序号 递增 --------
   JT808Conf_struct.Msg_Float_ID++;   
  //------------------------------ 
} 
//--------------------------------------------------------------------------------------
u8  Stuff_DevCommmonACK_0001H(void)      
{    
	// 1. Head
	  if(!Protocol_Head(MSG_0x0001,Packet_Normal))  return false;     //终端通用应答
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
	//  终端型号 20 Bytes      -- 补充协议里做更改
	memcpy(Original_info+Original_info_Wr,"Tianjin TCB TW701-BD",20);  
    Original_info_Wr+=20;      
    //  终端ID   7 Bytes    ,    
    memcpy(Original_info+Original_info_Wr,DeviceNumberID+5,7);   
    Original_info_Wr+=7;  
	//  车牌颜色  
	Original_info[Original_info_Wr++]=JT808Conf_struct.Vechicle_Info.Dev_Color;
	//  车牌
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
    memcpy(Original_info+Original_info_Wr,"70103",5);  //北京中斗  70104
    Original_info_Wr+=5;
	//  终端型号 20 Bytes      -- 补充协议里做更改
	memcpy(Original_info+Original_info_Wr,"HVT100BD3",9);   //ZD-V01H  HVT100BD1
    Original_info_Wr+=9;      
    for(i=0;i<11;i++)
	 Original_info[Original_info_Wr++]=0x00;	 
    //  终端ID   7 Bytes    ,    
    memcpy(Original_info+Original_info_Wr,IMSI_CODE+8,7);     
    Original_info_Wr+=7;  
	//  车牌颜色  
	Original_info[Original_info_Wr++]=2;//JT808Conf_struct.Vechicle_Info.Dev_Color;
	
	if(JT808Conf_struct.Vechicle_Info.Dev_Color!=0)
	{
		//  车牌
		memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Vechicle_Info.Vech_Num,13);  
		Original_info_Wr+=13;
	}
	else
	{
	       rt_kprintf("\r\n  车辆颜色:0     Need Vin\r\n");
		// 车辆VIN 
		memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Vechicle_Info.Vech_VIN,17);
		Original_info_Wr+=17;
    }

 
 //  3. Send 
  Protocol_End(Packet_Normal,LinkNum);   
  if(DispContent)
  {      
       rt_kprintf("\r\n	SEND Reigster Packet! \r\n");  
	 rt_kprintf("\r\n  注册发送时间 %d-%d-%d %02d:%02d:%02d\r\n", time_now.year+2000, time_now.month, time_now.day, \
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
 	  return false; //终端注销 
	 // 2. content  is null 
	 //  3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	Deregister  注销! \r\n");  
	 return true;    
}
//------------------------------------------------------------------------------------
u8  Stuff_DevLogin_0102H(void)      
{    
	// 1. Head
	if(!Protocol_Head(MSG_0x0102,Packet_Normal))  
 	  return false; //终端鉴权 
	 // 2. content  
	  
	 memcpy(Original_info+Original_info_Wr,JT808Conf_struct.ConfirmCode,strlen((const char*)JT808Conf_struct.ConfirmCode));   // 鉴权码  string Type
	 Original_info_Wr+=strlen((const char*)JT808Conf_struct.ConfirmCode); 
	 //  3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	 发送鉴权! \r\n");   
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
 Original_info_Wr+=28;   // 内容只有28 
 memcpy(spd_sensorReg,Original_info+Original_info_Wr,2);// 把读取传感器速度 
 
 
  //----------- 附加信息  ------------ 
  //  附加信息 1  -----------------------------    
  //  附加信息 ID
  Original_info[Original_info_Wr++]=0x03; // 行驶记录仪的速度
  //  附加信息长度
  Original_info[Original_info_Wr++]=2;
  //  类型
  Original_info[Original_info_Wr++]=(u8)(Speed_cacu>>8); 
  Original_info[Original_info_Wr++]=(u8)(Speed_cacu);	   
  //rt_kprintf("\r\n GPS速度=%d km/h , 传感器速度=%d km/h\r\n",Speed_gps,Speed_cacu); 
   //  附加信息 2  -----------------------------	
   //  附加信息 ID
   Original_info[Original_info_Wr++]=0x01; // 车上的行驶里程
   //  附加信息长度
   Original_info[Original_info_Wr++]=4; 
   //  类型
   Dis_01km=JT808Conf_struct.Distance_m_u32/100;
   Original_info[Original_info_Wr++]=(Dis_01km>>24); 
   Original_info[Original_info_Wr++]=(Dis_01km>>16); 
   Original_info[Original_info_Wr++]=(Dis_01km>>8); 
   Original_info[Original_info_Wr++]=Dis_01km; 
 
 
  //  附加信息 3 
  if(Warn_Status[1]&0x10)
 {
   //  附加信息 ID
   Original_info[Original_info_Wr++]=0x12; //  进出区域/路线报警
   //  附加信息长度 
   Original_info[Original_info_Wr++]=6;
   //  类型
   Original_info[Original_info_Wr++]=InOut_Object.TYPE;
   Original_info[Original_info_Wr++]=(InOut_Object.ID>>24);
   Original_info[Original_info_Wr++]=(InOut_Object.ID>>16);
   Original_info[Original_info_Wr++]=(InOut_Object.ID>>8);
   Original_info[Original_info_Wr++]=InOut_Object.ID;
   Original_info[Original_info_Wr++]=InOut_Object.InOutState;  
   rt_kprintf("\r\n ----- 0x0200 附加信息 \r\n");	 
 }
 
  //  附件信息4
  if(Warn_Status[3]&0x02)
  { 	 
	//	附加信息 ID
	Original_info[Original_info_Wr++]=0x11; //	进出区域/路线报警
	//	附加信息长度 
	Original_info[Original_info_Wr++]=1; 
	//	类型
	Original_info[Original_info_Wr++]=0; //  无特定位置   
 
  }


	   //  附加信息 5  -----------------------------	  
	 //  附加信息 ID
	 Original_info[Original_info_Wr++]=0xFE; //信号强度
	 //  附加信息长度
	 Original_info[Original_info_Wr++]=2; 
	 //  类型
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_SignalValue; 
	 Original_info[Original_info_Wr++]=0x00;  //  保留 

        //if(DispContent)
         //     printf("\r\n---- Satelitenum: %d , CSQ:%d\r\n",Satelite_num,ModuleSQ);  

        //  附加信息 6  -----------------------------	  
	 //  附加信息 ID
	 Original_info[Original_info_Wr++]=0xFF; //自定义模拟量上传
	 //  附加信息长度
	 Original_info[Original_info_Wr++]=6; 
	 //  类型
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_IO_1; 
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_IO_2;  
	 Original_info[Original_info_Wr++]=(BD_EXT.AD_0>>8);  // 模拟量 1
	 Original_info[Original_info_Wr++]=BD_EXT.AD_0;
	 Original_info[Original_info_Wr++]=(BD_EXT.AD_1>>8);  // 模拟量 2
	 Original_info[Original_info_Wr++]=BD_EXT.AD_1;

 
 //  3. Send 
 Protocol_End(Packet_Normal ,0);
   	
 return true; 

}
//------------------------------------------------------------------------------------
u8  Stuff_Current_Data_0200H(void)   //  发送即时数据不存储到存储器中
{  
  
  u32  Dis_01km=0;
  
    if( GPS_speed <=( JT808Conf_struct.Speed_warn_MAX*10) )
		StatusReg_SPD_NORMAL(); 

 //  1. Head	
 if(!Protocol_Head(MSG_0x0200,Packet_Normal)) 
 	  return false; 
 // 2. content 
    //------------------------------- Stuff ----------------------------------------
	// 1. 告警标志  4
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Warn_Status,4 );    
	Original_info_Wr += 4;
	// 2. 状态  4
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Car_Status,4 );   
	Original_info_Wr += 4;
	// 3.  纬度
    memcpy( ( char * ) Original_info+ Original_info_Wr,( char * )  Gps_Gprs.Latitude, 4 );//纬度   modify by nathan
	Original_info_Wr += 4;
	// 4.  经度
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )  Gps_Gprs.Longitude, 4 );	  //经度    东经  Bit 7->0   西经 Bit 7 -> 1
	Original_info_Wr += 4;
	// 5.  高程
	Original_info[Original_info_Wr++]=(u8)(GPS_Hight<<8);
	Original_info[Original_info_Wr++]=(u8)GPS_Hight;
	// 6.  速度    0.1 Km/h
	Original_info[Original_info_Wr++]=(u8)(Speed_gps>>8);//(GPS_speed>>8); 
	Original_info[Original_info_Wr++]=(u8)(Speed_gps);//GPS_speed;     
	// 7. 方向   单位 1度
	Original_info[Original_info_Wr++]=(GPS_direction>>8);  //High 
	Original_info[Original_info_Wr++]=GPS_direction; // Low
	// 8.  日期时间	
	Original_info[Original_info_Wr++]=(((Gps_Gprs.Date[0])/10)<<4)+((Gps_Gprs.Date[0])%10);		
	Original_info[Original_info_Wr++]=((Gps_Gprs.Date[1]/10)<<4)+(Gps_Gprs.Date[1]%10); 
	Original_info[Original_info_Wr++]=((Gps_Gprs.Date[2]/10)<<4)+(Gps_Gprs.Date[2]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[0]/10)<<4)+(Gps_Gprs.Time[0]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[1]/10)<<4)+(Gps_Gprs.Time[1]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[2]/10)<<4)+(Gps_Gprs.Time[2]%10);	 

	//----------- 附加信息  ------------
    //  附加信息 1  -----------------------------    
	//	附加信息 ID
	Original_info[Original_info_Wr++]=0x03; // 行驶记录仪的速度
	//	附加信息长度
	Original_info[Original_info_Wr++]=2;
	//	类型
	Original_info[Original_info_Wr++]=(u8)(Speed_cacu>>8); 
	Original_info[Original_info_Wr++]=(u8)(Speed_cacu);	     
	//rt_kprintf("\r\n GPS速度=%d km/h , 传感器速度=%d km/h\r\n",Speed_gps,Speed_cacu); 
     //  附加信息 2  -----------------------------	  
	 //  附加信息 ID
	 Original_info[Original_info_Wr++]=0x01; // 车上的行驶里程
	 //  附加信息长度
	 Original_info[Original_info_Wr++]=4; 
	 //  类型
	 Dis_01km=JT808Conf_struct.Distance_m_u32/100;
	 Original_info[Original_info_Wr++]=(Dis_01km>>24); 
	 Original_info[Original_info_Wr++]=(Dis_01km>>16); 
	 Original_info[Original_info_Wr++]=(Dis_01km>>8); 
	 Original_info[Original_info_Wr++]=Dis_01km; 
  
   
	//  附加信息 3 
	if(Warn_Status[1]&0x10)
   {
     //  附加信息 ID
     Original_info[Original_info_Wr++]=0x12; //  进出区域/路线报警
     //  附加信息长度 
     Original_info[Original_info_Wr++]=6;
	 //  类型
	 Original_info[Original_info_Wr++]=InOut_Object.TYPE;
	 Original_info[Original_info_Wr++]=(InOut_Object.ID>>24);
	 Original_info[Original_info_Wr++]=(InOut_Object.ID>>16);
	 Original_info[Original_info_Wr++]=(InOut_Object.ID>>8);
	 Original_info[Original_info_Wr++]=InOut_Object.ID;
	 Original_info[Original_info_Wr++]=InOut_Object.InOutState;  
	 rt_kprintf("\r\n ----- 0x0200 current 附加信息 \r\n");    
   }

    //  附件信息4
    if(Warn_Status[3]&0x02)
    {      
	  //  附加信息 ID
	  Original_info[Original_info_Wr++]=0x11; //  进出区域/路线报警
	  //  附加信息长度 
	  Original_info[Original_info_Wr++]=1; 
	  //  类型
	  Original_info[Original_info_Wr++]=0; //  无特定位置   

    }


	   //  附加信息 5  -----------------------------	  
	 //  附加信息 ID
	 Original_info[Original_info_Wr++]=0xFE; //信号强度
	 //  附加信息长度
	 Original_info[Original_info_Wr++]=2; 
	 //  类型
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_SignalValue; 
	 Original_info[Original_info_Wr++]=0x00;  //  保留 

        //if(DispContent)
         //     printf("\r\n---- Satelitenum: %d , CSQ:%d\r\n",Satelite_num,ModuleSQ);  

        //  附加信息 6  -----------------------------	  
	 //  附加信息 ID
	 Original_info[Original_info_Wr++]=0xFF; //自定义模拟量上传
	 //  附加信息长度
	 Original_info[Original_info_Wr++]=6; 
	 //  类型
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_IO_1; 
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_IO_2;  
	 Original_info[Original_info_Wr++]=(BD_EXT.AD_0>>8);  // 模拟量 1
	 Original_info[Original_info_Wr++]=BD_EXT.AD_0;
	 Original_info[Original_info_Wr++]=(BD_EXT.AD_1>>8);  // 模拟量 2
	 Original_info[Original_info_Wr++]=BD_EXT.AD_1;
	
 //  3. Send 
 Protocol_End(Packet_Normal ,0);

  if(SleepState==1)  	
  {   
      rt_kprintf("\r\n休眠时发送时间 %d-%d-%d %02d:%02d:%02d\r\n", time_now.year+2000, time_now.month, time_now.day, \
      time_now.hour, time_now.min, time_now.sec);
  }
 return true; 

}
//-----------------------------------------------------------------------
u8  Stuff_Current_Data_0201H(void)   //   位置信息查询回应
{  
   u32  Dis_01km=0; 
 //  1. Head	
 if(!Protocol_Head(MSG_0x0201,Packet_Normal)) 
 	  return false; 
 // 2. content 
    //------------------------------- Stuff ----------------------------------------
    //   float ID                                                // 对应中心应答消息的流水号
	Original_info[Original_info_Wr++]=(u8)(Centre_FloatID>>8);
	Original_info[Original_info_Wr++]=(u8)Centre_FloatID;

	// 1. 告警标志  4
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Warn_Status,4 );    
	Original_info_Wr += 4;
	// 2. 状态  4
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Car_Status,4 );   
	Original_info_Wr += 4;
	// 3.  纬度
    memcpy( ( char * ) Original_info+ Original_info_Wr,( char * )  Gps_Gprs.Latitude, 4 );//纬度   modify by nathan
	Original_info_Wr += 4;
	// 4.  经度
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )  Gps_Gprs.Longitude, 4 );	  //经度    东经  Bit 7->0   西经 Bit 7 -> 1
	Original_info_Wr += 4;
	// 5.  高程
	Original_info[Original_info_Wr++]=(u8)(GPS_Hight<<8);
	Original_info[Original_info_Wr++]=(u8)GPS_Hight;
	// 6.  速度    0.1 Km/h
	Original_info[Original_info_Wr++]=(u8)(Speed_gps>>8);
	Original_info[Original_info_Wr++]=(u8)Speed_gps;   
	// 7. 方向   单位 1度
	Original_info[Original_info_Wr++]=(GPS_direction>>8);  //High 
	Original_info[Original_info_Wr++]=GPS_direction; // Low
	// 8.  日期时间	
	Original_info[Original_info_Wr++]=(((Gps_Gprs.Date[0])/10)<<4)+((Gps_Gprs.Date[0])%10);		
	Original_info[Original_info_Wr++]=((Gps_Gprs.Date[1]/10)<<4)+(Gps_Gprs.Date[1]%10); 
	Original_info[Original_info_Wr++]=((Gps_Gprs.Date[2]/10)<<4)+(Gps_Gprs.Date[2]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[0]/10)<<4)+(Gps_Gprs.Time[0]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[1]/10)<<4)+(Gps_Gprs.Time[1]%10);
	Original_info[Original_info_Wr++]=((Gps_Gprs.Time[2]/10)<<4)+(Gps_Gprs.Time[2]%10);	 

	
		//----------- 附加信息  ------------
    //  附加信息 1  -----------------------------    
	//	附加信息 ID
	Original_info[Original_info_Wr++]=0x03; // 行驶记录仪的速度
	//	附加信息长度
	Original_info[Original_info_Wr++]=2;
	//	类型
	Original_info[Original_info_Wr++]=(u8)(Speed_cacu>>8); 
	Original_info[Original_info_Wr++]=(u8)(Speed_cacu);	     
	//rt_kprintf("\r\n GPS速度=%d km/h , 传感器速度=%d km/h\r\n",Speed_gps,Speed_cacu); 
     //  附加信息 2  -----------------------------	  
	 //  附加信息 ID
	 Original_info[Original_info_Wr++]=0x01; // 车上的行驶里程
	 //  附加信息长度
	 Original_info[Original_info_Wr++]=4; 
	 //  类型
	 Dis_01km=JT808Conf_struct.Distance_m_u32/100;
	 Original_info[Original_info_Wr++]=(Dis_01km>>24); 
	 Original_info[Original_info_Wr++]=(Dis_01km>>16); 
	 Original_info[Original_info_Wr++]=(Dis_01km>>8); 
	 Original_info[Original_info_Wr++]=Dis_01km; 
  
   
	//  附加信息 3 
	if(Warn_Status[1]&0x10)
   {
     //  附加信息 ID
     Original_info[Original_info_Wr++]=0x12; //  进出区域/路线报警
     //  附加信息长度 
     Original_info[Original_info_Wr++]=6;
	 //  类型
	 Original_info[Original_info_Wr++]=InOut_Object.TYPE;
	 Original_info[Original_info_Wr++]=(InOut_Object.ID>>24);
	 Original_info[Original_info_Wr++]=(InOut_Object.ID>>16);
	 Original_info[Original_info_Wr++]=(InOut_Object.ID>>8);
	 Original_info[Original_info_Wr++]=InOut_Object.ID;
	 Original_info[Original_info_Wr++]=InOut_Object.InOutState;  
	 rt_kprintf("\r\n ----- 0x0201 附加信息 \r\n");    
   }

    //  附件信息4
    if(Warn_Status[3]&0x02)
    {      
	  //  附加信息 ID
	  Original_info[Original_info_Wr++]=0x11; //  进出区域/路线报警
	  //  附加信息长度 
	  Original_info[Original_info_Wr++]=1; 
	  //  类型
	  Original_info[Original_info_Wr++]=0; //  无特定位置   

    }

    
	   //  附加信息 5  -----------------------------	  
	 //  附加信息 ID
	 Original_info[Original_info_Wr++]=0xFE; //信号强度
	 //  附加信息长度
	 Original_info[Original_info_Wr++]=2; 
	 //  类型
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_SignalValue; 
	 Original_info[Original_info_Wr++]=0x00;  //  保留 

        //if(DispContent)
         //     printf("\r\n---- Satelitenum: %d , CSQ:%d\r\n",Satelite_num,ModuleSQ);  

        //  附加信息 6  -----------------------------	  
	 //  附加信息 ID
	 Original_info[Original_info_Wr++]=0xFF; //自定义模拟量上传
	 //  附加信息长度
	 Original_info[Original_info_Wr++]=6; 
	 //  类型
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_IO_1; 
	 Original_info[Original_info_Wr++]= BD_EXT.FJ_IO_2;  
	 Original_info[Original_info_Wr++]=(BD_EXT.AD_0>>8);  // 模拟量 1
	 Original_info[Original_info_Wr++]=BD_EXT.AD_0;
	 Original_info[Original_info_Wr++]=(BD_EXT.AD_1>>8);  // 模拟量 2
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
 	  return false; // 终端参数上传

  //  2. content 
       //   float ID
	    Original_info[Original_info_Wr++]=(u8)(Centre_FloatID>>8);
	    Original_info[Original_info_Wr++]=(u8)Centre_FloatID;
	   //   参数个数	
	    Original_info[Original_info_Wr++]=4;
	   //   参数列表

	   //   2.1   车牌号
       /* Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
        Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x83;
		Original_info[Original_info_Wr++]=strlen((const char*)JT808Conf_struct.Vechicle_Info.Vech_Num); // 参数长度
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )JT808Conf_struct.Vechicle_Info.Vech_Num,strlen((const char*)JT808Conf_struct.Vechicle_Info.Vech_Num) ); // 参数值
		Original_info_Wr+=strlen((const char*)JT808Conf_struct.Vechicle_Info.Vech_Num); 
        */
		//   2.2  主服务器IP    
        Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
        Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x13;    
		                                         // 参数长度			    
		memset(reg_str,0,sizeof(reg_str));
	    IP_Str((char*)reg_str, *( u32 * ) RemoteIP_aux);   
		Original_info[Original_info_Wr++]=strlen((const char*)reg_str);
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )reg_str,strlen((const char*)reg_str));  // 参数值	 
		Original_info_Wr+=strlen((const char*)reg_str);

		//   2.3   主服务TCP端口
        Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
        Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x18;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=0x00;   // 参数值
		Original_info[Original_info_Wr++]=0x00; 
		Original_info[Original_info_Wr++]=(RemotePort_main>>8); 
		Original_info[Original_info_Wr++]=RemotePort_main; 
													    
		 //   2.4  APN 字符串
        Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
        Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x10;
		Original_info[Original_info_Wr++]=strlen((const char*)APN_String); // 参数长度
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )APN_String,strlen((const char*)APN_String)); // 参数值
		Original_info_Wr+=strlen((const char*)APN_String);

        //  2.5   备用IP		 
        Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
        Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x17;    
		                                         // 参数长度		 	    
		memset(reg_str,0,sizeof(reg_str));
	    IP_Str((char*)reg_str, *( u32 * ) RemoteIP_aux);
		Original_info[Original_info_Wr++]=strlen((const char*)reg_str);
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )reg_str,strlen((const char*)reg_str));  // 参数值	 
		Original_info_Wr+=strlen((const char*)reg_str);
		
       /*   
		 //   2.4   缺省时间上报间隔
        Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
        Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x29;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Default_Dur>>24);   // 参数值 
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Default_Dur>>16);
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Default_Dur>>8);
		Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Default_Dur);     
		
		 //   2.5   中心监控号码
        Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
        Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x40;
		Original_info[Original_info_Wr++]=strlen((const char*)JT808Conf_struct.LISTEN_Num); // 参数长度
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )JT808Conf_struct.LISTEN_Num,strlen((const char*)JT808Conf_struct.LISTEN_Num)); // 参数值
		Original_info_Wr+=strlen((const char*)JT808Conf_struct.LISTEN_Num);
		 //   2.6   最大速度门限
        Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
        Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x55;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=( JT808Conf_struct.Speed_warn_MAX>>24);   // 参数值
		Original_info[Original_info_Wr++]=( JT808Conf_struct.Speed_warn_MAX>>16);  
		Original_info[Original_info_Wr++]=( JT808Conf_struct.Speed_warn_MAX>>8);
		Original_info[Original_info_Wr++]=( JT808Conf_struct.Speed_warn_MAX);
		 //   2.7   连续驾驶门限
        Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
        Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x00;
		Original_info[Original_info_Wr++]=0x57;
		Original_info[Original_info_Wr++]=4  ; // 参数长度
		Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DrvKeepingSec>>24);   // 参数值
		Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DrvKeepingSec>>16);  
		Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DrvKeepingSec>>8);
		Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DrvKeepingSec);
		 //   2.8   最小休息时间
		 Original_info[Original_info_Wr++]=0x00;   // 参数ID 4Bytes
		 Original_info[Original_info_Wr++]=0x00;
		 Original_info[Original_info_Wr++]=0x00;
		 Original_info[Original_info_Wr++]=0x59;
		 Original_info[Original_info_Wr++]=4  ; // 参数长度
		 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MinSleepSec>>24);	 // 参数值
		 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MinSleepSec>>16);	
		 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MinSleepSec>>8);
		 Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MinSleepSec);         
		 */
  //  3. Send 
  Protocol_End(Packet_Normal ,0);
  if(DispContent)
	rt_kprintf("\r\n	发送参数查询信息! \r\n");   

  return true;
}
//--------------------------------------------------------------------------
u8  Stuff_EventACK_0301H(void)      
{    
	// 1. Head
	if(!Protocol_Head(MSG_0x0301,Packet_Normal)) 
 	  return false; 
	 // 2. content   
	   Original_info[Original_info_Wr++]=EventObj.Event_ID;// 返回事件ID
	    
	 //  3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	 事件结果返回  \r\n");    
	 return true; 

}

u8  Stuff_ASKACK_0302H(void)      
{  
  
	// 1. Head
	if(!Protocol_Head(MSG_0x0302,Packet_Normal)) 
 	  return false; 
	 // 2. content   
	   //  应答流水号
	   Original_info[Original_info_Wr++]=(ASK_Centre.ASK_floatID>>8);// 返回事件ID
	   Original_info[Original_info_Wr++]=ASK_Centre.ASK_floatID;
       Original_info[Original_info_Wr++]=ASK_Centre.ASK_answerID;	    
	 //  3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	发送提问选择结果 \r\n");     
	 return true; 

}

u8  Stuff_MSGACK_0303H(void)      
{  
 
	// 1. Head
	if(!Protocol_Head(MSG_0x0303,Packet_Normal)) 
 	  return false; 
	 // 2. content   
	   //  应答流水号
	   Original_info[Original_info_Wr++]=MSG_BroadCast_Obj.INFO_TYPE; 
	   Original_info[Original_info_Wr++]=MSG_BroadCast_Obj.INFO_PlyCancel;  //  0  取消  1 点播 
	 //  3. Send 
	 Protocol_End(Packet_Normal ,0);
	  if(DispContent)
	 rt_kprintf("\r\n	点播取消回复  \r\n");     
	 return true;    

}

u8  Stuff_ControlACK_0500H(void)   //   车辆控制应答
{  

 //  1. Head	
   if(!Protocol_Head(MSG_0x0500,Packet_Normal)) 
 	  return false; 
 // 2. content 
    //------------------------------- Stuff ----------------------------------------
    //   float ID                                                // 对应中心应答消息的流水号
	Original_info[Original_info_Wr++]=(u8)(Vech_Control.CMD_FloatID>>8);
	Original_info[Original_info_Wr++]=(u8)Vech_Control.CMD_FloatID;  

	// 1. 告警标志  4
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Warn_Status,4 );    
	Original_info_Wr += 4;
	// 2. 状态  4
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Car_Status,4 );   
	Original_info_Wr += 4;
	// 3.  纬度
    memcpy( ( char * ) Original_info+ Original_info_Wr,( char * )  Gps_Gprs.Latitude, 4 );//纬度   modify by nathan
	Original_info_Wr += 4;
	// 4.  经度
	memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )  Gps_Gprs.Longitude, 4 );	  //经度    东经  Bit 7->0   西经 Bit 7 -> 1
	Original_info_Wr += 4;
	// 5.  高程
	Original_info[Original_info_Wr++]=(u8)(GPS_Hight<<8);
	Original_info[Original_info_Wr++]=(u8)GPS_Hight;
	// 6.  速度    0.1 Km/h
	Original_info[Original_info_Wr++]=(u8)(Speed_gps>>8);
	Original_info[Original_info_Wr++]=(u8)Speed_gps;   
	// 7. 方向   单位 1度
	Original_info[Original_info_Wr++]=(GPS_direction>>8);  //High 
	Original_info[Original_info_Wr++]=GPS_direction; // Low
	// 8.  日期时间	
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

u8  Stuff_RecoderACK_0700H(void)   //   行车记录仪数据上传
{  

   u16	SregLen=0,Swr=0;//,Gwr=0; // S:serial  G: GPRS   
   u8	       Sfcs=0;     
   u16	i=0;   
   u32	regdis=0,reg2=0;   
   u8   	Reg[70];  
   u8       QueryRecNum=0;  // 查询记录数目

 //  1. Head	
 if(!Protocol_Head(MSG_0x0700,Packet_Normal)) 
 	  return false; 
 // 2. content 
    //------------------------------- Stuff ----------------------------------------
    //   float ID                                                // 对应中心应答消息的流水号
	Original_info[Original_info_Wr++]=(u8)(Recode_Obj.Float_ID>>8);
	Original_info[Original_info_Wr++]=(u8)Recode_Obj.Float_ID;  
	//   命令字
	Original_info[Original_info_Wr++]=Recode_Obj.CMD;   // 命令字  

    //   记录仪 数据块
	//---------------填写 A 协议头 ------------------------
	Swr=Original_info_Wr;	// reg save 
	Original_info[Original_info_Wr++]=0x55;  // 起始头
	Original_info[Original_info_Wr++]=0x7A; 
	//---------------根据类型分类填写内容------------------
	switch(Recode_Obj.CMD)  
	  {
		  //---------------- 上传数类型  -------------------------------------
		  case	A_Up_DrvInfo  : 	 //  当前驾驶员代码及对应的机动车驾驶证号
							   Original_info[Original_info_Wr++]=Recode_Obj.CMD; //命令字
	
							   SregLen=0x00;		   // 信息长度
							   Original_info[Original_info_Wr++]=0x00;    // Hi
							   Original_info[Original_info_Wr++]=21;	   // Lo
							   
							   Original_info[Original_info_Wr++]=0x00;    // 保留字 
							   
							   
							   memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Driver_Info.DriveCode,3);  
							   Original_info_Wr+=3;							   
							   memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Driver_Info.DriverCard_ID,18); //信息内容
							   Original_info_Wr+=18;  
							   break;
		  case	A_Up_RTC	  : 	 //  采集记录仪的实时时钟
							  Original_info[Original_info_Wr++]=Recode_Obj.CMD; //命令字
	
							   SregLen=0x00;		   // 信息长度
							   Original_info[Original_info_Wr++]=0x00;    // Hi
							   Original_info[Original_info_Wr++]=6;	  // Lo
							   
							   Original_info[Original_info_Wr++]=0x00;    // 保留字
							   
							   Time2BCD(Original_info+Original_info_Wr);	//信息内容	
							   Original_info_Wr+=6;
							   break;
		  case	A_Up_Dist	  :    //  采集360h内里程
							   Original_info[Original_info_Wr++]=Recode_Obj.CMD; //命令字
	
							   SregLen=0x00;		   // 信息长度
							   Original_info[Original_info_Wr++]=0x00;    // Hi
							   Original_info[Original_info_Wr++]=8;	   // Lo
							   
							   Original_info[Original_info_Wr++]=0x00;    // 保留字
							   //	信息内容
							    // -- 里程 3个字节 单位0.1km    6位
							   regdis=JT808Conf_struct.Distance_m_u32/100;  //单位0.1km 
							   reg2=regdis/100000;
							  Original_info[Original_info_Wr++]=(reg2<<4)+(regdis%100000/10000);
							  Original_info[Original_info_Wr++]=((regdis%10000/1000)<<4)+(regdis%1000/100);
							  Original_info[Original_info_Wr++]=((regdis%100/10)<<4)+(regdis%10);   
							    //  --读出里程时的RTC --- 
							  Time2BCD(Original_info+Original_info_Wr);	//信息内容	
							  Original_info_Wr+=5;   // 只要求到分 所以是 5个字节
							  break; 
	
		  case	A_Up_PLUS	  :    //  采集记录仪特征系数
							   Original_info[Original_info_Wr++]=Recode_Obj.CMD; //命令字
	
							   SregLen=0x00;		   // 信息长度
							   Original_info[Original_info_Wr++]=0x00;    // Hi
							   Original_info[Original_info_Wr++]=3;	   // Lo
							   
							   Original_info[Original_info_Wr++]=0x00;    // 保留字
	
							   //  信息内容
							   Original_info[Original_info_Wr++]=(u8)(JT808Conf_struct.Vech_Character_Value>>16);
							   Original_info[Original_info_Wr++]=(u8)(JT808Conf_struct.Vech_Character_Value>>8); 
							   Original_info[Original_info_Wr++]=(u8)(JT808Conf_struct.Vech_Character_Value);  
							   
							   break;
		  case	A_Up_VechInfo :    //  车辆信息
							   Original_info[Original_info_Wr++]=Recode_Obj.CMD; //命令字
	
							   SregLen=0x00;		   // 信息长度
							   Original_info[Original_info_Wr++]=0x00;    // Hi
							   Original_info[Original_info_Wr++]=41;	   // Lo
							   
							   Original_info[Original_info_Wr++]=0x00;    // 保留字							  
							   
							   memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Vechicle_Info.Vech_VIN,17); //信息内容
							   Original_info_Wr+=17;
							   memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Vechicle_Info.Vech_Num,12);
							   Original_info_Wr+=12;
							   memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Vechicle_Info.Vech_Type,12);	
							   Original_info_Wr+=12; 
	
							   break;
	      case	A_Up_Doubt	  :    //  事故疑点数据
						     Original_info[Original_info_Wr++]=Recode_Obj.CMD; //命令字
  
							 SregLen=206;		 // 信息长度
							 Original_info[Original_info_Wr++]=(u8)(SregLen>>8);	 // Hi
							 Original_info[Original_info_Wr++]=(u8)SregLen;		 // Lo		 
							 
							 Original_info[Original_info_Wr++]=0x00;	 // 保留字		
							 //------- 信息内容 ------		
							 if(Api_DFdirectory_Read(doubt_data, Original_info+Original_info_Wr,207,0,0)==1)
							 	  Original_info_Wr+=206; //	 内容是206
							 else
							 {  //该填写长度
				                                   Original_info[Original_info_Wr-3]=0x00;//Hi
				                                   Original_info[Original_info_Wr-2]=0x00;//lo
							 } 
	
							   break;							   
		  case	A_Up_N2daysDist:    //  最近2 天累计行驶里程
							   Original_info[Original_info_Wr++]=Recode_Obj.CMD; //命令字
	
							   SregLen=0x00;		   // 信息长度
							   Original_info[Original_info_Wr++]=0x00;    // Hi
							   Original_info[Original_info_Wr++]=8;	   // Lo
							   
							   Original_info[Original_info_Wr++]=0x00;    // 保留字
							   //	信息内容
							    // -- 里程 3个字节 单位0.1km    6位
							   regdis=(JT808Conf_struct.Distance_m_u32-JT808Conf_struct.DayStartDistance_32)/100;  //单位0.1km 
							   reg2=regdis/100000;
							   Original_info[Original_info_Wr++]=(reg2<<4)+(regdis%100000/10000);
							   Original_info[Original_info_Wr++]=((regdis%10000/1000)<<4)+(regdis%1000/100);
							   Original_info[Original_info_Wr++]=((regdis%100/10)<<4)+(regdis%10);   
							    //  --读出里程时的RTC --- 
							  Time2BCD(Original_info+Original_info_Wr);	//信息内容	
							  Original_info_Wr+=5;   // 只要求到分 所以是 5个字节
							  // Original_info_Wr+=18;		   //  18字节内不足填写00H
							   break; 
	      case  A_Up_N2daysSpd:	  // 最近2天内的行驶速度   7小时
									  Original_info[Original_info_Wr++]=Recode_Obj.CMD; //命令字
		   
									  SregLen=455;		  // 信息长度
									  Original_info[Original_info_Wr++]=(u8)(SregLen>>8);	  // Hi
									  Original_info[Original_info_Wr++]=(u8)SregLen;	  // Lo    65x7    
									  
									  Original_info[Original_info_Wr++]=0x00;	  // 保留字 	
									  //-----------  信息内容  --------------		
                                                                   //----------------------
										  QueryRecNum=Api_DFdirectory_Query(spdpermin,0);   //查询当前疲劳驾驶记录数目
										 if(QueryRecNum>7)								   
										        QueryRecNum=7;			
								
										    SregLen=QueryRecNum*65;
										      // 改写信息长度
			                                                      Original_info[Original_info_Wr-3]=(u8)(SregLen>>8);	  // Hi
			                                                      Original_info[Original_info_Wr-2]=(u8)SregLen;	  // Lo    65x7    
										 for(i=0;i<QueryRecNum;i++)			   // 从最新处读取存储填写
										  {
											 Api_DFdirectory_Read(spdpermin,Reg,70,0,i); // 从new-->old  读取
											  memcpy(Original_info+Original_info_Wr,Reg+5,60);	// 只填写速度
											Original_info_Wr+=65;	    
										  }
									       //------------------------------
									  
							break; 

		  case	A_Up_AvrgMin  : 	 //  360h 对应每分钟的平均速度	 // 默认填写最新7hour记录 
										 Original_info[Original_info_Wr++]=Recode_Obj.CMD; //命令字
			  
										 SregLen=435;		 // 信息长度 
										 Original_info[Original_info_Wr++]=(u8)(SregLen>>8);	 // Hi
										 Original_info[Original_info_Wr++]=(u8)SregLen;		 // Lo	  65x7	  
										 
										 Original_info[Original_info_Wr++]=0x00;	 // 保留字	   
										 //-----------	信息内容  --------------	
                                                                                  //----------------------
										  QueryRecNum=Api_DFdirectory_Query(spdpermin,0);   //查询当前疲劳驾驶记录数目
										 if(QueryRecNum>7)								   
										        QueryRecNum=7;			
								
										    SregLen=QueryRecNum*65;     // 改写信息长度
			                                                      Original_info[Original_info_Wr-3]=(u8)(SregLen>>8);	  // Hi
			                                                      Original_info[Original_info_Wr-2]=(u8)SregLen;	  // Lo    65x7    

																  
										 for(i=0;i<QueryRecNum;i++)			   // 从最新处读取存储填写
										  {
											 Api_DFdirectory_Read(spdpermin,Reg,70,1,i); // 从new-->old  读取
											  memcpy(Original_info+Original_info_Wr,Reg+5,60);	// 只填写速度
											Original_info_Wr+=65;	    
										  }
									       //------------------------------
										 
							   break; 
	
		  case	A_Up_Tired	  : 	//	疲劳驾驶记录
										 Original_info[Original_info_Wr++]=Recode_Obj.CMD; //命令字
			  
										 SregLen=90;		 // 信息长度
										 Original_info[Original_info_Wr++]=(u8)(SregLen>>8);	 // Hi
										 Original_info[Original_info_Wr++]=(u8)SregLen;		 // Lo	  30x6	 
										 
										 Original_info[Original_info_Wr++]=0x00;	 // 保留字		 
										 //------------ 
										 memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Driver_Info.DriverCard_ID,18); //机动车驾驶证号
							                      Original_info_Wr+=18;  
										 QueryRecNum=Api_DFdirectory_Query(tired_warn,0);   //查询当前疲劳驾驶记录数目
										 if(QueryRecNum>6)								   
										        QueryRecNum=6;		
										 
										    SregLen=QueryRecNum*12+18;     // 改写信息长度
			                                                      Original_info[Original_info_Wr-3]=(u8)(SregLen>>8);	  // Hi
			                                                      Original_info[Original_info_Wr-2]=(u8)SregLen;	  // Lo    65x7    

										 
										 for(i=0;i<QueryRecNum;i++)			   // 从最新处读取存储填写
										  {
											 Api_DFdirectory_Read(tired_warn,Reg,31,0,i); // 从new-->old  读取
											  memcpy(Original_info+Original_info_Wr,Reg+18,12);	//只要起始时间
											  Original_info_Wr+=12;	     
										  }
	
							   break; 
		  //------------------------ 下传数据相关命令 ---------------------
		  case	A_Dn_DrvInfo  : 	  //  设置 驾驶员信息
		  case  A_Dn_VehicleInfo:     //  设置车辆信息							 
		  case	A_Dn_RTC	  : 	   //  设置记录仪时间
		  case	A_Dn_Plus	  : 	   //  设置速度脉冲系数
							   
						  //   JT808Conf_struct.Vech_Character_Value=((u32)(*InStr)<<24)+((u32)(*InStr+1)<<16)+((u32)(*InStr+2)<<8)+(u32)(*InStr+3); // 特征系数   速度脉冲系数
						  //   Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
							   //-------------------------------------------------------------	
							   Original_info[Original_info_Wr++]=Recode_Obj.CMD; //命令字
												  // 信息长度
							   Original_info[Original_info_Wr++]=0x00;    // Hi
							   Original_info[Original_info_Wr++]=0;	  // Lo   20x8							   
							   Original_info[Original_info_Wr++]=0x00;    // 保留字   
							   break;
		  default :
							 //  rt_kprintf("Error:	Device Type Error! \r\n");
							   return  false; 
							   
	  }
	//---------------  填写计算 A 协议	Serial Data   校验位  -------------------------------------
	Sfcs=0; 					  //  计算S校验 从Ox55 开始 
	for(i=Swr;i<Original_info_Wr;i++)
	   Sfcs^=Original_info[i];
	Original_info[Original_info_Wr++]=Sfcs;		   // 填写FCS  


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
	 //  应答流水号
	Original_info[Original_info_Wr++]=DataTrans.TYPE;// 返回透传数据的类型
	memcpy(Original_info+Original_info_Wr,DataTrans.Data_Tx,DataTrans.Data_TxLen);
	Original_info_Wr+=DataTrans.Data_TxLen;
	 //  3. Send 
	Protocol_End(Packet_Normal ,0);
    if(DispContent)
	     rt_kprintf("\r\n	发送透传  \r\n");    
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
		   case 0 : // 图像
				   MediaObj.Media_totalPacketNum=Photo_sdState.Total_packetNum;  // 图片总包数
				   MediaObj.Media_currentPacketNum=Photo_sdState.SD_packetNum;	// 图片当前报数
				   MediaObj.Media_ID=1;   //  多媒体ID
				   MediaObj.Media_Channel=Camera_Number;  // 图片摄像头通道号
				   break;
		   case 1 : // 音频
		          
				  MediaObj.Media_totalPacketNum=Sound_sdState.Total_packetNum;	// 图片总包数
				  MediaObj.Media_currentPacketNum=Sound_sdState.SD_packetNum;  // 图片当前报数
				  MediaObj.Media_ID=1;	 //  多媒体ID
				  MediaObj.Media_Channel=1;  // 图片摄像头通道号
		         // rt_kprintf(" \r\n 申请上传音频信息 \r\n");  
				   break;
		   case 2 : // 视频
		          MediaObj.Media_totalPacketNum=Video_sdState.Total_packetNum;	// 图片总包数
				  MediaObj.Media_currentPacketNum=Video_sdState.SD_packetNum;  // 图片当前报数
				  MediaObj.Media_ID=1;	 //  多媒体ID
				  MediaObj.Media_Channel=1;  // 图片摄像头通道号
		         // rt_kprintf(" \r\n 申请上传视频信息 \r\n");  
		 
		 
				   break;
		   default:
				   return false;
		  }
	 
	   //  MediaID
	   Original_info[Original_info_Wr++]=(MediaObj.Media_ID>>24);// 返回事件ID
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
	 rt_kprintf("\r\n	发送多媒体事件信息上传  \r\n");    
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
	   Original_info[Original_info_Wr++]=(MediaObj.Media_ID>>24);//  多媒体ID
	   Original_info[Original_info_Wr++]=(MediaObj.Media_ID>>16);
       Original_info[Original_info_Wr++]=(MediaObj.Media_ID>>8);    	
	   Original_info[Original_info_Wr++]=MediaObj.Media_ID;
	   //  Type	   
	   Original_info[Original_info_Wr++]=MediaObj.Media_Type;    // 多媒体类型
	   //  MediaCode Type 
	    Original_info[Original_info_Wr++]=MediaObj.Media_CodeType; // 多媒体编码格式
	    Original_info[Original_info_Wr++]=MediaObj.Event_Code;     // 多媒体事件编码
	    Original_info[Original_info_Wr++]=MediaObj.Media_Channel;  // 通道ID  

	    //  Position Inifo	    
		//  告警标志  4
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Warn_Status,4 );    
		Original_info_Wr += 4;
		// . 状态  4
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )Car_Status,4 );   
		Original_info_Wr += 4;
		//   纬度
	    memcpy( ( char * ) Original_info+ Original_info_Wr,( char * )  Gps_Gprs.Latitude, 4 );//纬度   modify by nathan
		Original_info_Wr += 4;
		//   经度
		memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )  Gps_Gprs.Longitude, 4 );	  //经度    东经  Bit 7->0   西经 Bit 7 -> 1
		Original_info_Wr += 4;
		//   高程
		Original_info[Original_info_Wr++]=(u8)(GPS_Hight<<8);
		Original_info[Original_info_Wr++]=(u8)GPS_Hight;
		//   速度    0.1 Km/h
		Original_info[Original_info_Wr++]=(u8)(Speed_gps>>8);//(GPS_speed>>8); 
		Original_info[Original_info_Wr++]=(u8)(Speed_gps);//GPS_speed;     
		//   方向   单位 1度
		Original_info[Original_info_Wr++]=(GPS_direction>>8);  //High 
		Original_info[Original_info_Wr++]=GPS_direction; // Low
		//   日期时间	
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
          case 0 : // 图像
                   
				   if(((Photo_sdState.photo_sending)==enable)&&((Photo_sdState.SD_flag)==enable))
				   {
					   Photo_sdState.SD_flag=disable;// clear    
				   }
				   else
				   	 return false; 
				   //  ---------------  填写内容  ---------------
				   //			read		Photo_sdState.SD_packetNum从1开始计数
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
 
				   	
      
				      inadd=(Photo_sdState.SD_packetNum-1)<<9; //乘以512
				   	  if(PicFileSize>inadd)
				   	  	{
                            if((PicFileSize-inadd)>512)
                                 readsize=512;
							else
							 {	 
							    readsize=PicFileSize-inadd; // 最后一包  
							    rt_kprintf("\r\n   最后一包 readsize =%d \r\n",readsize);
							 }
				   	  	}
					  else
					  	 return false;
                  	}
				  else
				   if(TF_Card_Status()==1)  
				   	{  ;
				   	 /* inadd=(Photo_sdState.SD_packetNum-1)<<9; //乘以512
				   	  if(PicFileSize>inadd)
				   	  	{
                            if((PicFileSize-inadd)>512)
                                 readsize=512;
							else
							 {	 
							    readsize=PicFileSize-inadd; // 最后一包  
							    rt_kprintf("\r\n   最后一包 readsize =%d \r\n",readsize);
							 }
				   	  	}
					  else
					  	 return false;
                      i=read_file(PictureName,inadd,readsize,Original_info + Original_info_Wr); 
					  if(i==false)
					  	{
                          rt_kprintf("\r\n 图片文件: %s   读取失败\r\n",PictureName); 
                          return false;
					  	} */
				   	} 
			 Original_info_Wr+=readsize;		 //         
		          break;
		  case 1 : // 音频
			  if(((Sound_sdState.photo_sending)==enable)&&((Sound_sdState.SD_flag)==enable))
			   {
				   Sound_sdState.SD_flag=disable;// clear	   
			   }
			   else
				 return false; 
                       //------------------------------------------------------------------------
			   //  ---------------  填写内容  ---------------
			   //			read		Photo_sdState.SD_packetNum从1开始计数
			   //			content_startoffset 	picpage_offset				 contentpage_offset
	                  if(TF_Card_Status()==0) 
	                  {
				       Api_DFdirectory_Read(voice, Original_info + Original_info_Wr, 512,1,MediaObj.Media_currentPacketNum);  
					inadd=(Sound_sdState.SD_packetNum-1)<<9; //乘以512
					if(SrcFileSize>inadd)
					 {
			                           if((SrcFileSize-inadd)>512)
			                                 readsize=512;
							else
							 {	 
							     readsize=SrcFileSize-inadd; // 最后一包  
							     rt_kprintf("\r\n   最后一包 readsize =%d \r\n",readsize);  
							 }
					  }
					  else
						  return false;    
	                  	}
				 rt_kprintf("\r\n Sound_sdState.SD_packetNum= %d   filesize=%d  readsize=%d  \r\n",Sound_sdState.SD_packetNum,SrcFileSize,SrcFileSize-inadd);  
                              Original_info_Wr+=readsize;	  


			 //-------------------------------------------------------------------------		   
			    /*
			   //  ---------------	填写内容  ---------------		
			   if(TF_Card_Status()==1)
				{
				  if(mp3_sendstate==0)
				  {
						  if(Sound_sdState.SD_packetNum==1)
						  	{  // wav tou
						  	  
							  inadd=WaveFile_EncodeHeader(SrcFileSize ,Original_info + Original_info_Wr);
							  Original_info_Wr+=inadd;
							  rt_kprintf("\r\n 写入文件头大小为 wav fileheadersize=%d  \r\n",inadd);    

						  	}				
						  //---------------------------------------------------------
						  soundpage=(Sound_sdState.SD_packetNum-1)/5;// 得到page 
						  sounddelta=((Sound_sdState.SD_packetNum-1)%5)*SpxGet_Size; // 得到页内偏移
		                   rt_kprintf("\r\n inadd=%d  soundpage =%d  inpageoffset=%d \r\n",inadd,soundpage,sounddelta);
						 //  i=read_file(SpxSrcName,(soundpage<<9),512,SpxBuf); 
						 //  if(i==false)
							// {
							  // rt_kprintf("\r\n spx文件: %s   读取失败--2\r\n",SpxSrcName);  
							   //return false;
							 //} 
						 Api_Config_read(voice, Sound_sdState.SD_packetNum, SpxBuf,500);   
						 memcpy(instr,SpxBuf+sounddelta,SpxGet_Size);   		
		                 //---------  spx Decode  5  包 ---------                 
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
						inadd=(Sound_sdState.SD_packetNum-1)<<9; //乘以512
						if(mp3_fsize>inadd)
						  {
							  if((mp3_fsize-inadd)>512)
								   readsize=512;
							  else
							   {   
								  readsize=mp3_fsize-inadd; // 最后一包	
								  rt_kprintf("\r\n	 最后一包 mp3size =%d \r\n",readsize);
							   }
						  }
						else
						   return false;
						//rt_kprintf("\r\n 读取文件\r\n");
						i=read_file(SpxSrcName,inadd,readsize,Original_info + Original_info_Wr); 
						//rt_kprintf("\r\n 读取文件完毕\r\n");
						if(i==false)
						  {
							rt_kprintf("\r\n mp3文件: %s	读取失败\r\n",SpxSrcName); 
							return false;
						  } 					 
					  Original_info_Wr+=readsize;//	
                      

				 	}
			}
              else
			  	  return false; 
       */
		          break;
		  case 2 : // 视频
                  if(TF_Card_Status()==1)
                  	{  ;
					 /*	inadd=(Video_sdState.SD_packetNum-1)<<9; //乘以512
						if(wmv_fsize>inadd)
						  {
							  if((wmv_fsize-inadd)>512)
								   readsize=512;
							  else
							   {   
								  readsize=wmv_fsize-inadd; // 最后一包	
								  rt_kprintf("\r\n	 最后一包 wmvsize =%d \r\n",readsize);
							   }
						  }
						else
						   return false;
						i=read_file(SpxSrcName,inadd,readsize,Original_info + Original_info_Wr); 
						if(i==false)
						  {
							rt_kprintf("\r\n mp3文件: %s	读取失败\r\n",SpxSrcName); 
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
			rt_kprintf("\r\n Media 最后一个block\r\n");
			
			if(0==MediaObj.RSD_State)	// 如果在顺序传输模式下，则改为停止状态,等待中心下重传
			 {
			    MediaObj.RSD_State=2;	
			    MediaObj.RSD_Timer=0;			   
			 } 
			
		 }							 

	  }  
     //----------  累加发送报数 --------------------
     if(0==MediaObj.RSD_State)
     {
		 if(MediaObj.Media_currentPacketNum<MediaObj.Media_totalPacketNum)
		  {
		    //  图片
		    if(Photo_sdState.photo_sending==enable)
		       Photo_sdState.SD_packetNum++;
			//  音频 
			if(Sound_sdState.photo_sending==enable)
			Sound_sdState.SD_packetNum++;
			//视频
			if(Video_sdState.photo_sending==enable)
			 Video_sdState.SD_packetNum++; 
		  }
     }
	 else
	 if(1==MediaObj.RSD_State)
	 {
	   MediaObj.RSD_Reader++;
	   if(MediaObj.RSD_Reader==MediaObj.RSD_total)
	   	    MediaObj.RSD_State=2; //  置位等待状态，等待着中心再发重传指令
	 }	
	 //----------  返回  -------------------
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
        //   float ID  应答流水号 
	    Original_info[Original_info_Wr++]=(u8)(Centre_FloatID>>8);
	    Original_info[Original_info_Wr++]=(u8)Centre_FloatID;
          
		//------- 多媒体总项数 ----
		lenregwr=Original_info_Wr;		
	    Original_info[Original_info_Wr++]=(u8)(totalNum>>8); // 临时占上位置
	    Original_info[Original_info_Wr++]=(u8)totalNum;

		//----- 查找有效效位置 ----
		totalNum=0;
		  for(i=0;i<8;i++)
		  {
			  if(SD_ACKflag.f_MediaIndexACK_0802H==1)  // 图像
			  {		
				 Api_RecordNum_Read(pic_index,i,(u8*)&MediaIndex, sizeof(MediaIndex));   
			  }
			  else
			  if(SD_ACKflag.f_MediaIndexACK_0802H==2) // 音频
			  {
			       Api_RecordNum_Read(voice_index,i,(u8*)&MediaIndex, sizeof(MediaIndex));   
			  }  
			 // rt_kprintf("\r\n Effective_Flag %d  f_QueryEventCode %d  EventCode %d  \r\n",MediaIndex.Effective_Flag,SD_ACKflag.f_QueryEventCode,MediaIndex.EventCode);
			  if((MediaIndex.Effective_Flag==1)&&(SD_ACKflag.f_QueryEventCode==MediaIndex.EventCode))
			  { //  查找有效的索引和相对应类型的索引
			    Original_info[Original_info_Wr++]=(u8)(MediaIndex.MediaID>>24); //  多媒体ID dworrd
			    Original_info[Original_info_Wr++]=(u8)(MediaIndex.MediaID>>16);
				Original_info[Original_info_Wr++]=(u8)(MediaIndex.MediaID>>8);
				Original_info[Original_info_Wr++]=(u8)(MediaIndex.MediaID);
			    Original_info[Original_info_Wr++]=MediaIndex.Type; //  多媒体类型
				Original_info[Original_info_Wr++]=MediaIndex.ID;   //  通道
			    Original_info[Original_info_Wr++]=MediaIndex.EventCode;
				memcpy(Original_info+Original_info_Wr,MediaIndex.PosInfo,28);
			    Original_info_Wr+=28;    
				totalNum++;    
			  }
			  
		  }

          //---------   补上总项数  -------- 
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
        //   驾驶员姓名长度
        i=strlen((const char*) JT808Conf_struct.Driver_Info.DriveName);
	    Original_info[Original_info_Wr++]=i;
	    memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Driver_Info.DriveName,i);// name
		Original_info_Wr+=i;
        memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Driver_Info.Driver_ID,20);// 身份证号码
        Original_info_Wr+=20;
		memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Driver_Info.Drv_CareerID,40);//从业资格证
        Original_info_Wr+=40;
		i=strlen((const char*)JT808Conf_struct.Driver_Info.Comfirm_agentID); // 机构名称
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
        //   信息长度
	     listlen=207;
		 Original_info[Original_info_Wr++]=(listlen>>24);// 返回事件ID
		 Original_info[Original_info_Wr++]=(listlen>>16);
		 Original_info[Original_info_Wr++]=(listlen>>8);		  
		 Original_info[Original_info_Wr++]=listlen;
	    
        memcpy(Original_info+Original_info_Wr,"托运单位:天津七一二通信广播有限公司 电话:022-26237216  ",55);
        Original_info_Wr+=55;		
        memcpy(Original_info+Original_info_Wr,"承运单位:天津物流运输公司 电话:022-86692666  ",45);
        Original_info_Wr+=45;
		memcpy(Original_info+Original_info_Wr,"物品名称:GPS车载终端  包装方式:  箱式   每箱数量: 20   总量: 30箱  ",67);
        Original_info_Wr+=67;
		memcpy(Original_info+Original_info_Wr,"车型:箱式小货车 运达日期 :  2012-1-11   ",40); 
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
	//	应答流水号
   Original_info[Original_info_Wr++]=1;// 返回透传数据的类型 1	表示远程下载
	// 3. 内容
		//	3.1. Sub Head  
		   Rec_Num=Original_info_Wr; //reglen 
			memcpy(Original_info+Original_info_Wr,"*GB",3); // GPRS 起始头
		   Original_info_Wr+=3;
		   
		   Original_info[Original_info_Wr++]=0x00;	// SIM 号码   最先两个字节填写0x00
		   Original_info[Original_info_Wr++]=0x00;
		   memcpy(Original_info+Original_info_Wr,SIM_code,6);
		   Original_info_Wr+=6;  
  
		   Original_info_Wr+=2;   // 长度增加两个字节，但内容随后填写
		   
		   Original_info[Original_info_Wr++]=0x00; //消息序号 默认 0x00
		 // 3.2 Sub Info			 
			 Original_info[Original_info_Wr-1]=0x01; //消息序号 默认 0x00
			 Original_info[Original_info_Wr++]=0x30; //参数 	必须应答
			 
			 Original_info[Original_info_Wr++]=AckType; // 命令字	

			 switch(AckType)
			 	{
                  case  ISPACK_92:
				  case  ISPoverACK_96:    // 内容为空
				  	               break;
                  case  ISPinfoACK_94:                        
								Original_info[Original_info_Wr++]=(u8)(ISP_current_packnum>>8); // 当前包数
								Original_info[Original_info_Wr++]=(u8)ISP_current_packnum;
								Original_info[Original_info_Wr++]=(u8)(ISP_total_packnum>>8); // 总包数
								Original_info[Original_info_Wr++]=(u8)ISP_total_packnum;
								Original_info[Original_info_Wr++]=ISP_ack_resualt;	// 返回的状态	 
					               break;
                  default:
				  	      return false;
			 	}
  
		 // 3.3 Sub end
			 TX_NUM=Original_info_Wr-Rec_Num; // 子协议的长度
				//----stuff Ginfolen ---- 
			 Original_info[Rec_Num+11]=(u8)((TX_NUM-2)>>8);  // 长度不算头 
			 Original_info[Rec_Num+12]=(u8)(TX_NUM-2); 
			 //rt_kprintf("\r\n   测试	A=%X B=%X	reg=%d Reg=%d  REg=%d \r\n",GPRS_info[11],GPRS_info[12],reg,(u8)(reg<<8),reg&0x00ff);
			
			  Gfcs=0;				  //  计算从电话号码开始到校验前数据的异或和  G 协议校验 
			  for(i=3;i<Original_info_Wr;i++)
				   Gfcs^=Original_info[i]; 
			  Original_info[Original_info_Wr++]=Gfcs;  // 填写G校验位
			
			   
			  Original_info[Original_info_Wr++]=0x0D;  // G 协议尾	
			  Original_info[Original_info_Wr++]=0x0A;						 
	
	//	3. Send 
    Protocol_End(Packet_Normal,1 ); 
   if(DispContent)
   	{
      		 switch(AckType)
			 	{
                  case  ISPACK_92:
				  	              rt_kprintf("\r\n	 发送透传  ISP ID=%2X  ACK \r\n",ISPACK_92);       
				  	               break;
				  case  ISPoverACK_96:    // 内容为空
				                  rt_kprintf("\r\n	 发送透传 ISP ID=%2X  overACK \r\n",ISPoverACK_96); 
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
     //   读取第50 页信息
     	  Device_type=((u32)instr[1]<<24)+((u32)instr[2]<<16)+((u32)instr[3]<<8)+(u32)instr[4];
	  Firmware_ver=((u32)instr[5]<<24)+((u32)instr[6]<<16)+((u32)instr[7]<<8)+(u32)instr[8];
	  rt_kprintf("	\r\n 设备类型: %x  软件版本:%x \r\n",Device_type,Firmware_ver); 
	 
	 if(Device_type!=STM32F407_Recoder_32MbitDF)   
	  { 
		 rt_kprintf( "\r\n 设备类型不匹配不予更新" );
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
		   序号   字节数	名称			  备注
		  1 		  1    更新标志 	 1 表示需要更新   0 表示不需要更新
		  2-5			  4   设备类型				 0x0000 0001  ST712   TWA1
											0x0000 0002   STM32  103  新A1
											0x0000 0003   STM32  101  简易型
											0x0000 0004   STM32  A3  sst25
											0x0000 0005   STM32  行车记录仪 
		  6-9		 4	   软件版本 	 每个设备类型从  0x0000 00001 开始根据版本依次递增
		  10-29 	  20	日期		' mm-dd-yyyy HH:MM:SS'
		  30-31 	  2    总页数		   不包括信息页
		  32-35 	  4    程序入口地址    
		  36-200	   165	  预留	  
		  201-		  n    文件名  
	 
	  */
	  //------------   Type check  ---------------------
	  Device_type=((u32)ISP_buffer[1]<<24)+((u32)ISP_buffer[2]<<16)+((u32)ISP_buffer[3]<<8)+(u32)ISP_buffer[4];
	  Firmware_ver=((u32)ISP_buffer[5]<<24)+((u32)ISP_buffer[6]<<16)+((u32)ISP_buffer[7]<<8)+(u32)ISP_buffer[8];
	  rt_kprintf("	\r\n 设备类型: %x  软件版本:%x \r\n",Device_type,Firmware_ver); 
	 
	 if(Device_type!=STM32F407_Recoder_32MbitDF)   
	  { 
		 rt_kprintf( "\r\n 设备类型不匹配不予更新" );
		 ISP_buffer[0]=0;	 // 不更新下载的程序
		  ISP_Write(ISP_APP_Addr,ISP_buffer,PageSIZE); 
	  }  		   
	 	 
	  rt_kprintf( "\r\n 文件日期: " );					  
	  rt_kprintf("%20s",(const char*)(ISP_buffer+10));	 
	  rt_kprintf( "\r\n");	
	  rt_kprintf( "\r\n 文件名: " );
	  rt_kprintf("%100s",(const char*)(ISP_buffer+201));      
	  rt_kprintf( "\r\n");	
	 if(Device_type==STM32F407_Recoder_32MbitDF)         
	 {
		Systerm_Reset_counter= (Max_SystemCounter-5);	 // 准备重启更新最新程序
		ISP_resetFlag=1;//准备重启 
		rt_kprintf( "\r\n 准备重启更新程序!\r\n" );	 
	 }	 
   
 }



//----------------------------------------------------------------------------------
void Stuff_O200_Info_Only( u8* Instr) 
{
   u8  Infowr=0;
   
	// 1. 告警标志  4
	memcpy( ( char * ) Instr+ Infowr, ( char * )Warn_Status,4 );    
	Infowr += 4;
	// 2. 状态  4
	memcpy( ( char * ) Instr+ Infowr, ( char * )Car_Status,4 );   
	Infowr += 4;
	// 3.  纬度
    memcpy( ( char * ) Instr+ Infowr,( char * )  Gps_Gprs.Latitude, 4 );//纬度   modify by nathan
	Infowr += 4;
	// 4.  经度
	memcpy( ( char * ) Instr+ Infowr, ( char * )  Gps_Gprs.Longitude, 4 );	  //经度    东经  Bit 7->0   西经 Bit 7 -> 1
	Infowr += 4;
	// 5.  高程
	Instr[Infowr++]=(u8)(GPS_Hight<<8);
	Instr[Infowr++]=(u8)GPS_Hight;
	// 6.  速度    0.1 Km/h
	Instr[Infowr++]=(u8)(Speed_gps>>8);
	Instr[Infowr++]=(u8)Speed_gps;   
	// 7. 方向   单位 1度
	Instr[Infowr++]=(GPS_direction>>8);  //High 
	Instr[Infowr++]=GPS_direction; // Low
	// 8.  日期时间	
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
  
  //----- 查找无效位置 ----
  for(i=0;i<8;i++)
  {
	  if(type==0)  // 图像
	  {		
		   Api_RecordNum_Read(pic_index,i,(u8*)&MediaIndex, sizeof(MediaIndex));    
	  }
	  else
	  if(type==1) // 音频
	  {
	         Api_RecordNum_Read(voice_index,1,(u8*)&MediaIndex, sizeof(MediaIndex));    
	  }    
	  if(MediaIndex.Effective_Flag==0)
	  	 break;
  }
   if(i==8)  // 如果都满了则从第一个开始
   	   i=0;
  //----  填写信息 -------------
  memset((u8*)&MediaIndex,0,sizeof(MediaIndex));
  MediaIndex.MediaID= JT808Conf_struct.Msg_Float_ID;   
  MediaIndex.Type=type;
  MediaIndex.ID=ID;
  MediaIndex.Effective_Flag=1;
  MediaIndex.EventCode=Evencode;
  memcpy(MediaIndex.FileName,name,strlen((const char*)name));
  Stuff_O200_Info_Only(MediaIndex.PosInfo);
  
  if(type==0)  // 图像
  {
        Api_RecordNum_Write(pic_index,i,(u8*)&MediaIndex, sizeof(MediaIndex));    
  }
  else
  if(type==1) // 音频
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
  

   rt_kprintf("\r\n    收到中心设置命令 SubID=%X \r\n",SubID);  
	   
  switch(SubID)
   {
     case 0x0001:  // 终端心跳包发送间隔  单位:s
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.Heart_Dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
                                   rt_kprintf("\r\n 心跳包间隔: %d s\r\n",JT808Conf_struct.DURATION.Heart_Dur);
                    break;
	 case 0x0002:  // TCP 消息应答超时时间  单位:s	                
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.TCP_ACK_Dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n TCP消息应答间隔: %d s\r\n",JT808Conf_struct.DURATION.TCP_ACK_Dur); 
	                break;
	 case 0x0003:  //  TCP 消息重传次数	                
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.TCP_ReSD_Num=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n TCP重传次数: %d\r\n",JT808Conf_struct.DURATION.TCP_ReSD_Num); 
	                break;
	 case 0x0004:  // UDP 消息应答超时时间  单位:s	                
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.UDP_ACK_Dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
                    rt_kprintf("\r\n UDP应答超时: %d\r\n",JT808Conf_struct.DURATION.UDP_ACK_Dur);
	                break;
	 case 0x0005:  //  UDP 消息重传次数	                
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.UDP_ReSD_Num=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n UDP重传次数: %d\r\n",JT808Conf_struct.DURATION.UDP_ReSD_Num); 
					break;
	 case 0x0010:  //  主服务器APN 
                                    if(infolen==0)
					  	break;
					  memset(APN_String,0,sizeof(APN_String));					  
					  memcpy(APN_String,(char*)Content,infolen);  
					  memset((u8*)SysConf_struct.APN_str,0,sizeof(APN_String));	
					  memcpy((u8*)SysConf_struct.APN_str,(char*)Content,infolen);  
					 Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));


                                     DataLink_APN_Set(APN_String,1); 
									 
	                break; 
	 case 0x0013:  //  主服务器地址  IP 或域名
                      memset(reg_in,0,sizeof(reg_in)); 
					  memcpy(reg_in,Content,infolen);
					 //----------------------------	
					 
					 i =str2ip((char*)reg_in, RemoteIP_main);
					 if (i <= 3)
					 {
					  rt_kprintf("\r\n  域名: %s \r\n",reg_in); 
					  
					  memset(DomainNameStr,0,sizeof(DomainNameStr));					  
					  memset(SysConf_struct.DNSR,0,sizeof(DomainNameStr));
					  memcpy(DomainNameStr,(char*)Content,infolen);   
					  memcpy(SysConf_struct.DNSR,(char*)Content,infolen);   
					 Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));

                                     //----- 传给 GSM 模块------
                                    DataLink_DNSR_Set(SysConf_struct.DNSR,1); 

									 
					  SD_ACKflag.f_CentreCMDack_0001H=2 ;//DataLink_EndFlag=1; //AT_End(); 先返回结果再挂断  
					  break;
					 }
					 memset(reg_str,0,sizeof(reg_str));
					 IP_Str((char*)reg_str, *( u32 * ) RemoteIP_main); 		  
					 strcat((char*)reg_str, " :");		 
					 sprintf((char*)reg_str+strlen((const char*)reg_str), "%u\r\n", RemotePort_main);  
					 memcpy(SysConf_struct.IP_Main,RemoteIP_main,4);
					 SysConf_struct.Port_main=RemotePort_main;
					 
					 Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
					 rt_kprintf("\r\n 中心设置主服务器 IP \r\n");
					 rt_kprintf("\r\n SOCKET :");  
					 rt_kprintf((char*)reg_str);   
					  //-----------  Below add by Nathan  ----------------------------				  
					  rt_kprintf("\r\n		   备用IP: %d.%d.%d.%d : %d \r\n",RemoteIP_aux[0],RemoteIP_aux[1],RemoteIP_aux[2],RemoteIP_aux[3],RemotePort_main);   
					 
					  //-----------  Below add by Nathan  ----------------------------
					  DataLink_MainSocket_set(RemoteIP_main,RemotePort_main,1);
	                              //-------------------------------------------------------------	
					   
					   SD_ACKflag.f_CentreCMDack_0001H=2 ;//DataLink_EndFlag=1; //AT_End(); 先返回结果再挂断  

	                break;
	 case 0x0014:  // 备份服务器 APN

	                break;
	 case 0x0017:  // 备份服务器  IP                    
				  memset(reg_in,0,sizeof(reg_in)); 
				  memcpy(reg_in,Content,infolen);
				 //---------------------------- 
				  i =str2ip((char*)reg_in, RemoteIP_aux);
				  if (i <= 3)
				 {
					  rt_kprintf("\r\n  域名aux: %s \r\n",reg_in); 					  
					 memset(DomainNameStr_aux,0,sizeof(DomainNameStr_aux));					  
					 memset(SysConf_struct.DNSR_Aux,0,sizeof(DomainNameStr_aux));     
					  memcpy(DomainNameStr_aux,(char*)Content,infolen);   
					  memcpy(SysConf_struct.DNSR_Aux,(char*)Content,infolen);    
					 Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
                                     //----- 传给 GSM 模块------
                                    DataLink_DNSR2_Set(SysConf_struct.DNSR_Aux,1);
									 
					  SD_ACKflag.f_CentreCMDack_0001H=2 ;//DataLink_EndFlag=1; //AT_End(); 先返回结果再挂断  
					  break;
				 }  				  
				  memset(reg_str,0,sizeof(reg_str));
				  IP_Str((char*)reg_str, *( u32 * ) RemoteIP_aux);		   
				  strcat((char*)reg_str, " :"); 	  
				  sprintf((char*)reg_str+strlen((const char*)reg_str), "%u\r\n", RemotePort_aux);  
				 
				  Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
				  rt_kprintf("\r\n 中心设置备用服务器 IP \r\n");
				  rt_kprintf("\r\nUDP SOCKET :");  
				  rt_kprintf((char*)reg_str);  
				  DataLink_AuxSocket_set(RemoteIP_aux,RemotePort_aux,1);
				   //-----------  Below add by Nathan  ----------------------------				   
				   rt_kprintf("\r\n 		备用IP: %d.%d.%d.%d : %d \r\n",RemoteIP_aux[0],RemoteIP_aux[1],RemoteIP_aux[2],RemoteIP_aux[3],RemotePort_aux);	
	                break;
	 case 0x0018:  //  服务器 TCP 端口
                    //----------------------------	
                      if(infolen!=4)
						break;	
					  RemotePort_main=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];

					  Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
					  rt_kprintf("\r\n 中心设置主服务器 PORT \r\n");
					  rt_kprintf("\r\nUDP SOCKET :");  
					  rt_kprintf((char*)reg_str);  
					   //-----------  Below add by Nathan  ----------------------------
                                     DataLink_MainSocket_set(RemoteIP_main,RemotePort_main,1);					   //-------------------------------------------------------------		   
					   SD_ACKflag.f_CentreCMDack_0001H=2 ;//DataLink_EndFlag=1; //AT_End(); 先返回结果再挂断	
	                break;
	 case 0x0019:  //  服务器 UDP 端口
                    
					if(infolen!=4)
					  break;  
					RemotePort_aux=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
					
					 Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
					rt_kprintf("\r\n 中心设置UDP服务器 PORT \r\n");
					rt_kprintf("\r\nUDP SOCKET :");   
					rt_kprintf((char*)reg_str);    
					rt_kprintf("\r\n		 备用IP: %d.%d.%d.%d : %d \r\n",RemoteIP_aux[0],RemoteIP_aux[1],RemoteIP_aux[2],RemoteIP_aux[3],RemotePort_aux);	 
	                break;
     case 0x0020:  //  汇报策略  0 定时汇报  1 定距汇报 2 定时和定距汇报
                    if(infolen!=4)
						break;	
                    resualtu32=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
					switch(resualtu32)
						{
						   case 0:rt_kprintf("\r\n 定时汇报 \r\n");
						   	      break;
						   case 1:rt_kprintf("\r\n 定距汇报 \r\n");
						   	      break;
						   case 2:rt_kprintf("\r\n 定时和定距汇报\r\n"); 
						   	      break;
						   default:
						   	      break;

						}
	                break;
	 case 0x0021:  //  位置汇报方案  0 根据ACC上报  1 根据ACC和登录状态上报 

	                break;
       //--------
					
	 case 0x0022:  //  驾驶员未登录 汇报时间间隔 单位:s    >0	                  
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.NoDrvLogin_Dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
                                   rt_kprintf("\r\n 驾驶员未登录汇报间隔: %d\r\n",JT808Conf_struct.DURATION.NoDrvLogin_Dur);  
	                break;
	 case 0x0027:   //  休眠时汇报时间间隔，单位 s  >0	                
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.Sleep_Dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
                    rt_kprintf("\r\n 休眠汇报时间间隔: %d \r\n",JT808Conf_struct.DURATION.Sleep_Dur);   
	                break;
	 case 0x0028:   //  紧急报警时汇报时间间隔  单位 s	                
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.Emegence_Dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                   Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct)); 
                    rt_kprintf("\r\n 紧急报警时间间隔: %d \r\n",JT808Conf_struct.DURATION.Emegence_Dur);   
	                break;
	 case 0x0029:   //  缺省时间汇报间隔  单位 s
	                if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.Default_Dur=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
					rt_kprintf("\r\n 缺省汇报时间间隔: %d \r\n",JT808Conf_struct.DURATION.Default_Dur);   
	                break;
       //---------
					
	 case 0x002C:   //  缺省距离汇报间隔  单位 米
	                if(infolen!=4)
						break;					
					JT808Conf_struct.DISTANCE.Defalut_DistDelta=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n 缺省距离汇报间隔: %d m\r\n",JT808Conf_struct.DISTANCE.Defalut_DistDelta); 
	                break;
	 case 0x002D:   //  驾驶员未登录汇报距离间隔 单位 米
	               if(infolen!=4)
						break;					
					JT808Conf_struct.DISTANCE.NoDrvLogin_Dist=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct)); 
					rt_kprintf("\r\n 驾驶员未登录汇报距离: %d m\r\n",JT808Conf_struct.DISTANCE.NoDrvLogin_Dist); 
	                break;
	 case 0x002E:   //  休眠时汇报距离间隔  单位 米
	               if(infolen!=4)
						break;					
					JT808Conf_struct.DISTANCE.Sleep_Dist=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n 休眠时定距上报间隔: %d m\r\n",JT808Conf_struct.DISTANCE.Sleep_Dist); 
	                break;
	 case 0x002F:   //  紧急报警时汇报距离间隔  单位 米
	               if(infolen!=4)
						break;					
					JT808Conf_struct.DISTANCE.Emergen_Dist=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n 紧急报警时定距上报间隔: %d m\r\n",JT808Conf_struct.DISTANCE.Emergen_Dist); 
	                break;
	 case 0x0030:   //  拐点补传角度 , <180
                    if(infolen!=4)
						break;					
					JT808Conf_struct.DURATION.SD_Delta_maxAngle=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n 拐点补传角度: %d 度\r\n",JT808Conf_struct.DISTANCE.Emergen_Dist);  
	                break;
	 case 0x0031: 		 	            
                    if(infolen!=2)
						break;					
					JT808Conf_struct.DURATION.IllgleMovo_disttance=(Content[0]<<8)+Content[1]; 
                    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n 电子围栏半径(非法移动阈值): %d m\r\n",JT808Conf_struct.DURATION.IllgleMovo_disttance);  
        //---------
	 case 0x0040:   //   监控平台电话号码  
	                 if(infolen==0)
					 	 break;
					i=strlen((const char*)JT808Conf_struct.LISTEN_Num);
					rt_kprintf("\r\n old: %s \r\n",JT808Conf_struct.LISTEN_Num);
					 
					memset(JT808Conf_struct.LISTEN_Num,0,sizeof(JT808Conf_struct.LISTEN_Num));
					memcpy(JT808Conf_struct.LISTEN_Num,Content,infolen);											
					Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));  
 					rt_kprintf("\r\n new: %s \r\n",JT808Conf_struct.LISTEN_Num);
                     
	                //CallState=CallState_rdytoDialLis;  // 准备开始拨打监听号码 
	                rt_kprintf("\r\n 设置监控平台号码: %s \r\n",JT808Conf_struct.LISTEN_Num);   

	                break;
	 case 0x0041:   //   复位电话号码，可采用此电话号码拨打终端电话让终端复位
                    if(infolen==0)
					 	 break;
					memset(reg_str,0,sizeof(reg_str)); 
					memcpy(reg_str,Content,infolen);											
 					rt_kprintf("\r\n 复位电话号码 %s \r\n",reg_str);  
	                break;
	 case 0x0042:   //   恢复出厂设置电话，可采用该电话号码是终端恢复出厂设置

	                break;
	 case 0x0045:   //  终端电话接听策略 0 自动接听  1 ACC ON自动接听 OFF时手动接听

	                break;
	 case 0x0046:   //  每次通话最长时间 ，单位  秒
                    
	                break;
	 case 0x0047:   //  当月最长通话时间，单位  秒

	                break;
	 case 0x0048:   //  监听电话号码                    
					if(infolen==0)
						 break;
					memset(JT808Conf_struct.LISTEN_Num,0,sizeof(JT808Conf_struct.LISTEN_Num));
					memcpy(JT808Conf_struct.LISTEN_Num,Content,infolen); 										
					Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));  
					CallState=CallState_rdytoDialLis;  // 准备开始拨打监听号码 
					rt_kprintf("\r\n 立即拨打监听号码: %s \r\n",JT808Conf_struct.LISTEN_Num);  
	                break;  

	    //----------				
	 case 0x0050:  //  报警屏蔽字， 与位置信息中报警标志相对应。相应位为1时报警被屏蔽---
                    
                    if(infolen!=4)
						break;	
                    resualtu32=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    rt_kprintf("\r\n 报警屏蔽字: %x \r\n",resualtu32);    
					break;
	 case 0x0052:  //  报警拍照开关， 与报警标志对应的位1时，拍照

	                break;
	 case 0x0053:  //  报警拍照存储  	与报警标志对应的位1时，拍照存储 否则实时上传

	                break;
	 case 0x0054:  //  关键标志  		与报警标志对应的位1  为关键报警

	                break;
        //---------  
					
	 case 0x0055:  //  最高速度   单位   千米每小时
                   if(infolen!=4)
						break;
				    JT808Conf_struct.Speed_warn_MAX=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)	+Content[3];
				   memset(reg_str,0,sizeof(reg_str));
				   memcpy(reg_str,& JT808Conf_struct.Speed_warn_MAX,4);
				   memcpy(reg_str+4,&JT808Conf_struct.Spd_Exd_LimitSeconds,4);
				    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
				   rt_kprintf("\r\n 最高速度: %d km/h \r\n", JT808Conf_struct.Speed_warn_MAX); 
				   Spd_ExpInit();
	                break;
	 case 0x0056:  //  超速持续时间    单位 s
                   if(infolen!=4)
						break;
				   JT808Conf_struct.Spd_Exd_LimitSeconds=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)	+Content[3];
				   memset(reg_str,0,sizeof(reg_str));
				   memcpy(reg_str,& JT808Conf_struct.Speed_warn_MAX,4);
				   memcpy(reg_str+4,&JT808Conf_struct.Spd_Exd_LimitSeconds,4); 
				    Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
				   rt_kprintf("\r\n 超时持续时间: %d s \r\n",JT808Conf_struct.Spd_Exd_LimitSeconds); 
				   Spd_ExpInit();
	                break;
	 case 0x0057:  //  连续驾驶时间门限 单位  s
					 if(infolen!=4)
						  break;
				   TiredConf_struct.TiredDoor.Door_DrvKeepingSec=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)   +Content[3];
                               Api_Config_write(tired_config,0,(u8*)&TiredConf_struct,sizeof(TiredConf_struct));				   rt_kprintf("\r\n 连续驾驶时间门限: %d s \r\n",TiredConf_struct.TiredDoor.Door_DrvKeepingSec);  
				   Warn_Status[3]&=~0x04;  //BIT(2)	接触疲劳驾驶报警 
				   TIRED_Drive_Init();    // 清除疲劳驾驶状态      
	               break;
	 case 0x0058:  //  当天累计驾驶时间门限  单位  s
				   if(infolen!=4)
						break;
				   TiredConf_struct.TiredDoor.Door_DayAccumlateDrvSec=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)   +Content[3];
                               Api_Config_write(tired_config,0,(u8*)&TiredConf_struct,sizeof(TiredConf_struct));				   rt_kprintf("\r\n 连续驾驶时间门限: %d s \r\n",TiredConf_struct.TiredDoor.Door_DrvKeepingSec);  
				   TiredConf_struct.Tired_drive.ACC_ONstate_counter=0;
				   TiredConf_struct.Tired_drive.ACC_Offstate_counter=0;
                                Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct)); 
                               rt_kprintf("\r\n 当天累计驾驶时间: %d s \r\n",TiredConf_struct.TiredDoor.Door_DayAccumlateDrvSec); 
	                break;
     case 0x0059:  //  最小休息时间  单位 s
				   if(infolen!=4)
						break;
				    TiredConf_struct.TiredDoor.Door_MinSleepSec=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)   +Content[3];
                               Api_Config_write(tired_config,0,(u8*)&TiredConf_struct,sizeof(TiredConf_struct));				   rt_kprintf("\r\n 连续驾驶时间门限: %d s \r\n",TiredConf_struct.TiredDoor.Door_DrvKeepingSec);  
					rt_kprintf("\r\n 最小休息时间: %d s \r\n",TiredConf_struct.TiredDoor.Door_MinSleepSec);  
	                break;
	 case 0x005A:  //  最长停车时间   单位 s
					 if(infolen!=4)
						  break;
					TiredConf_struct.TiredDoor.Door_MaxParkingSec=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8)	 +Content[3];
                                   Api_Config_write(tired_config,0,(u8*)&TiredConf_struct,sizeof(TiredConf_struct));				   rt_kprintf("\r\n 连续驾驶时间门限: %d s \r\n",TiredConf_struct.TiredDoor.Door_DrvKeepingSec);  
					TiredConf_struct.TiredDoor.Parking_currentcnt=0; 
					 Warn_Status[1]&=~0x08;  // 清除超时触发 
					rt_kprintf("\r\n 最长停车时间: %d s \r\n",TiredConf_struct.TiredDoor.Door_MaxParkingSec);   
	                break;
	     //--------- 
	 case  0x0070: //  图像/视频质量  1-10  1 最好

	                break;
	 case  0x0071: //  亮度  0-255

	                break;
	 case  0x0072: //  对比度  0-127

	                break;
	 case  0x0073: // 饱和度  0-127

	                break;
	 case  0x0074: // 色度   0-255

	                break;
		  //---------
	 case  0x0080: // 车辆里程表读数   1/10 km

	                break;
	 case  0x0081: // 车辆所在的省域ID
	                if(infolen!=2)
						  break;
					JT808Conf_struct.Vechicle_Info.Dev_ProvinceID=(Content[0]<<8)+Content[1];
                                   Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n 车辆所在省域ID: 0x%X \r\n",JT808Conf_struct.Vechicle_Info.Dev_ProvinceID); 
	                break;
	 case  0x0082: // 车辆所在市域ID
	                if(infolen!=2)
						  break;
					JT808Conf_struct.Vechicle_Info.Dev_ProvinceID=(Content[0]<<8)+Content[1];
                                   Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n 车辆所在市域ID: 0x%X \r\n",JT808Conf_struct.Vechicle_Info.Dev_ProvinceID); 
	                break;
	 case  0x0083: // 公安交通管理部门颁发的机动车号牌
	                if(infolen<4)
						  break;					
					memset(JT808Conf_struct.Vechicle_Info.Vech_Num,0,sizeof(JT808Conf_struct.Vechicle_Info.Vech_Num));
					memcpy(JT808Conf_struct.Vechicle_Info.Vech_Num,Content,infolen);
                                   Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n 机动车驾驶证号: %s  \r\n",JT808Conf_struct.Vechicle_Info.Vech_Num);  
	                break;
	 case  0x0084: // 车牌颜色  按照国家规定
	                if(infolen!=1)
						  break;
					JT808Conf_struct.Vechicle_Info.Dev_Color=Content[0];           
                                   Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					rt_kprintf("\r\n 车辆颜色: %d  \r\n",JT808Conf_struct.Vechicle_Info.Dev_Color);                 
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
      case 1:  //  无线升级参数  参数之间采用分号分隔   指令格式如下:
             /*
URL 地址；拨号名称；拨号用户名；拨号密码；地址；TCP端口；UDP端口；制造商ID; 硬件版本；固件版本；连接到指定服务器指定是服务器时限；
           若某个参数无数值，则放空
               */
             rt_kprintf("\r\n 无线升级 \r\n");  
			 rt_kprintf("\r\n 内容: %s\r\n",Content); 
		     break;
	  case 2:  // 控制终端连接指定服务器
	        /*
连接控制；监管平台鉴权码；拨号点名称； 拨号用户名；拨号密码；地址；TCP端口；UDP端口；连接到指定服务器时限
           若每个参数无数值，则放空
	         */  
	         rt_kprintf("\r\n 终端控制连接指定服务器\r\n");  
			 rt_kprintf("\r\n 内容: %s\r\n",Content);
			 break;
	  case 3:  //  终端关机
              SD_ACKflag.f_CentreCMDack_0001H=5; 
			  rt_kprintf("\r\n 终端关机 \r\n");  
	         break;
      case 4:  //  终端复位
               SD_ACKflag.f_CentreCMDack_0001H=3;
			   rt_kprintf("\r\n 终端复位 \r\n");          
	         break;
	  case 5: //   终端恢复出厂设置		 		  	       
	           /* if(SysConf_struct.Version_ID==SYSID)   //  check  wether need  update  or not 
	          	{                    
					SysConf_struct.Version_ID=SYSID+1;   
					Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
					Systerm_Reset_counter=Max_SystemCounter;
					ISP_resetFlag=2;    //   借助远程下载重启机制复位系统 
	          	}
				rt_kprintf("\r\n 恢复出厂设置 \r\n"); */ 
			 break;
	  case 6: //   关闭数据通信
             SD_ACKflag.f_CentreCMDack_0001H=5; 
			 rt_kprintf("\r\n 关闭数据通信 \r\n"); 
	         break;
	  case 7: //   关闭所有无线通信
             SD_ACKflag.f_CentreCMDack_0001H=5; 
			 rt_kprintf("\r\n 关闭所有通信 \r\n");   
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
     case 0x81: //	  中心设置 驾驶员代码  驾驶证号码
				memset(JT808Conf_struct.Driver_Info.DriverCard_ID,0,18);
                //  驾驶员代码没有处理 3个字节 
				memcpy(JT808Conf_struct.Driver_Info.DriveCode,Instr,3); 						   
			       memcpy(JT808Conf_struct.Driver_Info.DriverCard_ID,Instr+3,18); //只要驾驶证号码
				 Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
				break;
   
	 case 0x82: //	  中心设置车牌号  
				memset((u8*)&JT808Conf_struct.Vechicle_Info,0,sizeof(JT808Conf_struct.Vechicle_Info));		
			
                memcpy(JT808Conf_struct.Vechicle_Info.Vech_VIN,Instr,17);
				memcpy(JT808Conf_struct.Vechicle_Info.Vech_Num,Instr+17,12);
				memcpy(JT808Conf_struct.Vechicle_Info.Vech_Type,Instr+29,12); 

				Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));    
				break;
	 case 0xC2: //设置记录仪时钟
			    // 没啥用，给个回复就行，俺有GPS校准就够了 

		        break;

	 case 0xC3: //车辆速度脉冲系数（特征系数）
				 JT808Conf_struct.Vech_Character_Value=(u32)(Instr[0]<<16)+(u32)(Instr[1]<<8)+(u32)Instr[2]; // 特征系数  速度脉冲系数
				
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


   rt_kprintf("\r\n    收到扩展终端设置命令 SubID=%X \r\n",SubID);  
	   
  switch(SubID)
   {
     case 0x0001:  //GNSS 定位模式切换
                    if(infolen!=4)
						break;			 		
		      BD_EXT.BD_Mode=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];

		      switch(BD_EXT.BD_Mode)
		      	{
                         case 0x00:  // 单 GPS 定位模式                        
						   gps_mode("2");
						 break;
			    case  0x01:  // 	单BD2 定位模式
			                        gps_mode("1");
				               break;
			    case  0x02: //  BD2+GPS 定位模式
						   gps_mode("3");
				               break;							   	
		      	}
                    //BD_EXT_Write();   
                    break;
	 case 0x0002:  // GNSS 波特率设置            
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
		      	 //---UART_GPS_Init(baud);  //   修改串口波特率 
		      	 rt_thread_delay(20);
                      // BD_EXT_Write();   
			rt_kprintf("\r\n 中心设置GNSS 波特率:  %d s\r\n",baud); 
	                break;
	 case 0x0003:  //  BNSS NMEA 输出更新率               
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
			      rt_kprintf("\r\n  GNSS 输出更新: %dms\r\n",baud);  
	                break;
	 case 0x0004:  // GNSS 采集NMEA 数据频率               
                    if(infolen!=4)
						break;					
			 BD_EXT.BD_SampleFrea=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                       rt_kprintf("\r\n GNSS 采集频率: %d s\r\n", BD_EXT.BD_SampleFrea);   
	                break;
	 case 0x0005:  //  CAN 1  参数设置               
                    if(infolen!=4)
						break;					
			BD_EXT.CAN_1_Mode=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                     CAN_App_Init();
			//BD_EXT_Write();   
			rt_kprintf("\r\n 中心设置CAN1 : 0x%08X\r\n",BD_EXT.CAN_1_Mode); 
			break;
	 case 0x0006:  //  CAN2   参数设置    ----- 要和小板通信
                    if(infolen!=4)
						break;					
			BD_EXT.CAN_2_Mode=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                    // BD_EXT_Write();  
			memset( u3_Txstr,0,sizeof(u3_Txstr));
			memset( Destu3_Txstr,0,sizeof(Destu3_Txstr));		 
			u3_Txstr[0]=0x7E;       // 头
		       u3_Txstr[1]=0x33;		//  CAN 相关
		       u3_Txstr[2]=0x01;    //  参数设置 
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
			 // U3_PutData(Destu3_Txstr,len+2);  //  发送给串口	
			rt_device_write(&Device_CAN2,0, Destu3_Txstr,len+2);       
			rt_kprintf("\r\n 中心设置CAN2 : 0x%08X\r\n",BD_EXT.CAN_2_Mode);    
	                break; 
        case 0x0007:  //  碰撞参数设置
                    if(infolen!=4)
						break;					
			BD_EXT.Collision_Check=(Content[0]<<24)+(Content[1]<<16)+(Content[2]<<8) +Content[3];
                     //BD_EXT_Write();   
			memset( u3_Txstr,0,sizeof(u3_Txstr));
			memset( Destu3_Txstr,0,sizeof(Destu3_Txstr));		 
			u3_Txstr[0]=0x7E;       // 头
		       u3_Txstr[1]=0x32;		//  CAN 相关 
		       u3_Txstr[2]=0x01;    //  参数设置 
		       u3_Txstr[3]=(BD_EXT.Collision_Check>>24);
			u3_Txstr[4]=(BD_EXT.Collision_Check>>16);   	
	              u3_Txstr[5]=(BD_EXT.Collision_Check>>8);	       
                     u3_Txstr[6]=BD_EXT.Collision_Check;	 
			u3_Txstr[7]=0x7E;	               				 

			Destu3_Txstr[0]=0x7E;
			len=Protocol_808_Encode(Destu3_Txstr+1, u3_Txstr+1, 6);
			Destu3_Txstr[len+1]=0x7E;

			//   设置配置碰撞参数
			mma8451_config((uint16_t) (BD_EXT.Collision_Check>>16),(uint16_t) (BD_EXT.Collision_Check));
			rt_kprintf("\r\n 中心设置碰撞参数: 0x%08X\r\n",BD_EXT.Collision_Check);; 
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


   rt_kprintf("\r\n    收到扩展终端参数设置1 命令 SubID=%X \r\n",SubID);  
	   
  switch(SubID)
   {
     case 0x0001:  // CAN ID 设置
                    if(infolen<4)
						break;			 		
		        // Content[0];  //  设置属性
			  //Content[1] ;//本消息中包含CAN ID 的数量
		     //------------------------------------	  
			//Content[2];  //  CAN ID 的ID 			
		     if(Content[3]&0x80)//  CAN ID   的属性  
		     	{
			      BD_EXT.CAN_2_ID=(Content[4]<<24)+(Content[5]<<16)+(Content[6]<<8)+Content[7];// 
                            if(Content[3]&0x40)
					 BD_EXT.CAN_2_Type=1; // 标准帧
				else 
					 BD_EXT.CAN_2_Type=0;  
			      BD_EXT.CAN_2_Duration=(Content[9]<<8)+Content[10];   // 0   表示停止

				 
				//     得有个和小板通信的处理
				memset( Destu3_Txstr,0,sizeof(Destu3_Txstr));		 
				u3_Txstr[0]=0x7E;       // 头
			       u3_Txstr[1]=0x33;		//  CAN 相关
			       u3_Txstr[2]=0x02;    //  参数设置 
			       u3_Txstr[3]=Content[3]; // 属性
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
				   
			       // U3_PutData(Destu3_Txstr,len+2);  //  发送给串口	
			       rt_device_write(&Device_CAN2,0, Destu3_Txstr,len+2);       
				rt_kprintf("\r\n 中心设置CAN2 : 0x%08X   Dur: %d s\r\n",BD_EXT.CAN_2_ID,BD_EXT.CAN_2_Duration);     
			        //----------------------------------
			 } 
			 else
     			   {
      			       BD_EXT.CAN_1_ID=(Content[4]<<24)+(Content[5]<<16)+(Content[6]<<8)+Content[7];// 
                             if(Content[3]&0x40)   
					 BD_EXT.CAN_1_Type=1; // 标准帧
				else 
					 BD_EXT.CAN_1_Type=0;   
				 BD_EXT.CAN_1_Duration=(Content[9]<<8)+Content[10];  // 0   表示停止  

				rt_kprintf("\r\n 中心设置CAN1 : 0x%08X   Dur: %d s\r\n",BD_EXT.CAN_1_ID,BD_EXT.CAN_1_Duration);    

			   }   
                    BD_EXT_Write();    
                    break;
	 case 0x0002:  // 文本信息标志含义     
	            // Content[0]; // 没用
	              //if((Content[1]==0)||(Content[1]==1))  //  部分类型都给显示屏 
	             	{
				memset( TextInfo.TEXT_Content,0,sizeof(TextInfo.TEXT_Content));
				DwinLCD.TxInfolen=AsciiToGb(TextInfo.TEXT_Content,infolen-2,Content+2);
				//memcpy(TextInfo.TEXT_Content,Content+2,infolen-2);
				TextInfo.TEXT_SD_FLAG=1;	// 置发送给显示屏标志位  // ||||||||||||||||||||||||||||||||||
                            rt_kprintf("\r\n  CAN 文本:  "); 
				for(i=0;i<DwinLCD.TxInfolen;i++) 
							 rt_kprintf("%c",TextInfo.TEXT_Content[i]);
				  rt_kprintf("\r\n ");
			}	

		          #ifdef LCD_5inch
						           //======  信息都在屏幕上显示  
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
  		   case  0x91:     //   中心下发远程下载开始命令
					       ISP_rxCMD=New_CMD; 
						f_ISP_ACK=1;						
							rt_kprintf( "\r\n ISP request! \r\n" ); 					   
						//------------- ISP state ------------------------	
						ISP_running_state=1;
						ISP_running_counter=0;
						ISP_RepeatCounter=0;
					  break;
		   case  0x93:     //   中心下发远程下载数据内容命令
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
		   case  0x95:     //   中心下发远程下载结束命令
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
   if(strncmp((char*)Instr, "*GB",3) != 0)           //   过滤头 
  	   return;
 
  //  2.   get infolen    CMD 
  infolen=( u16 )(Instr[11] << 8 ) + ( u16 ) Instr[12]; 
//  contentlen=infolen-14;  //  8+2+1+1+1+1
  New_CMD = Instr[15];  

  //  3.   Check  Fcs  
	  FCS_RX_UDP_sub=0;
	  for ( i=0; i <(infolen-1); i++ ) //先算出收到数据的异或和  长度包括校验所以信息长度应减1
		  {
			  FCS_RX_UDP_sub^=Instr[3+i];	
		  } 
	  // ------- FCS filter -----------------
	  if( Instr[infolen+2]!= FCS_RX_UDP_sub ) 	   
		  {
			  rt_kprintf("\r\n ISP校验错误	  Caucate %x  ,RX  %x\r\n",FCS_RX_UDP_sub,Instr[infolen+2]); 
				//-----------------  memset  -------------------------------------
			  return; 
		  }
	  else		 
		  rt_kprintf("\r\nISP校验正确	  Caucate %x  ,RX  %x\r\n",FCS_RX_UDP_sub,Instr[infolen+2]);	  
   
 //   4 .  Classify  Process 
   // rt_kprintf("\r\n New_CMD=%X",New_CMD);
     switch(New_CMD)
   {
  		   case  0x91:     //   中心下发远程下载开始命令
					       ISP_rxCMD=New_CMD; 
						f_ISP_ACK=1;						
							rt_kprintf( "\r\n ISP request! \r\n" ); 					   
						//------------- ISP state ------------------------	
						ISP_running_state=1;
						ISP_running_counter=0;
						ISP_RepeatCounter=0;
					  break;
		   case  0x93:     //   中心下发远程下载数据内容命令
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
									//rt_kprintf("\r\n 写入Page=%d\r\n",DF_PageAddr);
						
									//-----------  protect	RAM data  ------
									if((DF_PageAddr>=DF_BL_PageNo)&&(DF_PageAddr<DF_PageAddr))
										break; 
									
									//---------------- write APP data --------- 
									ISP_Format(DF_PageAddr,0,Instr+22,PageSIZE);
					
									//rt_kprintf("\r\n写内容:\r\n ");
									//for(i=0;i<PageSIZE;i++)
										//rt_kprintf("%X ",Instr[22+i]);
								    // rt_kprintf("\r\n读内容:\r\n ");	
								      rt_thread_delay(5); 
								       rt_kprintf("\r\n清除ISP 区域\r\n ");	
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
											   rt_kprintf( "\r\n i=%d  读取和写不匹配! \r\n",i );  
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
										    rt_kprintf("\r\n 超过最大错误5次数，清除远程升级区域!\r\n");
                                                                          DF_ClearUpdate_Area();  
									   	}
									 } 
									else  
										
									 {
									   ISP_ack_resualt=0x01;   // ok		
									   ISP_RepeatCounter=0;
									 } 
								   //---------------------------------
								   rt_thread_delay(10);  // 不加会导致堆栈溢出
								   
								   f_ISP_88_ACK=1;								   
						  }
				             }
						//------------- ISP state ------------------------
					#endif	
				        ISP_running_state=1;
					 ISP_running_counter=0;   			                
					 f_ISP_88_ACK=1;			
					  break;
		   case  0x95:     //   中心下发远程下载结束命令
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
  MediaObj.Media_Type=MdType;//	指定当前传输多媒体的类型   0  表示图片
  MediaObj.Media_CodeType=MdCodeType;  //  多媒体编码格式   0  表示JPEG	格式 
  MediaObj.SD_media_Flag=1;   //  置多媒体事件信息发送标志位  ，开始图片传输  
  MediaObj.SD_Eventstate=1;		  //  开始处于发送状态 
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
	   Photo_sdState.SD_packetNum=1; // 第一包开始  
	   rt_kprintf("\r\n 开始上传照片! ....\r\n");  												
    }
  */
   //---------------------------------------------------- 
}

void Media_Clear_State(void)
{
     // 不清楚Meia Type
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
				  MediaObj.SD_Data_Flag=1; // 置重传发送多媒体信息标志位
				  switch(MediaObj.Media_Type)    //   图片重传包数
				  	{
				  	 case 0://  图像
				  	         Photo_sdState.SD_packetNum=MediaObj.Media_ReSdList[MediaObj.RSD_Reader];
	                         Photo_sdState.SD_flag=1;
							 break;
					 case 1:// 音频                             
				  	         Sound_sdState.SD_packetNum=MediaObj.Media_ReSdList[MediaObj.RSD_Reader];
	                         Sound_sdState.SD_flag=1;  
					         break;
					 case 2:// 视频
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
		  if(MediaObj.RSD_Timer>120)   //   如果状态一直在等待且超过30s择清除状态 
		  { 
		     switch (MediaObj.Media_Type)
	         {
	          case 0 : // 图像
	                  Photo_send_end();  // 拍照上传结束
	                  
			          break;
			  case 1 : // 音频
			         Sound_send_end();  
			          break;
			  case 2 : // 视频
                               Video_send_end();
			          break;
			  default:
			  	      break;
	         } 
            Media_Clear_State();
			rt_kprintf("\r\n 信息重传超时结束! \r\n");
			
			Check_MultiTakeResult_b4Trans();  // 多路摄像头拍照状态检测
			
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
	 Photo_sdState.SD_packetNum=1; // 从1 开始
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
	 //pic_current_page++; //指向文件内容 
	 //pic_PageIn_offset=0;// 图片读取页内偏移地址 
	// rt_kprintf("\r\n    open Pic =%s",PictureName); 
	  
	  if(PicFileSize%512)
	     Photo_sdState.Total_packetNum=PicFileSize/512+1;
	  else
	  	 Photo_sdState.Total_packetNum=PicFileSize/512;

	 rt_kprintf("\r\n    Camera %d  ReadpicStart total :%d ，Pagesize: %d Bytes\r\n\r\n",Camera_Number,Photo_sdState.Total_packetNum,PicFileSize);    
	 if((Camera_Number==0)||(Photo_sdState.Total_packetNum==0))
	 {
	   Photo_send_end(); // clear  state 
       rt_kprintf("\r\n  图片总包数为空 ，摄像头序号为0 ，发送拍照失败到中心 \r\n");    
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
  Sound_sdState.SD_packetNum=1; // 从1 开始
  Sound_sdState.Exeption_timer=0;
   
 //---  Speex_Init();	 // speachX 初始化	 
  // 1. 查找最新的文件
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
	rt_kprintf("\r\n 索引filename:%s\r\n",MediaIndex.FileName);
  }	
  else
  	{
  	   rt_kprintf("\r\n 没有已存储的音频文件 \r\n");
  	   return false	;
  	}

*/
    
	//	2.	创建wav 文件   

	//	3. 文件大小
	  // file name
	memset(sound_name,0,sizeof(sound_name));
	DF_ReadFlash(SoundStart_offdet, 4, sound_name,20);
       SrcFileSize=Api_DFdirectory_Query(voice, 1);
     //  Sound_sdState.Total_packetNum=(SrcFileSize/512); // 每包100个字节
        if(SrcFileSize%512)
	     Sound_sdState.Total_packetNum=SrcFileSize/512+1;
	  else
	  	 Sound_sdState.Total_packetNum=SrcFileSize/512; 
	rt_kprintf("\r\n	文件名: %s大小: %d Bytes  totalpacketnum=%d \r\n",sound_name,SrcFileSize,Sound_sdState.Total_packetNum);
  
    // -------  MultiMedia Related --------
      Media_Start_Init(1,3); // 音频  wav 格式   0:JPEG ;   1: TIF ;   2:MP3;  3:WAV  4: WMV  其他保留   
                                           //    5   amr
     return true;  
}

u8  MP3_send_start(void) 
{
   u8   mp3_name[13];
  mp3_sendstate=1;
  Sound_sdState.photo_sending=disable;  
  Sound_sdState.SD_flag=0;  
  Sound_sdState.SD_packetNum=1; // 从1 开始
    Sound_sdState.Exeption_timer=0;

    memset(mp3_name,0,sizeof(mp3_name));
	memcpy(mp3_name,"ch12.mp3",8);
	
	if(mp3_fsize%512)
	   Sound_sdState.Total_packetNum=mp3_fsize/512+1;
	else
	   Sound_sdState.Total_packetNum=mp3_fsize/512;

	rt_kprintf("\r\n  mp3文件名称:%s    文件大小  %d Bytes  \r\n",mp3_name,mp3_fsize);
  
    // -------  MultiMedia Related --------
    Media_Start_Init(1,2); // 音频  wav 格式       
	return true;
}

u8  Video_send_start(void)
{
   u8 video_name[13];
   	
  wmv_sendstate=1;
  Video_sdState.photo_sending=disable;  
  Video_sdState.SD_flag=0;  
  Video_sdState.SD_packetNum=1; // 从1 开始
  Video_sdState.Exeption_timer=0;

    memset(video_name,0,sizeof(video_name));
	memcpy(video_name,"ch1.wmv",7);
	
    if(wmv_fsize%512)
	   Video_sdState.Total_packetNum=wmv_fsize/512+1;
	else
	   Video_sdState.Total_packetNum=wmv_fsize/512;  

	rt_kprintf("\r\n  wmv文件名称:%s    文件大小  %d Bytes  \r\n",video_name,wmv_fsize);
  
    // -------  MultiMedia Related --------
    Media_Start_Init(2,4); // 音频  wav 格式       
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
  
  if((Video_sdState.photo_sending==enable)&&(2==MediaObj.Media_Type)) // 视频
  {
	 if((Video_sdState.SD_packetNum<=Video_sdState.Total_packetNum+1)&&(2!=MediaObj.RSD_State))  
	  {  //  一下定时器在	在顺序发送过过程中	 和   收到重传开始后有效
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
  
  if((Sound_sdState.photo_sending==enable)&&(1==MediaObj.Media_Type)) // 音频
  {
	 if((Sound_sdState.SD_packetNum<=Sound_sdState.Total_packetNum+1)&&(2!=MediaObj.RSD_State))  
	  {  //  一下定时器在	在顺序发送过过程中	 和   收到重传开始后有效
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
	   	{  //  一下定时器在   在顺序发送过过程中   和   收到重传开始后有效 
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
void TCP_RX_Process( u8  LinkNum)  //  ---- 808  标准协议 
{
  	  u16	i=0;//,DF_PageAddr;
	  u16  infolen=0,contentlen=0;
	 // u8   ireg[5]; 
	  u8   Ack_Resualt=1;
	  u16  Ack_CMDid_8001=0;
	  u8   Total_ParaNum=0;        // 中心设置参数总数 
	  u8   Process_Resualt=0;  //  bit 表示   bit0 表示 1  bit 1 表示2
	  u8   ContentRdAdd=0;   // 当前读取到的地址
	  u8   SubInfolen=0;     // 子信息长度
	  u8   Reg_buf[22];
	  //u8   CheckResualt=0;
      u32  reg_u32=0;
  //----------------      行车记录仪808 协议 接收处理   -------------------------- 

  //  0.  Decode     
  Protocol_808_Decode();
  //  1.  fliter head
  if(UDP_HEX_Rx[0]!=0x7e)           //   过滤头 
  	   return; 
  //  2.  check Centre Ack
  Centre_CmdID=(UDP_HEX_Rx[1]<<8)+UDP_HEX_Rx[2];  // 接收到中心消息ID   
  Centre_FloatID=(UDP_HEX_Rx[11]<<8)+UDP_HEX_Rx[12];  // 接收到中心消息流水号        

  //  分包判断
  if(UDP_HEX_Rx[3]&0x20) 
  	{   //  分包判断
        ;  
  	} 

  //  3.   get infolen    ( 长度为消息体的长度)    不分包的话  消息头长度为12 则参与计算校验的长度 =infolen+12
  //infolen =( u16 )((UDP_HEX_Rx[3]&0x3f) << 8 ) + ( u16 ) UDP_HEX_Rx[4];  
  infolen =( u16 )((UDP_HEX_Rx[3]&0x03) << 8 ) + ( u16 ) UDP_HEX_Rx[4];  
  contentlen=infolen+12;    //  参与校验字节的长度   

  //  4.   Check  Fcs 
	  FCS_RX_UDP=0; 
  	  //nop;nop;
	  for ( i=0; i<(UDP_DecodeHex_Len-3); i++ ) //先算出收到数据的异或和  
		  {
			  FCS_RX_UDP^=UDP_HEX_Rx[1+i];	  
		  } 
	  //nop;
	  // ------- FCS filter -----------------
	/*  if( UDP_HEX_Rx[UDP_DecodeHex_Len-2]!=FCS_RX_UDP ) //判断校验结果	    
		  {
			  rt_kprintf("\r\n  infolen=%d ",infolen);  
			  rt_kprintf("\r\n808协议校验错误	  Caucate %x  ,RX  %x\r\n",FCS_RX_UDP,UDP_HEX_Rx[UDP_DecodeHex_Len-2]); 
				//-----------------  memset  -------------------------------------
			  memset(UDP_HEX_Rx, 0, sizeof(UDP_HEX_Rx));    
			  UDP_hexRx_len= 0; 
			  return; 
		  }	
	  */
	//  else		  
		 // rt_kprintf("\r\n 808协议校验正确	  Caucate %x  ,RX  %x\r\n",FCS_RX_UDP,UDP_HEX_Rx[UDP_DecodeHex_Len-2]);	   
   
 //   5 .  Classify  Process 
         rt_kprintf("\r\n           CentreCMD = 0x%X  \r\n",Centre_CmdID);  // add for  debug  
         
     switch(Centre_CmdID)
     	{
     	   case  0x8001:  //平台通用应答
     	             // 若没有分包处理的话  消息头长12  从0开始计算第12个字节是消息体得主体

					  //  13 14  对应的终端消息流水号  
					  //  15 16  对应终端的消息
                       Ack_CMDid_8001=(UDP_HEX_Rx[15]<<8)+UDP_HEX_Rx[16];

					   switch(Ack_CMDid_8001)   // 判断对应终端消息的ID做区分处理
					   	{ 
					   	   case 0x0200:    //  对应位置消息的应答
                                      //----- 判断确认是否成功
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
											   rt_kprintf( "\r\n紧急报警收到应答，得清除!\r\n");   
									  }
									  //------------------------------------
                                     if(Warn_Status[1]&0x10)// 进出区域报警
                                     {
						                  InOut_Object.TYPE=0;//圆形区域
						                  InOut_Object.ID=0; //  ID
						                  InOut_Object.InOutState=0;//  进报警 
						                  Warn_Status[1]&=~0x10;
                                     } 
									   if(Warn_Status[1]&0x20)// 进出路线报警
                                     {
						                  InOut_Object.TYPE=0;//圆形区域
						                  InOut_Object.ID=0; //  ID
						                  InOut_Object.InOutState=0;//  进报警 
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
									  /* cycle_read++;   //  收到应答才递增
									   if(cycle_read>=Max_CycleNum)
											   cycle_read=0;
									   ReadCycle_status=RdCycle_Idle;
                                                                 */
                                        //--------------  多媒体上传相关  --------------                                       
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
                                                                                     rt_kprintf("\r\n  手动上报多媒体上传处理\r\n");
											  }	
											 rt_kprintf("\r\n  多媒体信息前的多媒体发送完毕 \r\n");  
									   	  }	
									   
								       break;
				  case 0x0002:  //  心跳包的应答
                                       //  不用判结果了  ---     
                                        JT808Conf_struct.DURATION.TCP_ACK_DurCnter=0;//clear
                                        JT808Conf_struct.DURATION.TCP_SD_state=0; //clear
                                         rt_kprintf( "\r\n  Centre  Heart ACK!\r\n");  
							           break;
                            case 0x0101:  //  终端注销应答
                                        if(0==UDP_HEX_Rx[17])
                                        {  // 注销成功
                                          
										 memset(Reg_buf,0,sizeof(Reg_buf));
 										 memcpy(Reg_buf,JT808Conf_struct.ConfirmCode,20);
 										 JT808Conf_struct.Regsiter_Status=0; 
 										 Reg_buf[20]=JT808Conf_struct.Regsiter_Status; 
 										Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
										 rt_kprintf("\r\n  终端注销成功!  \r\n"); 
                                        }  

							           break;
						    case 0x0102:  //  终端鉴权
						                
 						                rt_kprintf("\r\n 收到鉴权结果: %x \r\n",UDP_HEX_Rx[17]); 
						                if(0==UDP_HEX_Rx[17])
                                         {  // 鉴权成功 
                                                          DEV_Login.Operate_enable=2; // 鉴权完成
					                        if(DataLink_Status())   
							                   DataLinkOK_Process();
											 rt_kprintf("\r\n  终端鉴权成功!  \r\n");   
						                }
									   break;
                            case 0x0800:  // 多媒体事件信息上传
                                             rt_kprintf("\r\n 多媒体事件信息上传回应! \r\n");											 
											 Media_Clear_State();  //  clear 
											 
                                            if(0==MediaObj.Media_Type)
											{
											   MediaObj.Media_transmittingFlag=1; 
											   PositionSD_Enable(); 
											   Current_UDP_sd=1;
											   
											   Photo_sdState.photo_sending=enable; 
											   Photo_sdState.SD_packetNum=1; // 第一包开始 
											   PositionSD_Enable();  //   使能上报
											   rt_kprintf("\r\n 开始上传照片! ....\r\n");   												
                            	            }
											else
											if(1==MediaObj.Media_Type)
											{
											   MediaObj.Media_transmittingFlag=1;  
											   
											   Sound_sdState.photo_sending=enable; 
											   Sound_sdState.SD_packetNum=1; // 第一包开始 
											   PositionSD_Enable();  //   使能上报 
											   Current_UDP_sd=1;
											   rt_kprintf("\r\n 开始上传音频! ....\r\n");    												
                            	            }	
											else
											if(2==MediaObj.Media_Type)
											{
											   MediaObj.Media_transmittingFlag=1;   
											   PositionSD_Enable();  //   使能上报
											   Current_UDP_sd=1;
											   Video_sdState.photo_sending=enable; 
											   Video_sdState.SD_packetNum=1; // 第一包开始   
											   rt_kprintf("\r\n 开始上传视频! ....\r\n");    												
                            	            }	
											break;
						    case 0x0702: 
                                          rt_kprintf("\r\n  驾驶员信息上报---中心应答!  \r\n"); 							       
											 
							           break;
							case 0x0701: 
                                           rt_kprintf("\r\n  电子运单上报---中心应答!  \r\n"); 							       
											 
							           break;		   
									   
							default	 :
								       break;
					   	}


					    //---------------------
						 ACKFromCenterCounter=0; 
						 fCentre_ACK=0;
						 ACK_timer=0;
				        break;
           case  0x8100:    //  监控中心对终端注册消息的应答		
                       //-----------------------------------------------------------
					   switch(UDP_HEX_Rx[15])
                        	{
                               case 0: rt_kprintf("\r\n   ----注册成功\r\n");
							           memset(JT808Conf_struct.ConfirmCode,0,sizeof(JT808Conf_struct.ConfirmCode));
                                       memcpy(JT808Conf_struct.ConfirmCode,UDP_HEX_Rx+16,infolen-3);

									   memset(Reg_buf,0,sizeof(Reg_buf));
							           memcpy(Reg_buf,JT808Conf_struct.ConfirmCode,20);
									   JT808Conf_struct.Regsiter_Status=1; 
							           Reg_buf[20]=JT808Conf_struct.Regsiter_Status;
                                Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
									   rt_kprintf("鉴权码: %s\r\n		   鉴权码长度: %d\r\n",JT808Conf_struct.ConfirmCode,strlen((const char*)JT808Conf_struct.ConfirmCode)); 
                                       //-------- 开始鉴权 ------
									   DEV_Login.Operate_enable=1;  
									   break;
							   case 1: rt_kprintf("\r\n   ----车辆已被注册\r\n");
									   break;	
							   case 2: rt_kprintf("\r\n   ----数据库中无该车辆\r\n");
									   break;
							   case 3: rt_kprintf("\r\n   ----终端已被注册\r\n");  
							           if(0==JT808Conf_struct.Regsiter_Status)  
                                        {
                                          ;//JT808Conf_struct.Regsiter_Status=2;  // not  1
                                          //DEV_regist.DeRegst_sd=1;
							           	}  
									   else
									   if(1==JT808Conf_struct.Regsiter_Status) 
							               DEV_Login.Operate_enable=1; //开始鉴权 
							          
									   break;		    
                               case 4: rt_kprintf("\r\n   ----数据库中无该终端\r\n");  
									   break;
                        	}					    
		              break;
		   case  0x8103:    //  设置终端参数   
		               //  Ack_Resualt=0;
                      if(contentlen)
                       {                       // 和中心商议好了每次只下发操作一项设置
                         Total_ParaNum=UDP_HEX_Rx[13]; // 中心设置参数总数
                         rt_kprintf("\r\n Set ParametersNum =%d  \r\n",Total_ParaNum);
						 //------------------------------------------------------------------- 
						 ContentRdAdd=14;
						 Process_Resualt=0;  // clear resualt
						 for(i=0;i<Total_ParaNum;i++)
						 {
						   //  数类型是DWORD 4 个字节
						   SubCMD_8103H=(UDP_HEX_Rx[ContentRdAdd]<<24)+(UDP_HEX_Rx[ContentRdAdd+1]<<16)+(UDP_HEX_Rx[ContentRdAdd+2]<<8)+UDP_HEX_Rx[ContentRdAdd+3]; 
                           //  子信息长度
						   SubInfolen=UDP_HEX_Rx[ContentRdAdd+4];
						   //  处理子信息 如果设置成功把相应Bit 位置为 1 否则保持 0
						   if(CentreSet_subService_8103H(SubCMD_8103H,SubInfolen,UDP_HEX_Rx+ContentRdAdd+5))
                                Process_Resualt|=(0x01<<i);						   
						   //  移动偏移地址
						   ContentRdAdd+=5+UDP_HEX_Rx[ContentRdAdd+4]; // 偏移下标
						 } 						   

                         //--------------判断所有的设置结果  ---------------
                        /* for(i=0;i<Total_ParaNum;i++)
                         {
                             if(!((Process_Resualt>>0)&0x01))
						     	{
                                  Ack_Resualt=1; //  1  表示失败
                                  break; 
						     	}
							 if(i==(Total_ParaNum-1))  //  设置到最后一个确认成功
							 	Ack_Resualt=0;  //  成功/确认
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
		   case  0x8104:    //  查询终端参数
                      SD_ACKflag.f_SettingPram_0104H=1;  // 不管什么内容回复统一结果
                      rt_kprintf("\r\n  中心查询终端参数 !\r\n");   
			          break;
           case  0x8105:     // 终端控制
                    // Ack_Resualt=0; 
                     if(contentlen)
                      {                       // 和中心商议好了每次只下发操作一项设置
                         Total_ParaNum=UDP_HEX_Rx[13]; //  终端控制命令字 
                         rt_kprintf("\r\n Set ParametersNum =%d  \r\n",Total_ParaNum); 
						 //------------------------------------------------------------------- 
						 if(CentreSet_subService_8105H(Total_ParaNum,contentlen-1,UDP_HEX_Rx+14)) 
                                Ack_Resualt=0;   // 返回成功						   
                       }	
					 
						 //-------------------------------------------------------------------
						 Ack_Resualt=0;
					   if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {
						     SD_ACKflag.f_CentreCMDack_0001H=1;
							 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
                         }	 
						  rt_kprintf("\r\ny  终端控制 !\r\n"); 
						  
		              break;
		   case  0x8201:     // 位置信息查询    位置信息查询消息体为空
		               SD_ACKflag.f_CurrentPosition_0201H=1;		
					   rt_kprintf("\r\n  位置信息查询 !\r\n");  
		              break;
		   case  0x8202:     // 临时位置跟踪控制
		                 Ack_Resualt=0;

						 //  13 14  时间间隔 
					     JT808Conf_struct.RT_LOCK.Lock_Dur=(UDP_HEX_Rx[13]<<8)+UDP_HEX_Rx[14];
					    //  15 16  17 18 对应终端的消息
                         JT808Conf_struct.RT_LOCK.Lock_KeepDur=(UDP_HEX_Rx[15]<<24)+(UDP_HEX_Rx[16]<<16)+(UDP_HEX_Rx[17]<<8)+UDP_HEX_Rx[18];

                         JT808Conf_struct.RT_LOCK.Lock_state=1;    // Enable Flag
						 JT808Conf_struct.RT_LOCK.Lock_KeepCnter=0;  //  保持计数器
						 Current_SD_Duration=JT808Conf_struct.RT_LOCK.Lock_KeepDur;  //更改发送间隔						 

						 JT808Conf_struct.SD_MODE.DUR_TOTALMODE=1;   // 更新定时相关状态位
						 JT808Conf_struct.SD_MODE.Dur_DefaultMode=1;
						 //  保存配置
						 Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
						 
		                // if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {
						     Ack_Resualt=0;
						     SD_ACKflag.f_CentreCMDack_0001H=1;
							 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;      
                         }
                      rt_kprintf("\r\n  临时位置跟踪控制!\r\n"); 
		              break;
		   case  0x8300:    //  文本信息下发
		                  Ack_Resualt=0;
                          TextInfo.TEXT_FLAG=UDP_HEX_Rx[13];
						  if(TextInfo.TEXT_FLAG&0x09)  // 检测是否给TTS终端  ，紧急也给TTS播报
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
						 if((TextInfo.TEXT_FLAG&0x04)||(TextInfo.TEXT_FLAG&0x01))  // 检测是否给终端显示器
						 {                            
							memset( TextInfo.TEXT_Content,0,sizeof(TextInfo.TEXT_Content));
							memcpy(TextInfo.TEXT_Content,UDP_HEX_Rx+14,infolen-1);
							TextInfo.TEXT_SD_FLAG=1;	// 置发送给显示屏标志位  // ||||||||||||||||||||||||||||||||||

							//========================================
							TextInforCounter++;
							rt_kprintf("\r\n写入收到的第 %d 条信息,消息长度=%d,消息:%s",TextInforCounter,infolen-1,TextInfo.TEXT_Content);
							TEXTMSG_Write(TextInforCounter,1,infolen-1,TextInfo.TEXT_Content);				
							//========================================
						 } 

						  #ifdef LCD_5inch
						           //======  信息都在屏幕上显示  
                                                	  DwinLCD.Type=LCD_SDTXT;
							  memset(DwinLCD.TXT_content,0,sizeof(DwinLCD.TXT_content)); 
                                                   DwinLCD.TXT_contentLen=AsciiToGb(DwinLCD.TXT_content,infolen-1,UDP_HEX_Rx+14);  						  
   						#endif    

                          //------- 返回 ----
						 //  if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {   
						      Ack_Resualt=0;
						     SD_ACKflag.f_CentreCMDack_0001H=1;
							 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
                         }
						   rt_kprintf("\r\n 文本信息: %s\r\n",TextInfo.TEXT_Content);
		              break; 
		   case  0x8301:    //  事件设置 
					   if(contentlen)
                      {                       
                         //--- 设置类型--
                         switch(UDP_HEX_Rx[13])
                         {
                            case 0 :  //  删除终端现有所有事件，该命令后不带后继字符
                                      Event_Init(1); 	
                                      rt_kprintf("\r\n 删除所有事件\r\n");
                                      break; 
							case 1:  // 更新事件
									 if(UDP_HEX_Rx[13]==1)
									 	rt_kprintf("\r\n 更新事件\r\n");
							         //break;
							case 2:  // 追加事件
                                     if(UDP_HEX_Rx[13]==2)
									 	rt_kprintf("\r\n 追加事件\r\n");
							          //break;
							case 3:  // 修改事件
                                     if(UDP_HEX_Rx[13]==3)
									 	 rt_kprintf("\r\n 修改事件\r\n");  
							          //break;
							case 4:  // 删除特定事件       
							         if(UDP_HEX_Rx[13]==4)
									 	 rt_kprintf("\r\n 删除特点事件\r\n"); 
									  Total_ParaNum=UDP_HEX_Rx[14]; // 中心设置参数总数
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
                                                                 Api_RecordNum_Write(event_808, EventObj.Event_ID, (u8*)&EventObj,sizeof(EventObj));									  rt_kprintf("\r\n 事件内容:%s\r\n",EventObj.Event_Str);   
									  rt_kprintf("\r\n 事件内容:%s\r\n",EventObj.Event_Str);   
							          break;
							default:
								      break;

                         }

                          //---------返回 -------
                         // if(SD_ACKflag.f_CentreCMDack_0001H==0) // 一般回复
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
		   case  0x8302:    // 提问下发
						 // if(UDP_HEX_Rx[13]&0x08)  // 检测标志是否给显示终端 
						   rt_kprintf("\r\n  中心下发提问 \r\n"); 
						  {
						     ASK_Centre.ASK_infolen=UDP_HEX_Rx[14];
							 memset(ASK_Centre.ASK_info,0,sizeof(ASK_Centre.ASK_info));
							 memcpy(ASK_Centre.ASK_info,UDP_HEX_Rx+15,ASK_Centre.ASK_infolen);
							 rt_kprintf("\r\n  问题: %s \r\n",ASK_Centre.ASK_info); 
							 memset(ASK_Centre.ASK_answer,0,sizeof(ASK_Centre.ASK_answer));
							 memcpy(ASK_Centre.ASK_answer,UDP_HEX_Rx+15+ASK_Centre.ASK_infolen,infolen-2-ASK_Centre.ASK_infolen);	 

							 ASK_Centre.ASK_SdFlag=1;   // ||||||||||||||||||||||||||||||||||
							 ASK_Centre.ASK_floatID=Centre_FloatID; // 备份 FloatID	 
                                                  ASK_Centre.ASK_disp_Enable=1;
							 rt_kprintf("\r\n 提问Answer:%s\r\n",ASK_Centre.ASK_answer+3);
       
                                                  Api_RecordNum_Write(ask_quesstion,1, (u8*)&ASK_Centre,sizeof(ASK_Centre)); 	
                                                  
						  }
						   
						// if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {
						     Ack_Resualt=0; 
						     SD_ACKflag.f_CentreCMDack_0001H=1;
					          SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;  
                                           } 

					
		              break;
           case  0x8303:    //  信息点播菜单设置
                              //--- 设置类型--
                         switch(UDP_HEX_Rx[13])
                         {
                            case 0 :  //  删除终端现有所有信息
                                      MSG_BroadCast_Init(1); 
                                      rt_kprintf("\r\n 删除信息\r\n");
                                      break;
							case 1:  // 更新菜单
									  if(UDP_HEX_Rx[13]==1)
									 	  rt_kprintf("\r\n 更新菜单\r\n");
							         //break;
							case 2:  // 追加菜单
                                       if(UDP_HEX_Rx[13]==2)
									 	  rt_kprintf("\r\n 追加菜单\r\n");
							          //break;
							case 3:  // 修改菜单
							          if(UDP_HEX_Rx[13]==3)
									 	  rt_kprintf("\r\n 修改菜单\r\n");
									  Total_ParaNum=UDP_HEX_Rx[14];           // 消息项总数
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
									  rt_kprintf("\r\n 信息点播内容:%s\r\n",MSG_BroadCast_Obj.INFO_STR); 
							          break;
							default:
								      break;

                         }

                          //---------返回 -------
                          // if(SD_ACKflag.f_CentreCMDack_0001H==0) // 一般回复
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
		   case  0x8304:    //  信息服务
                          Ack_Resualt=0;
                          MSG_BroadCast_Obj.INFO_TYPE=UDP_HEX_Rx[13];  //  信息类型
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
						   rt_kprintf("\r\n 信息服务内容:%s\r\n",Dev_Voice.Play_info); 

                           // --------  发送给文本信息  --------------    
						   memset( TextInfo.TEXT_Content,0,sizeof(TextInfo.TEXT_Content));
						   memcpy(TextInfo.TEXT_Content,UDP_HEX_Rx+16,infolen-3);
						   TextInfo.TEXT_SD_FLAG=1;    // 置发送给显示屏标志位	// ||||||||||||||||||||||||||||||||||
						   
 
                          //------- 返回 ----
						 //  if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {
						     Ack_Resualt=0;
						     SD_ACKflag.f_CentreCMDack_0001H=1;
							 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
                         }
		              break;	
		   case  0x8400:    //  电话回拨
                       
					   if(infolen==0)
							 break;
					   if(0==UDP_HEX_Rx[13])   // 普通通话
					   	   rt_kprintf("\r\n   电话回拨-->普通通话\r\n");
					   else
					   if(1==UDP_HEX_Rx[13])  //  监听
					       rt_kprintf("\r\n   电话回拨-->监听");
					   else
					       break;
	 					  memset(JT808Conf_struct.LISTEN_Num,0,sizeof(JT808Conf_struct.LISTEN_Num));
	 					  memcpy(JT808Conf_struct.LISTEN_Num,UDP_HEX_Rx+14,infolen-1);  										  
                                            Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
                                            CallState=CallState_rdytoDialLis;  // 准备开始拨打监听号码 

						 // if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {
						     SD_ACKflag.f_CentreCMDack_0001H=1;
							 SD_ACKflag.f_CentreCMDack_resualt=0;      
                         }
		              break;
		   case  0x8401:  //   设置电话本
                      
					  switch(UDP_HEX_Rx[13])
					  {
						 case 0 :  //  删除终端现有所有信息
								  // PhoneBook_Init(1); 
								   rt_kprintf("\r\n 删除电话本\r\n"); 
								   break;
						 case 1:  // 更新菜单
						          if(UDP_HEX_Rx[13]==1)
									 	 rt_kprintf("\r\n 更新电话本\r\n"); 
						 case 3:  // 修改菜单
						           if(UDP_HEX_Rx[13]==3)
									 	 rt_kprintf("\r\n 修改电话本\r\n"); 
						           Rx_PhoneBOOK.CALL_TYPE=UDP_HEX_Rx[15]; // 标志 ，呼入呼出
                                   Rx_PhoneBOOK.NumLen=UDP_HEX_Rx[16];								   
								   memset(Rx_PhoneBOOK.NumberStr,0,sizeof(Rx_PhoneBOOK.NumberStr)); 
								   memcpy(Rx_PhoneBOOK.NumberStr,UDP_HEX_Rx+17,Rx_PhoneBOOK.NumLen);
								   Rx_PhoneBOOK.UserLen=UDP_HEX_Rx[17+Rx_PhoneBOOK.NumLen]; 								   
								   memset(Rx_PhoneBOOK.UserStr,0,sizeof(Rx_PhoneBOOK.UserStr));								   
								   memcpy(Rx_PhoneBOOK.UserStr,UDP_HEX_Rx+18+Rx_PhoneBOOK.NumLen,Rx_PhoneBOOK.UserLen);
								   
								   for(i=0;i<8;i++)
								   {
                                    PhoneBook.CALL_TYPE=2; //类型定义为输出 
								    PhoneBook.NumLen=0;    // 号码长度
								    memset(PhoneBook.NumberStr,0,sizeof(PhoneBook.NumberStr));  
								    PhoneBook.UserLen=0;		
								    memset(PhoneBook.UserStr,0,sizeof(PhoneBook.UserStr)); 	 
									   Api_RecordNum_Read(phonebook, i+1, (u8*)&PhoneBook,sizeof(PhoneBook)); 
									if(strncmp((char*)PhoneBook.UserStr,(const char*)Rx_PhoneBOOK.UserStr,Rx_PhoneBOOK.UserLen) == 0) 
									{ // 找到相同名字的把以前的删除用新的代替
									   Api_RecordNum_Write(phonebook, i+1, (u8*)&Rx_PhoneBOOK,sizeof(Rx_PhoneBOOK)); 
									   break;  // 跳出for 
									}

								   }
								   break;
						 case 2:  // 追加菜单	
						          if(UDP_HEX_Rx[13]==2)
									 	 rt_kprintf("\r\n 追加电话本\r\n"); 
						           Rx_PhoneBOOK.CALL_TYPE=UDP_HEX_Rx[15]; // 标志 ，呼入呼出
                                   Rx_PhoneBOOK.NumLen=UDP_HEX_Rx[16];								   
								   memset(Rx_PhoneBOOK.NumberStr,0,sizeof(Rx_PhoneBOOK.NumberStr)); 
								   memcpy(Rx_PhoneBOOK.NumberStr,UDP_HEX_Rx+17,Rx_PhoneBOOK.NumLen);
								   Rx_PhoneBOOK.UserLen=UDP_HEX_Rx[17+Rx_PhoneBOOK.NumLen]; 								   
								   memset(Rx_PhoneBOOK.UserStr,0,sizeof(Rx_PhoneBOOK.UserStr));		
								   Rx_PhoneBOOK.Effective_Flag=1; // 有效标志位  
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
           case  0x8500:    //  车辆控制
                      Vech_Control.Control_Flag=UDP_HEX_Rx[13];
					  if(UDP_HEX_Rx[13]&0x01)
					  	{ // 车门加锁       bit 12
                            Car_Status[2]|=0x10;     // 需要控制继电器
							rt_kprintf("\r\n  车辆加锁 \r\n"); 
					  	}
					  else
					  	{ // 车门解锁
                            Car_Status[2]&=~0x10;    // 需要控制继电器
                            rt_kprintf("\r\n  车辆解锁 \r\n"); 
					  	}					  
					  Vech_Control.ACK_SD_Flag=1;    
		              break;
		   case  0x8600:    //  设置圆形区域
                       rt_kprintf("\r\n  设置圆形区域 \r\n"); 
		              if(UDP_HEX_Rx[14]==1)  //  现在支持设置一个区域
		              { 
	                       switch(UDP_HEX_Rx[13])
	                       	{	                       	      
		                          case 1:  // 追加区域
                                           for(i=0;i<8;i++)
                                           	{                                                
												memset((u8*)&Rail_Cycle,0,sizeof(Rail_Cycle));
												Api_RecordNum_Write(Rail_cycle,Rail_Cycle.Area_ID, (u8*)&Rail_Cycle,sizeof(Rail_Cycle)); 	 
												if(Rail_Cycle.Area_attribute) // 找出来未使用的
													break;
                                           	}
										   if(8==i)  //  如果都满了，那么用 0
										   	{
										   	  i=0;
										   	}
								         
		                          case 0:  // 更新区域
		                          case 2:  // 修改区域                                      
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
                		 //------- 返回 ----
						//   if(SD_ACKflag.f_CentreCMDack_0001H==0)
						 {
						     SD_ACKflag.f_CentreCMDack_0001H=1;
							 Ack_Resualt=0;
							 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
                         }
				      break;
           case  0x8601:    //  删除圆形区域 
                         rt_kprintf("\r\n  删除圆形区域 \r\n"); 
						 if(0==UDP_HEX_Rx[13])   // 区域数
                                                    RailCycle_Init();  // 删除所有区域  
						 else
						 	{						 	  
							  memset((u8*)&Rail_Cycle,0,sizeof(Rail_Cycle));  //  clear all  first
						 	  for(i=0;i<UDP_HEX_Rx[13];i++)
						 	  {
						 	    Rail_Cycle.Area_ID=(UDP_HEX_Rx[14+i]<<24)+(UDP_HEX_Rx[15+i]<<16)+(UDP_HEX_Rx[16+i]<<8)+UDP_HEX_Rx[17+i];
                                if((Rail_Cycle.Area_ID>8)||(Rail_Cycle.Area_ID==0))
										   	Rail_Cycle.Area_ID=1;	
								Rail_Cycle.Effective_flag=0; // clear
								 Api_RecordNum_Write(Rail_cycle,Rail_Cycle.Area_ID, (u8*)&Rail_Cycle,sizeof(Rail_Cycle)); // 删除对应的围栏
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
		   case  0x8602:    //  设置矩形区域
		               rt_kprintf("\r\n  设置矩形区域 \r\n"); 
                        if(UDP_HEX_Rx[14]==1)  //  现在支持设置一个区域
		              { 
	                       switch(UDP_HEX_Rx[13])
	                       	{	                       	      
		                          case 1:  // 追加区域
                                  case 0:  // 更新区域
		                          case 2:  // 修改区域                                      
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

										   rt_kprintf("\r\n   中心设置  矩形围栏 leftLati=%d leftlongi=%d\r\n",Rail_Rectangle.LeftUp_Latitude,Rail_Rectangle.LeftUp_Longitude);  
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
		   case  0x8603:    //  删除矩形区域
                           rt_kprintf("\r\n  删除矩形区域 \r\n");    
					   	  if(0==UDP_HEX_Rx[13])   // 区域数
                             RailRect_Init();  // 删除所有区域
						   else
						 	{						 	  
							  memset((u8*)&Rail_Rectangle,0,sizeof(Rail_Rectangle));  //  clear all  first
						 	  for(i=0;i<UDP_HEX_Rx[13];i++)
						 	  {
						 	    Rail_Rectangle.Area_ID=(UDP_HEX_Rx[14+i]<<24)+(UDP_HEX_Rx[15+i]<<16)+(UDP_HEX_Rx[16+i]<<8)+UDP_HEX_Rx[17+i];
                                if((Rail_Rectangle.Area_ID>8)||(Rail_Rectangle.Area_ID==0))
										   	Rail_Rectangle.Area_ID=1;	
								Rail_Rectangle.Effective_flag=0;
								Api_RecordNum_Write(Rail_rect,Rail_Rectangle.Area_ID, (u8*)&Rail_Rectangle,sizeof(Rail_Rectangle)); // 删除对应的围栏
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
		   case  0x8604:	//  多边形区域
		                     rt_kprintf("\r\n  设置多边形区域 \r\n");
			                 if(UDP_HEX_Rx[14]==1)  //  现在支持设置一个区域
							 { 
								  switch(UDP_HEX_Rx[13])
								   {								 
										 case 1:  // 追加区域
										 case 0:  // 更新区域
										 case 2:  // 修改区域									   
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
		   case  0x8605:    //  删除多边区域
                         rt_kprintf("\r\n  删除多边形区域 \r\n");
                        if(0==UDP_HEX_Rx[13])   // 区域数
                             RailPolygen_Init();  // 删除所有区域
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
		   case  0x8606:    //  设置路线
		                 rt_kprintf("\r\n  设置路线 \r\n");
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
						for(i=0;i<6;i++)  // 拐点数目
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
						  Api_RecordNum_Write(route_line,ROUTE_Obj.Route_ID, (u8*)&ROUTE_Obj,sizeof(ROUTE_Obj));// 删除对应的围栏 



                           //----------------
                    // if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
		              break;
		   case  0x8607:    //  删除路线
                         rt_kprintf("\r\n  删除路线 \r\n");  
                         if(0==UDP_HEX_Rx[13])   // 区域数
                             RouteLine_Init();  // 删除所有区域
						 else
						 	{						 	  
							  memset((u8*)&ROUTE_Obj,0,sizeof(ROUTE_Obj));  //  clear all  first
						 	  for(i=0;i<UDP_HEX_Rx[13];i++)
						 	  {
						 	    ROUTE_Obj.Route_ID=(UDP_HEX_Rx[14+i]<<24)+(UDP_HEX_Rx[15+i]<<16)+(UDP_HEX_Rx[16+i]<<8)+UDP_HEX_Rx[17+i];
                                if((ROUTE_Obj.Route_ID>Route_Mum)||(ROUTE_Obj.Route_ID==0))
										   	ROUTE_Obj.Route_ID=1;	
								ROUTE_Obj.Effective_flag=0;       
								     Api_RecordNum_Write(route_line,ROUTE_Obj.Route_ID, (u8*)&ROUTE_Obj,sizeof(ROUTE_Obj)); 	  // 删除对应的围栏
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
           case  0x8700:    //  行车记录仪数据采集命令
                       rt_kprintf("\r\n  记录仪采集命令 \r\n");
                       Recode_Obj.Float_ID=Centre_FloatID;
					   Recode_Obj.CMD=UDP_HEX_Rx[13];
					   Recode_Obj.SD_Data_Flag=1;
					   
		              break;
		   case  0x8701:    //  行驶记录仪参数下传命令
		               rt_kprintf("\r\n  记录仪参数下传 \r\n");
		               Recode_Obj.Float_ID=Centre_FloatID;
					   Recode_Obj.CMD=UDP_HEX_Rx[13];    
					   Recode_Obj.SD_Data_Flag=1;  
					   CenterSet_subService_8701H(Recode_Obj.CMD,UDP_HEX_Rx+14); //跳过2B长度和1 保留字
			          break; 
		   case  0x8800:    //   多媒体数据上传应答
		              if(infolen==5)
		              	{  //  判断是否有重传ID列表，如果没有则表示中心接收完成!						
                                switch (MediaObj.Media_Type)
						         {
						          case 0 : // 图像
						                  Photo_send_end();  // 拍照上传结束
						                  rt_kprintf("\r\n 图像传输结束! \r\n");
                                          //------------多路拍照处理  -------
                                         // CheckResualt=Check_MultiTakeResult_b4Trans();  
						                 				   
								          break;
								  case 1 : // 音频
								          Sound_send_end();
										  rt_kprintf("\r\n 音频传输结束! \r\n");
								          break;
								  case 2 : // 视频
									      Video_send_end();
  								          rt_kprintf("\r\n 视频传输结束! \r\n");  
								          break;
								  default:
								  	      break;
						         }
						// if(CheckResualt==0)	
						   	   Media_Clear_State(); 
		              	}
					  else
					  	{  //  重传包ID 列表 
					  	  if(UDP_HEX_Rx[17]!=0)
					  	  {
						  	   MediaObj.RSD_State=1;   //   进入重传状态
						  	   MediaObj.RSD_Timer=0;   //   清除重传定时器   
						  	   MediaObj.RSD_Reader=0;
							   MediaObj.RSD_total=UDP_HEX_Rx[17];    // 重传包总数
						  	   memset(MediaObj.Media_ReSdList,0,125);
							   memcpy(MediaObj.Media_ReSdList,UDP_HEX_Rx+18,UDP_HEX_Rx[17]); 
	                            rt_kprintf("\r\n  重传列表: ");
							   for(i=0;i<MediaObj.RSD_total;i++)
							     rt_kprintf(" %d",UDP_HEX_Rx[18+i]); 
							   rt_kprintf("\r\n"); 
					  	   } 
						}
                        
		              break;
		   case  0x8801:   //    摄像头立即拍照		  

				     Camera_Obj.Channel_ID=UDP_HEX_Rx[13];     //   通道
					 Camera_Obj.Operate_state=UDP_HEX_Rx[18];  //   是否保存标志位  
		            //----------------------------------
				   
					if((Camera_Take_Enable())&&(Photo_sdState.photo_sending==0)) //图片传输中不能拍
					{
					   Camera_Number=UDP_HEX_Rx[13];
					   if((Camera_Number>Max_CameraNum)&&(Camera_Number<1)) 
						  break;
					 
					   Start_Camera(Camera_Number);   //开始拍照  
					   SingleCamera_TakeRetry=0;   // clear 
					}	   
					rt_kprintf("\r\n   中心及时拍照  Camera: %d    \r\n",Camera_Number);	
		              
                     // if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
                      break;
		   case  0x8802:   //    存储多媒体数据检索
		                 SD_ACKflag.f_QueryEventCode=UDP_HEX_Rx[15];    
                         switch(UDP_HEX_Rx[13])
                         {
                            case 0:  // 图像
                                     SD_ACKflag.f_MediaIndexACK_0802H=1;
									 rt_kprintf("\r\n  中心查询图像索引 \r\n");   
							       break;
							case 1:  //  音频
                                     SD_ACKflag.f_MediaIndexACK_0802H=2;
									  rt_kprintf("\r\n  中心查询音频索引 \r\n"); 	 
							case 2:  //  视频
							         SD_ACKflag.f_MediaIndexACK_0802H=3;
						    default:
								     break; 
						}						 
						
		              break;
		   case  0x8803:   //    存储多媒体数据上传命令
                      rt_kprintf("\r\n 多媒体数据上传\r\n"); 
                       switch(UDP_HEX_Rx[13])
                         {
                            case 0:  // 图像
									 rt_kprintf("\r\n   上传固有图片\r\n");    
							       break;
							case 1:  //  音频
                                     MP3_send_start();
									  rt_kprintf("\r\n  上传固有音频 \r\n"); 
								   break;	  
							case 2:  //  视频
							        // Video_send_start();
							         // MP3_send_start();
									  rt_kprintf("\r\n  上传固有视频  不操作了 用音频\r\n"); 
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
		   case  0x8804:   //    录音开始命令
		   
		            //#if  1			     
                       switch(UDP_HEX_Rx[13])
                       	{ 
                       	  case 0x00:  // 停录音
                       	                      // VOC_REC_STOP(void);
						       VOC_REC_Stop();
							/* 					  
	                                     Dev_Voice.CMD_Type='0';
										 Dev_Voice.info_sdFlag=0;    
										 Dev_Voice.Voice_FileOperateFlag=0;
										 Dev_Voice.Centre_RecordFlag=0; // 清空中心录音标志位
										 if(TF_Card_Status()==1)	
												{
												  //memset(Dev_Voice.Voice_Reg,0,512);//clear left
												  //edit_file(Dev_Voice.FileName,Dev_Voice.Voice_Reg,512); //写信息到TF
												  //Dev_Voice.Voice_FileSize+=500; //add  											  
												  rt_kprintf("\r\n ---------   语音文件的最后地址	  VoiceFileSize %d Bytes  \r\n",Dev_Voice.Voice_FileSize);
												}   
										Api_DFdirectory_Write(voice, Dev_Voice.Voice_Reg,500);  
		                                 Dev_Voice.Voice_FileSize+=500;
		                                 Sound_SaveEnd();  */
						            break;
						  case 0x01:  // 开始录音
                                                       
								     VOC_REC_Start();
						 					   
					 /*
						          if(MMedia2_Flag==0)
						          	{
                                       MP3_send_start();
									   rt_kprintf("\r\n  上传固有音频 \r\n"); 
									   MMedia2_Flag=1;
									   break;
						          	}
                                    Dev_Voice.Rec_Dur=(UDP_HEX_Rx[14]<<8)+UDP_HEX_Rx[15];
									Dev_Voice.SaveOrNotFlag=UDP_HEX_Rx[16];

                                    // ------   录音文件名 -----------
                                    if(TF_Card_Status()==1)
	                                { 
	                                    memset(Dev_Voice.FileName,0,sizeof(Dev_Voice.FileName)); 	    									
										sprintf((char*)Dev_Voice.FileName,"%d%d%d%d.spx",time_now.day,time_now.hour,time_now.min,time_now.sec);    
	                                   // creat_file(Dev_Voice.FileName); //创建文件名了 
	                                    Sound_SaveStart(); 
										rt_kprintf("\r\n			  中心录音文件名称: %s \r\n",Dev_Voice.FileName);	
										Save_MediaIndex(1,Dev_Voice.FileName,0,0);     
                                     }
                                    Dev_Voice.Centre_RecordFlag=1;//中心开始录音标志位
									              
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
		   case  0x8805:   //    单条存储多媒体数据检索上传命令	---- 补充协议要求

                      reg_u32=(UDP_HEX_Rx[13]<<24)+(UDP_HEX_Rx[14]<<16)+(UDP_HEX_Rx[15]<<8)+UDP_HEX_Rx[16];
					  rt_kprintf("\r\n单条存储多媒体数据检索上传 MeidiaID=%d 删除标志: %d ",reg_u32,UDP_HEX_Rx[17]);

					  	  //------------------------------
					 // if(SD_ACKflag.f_CentreCMDack_0001H==0)
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 }
		              break;
		 case  0x8900:   //    数据下行透传                      
		                      if(LinkNum==0)
		                     {
                                     DataTrans.TYPE=UDP_HEX_Rx[13];
					  memset(DataTrans.DataRx,0,sizeof(DataTrans.DataRx)); 
                                     memcpy(DataTrans.DataRx,UDP_HEX_Rx+14,infolen-1);
					  DataTrans.Data_RxLen=infolen-1;   

					  //--------- 送给小屏幕---------- 	
					  	memset( TextInfo.TEXT_Content,0,sizeof(TextInfo.TEXT_Content));
						AsciiToGb(TextInfo.TEXT_Content,infolen-1,UDP_HEX_Rx+14); 
						TextInfo.TEXT_SD_FLAG=1;	// 置发送给显示屏标志位  // ||||||||||||||||||||||||||||||||||

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
					  	  //  DataTrans_ISPService_8900H(UDP_HEX_Rx+14); //14 偏移是信息内容
					  	}
					  else
					  	{
					  	rt_kprintf("\r\n透传数据类型错误UDP_HEX_Rx[13]=%d",UDP_HEX_Rx[13]);
					  	}
				 }
		              break;
		   case  0x8A00:   //    平台RSA公钥

		   
                    // if(SD_ACKflag.f_CentreCMDack_0001H==0) 
 					 {
 						 SD_ACKflag.f_CentreCMDack_0001H=1;
 						 Ack_Resualt=0;
 						 SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
 					 } 
				      break;	
		  //----------     北斗相关扩展协议   ----------------------			  
		  case   0xFF01:   // 扩展终端参数设置
                                       if(contentlen)
                                      {                       // 和中心商议好了每次只下发操作一项设置
                                          Total_ParaNum=UDP_HEX_Rx[13]; // 中心设置参数总数
                                          rt_kprintf("\r\n Set ParametersNum =%d  \r\n",Total_ParaNum);
						 //------------------------------------------------------------------- 
						 ContentRdAdd=14;
						 Process_Resualt=0;  // clear resualt
						 for(i=0;i<Total_ParaNum;i++)
						 {
						   //  数类型是DWORD 4 个字节
						   SubCMD_FF01H=(UDP_HEX_Rx[ContentRdAdd]<<24)+(UDP_HEX_Rx[ContentRdAdd+1]<<16)+(UDP_HEX_Rx[ContentRdAdd+2]<<8)+UDP_HEX_Rx[ContentRdAdd+3]; 
                                            //  子信息长度
						   SubInfolen=UDP_HEX_Rx[ContentRdAdd+4];
						   //  处理子信息 如果设置成功把相应Bit 位置为 1 否则保持 0
						   if(CentreSet_subService_FF01H(SubCMD_FF01H,SubInfolen,UDP_HEX_Rx+ContentRdAdd+5))
                                              Process_Resualt|=(0x01<<i);						   
						   //  移动偏移地址
						   ContentRdAdd+=5+UDP_HEX_Rx[ContentRdAdd+4]; // 偏移下标 
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
		  case   0xFF02: // 扩展终端参数查询
                                  SD_ACKflag.f_BD_Extend_7F02H=1;
				      rt_kprintf("\r\n  中心查询北斗相关设置信息\r\n");			  
		                   break;
		  case  0xFF03:   //  扩展终端参数设置1 命令
                                   
                                    if(contentlen)
                                      {                       // 和中心商议好了每次只下发操作一项设置
                                          Total_ParaNum=UDP_HEX_Rx[13]; // 中心设置参数总数
                                          rt_kprintf("\r\n Set ParametersNum =%d  \r\n",Total_ParaNum);
						 //------------------------------------------------------------------- 
						 ContentRdAdd=14;
						 Process_Resualt=0;  // clear resualt
						 for(i=0;i<Total_ParaNum;i++)
						 {
						   //  数类型是DWORD 4 个字节
						   SubCMD_FF03H=(UDP_HEX_Rx[ContentRdAdd]<<24)+(UDP_HEX_Rx[ContentRdAdd+1]<<16)+(UDP_HEX_Rx[ContentRdAdd+2]<<8)+UDP_HEX_Rx[ContentRdAdd+3]; 
                                            //  子信息长度
						   SubInfolen=UDP_HEX_Rx[ContentRdAdd+4];
						   //  处理子信息 如果设置成功把相应Bit 位置为 1 否则保持 0
						   if(CentreSet_subService_FF03H(SubCMD_FF03H,SubInfolen,UDP_HEX_Rx+ContentRdAdd+5))
                                              Process_Resualt|=(0x01<<i);						   
						   //  移动偏移地址
						   ContentRdAdd+=5+UDP_HEX_Rx[ContentRdAdd+4]; // 偏移下标 
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
	   //------- 系统启动时处理   若 标志位为0 且年月日都为 0，表面是系统刚刚启动，这时获取当前时间把当前时间给BAK------
	if((0==Avrgspd_Mint.saveFlag)&&(Avrgspd_Mint.datetime_Bak[0]==0)&&(Avrgspd_Mint.datetime_Bak[1]==0)&&(Avrgspd_Mint.datetime_Bak[2]==0))
		{
		Time2BCD(Avrgspd_Mint.datetime);  
		memcpy(Avrgspd_Mint.datetime_Bak,Avrgspd_Mint.datetime,6);//将当前时间给BAK
		avgspd_Mint_Wr=Temp_Gps_Gprs.Time[1];//time_now.min;
		//rt_kprintf("\r\n-------------()avgspd_Mint_Wr=%d",avgspd_Mint_Wr);
		}	
	else
		{     // 若不是系统启动开始   
		if(min==0) //  把秒数写为非0 ，以便留出时间去存储上1小时的数据 
			{
			if(sec==5)  //存储上一小时每分钟的平均速度 ==2是因为要保证第59分钟速度已经被存储到相应的寄存器中
				{
				memcpy(Avrgspd_Mint.datetime_Bak,Avrgspd_Mint.datetime,6);//将当前时间给BAK
				Avrgspd_Mint.datetime_Bak[4]=0;// 分钟为0
				Avrgspd_Mint.datetime_Bak[5]=0;// 秒也为0 
				Avrgspd_Mint.saveFlag=1;//使能存储      到NandFlag  标志位
				Time2BCD(Avrgspd_Mint.datetime);//获取新的时间
				//rt_kprintf("\r\n存储前一小时平均速度,min=%d",min);
				} 
			if(sec==8)    //存储完后  清除时间下标  
				avgspd_Mint_Wr=0;
			}
		}
	
	if(sec==0)  // 秒为0时存储上1分钟的数据
		{
		//rt_kprintf("\r\n下一分钟开始,PerMinSpdTotal=%d,AspdCounter=%d",PerMinSpdTotal,AspdCounter);
		if(AspdCounter)
			{			
			//rt_kprintf("\r\n--------------PerMinSpdTotal=%d,AspdCounter=%d", PerMinSpdTotal,AspdCounter);  
			Avrgspd_Mint.avgrspd[avgspd_Mint_Wr]=PerMinSpdTotal/AspdCounter;//第几分钟的平均速度
			//=============================================================
			//为了好测试暂时把1分钟内速度总和当作平均速度来统计
			//Avrgspd_Mint.avgrspd[avgspd_Mint_Wr]=PerMinSpdTotal;
			//=============================================================
			}
		else
			{
			Avrgspd_Mint.avgrspd[avgspd_Mint_Wr]=0; 
			//rt_kprintf("\r\n-------------(速度无效)");
			}
		//rt_kprintf("\r\n--------------%d分,平均速度=%d \r\n", avgspd_Mint_Wr,Avrgspd_Mint.avgrspd[avgspd_Mint_Wr]);  
		//avgspd_Mint_Wr++;//=(Avrgspd_Mint.datetime[4]>>4)*10+(0x0f&Avrgspd_Mint.datetime[4]);  
		
		PerMinSpdTotal=0;  // clear   
		AspdCounter=0;	   // clear  
		
		//rt_kprintf("\r\n清0，速度总和=%d,次数=%d\r\n",PerMinSpdTotal,AspdCounter);
		
		}
	else
		{
		if(UDP_dataPacket_flag==0x02 ) 
			{
			avgspd_Mint_Wr=Temp_Gps_Gprs.Time[1];;//++;//=time_now.min+1;
			
			AspdCounter++;
			PerMinSpdTotal+=GPS_speed/10;   // 只要求精确到 km/h   所以要除 10
			//if(AspdCounter%10==0)
				//rt_kprintf("\r\nAspdCounter=%d,GPS_speed=%d,PerMinSpdTotal=%d \r\n",AspdCounter,GPS_speed/10,PerMinSpdTotal);  
		
			} 
		}  
}

//==================================================================================================
// 第四部分 :   以下是行车记录仪相关协议 即 附录A   
//==================================================================================================

//  1.  上载数据命令字相关  
/*
u8 In_Type  :   传输类型
u8* InStr   :   参数字符串
u8 TransType:   传输方式   串口  或  GPRS

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

  
  if(TransType)  //-------  GPRS  得添加 无线协议头
  {
    memcpy(UpReg+Up_wr,"*GB",3); // GPRS 起始头
    Up_wr+=3;
	
    UpReg[Up_wr++]=0x00;  // SIM 号码   最先两个字节填写0x00
	UpReg[Up_wr++]=0x00;
	memcpy(UpReg+Up_wr,SIM_code,6);
	Up_wr+=6;
	
	Greglen=Up_wr;                      // 长度
	Up_wr+=2;
	
	UpReg[Up_wr++]=0x00; //消息序号 默认 0x00

	UpReg[Up_wr++]=0x20; //参数  bit5 bit4 10 选择应答遵照协议 

    UpReg[Up_wr++]=0xF0; // 命令字  无线传输RS232数据

	//   以下是数据内容
  }  
  //---------------填写 A 协议头 ------------------------
  Swr=Up_wr;  // reg save
  UpReg[Up_wr++]=0xAA;  // 起始头
  UpReg[Up_wr++]=0x75;
  //---------------根据类型分类填写内容------------------
  switch(In_Type)  
  	{
  	    //---------------- 上传数类型  -------------------------------------
		case  A_Up_DrvInfo  :      //  当前驾驶人信息
		                     UpReg[Up_wr++]=In_Type; //命令字

							 SregLen=0x00;           // 信息长度
                             UpReg[Up_wr++]=0x00;    // Hi
                             UpReg[Up_wr++]=39;      // Lo
                             
                             UpReg[Up_wr++]=0x00;    // 保留字 
                             
                             
                             memcpy(UpReg+Up_wr,JT808Conf_struct.Driver_Info.DriverCard_ID,18); //信息内容
                             Up_wr+=18;
							 memcpy(UpReg+Up_wr,JT808Conf_struct.Driver_Info.DriveName,21);
							 Up_wr+=21;
							 
			                 break;
		case  A_Up_RTC      :      //  采集记录仪的实时时钟
                             UpReg[Up_wr++]=In_Type; //命令字

							 SregLen=0x00;           // 信息长度
                             UpReg[Up_wr++]=0x00;    // Hi
                             UpReg[Up_wr++]=6;      // Lo
                             
                             UpReg[Up_wr++]=0x00;    // 保留字
                             
                             Time2BCD(UpReg+Up_wr);   //信息内容  
                             Up_wr+=6;
                             break;
		case  A_Up_Dist     :    //  采集里程
                             UpReg[Up_wr++]=In_Type; //命令字

							 SregLen=0x00;           // 信息长度
                             UpReg[Up_wr++]=0x00;    // Hi
                             UpReg[Up_wr++]=16;      // Lo
                             
                             UpReg[Up_wr++]=0x00;    // 保留字
                             //   信息内容
                             Time2BCD(UpReg+Up_wr);
							 Up_wr+=6;
                             memcpy(UpReg+Up_wr,(u8*)JT808Conf_struct.FirstSetupDate,6);
							 Up_wr+=6;
							 regdis=JT808Conf_struct.Distance_m_u32/100;  //单位0.1km 
							 reg2=regdis/10000000;
							 UpReg[Up_wr++]=(reg2<<4)+((regdis%10000000)/1000000);
							 UpReg[Up_wr++]=((regdis%1000000/100000)<<4)+(regdis%100000/10000);
							 UpReg[Up_wr++]=((regdis%10000/1000)<<4)+(regdis%1000/100);
							 UpReg[Up_wr++]=((regdis%100/10)<<4)+(regdis%10); 
							 

		case  A_Up_PLUS     :    //  采集记录仪速度脉冲系数
                             UpReg[Up_wr++]=In_Type; //命令字

							 SregLen=0x00;           // 信息长度
                             UpReg[Up_wr++]=0x00;    // Hi
                             UpReg[Up_wr++]=10;      // Lo
                             
                             UpReg[Up_wr++]=0x00;    // 保留字

							 //  信息内容
							 Time2BCD(UpReg+Up_wr);
							 Up_wr+=6;
                             UpReg[Up_wr++]=(u8)(JT808Conf_struct.Vech_Character_Value<<24);
							 UpReg[Up_wr++]=(u8)(JT808Conf_struct.Vech_Character_Value<<16);
							 UpReg[Up_wr++]=(u8)(JT808Conf_struct.Vech_Character_Value<<8);
							 UpReg[Up_wr++]=(u8)(JT808Conf_struct.Vech_Character_Value); 
							 
                             break;
		case  A_Up_VechInfo :    //  车辆信息
		                     UpReg[Up_wr++]=In_Type; //命令字

							 SregLen=0x00;           // 信息长度
                             UpReg[Up_wr++]=0x00;    // Hi
                             UpReg[Up_wr++]=41;      // Lo
                             
                             UpReg[Up_wr++]=0x00;    // 保留字                              
                             
                             memcpy(UpReg+Up_wr,JT808Conf_struct.Vechicle_Info.Vech_VIN,17); //信息内容
                             Up_wr+=17;
							 memcpy(UpReg+Up_wr,JT808Conf_struct.Vechicle_Info.Vech_Num,12);
                             Up_wr+=12;
							 memcpy(UpReg+Up_wr,JT808Conf_struct.Vechicle_Info.Vech_Type,12);  
                             Up_wr+=12;

		                     break;
		case  A_Up_AvrgMin  :      //  每分钟平均速度记录      // 默认填写最新7分钟记录
                                       UpReg[Up_wr++]=In_Type; //命令字
			
									   SregLen=455;		   // 信息长度
									   UpReg[Up_wr++]=(u8)(SregLen>>8);    // Hi
									   UpReg[Up_wr++]=(u8)SregLen;		   // Lo	65x7    
									   
									   UpReg[Up_wr++]=0x00;    // 保留字	 
									   //-----------  信息内容  --------------		
									       //----------------------
										  QueryRecNum=Api_DFdirectory_Query(spdpermin,0);   //查询当前疲劳驾驶记录数目
										 if(QueryRecNum>7)								   
										        QueryRecNum=7;			
								
										    SregLen=QueryRecNum*65;     // 改写信息长度
			                                                      UpReg[Up_wr-3]=(u8)(SregLen>>8);	  // Hi
			                                                      UpReg[Up_wr-2]=(u8)SregLen;	  // Lo    65x7    

																  
										 for(i=0;i<QueryRecNum;i++)			   // 从最新处读取存储填写
										  {
											 Api_DFdirectory_Read(spdpermin,Reg,70,0,i); // 从new-->old  读取
											  memcpy(UpReg+Up_wr,Reg+5,60);	// 只填写速度
											Up_wr+=65;	    
										  }
									       //------------------------------
									   
		                     break; 
		case  A_Up_Tired    :     //  疲劳驾驶记录
		                               UpReg[Up_wr++]=In_Type; //命令字
			
									   SregLen=180;		   // 信息长度
									   UpReg[Up_wr++]=(u8)(SregLen>>8);    // Hi
									   UpReg[Up_wr++]=(u8)SregLen;		   // Lo	30x6   
									   
									   UpReg[Up_wr++]=0x00;    // 保留字	   


									   //----------------------------------									    
                                                                   	QueryRecNum=Api_DFdirectory_Query(tired_warn,0);   //查询当前疲劳驾驶记录数目
										 if(QueryRecNum>6)								   
										        QueryRecNum=6;		
										 
                                                                  SregLen=30*QueryRecNum;		   // 信息长度
									   UpReg[Up_wr-3]=(u8)(SregLen>>8);    // Hi
									   UpReg[Up_wr-2]=(u8)SregLen;		   // Lo	65x7  
										 
										 for(i=0;i<QueryRecNum;i++)			   // 从最新处读取存储填写
										  {
											 Api_DFdirectory_Read(tired_warn,Reg,31,0,i); // 从new-->old  读取
                                                                              memcpy(UpReg+Up_wr,Reg,30); 
											 Up_wr+=30; 	    
										  }
									  //----------------------------------	 
                                                                    

		                     break;
		//------------------------ 下传数据相关命令 ---------------------
	    case  A_Dn_DrvInfo  :       //  设置车辆信息
                              
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
                           
		case  A_Dn_RTC      :        //  设置记录仪时间
		                    
		                     UpReg[Up_wr++]=In_Type; //命令字

							                    // 信息长度
                             UpReg[Up_wr++]=0x00;    // Hi
                             UpReg[Up_wr++]=0;      // Lo   20x8
                             
                             UpReg[Up_wr++]=0x00;    // 保留字  

		                     break;
		case  A_Dn_Plus     :        //  设置速度脉冲系数
                             
						//	 JT808Conf_struct.Vech_Character_Value=((u32)(*InStr)<<24)+((u32)(*InStr+1)<<16)+((u32)(*InStr+2)<<8)+(u32)(*InStr+3); // 特征系数	速度脉冲系数
						//	 Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));  
                             //-------------------------------------------------------------  
							 UpReg[Up_wr++]=In_Type; //命令字
							                    // 信息长度
                             UpReg[Up_wr++]=0x00;    // Hi
                             UpReg[Up_wr++]=0;      // Lo   20x8                             
                             UpReg[Up_wr++]=0x00;    // 保留字   
		                     break;
		default :
                             rt_kprintf("Error:   Device Type Error! \r\n");
			                 return; 
							 
  	}
  //---------------  填写计算 A 协议  Serial Data   校验位  -------------------------------------
  Sfcs=0;						//	计算S校验 从OxAA 开始 
  for(i=Swr;i<Up_wr;i++)
	 Sfcs^=UpReg[i];
  UpReg[Up_wr++]=Sfcs;			 // 填写FCS  
  //-----------------------------------------------------------------------------
  if(TransType)  //-------通过  GPRS  
  {
    
    //----stuff Ginfolen ----
    UpReg[Greglen]=(u8)((Up_wr-2)<<8);  //长度不算头 
	UpReg[Greglen+1]=(u8)(Up_wr-2);   
	
    Gfcs=0;                 //  计算从电话号码开始到校验前数据的异或和  G 协议校验 
	for(i=3;i<Up_wr;i++)
		Gfcs^=UpReg[i];
	
    UpReg[Up_wr++]=Gfcs;  // 填写G校验位

	
    UpReg[Up_wr++]=0x0D;  // G 协议尾
	UpReg[Up_wr++]=0x0A;
    //=================================================================================================
    //---------通过 GPRS 发送  --------------    
	GPRS_infoWr_Tx=0;	//--------------  clear  --------- 
	//-----------------------------------------------------------------------
	memcpy(GPRS_info+GPRS_infoWr_Tx,UpReg,Up_wr); // 拷贝内容到发送缓存区
	GPRS_infoWr_Tx+=Up_wr; 	
	//-----------  Add for Debug ---------------------------------
#if  0   	
	 memset(UDP_AsciiTx,0,sizeof(UDP_AsciiTx)); 	 
	 strcat((char*)UDP_AsciiTx,"AT%IPSENDX=1,\"");
	 packet_len=14;//strlen((const char*)UDP_AsciiTx);	
	 UDP_AsciiTx_len=HextoAscii(GPRS_info, GPRS_infoWr_Tx,UDP_AsciiTx+packet_len);
	
	 packet_len+=UDP_AsciiTx_len;	 
	 strcat((char*)UDP_AsciiTx,"\"\r\n");  
 
	  以后再说吧
	 if(DispContent==2)
		Uart1_PutData((uint8_t *) UDP_AsciiTx, packet_len+3);		
	 GSM_PutData((uint8_t*)UDP_AsciiTx,packet_len+3);  
	 rt_kprintf("\r\n	SEND DriveRecord -UP-Data! \r\n");  
#endif	
  }  
  else	 
  {                 //-----  通过串口输出
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
   	  //  上行
      case 0x00:  // 采集行车记录仪执行标准版本号
                 Adata_ACKflag.A_Flag__Up_Ver_00H=0xff;
				 break;
	  case 0x01:  // 采集当前驾驶人信息
	             Adata_ACKflag.A_Flag_Up_DrvInfo_01H=instr[2];
				 break;
	  case 0x02:  // 采集记录仪的实时时钟
	             Adata_ACKflag.A_Flag_Up_RTC_02H=instr[2];
				 break;
	  case 0x03:  // 采集行驶里程
	             Adata_ACKflag.A_Flag_Up_Dist_03H=instr[2];
				 break;
	  case 0x04:  // 采集记录仪速度脉冲系数
	             Adata_ACKflag.A_Flag_Up_PLUS_04H=instr[2];
				 break;
	  case 0x06:  // 采集车辆信息
	             Adata_ACKflag.A_Flag_Up_VechInfo_06H=instr[2];
				 break;
	  case 0x08:  // 采集记录仪状态信号配置信息
	             Adata_ACKflag.A_Flag_Up_SetInfo_08H=instr[2];
				 break;
	  case 0x16:  // 采集记录仪唯一编号
	             Adata_ACKflag.A_Flag_Up_DevID_16H=instr[2];
				 break;
	  case 0x09:  // 采集指定的每秒钟平均速度记录            
	             Adata_ACKflag.A_Flag_Up_AvrgSec_09H=instr[2];  // 有起始结束时间
	  	         break;
	  case 0x05: // 采集指定的每分钟平均速度记录
	             Adata_ACKflag.A_Flag_Up_AvrgMin_05H=instr[2]; // 有起始结束时间
	             break;
	  case 0x13: // 采集指定的位置信息记录
	             Adata_ACKflag.A_Flag_Up_Posit_13H=instr[2];
				 break;
	  case 0x07: // 采集事故疑点记录
	             Adata_ACKflag.A_Flag_Up_Doubt_07H=instr[2];  // 有起始结束时间
	             break;
	  case 0x11: // 采集指定的疲劳驾驶记录
	             Adata_ACKflag.A_Flag_Up_Tired_11H=instr[2]; // 有起始结束时间
				 break;
	  case 0x10: // 采集指定的登录退出记录
	             Adata_ACKflag.A_Flag_Up_LogIn_10H=instr[2]; // 有起始结束时间
	             break;
	  case 0x14: // 采集指定的记录仪外部供电记录
	            Adata_ACKflag.A_Flag_Up_Powercut_14H=instr[2]; // 有起始结束时间
				break;
	  case 0x15: // 采集指定的记录仪参数修改记录
	            Adata_ACKflag.A_Flag_Up_SetMdfy_15H=instr[2];
				break;
	  //  下行
	  case 0x82: // 设置车辆信息                 
				    memset(JT808Conf_struct.Vechicle_Info.Vech_VIN,0,sizeof(JT808Conf_struct.Vechicle_Info.Vech_VIN));
				    memset(JT808Conf_struct.Vechicle_Info.Vech_Num,0,sizeof(JT808Conf_struct.Vechicle_Info.Vech_Num));
				    memset(JT808Conf_struct.Vechicle_Info.Vech_Type,0,sizeof(JT808Conf_struct.Vechicle_Info.Vech_Type)); 	
                 
					 //-----------------------------------------------------------------------
					 memcpy(JT808Conf_struct.Vechicle_Info.Vech_VIN,instr,17);
					 memcpy(JT808Conf_struct.Vechicle_Info.Vech_Num,instr+17,12);
					 memcpy(JT808Conf_struct.Vechicle_Info.Vech_Type,instr+29,12); 
					 
			
				 	Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   

                 Adata_ACKflag.A_Flag_Dn_DrvInfo_82H=instr[2];
				    
					Settingchg_Status=0x82;//设置车辆信息 					
					NandsaveFlg.Setting_SaveFlag=1;  //存储参数修改记录
	  case 0x83:  // 设置初次安装日期
                             memcpy(JT808Conf_struct.FirstSetupDate,instr,6); 
				  Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
                 Adata_ACKflag.A_Flag_Dn_SetupDate_83H=instr[2];
				 Settingchg_Status=0x83;//设置车辆信息 				 
				 NandsaveFlg.Setting_SaveFlag=1;  //存储参数修改记录
				 break;
	  case 0x84:  // 设置状态量信息	
	             Settingchg_Status=0x84;//设置车辆信息 	             
				 NandsaveFlg.Setting_SaveFlag=1;  //存储参数修改记录
	             break;
	  case 0xc2: // 设置记录仪时钟
                 Adata_ACKflag.A_Flag_Dn_RTC_C2H=instr[2];
				 Settingchg_Status=0xc2;//设置车辆信息 				 
				 NandsaveFlg.Setting_SaveFlag=1;  //存储参数修改记录
				 break;
	  case 0xc3: // 设置记录仪速度脉冲系数                 
				  JT808Conf_struct.Vech_Character_Value=(u32)(instr[0]<<24)+(u32)(instr[1]<<16)+(u32)(instr[2]<<8)+(u32)(instr[3]); // 特征系数	速度脉冲系数
				  Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
		         Adata_ACKflag.A_Flag_Dn_Plus_C3H=instr[2];   
				 Settingchg_Status=0xc3;//设置车辆信息  				 
				 NandsaveFlg.Setting_SaveFlag=1;  //存储参数修改记录
				 break;
	  default:

		         break;
				 
				 	


   	}
}

u8 Send_Device_Data(void)  //  给行车记录仪发送数据
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
			   Device_Data(0x00,1);	   // 采集行车记录仪执行标准版本号 命令为0x00 置位时填写0xFF,所以单独处理
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
	regstr[0]=0x55; // 协议头    
    regstr[1]=0x7A;
	regstr[2]=0xFE; // 命令字 ，用预留命令字表示调试输出	
	//  3,4 为长度字节最后填写
	regstr[5]=0x00; // 备用字  0x00

	reglen= vsprintf((char*)regstr+6, fmt, args);
	va_end(args); 
	regstr[3]=(u8)(reglen>>8);  // 填写长度  ，长度为信息内容的长度 
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

void SpeedWarnJudge(void)  //  速度报警判断 
{
  					 //--------  速度报警  -------	 
					 if(  JT808Conf_struct.Speed_warn_MAX >0 )   //> 0 
					 {
			            
						 //----- GPS  即时速度	0.1 km/h  ---------
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
														 //PositionsSD_Enable();	  //  回报GPS 信息	 

														 StatusReg_SPD_WARN(); //  超速报警状态
														 rt_kprintf("\r\n  超速报警\r\n"); 
													 } 
													 //---------------------------------------------
													 Time2BCD(speed_Exd.ex_startTime); //记录超速报警起始时间
													 if(speed_Exd.current_maxSpd<GPS_speed) //找最大速度
														  speed_Exd.current_maxSpd=GPS_speed;
													 speed_Exd.excd_status=1;
													 speed_Exd.dur_seconds++;		
			 
												   //---------------------------------------------- 
											 }
											 
											 if(speed_Exd.excd_status==1) // 使能flag 后开始计时 
											  {
												 speed_Exd.dur_seconds++; 
												 if(speed_Exd.current_maxSpd<GPS_speed) //找最大速度
														  speed_Exd.current_maxSpd=GPS_speed;
											   } 
									 
							 }
							 else
							 { 
							              StatusReg_SPD_NORMAL(); //  清除速度报警状态寄存器  
										  
										  if(speed_Exd.excd_status!=2)
										  {
										    StatusReg_SPD_NORMAL(); //  清除速度报警状态寄存器
											speed_Exd.dur_seconds = 0;
											speed_Exd.speed_flag = 0; 	
										  } 											   
											//----------------------------------------------
											if(speed_Exd.excd_status==1)  
		                                    {		
											    Time2BCD(speed_Exd.ex_endTime); //记录超速报警结束时间 
												speed_Exd.excd_status=2;
							 	            }
											else
											if(speed_Exd.excd_status==0)
												Spd_ExpInit();
											//---------------------------------------------- 
							 }
					  }//------- 速度报警 over	 ---


}


u16  Protocol_808_Encode(u8 *Dest,u8 *Src, u16 srclen)
{
  u16  lencnt=0,destcnt=0;

  for(lencnt=0;lencnt<srclen;lencnt++)
  {
     if(Src[lencnt]==0x7e)  // 7e 转义
     {
       Dest[destcnt++]=0x7d;
	   Dest[destcnt++]=0x02;
     }
     else
	 if(Src[lencnt]==0x7d) //  7d  转义 	
	 {
       Dest[destcnt++]=0x7d;
	   Dest[destcnt++]=0x01; 
	 }
	 else
        Dest[destcnt++]=Src[lencnt]; // 原始信息
  }

  return destcnt; //返回转义后的长度

}
//-------------------------------------------------------------------------------
void Protocol_808_Decode(void)  // 解析指定buffer :  UDP_HEX_Rx  
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
//---------  拐点补传测试程序  ---------------------------
//#if 0
/*
void Inflexion_Process(void)
{            //
  u16  once_delta=0;


	Inflexion_Current=GPS_direction;  //  update new
   //----------------------------------------------------------------------- 
    if(Inflexion_Current>Inflexion_Bak)   // 初步判断大小  
    	{  // 增大
            if((Inflexion_Current-Inflexion_Bak)>300)  // 判断是否倒置减小
			{   //  如果差值大于300 ，说明是小于
			     once_delta=Inflexion_Bak+360-Inflexion_Current;  //判断差值绝对值
			     InflexDelta_Accumulate+=once_delta;
				 if((once_delta>=15)&&(once_delta<=60)) // 角度最小变化率不得小于15度  拐点补传角度不大于180 要求连续3s  所以每秒不大于60
				 {
					    if((Inflexion_chgcnter>0)&&(InflexLarge_or_Small==2))  //判断之前是否一直是小于
					    	{
		                        Inflexion_chgcnter++;
								if((InflexDelta_Accumulate>=3)&&(InflexDelta_Accumulate>JT808Conf_struct.DURATION.SD_Delta_maxAngle)&&(InflexDelta_Accumulate<=180))
									{     //要求至少持续3s	累计拐角补传角度不得大于180
                                           InflexLarge_or_Small=0;
										   Inflexion_chgcnter=0;
										   InflexDelta_Accumulate=0;
										   PositionSD_Enable=1; // 发送拐点标志位
										   rt_kprintf("\r\n 拐点上报 --1\r\n");
									}
								else
								  InflexLarge_or_Small=2; // 这次是小于
					    	}
						else
							{
							   InflexLarge_or_Small=2;  // 这是第一次小于 
							   Inflexion_chgcnter=1;
							   InflexDelta_Accumulate=once_delta;
							}
				 }
				 else
				 {    //  小于 15 就算等于
				   InflexLarge_or_Small=0;
				   Inflexion_chgcnter=0;
				   InflexDelta_Accumulate=0;
				 }
				 	
            } 
			else		// current真真正正的比Bak 大
			{  
			   once_delta=Inflexion_Current-Inflexion_Bak;  //判断差值绝对值
			   InflexDelta_Accumulate+=once_delta;
			   if((once_delta>=15)&&(once_delta<=60)) // 角度最小变化率不得小于15度  拐点补传角度不大于180 要求连续3s  所以每秒不大于60
			   {
	               if((Inflexion_chgcnter>0)&&(InflexLarge_or_Small==1)) //判断之前是否一直大于
				   {
				       Inflexion_chgcnter++;					   
					   if((InflexDelta_Accumulate>=3)&&(InflexDelta_Accumulate>JT808Conf_struct.DURATION.SD_Delta_maxAngle)&&(InflexDelta_Accumulate<=180))
						   {	 //要求至少持续3s	累计拐角补传角度不得大于180
								  InflexLarge_or_Small=0;
								  Inflexion_chgcnter=0;
								  InflexDelta_Accumulate=0;
								  PositionSD_Enable(); // 发送拐点标志位
								  rt_kprintf("\r\n 拐点上报 --2\r\n");
						   }
					   else
					     InflexLarge_or_Small=1; // 这次是大于
	               }	
				   else
				   	{  
                       InflexLarge_or_Small=1;  // 这是第一次大于
					   Inflexion_chgcnter=1;
					   InflexDelta_Accumulate=once_delta; 
				   	}
			   }
			    else
				 {     // 小于15度就算等于
				   InflexLarge_or_Small=0;
				   Inflexion_chgcnter=0;
				   InflexDelta_Accumulate=0;
				 }

			}  
    	}
	else
	 if(Inflexion_Current<Inflexion_Bak)	
	 	{  // 减小
               if((Inflexion_Bak-Inflexion_Current)>300)  // 判断是否倒置增大
               { //  如果差值大于300 ，说明是大于
                  once_delta=Inflexion_Current+360-Inflexion_Bak;  //判断差值绝对值
			      InflexDelta_Accumulate+=once_delta;
				  if((once_delta>=15)&&(once_delta<=60)) // 角度最小变化率不得小于15度	拐点补传角度不大于180 要求连续3s  所以每秒不大于60
                  {   // 最小变化率 不小于 15
                     if((Inflexion_chgcnter>0)&&(InflexLarge_or_Small==1))  //判断之前是否一直是大于
					    	{
		                        Inflexion_chgcnter++;
								if((InflexDelta_Accumulate>=3)&&(InflexDelta_Accumulate>JT808Conf_struct.DURATION.SD_Delta_maxAngle)&&(InflexDelta_Accumulate<=180))
									{     //要求至少持续3s	累计拐角补传角度不得大于180
                                           InflexLarge_or_Small=0;
										   Inflexion_chgcnter=0;
										   InflexDelta_Accumulate=0;
										   PositionSD_Enable(); // 发送拐点标志位
										   rt_kprintf("\r\n 拐点上报 --3\r\n");
									}
								else
								  InflexLarge_or_Small=1; // 这次是大于
					    	}
						else
							{
							   InflexLarge_or_Small=1;  // 这是第一次大于 
							   Inflexion_chgcnter=1;
							   InflexDelta_Accumulate=once_delta;
							}

                  }
				  else
				  {    //  小于 15 就算等于
				   InflexLarge_or_Small=0;
				   Inflexion_chgcnter=0;
				   InflexDelta_Accumulate=0;
				  }
			  }//---------------------------
			   else 	   // current 真真正正的比Bak 小
			   {
				  once_delta=Inflexion_Bak-Inflexion_Current;  //判断差值绝对值
				  InflexDelta_Accumulate+=once_delta;
				  if((once_delta>=15)&&(once_delta<=60)) // 角度最小变化率不得小于15度	拐点补传角度不大于180 要求连续3s  所以每秒不大于60
				  {
					  if((Inflexion_chgcnter>0)&&(InflexLarge_or_Small==2)) //判断之前是否一直小于
					  {
						  Inflexion_chgcnter++; 					  
						  if((InflexDelta_Accumulate>=3)&&(InflexDelta_Accumulate>JT808Conf_struct.DURATION.SD_Delta_maxAngle)&&(InflexDelta_Accumulate<=180))
							  { 	//要求至少持续3s	累计拐角补传角度不得大于180
									 InflexLarge_or_Small=0;
									 Inflexion_chgcnter=0;
									 InflexDelta_Accumulate=0;
									 PositionSD_Enable(); // 发送拐点标志位
									 rt_kprintf("\r\n 拐点上报 --4\r\n"); 
							  }
						  else
						   InflexLarge_or_Small=2; // 这次是小于
					  }    
					  else
					   {  
						  InflexLarge_or_Small=2;  // 这是第一次小于
						  Inflexion_chgcnter=1;
						  InflexDelta_Accumulate=once_delta; 
					   }
				  }
				   else
					{	  // 小于15度就算等于
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
			  if(SleepCounter>15)   // 防抖处理
			 {
			      SleepCounter=0; 
			   	   SleepState=1;		
				   if(JT808Conf_struct.RT_LOCK.Lock_state!=1)
				   Current_SD_Duration=JT808Conf_struct.DURATION.Sleep_Dur; // 5分钟 
				   //JT808Conf_struct.DURATION.Heart_SDCnter=25; 
				   //JT808Conf_struct.DURATION.Heart_Dur=320; 
				  /*   天地通要求休眠不重新计时  ，不  
				    memcpy(BakTime,CurrentTime,3); // update  //gps开始计时 
				    systemTick_trigGPS_counter=0; // 系统定时 清除
				  */  
				   if(DataLink_Status())
	                                PositionSD_Enable();  //  在线就发送    河北天地通要求数据完整 
				    rt_kprintf("\r\n 进入休眠状态! \r\n");          
			 }  
		 }
}

void  Sleep_Mode_ConfigExit(void)
{
    if(SleepState==1)
		   	{
		   	   SleepState=0;
			   rt_kprintf("\r\n 车台唤醒! \r\n");      
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
	//  2. Wave 文件 大小  小端模式 
	Filesize=0x24+(inFilesize<<3); // 乘以16 加 36 wave 文件大小 
	rt_kprintf("\r\n .wav 文件大小: %d Rawdata: %d \r\n ",Filesize,(inFilesize<<3)); 
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
	//  7. NumChannels  通道数
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
	Filesize=(inFilesize<<3); // 乘以16 加 36 wave 文件大小 
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
       纬度没有差值    1纬度  111km
       40度纬度上 1经度为  85.3km   (北京地区)
   */
   u8 i=0;
   u32 Latitude=0,Longitude=0;
   u32 DeltaLatiDis=0,DeltaLongiDis=0,CacuDist=0;
   u8  InOutState=0;   //   0 表示 in   1  表示Out

   //  1. get value
      Latitude=(LatiStr[0]<<24)+(LatiStr[1]<<16)+(LatiStr[2]<<8)+LatiStr[3];
      Longitude=(LongiStr[0]<<24)+(LongiStr[1]<<16)+(LongiStr[2]<<8)+LongiStr[3]; 


 
  for(i=0;i<8;i++)
  {
         InOutState=0; 
		 memset((u8*)&Rail_Cycle,0,sizeof(Rail_Cycle));
		Api_RecordNum_Read(Rail_cycle,i+1, (u8*)&Rail_Cycle,sizeof(Rail_Cycle)); 	 
		// rt_kprintf("\r\n\r\n 圆形围栏 有效状态:%d  TYPE: %d    atrri=%d  lati: %d  longiti:%d  radicus:%d  maxspd: %d  keepdur:%d \r\n",Rail_Cycle.Effective_flag,Rail_Cycle.Area_ID,Rail_Cycle.Area_attribute,Rail_Cycle.Center_Latitude,Rail_Cycle.Center_Longitude,Rail_Cycle.Radius,Rail_Cycle.MaxSpd,Rail_Cycle.KeepDur);  
         

      if(Rail_Cycle.Effective_flag==1) 
      {
         
			 DeltaLatiDis=abs(Latitude-Rail_Cycle.Center_Latitude)/9; //  a/1000000*111000=a/9.009	除以一百万得到度数 再乘以 111000 米得到实际距离 
             
			 DeltaLongiDis=abs(Longitude-Rail_Cycle.Center_Longitude)*853/10000; // a/1000000*85300=a 853/10000 m 	  
			 
			 CacuDist=sqrt((DeltaLatiDis*DeltaLatiDis)+(DeltaLongiDis*DeltaLongiDis));  
             
			 rt_kprintf("\r\n  TemperLati  %d  TemperLongi	%d	  Centerlati %d  center longi %d\r\n",Latitude,Longitude,Rail_Cycle.Center_Latitude,Rail_Cycle.Center_Longitude); 
			 rt_kprintf("\r\n  he=%d heng=%d   shu=%d   juli=%d\r\n",abs(Longitude-Rail_Cycle.Center_Longitude),DeltaLongiDis,DeltaLatiDis,CacuDist);   
			 
		   if(DeltaLatiDis>Rail_Cycle.Radius ) 
		   {  // 如果纬度距离大于 半径肯定出
		     InOutState=1;
		   }	 
		    else
		   {
			   DeltaLongiDis=abs(Longitude-Rail_Cycle.Center_Longitude)*853/10000; // a/1000000*85300=a 853/10000 m		
			   if(DeltaLongiDis>Rail_Cycle.Radius )	 
			   	{  // 如果经度距离大于半径肯定出
                   InOutState=1;
			   	}
			   else  //  计算两点见距离
			    CacuDist=sqrt((DeltaLatiDis*DeltaLatiDis)+(DeltaLongiDis*DeltaLongiDis));
		    } 


		     // 1. 判断属性
		     if(Rail_Cycle.Area_attribute &0x0001) //Bit 0 根据时间
		     {
                if(CurrentTime_Judge(Rail_Cycle.StartTimeBCD,Rail_Cycle.EndTimeBCD)==false)
				{
				  rt_kprintf("\r\n 时段没在区间内 \r\n");
				  return;
                }  
			   //continue;
		     }
			 if(Rail_Cycle.Area_attribute &0x0002) //Bit 1 限速
			 {
                if(GPS_speed>Rail_Cycle.MaxSpd) 
                	{
                          StatusReg_SPD_WARN(); //  超速报警状态
						  rt_kprintf("\r\n  设定围栏超速报警\r\n"); 
                	}
				else
					 StatusReg_SPD_NORMAL();
	           //continue;
			 }
	         if(Rail_Cycle.Area_attribute &0x0004) //Bit 2 进区域报警给驾驶员
	         {


	           //continue;
	         }
			 if(Rail_Cycle.Area_attribute &0x0008) //Bit 3 进区域报警给平台 
			 {
               if((InOutState==0)&&(CacuDist<Rail_Cycle.Radius )&&(Rail_Cycle.MaxSpd>(Speed_gps/10)))
               {
                  Warn_Status[1]|=0x10;// 进出区域报警
                  InOut_Object.TYPE=1;//圆形区域
                  InOut_Object.ID=i; //  ID
                  InOut_Object.InOutState=0;//  进报警	
                  rt_kprintf("\r\n -----圆形电子围栏--入报警");   
			      break;
               }			 
	           //continue;
			 }
			  if(Rail_Cycle.Area_attribute &0x0010) //Bit 4 出区域报警给司机
			 {
	           ;   
	           //continue;
			 }
			 if((Rail_Cycle.Area_attribute &0x0020)&&(Rail_Cycle.MaxSpd>(Speed_gps/10))) //Bit 5 出区域报警给平台 
			 {
			   if((InOutState==1)||(CacuDist>Rail_Cycle.Radius )) 
			   	{
			   	  Warn_Status[1]|=0x10;// 进出区域报警
                  InOut_Object.TYPE=1;//圆形区域
                  InOut_Object.ID=i; //  ID
                  InOut_Object.InOutState=1;//  出报警 
                  rt_kprintf("\r\n -----圆形电子围栏--出报警");        
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
	u8	InOutState=1;	//	 0 表示 in	 1	表示Out
	
  
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
		 
		//  rt_kprintf("\r\n\r\n 判断矩形形围栏 有效:%d ID: %d  atrri=%X  leftlati: %d  leftlongiti:%d    rightLati:%d   rightLongitu: %d	\r\n",Rail_Rectangle.Effective_flag,i+1,Rail_Rectangle.Area_attribute,Rail_Rectangle.LeftUp_Latitude,Rail_Rectangle.LeftUp_Longitude,Rail_Rectangle.RightDown_Latitude,Rail_Rectangle.RightDown_Longitude);	
            if((Latitude>Rail_Rectangle.RightDown_Latitude)&&(Latitude<Rail_Rectangle.LeftUp_Latitude)&&(Longitude>Rail_Rectangle.LeftUp_Longitude)&&(Longitude<Rail_Rectangle.RightDown_Longitude))
		       InOutState=0;
            
			//rt_kprintf("\r\n  TemperLati  %d  TemperLongi  %d   res %d\r\n",Latitude,Longitude,InOutState); 
		     
			// 1. 判断属性
			if(Rail_Rectangle.Area_attribute &0x0001) //Bit 0 根据时间
			{
			
			  //continue;
			}
			if(Rail_Rectangle.Area_attribute &0x0002) //Bit 1 限速
			{
			
			  //continue;
			}
			if(Rail_Rectangle.Area_attribute &0x0004) //Bit 2 进区域报警给驾驶员
			{
			
			
			  //continue;
			}
			if(Rail_Rectangle.Area_attribute &0x0008) //Bit 3 进区域报警给平台 
			{
			  if(InOutState==0)
			  {
				 Warn_Status[1]|=0x10;// 进出区域报警
				 InOut_Object.TYPE=2;//矩形区域
				 InOut_Object.ID=i; //	ID
				 InOut_Object.InOutState=0;//  进报警		
				 rt_kprintf("\r\n -----矩形电子围栏--入报警"); 
				 break;
			  } 			
			  //continue;
			}
			 if(Rail_Rectangle.Area_attribute &0x0010) //Bit 4 出区域报警给司机
			{
			
			
			 // continue;
			}
			if(Rail_Rectangle.Area_attribute &0x0020) //Bit 5 出区域报警给平台 
			{
			  if(InOutState==1)
			   {
				 Warn_Status[1]|=0x10;// 进出区域报警
				 InOut_Object.TYPE=2;//矩形区域
				 InOut_Object.ID=i; //	ID
				 InOut_Object.InOutState=1;//  出报警		
				 rt_kprintf("\r\n -----矩形电子围栏--出报警"); 
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
       纬度没有差值    1纬度  111km
       40度纬度上 1经度为  85.3km   (北京地区)
   */
     u8 i=0;
      u8 route_cout=0, seg_count=0,seg_num=0; 
      u32 Latitude=0,Longitude=0;
    // u32 DeltaLatiDis=0,DeltaLongiDis=0,CacuDist=0;
    // u8  InOutState=0;   //   0 表示 in   1  表示Out
     u32  Route_Status=0;   // 每个bit 表示 一个路线 偏航状态默认为0
     u32  Segment_Status=0;  //  当前线路中，对应端的偏航情况， 默认为0
     u32  Distance=0;
//     u8    InAreaJudge=0; //  判断是否在判断区域 bit 0 经度范围 bit  1 纬度范围
     u32  Distance_Array[6]; //存储当条线路的最小距离，默认是个大数值

       //  1. get value
	 Latitude=(LatiStr[0]<<24)+(LatiStr[1]<<16)+(LatiStr[2]<<8)+LatiStr[3];
	 Longitude=(LongiStr[0]<<24)+(LongiStr[1]<<16)+(LongiStr[2]<<8)+LongiStr[3]; 

	 // rt_kprintf("\r\n 当前---->  Latitude:   %d     Longitude: %d\r\n",Latitude,Longitude); 

      //  2.  Judge 
      for(route_cout=0;route_cout<Route_Mum;route_cout++)      // 读取路线
     { 
                 
	        // 2.1  --------   读取路线-----------	        
	        memset((u8*)&ROUTE_Obj,0,sizeof(ROUTE_Obj));  //  clear all  first        
	        DF_ReadFlash(DF_Route_Page+route_cout, 0,(u8*)&ROUTE_Obj, sizeof(ROUTE_Obj));	  
		 DF_delay_us(20);  
		//rt_kprintf("\r\n -----> ROUTE_Obj.RouteID:   %d \r\n",ROUTE_Obj.Route_ID);   
              // 2.2  -----  判断是否有效  -------
	       if((ROUTE_Obj.Effective_flag==1) &&(ROUTE_Obj.Points_Num>1)) //  判断是否有效且有拐点，若无效不处理
	      {     
	              // 2.2.0    当前段距离付给一个大的数值
	              for(i=0;i<6;i++)
				  	  Distance_Array[i]=ROUTE_DIS_Default;  
		      // 2.2.1      计算段数
                     seg_num=ROUTE_Obj.Points_Num-1; // 线路段数目
                    //  2.2.2    判断路线中每一段的状态
                     Segment_Status=0;  // 清除段判断状态，每个线路重新开始一次
			for(seg_count=0;seg_count<seg_num;seg_count++)
			{      
			          if((ROUTE_Obj.RoutePoints[seg_count+1].POINT_Latitude==0)&&(ROUTE_Obj.RoutePoints[seg_count+1].POINT_Longitude==0))
				      	{
				      	     rt_kprintf("\r\n  该点为0 ，jump\r\n"); 
					     continue;
				      	}
			      //----- 开始做距离计算, 在没在区域在函数里边做了判断
                           Distance_Array[seg_count]=Distance_Point2Line(Latitude, Longitude,ROUTE_Obj.RoutePoints[seg_count].POINT_Latitude,ROUTE_Obj.RoutePoints[seg_count].POINT_Longitude,ROUTE_Obj.RoutePoints[seg_count+1].POINT_Latitude,ROUTE_Obj.RoutePoints[seg_count+1].POINT_Longitude);   

			}
		//=========================================================	
              //  2.4 ------  打印显示距离，找出最小数值----
              Distance=Distance_Array[0];  // 最小距离  
               for(i=0;i<6;i++)
		   {
		       if(Distance>=Distance_Array[i])  
                                    Distance=Distance_Array[i];			   
		      // rt_kprintf("\r\n  Distance[%d]=%d",i,Distance_Array[i]);
                 }	 
		  rt_kprintf("\r\n MinDistance =%d  Width=%d \r\n",Distance,(ROUTE_Obj.RoutePoints[seg_num].Width>>1));	  // 

		if(Distance<ROUTE_DIS_Default)
	      {
		       //  ---- 和路段宽度做对比	   
	                 if(Distance>(ROUTE_Obj.RoutePoints[seg_num].Width>>1))
	                {
	                       rt_kprintf("\r\n 路线偏离\r\n");   
	                        Segment_Status|=(1<<seg_num);   //  把相应的bit  置位    
	                }  
		}

                     //                    
	      }  
		// 2.4  根据 2.2 结果判断当期路线状态
		if(Segment_Status)
			Route_Status|=(1<<route_cout);   //  把相应的bit  置位 
     }
     // 3.  Result
     		if(Route_Status) 
		 {
		    if( (Warn_Status[1]&0x80)==0)   //  如果以前没触发，那么及时上报
			{	  
                          PositionSD_Enable();  
				 Current_UDP_sd=1;   
		    	}
			
		    Warn_Status[1]|=0x80;// 路线偏航报警 
		    rt_kprintf("\r\n    路径偏航触发 !\r\n");           
		   
		}
		else
		{
		     if (Warn_Status[1]&0x80)   //  如果以前没触发，那么及时上报
			{	  
                             PositionSD_Enable();  
				 Current_UDP_sd=1;   
		    	}

		    Warn_Status[1]&=~0x80;// 路线偏航报警      
		}
	  

}

//--------  D点到直线距离计算-------
/*
     P1(x1,y1)   P2(x2,y2)  ,把点P(x1,y2)作为坐标原点，即x1=0，y2=0；

     那么两点P1，P2 确定的直线方程(两点式)为:
             (x-x1)/(x2-x1) =(y-y1)/(y2-y1)                          (1)
             
    注:  标准式直线方程为 AX+BY+C=0;
             那么平面上任意一点P(x0,y0) 到直线的距离表示为
             d=abs(Ax0+By0+C)/sqrt(A^2+B^2)

    其中把方程式(1) 转换成标准式为:
            (y2-y1)x+(x1-x2)y+x1(y1-y2)+y1(x2-x1)=0; 

   由于点(x1,y2)为原点  即x1=0，y2=0；  P1(0,y1) , P2(x2,0)
    所以   A=-y1 ,  B=-x2, C=y1x2
    那么 直线的方程:
                  -y1x-x2y+y1x2=0;  (2)  
 
  =>     d=abs(-y1x0-x2y0+y1x2)/sqrt(y1^2+x2^2)       (3)

         其中 (3)  为最终应用的公式 

        注:  先根据经纬度折合计算出 x0，y0，x1,y1,x2,y2  的数值单位为: 米     
=>  区域判断:
           根据(2) 可以求出  过 P1(0,y1) , P2(x2,0) 点与已知直线垂直的两条直线方程
              P1(0,y1) :      x2x-y1y+y1^2=0  (4)
              P2(x2,0) :      x2x-y1y-x2^2=0  (5)

          如果 y1 >=0      直线(4)    在直线(5)  的上边
          那么 点在线段区域内的判断方法是
                       (4) <=  0    且  (5)  >=0 
       另                
           如果 y1 <=0      直线(5)    在直线(4)  的上边
          那么 点在线段区域内的判断方法是
                       (4) >=  0    且  (5)  <=0 
   //------------------------------------------------------------------------------------------     
     
       纬度没有差值    1纬度  111km
       40度纬度上 1经度为  85.3km   (北京地区)   

       X 轴为 经度(longitude) 差值
       Y 轴为纬度 (latitude)  差值
                
     
   //------------------------------------------------------------------------------------------
*/

u32   Distance_Point2Line(u32 Cur_Lat, u32  Cur_Longi,u32 P1_Lat,u32 P1_Longi,u32 P2_Lat,u32 P2_Longi)
{   //   输入当前点 ，返回点到既有直线的距离
      long  x0=0,y0=0,Line4_Resualt=0,Line5_Resualt=0;  // 单位: 米
      long  y1=0; 
      long  x2=0;	
      long   distance=0;	 
     // long  Rabs=0;
//      long  Rsqrt=0;  
     long  DeltaA1=0,DeltaA2=0,DeltaO1=0,DeltaO2=0; //  DeltaA : Latitude     DeltaO:  Longitude 	   
     // u32   Line4_Resualt2=0,Line5_Resualt2=0; 
      double   fx0=0,fy0=0,fy1=0,fx2=0;	  
      double   FLine4_Resualt2=0,FLine5_Resualt2=0,fRabs=0,fRsqrt=0; 	  

	  // 0.   先粗略的判断 
	       DeltaA1=abs(Cur_Lat-P1_Lat); 
	       DeltaA2=abs(Cur_Lat-P2_Lat); 
		DeltaO1=abs(Cur_Lat-P1_Longi); 
	       DeltaO2=abs(Cur_Lat-P2_Longi); 
	   /* if((DeltaA1>1000000) &&(DeltaA2>1000000))	    
            {  
                rt_kprintf("\r\n  Latitude 差太大\r\n");
                return   ROUTE_DIS_Default; 
	    }	
	     if((DeltaO1>1000000) &&(DeltaO2>1000000))	    
            {  
                rt_kprintf("\r\n  Longitude 差太大\r\n"); 
                return   ROUTE_DIS_Default; 
	    }	  	
       */
	 // 1.  获取  P1(0,y1)   P2(x2,0) ,和P(x0,y0)    P(x1,y2)为原点  即x1=0，y2=0；  P1(0,y1) , P2(x2,0)
	  x2=abs(P2_Longi-P1_Longi); // a/1000000*85300=a 853/10000 m =a x 0.0853
         if(P2_Longi<P1_Longi)
		 	x2=0-x2;
	  fx2=(double)((double)x2/1000);
	 //rt_kprintf("\r\n P2_L=%d,P1_L=%d   delta=%d \r\n",P2_Longi,P1_Longi,(P2_Longi-P1_Longi));
	 // if(P2_Longi
         y1=abs(P2_Lat-P1_Lat); //  a/1000000*111000=a/9.009	除以一百万得到度数 再乘以 111000 米得到实际距离 
         if(P2_Lat<P1_Lat) 
		 	y1=0-y1;
	   fy1=(double)((double)y1/1000);	  
	  //rt_kprintf("\r\n P2_LA=%d,P1_LA=%d   delta=%d \r\n",P2_Lat,P1_Lat,(P2_Lat-P1_Lat));

      //   rt_kprintf("\r\n 已知两点坐标: P1(0,%d)   P2(%d,0) \r\n", y1,x2); 
       //    当前点
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
        //   rt_kprintf("\r\n当前点坐标: P0(%d,%d)    \r\n", x0,y0);  
	  // 2. 判断y1  的大小， 求出过 P1(0,y1)   P2(x2,0) ,和已知直线的方程，并判断
	  //     当前点是否在路段垂直范围内
	  
             //  2.1   将当前点带入， 过 P1(0,y1)   的 直线方程(4)  求出结果
                                  Line4_Resualt=(x2*x0)-(y1*y0)+(y1*y1);
	                            FLine4_Resualt2=fx2*fx0-fy1*fy0+fy1*fy1;
	   //     rt_kprintf("\r\n Line4=x2*x0-y1*y0+y1*y1=(%d)*(%d)-(%d)*(%d)+(%d)*(%d)=%ld     x2*x0=%d    y1*y0=%d   y1*y1=%d  \r\n",x2,x0,y1,y0,y1,y1,Line4_Resualt,x2*x0,y1*y0,y1*y1); 
          //     rt_kprintf("\r\n FLine4=fx2*fx0-fy1*fy0+fy1*fy1=(%f)*(%f)-(%f)*(%f)+(%f)*(%f)=%f      fx2*fx0=%f    fy1*fy0=%f   fy1*fy1=%f  \r\n",fx2,fx0,fy1,fy0,fy1,fy1,FLine4_Resualt2,fx2*fx0,fy1*fy0,fy1*fy1); 
  
	      //   2.2   将当前点带入， 过P2(x2,0) 的 直线方程(5)  求出结果 
                                  Line5_Resualt=(x2*x0)-y1*y0-x2*x2;
		                    FLine5_Resualt2=fx2*fx0-fy1*fy0-fx2*fx2;    
		//rt_kprintf("\r\n Line5=x2*x0-y1*y0-x2*x2=(%d)*(%d)-(%d)*(%d)-(%d)*(%d)=%ld     Se : %ld   \r\n",x2,x0,y1,y0,x2,x2,Line5_Resualt,Line5_Resualt2); 
          //    rt_kprintf("\r\n FLine5=fx2*fx0-fy1*fy0-fx2*fx2=(%f)*(%f)-(%f)*(%f)-(%f)*(%f)=%f      fx2*fx0=%f    fy1*fy0=%f   fx2*fx2=%f  \r\n",fx2,fx0,fy1,fy0,fx2,fx2,FLine5_Resualt2,fx2*fx0,fy1*fy0,fx2*fx2);   
 	      // rt_kprintf("\r\n  Line4_Resualt=%d     Line5_Resualt=%d  \r\n",Line4_Resualt,Line5_Resualt);    

	     if(fy1>=0)      //  直线(4) 在上发
	  	{
               
			//   2.3   判断区域    (4) <=  0    且  (5)  >=0     // 判断条件取反
			     if((FLine4_Resualt2>0) ||(FLine5_Resualt2<0))
				 	 return   ROUTE_DIS_Default;      //  不满足条件返回最大数值
	  	}
	     else
	     	{      //  直线(5)

                   	//   2.4   判断区域     (4) >=  0    且  (5)  <=0     // 判断条件取反 
			     if((FLine4_Resualt2<0) ||(FLine5_Resualt2>0)) 
				 	 return   ROUTE_DIS_Default;      //  不满足条件返回最大数值 

	     	}	

              rt_kprintf("\r\n In judge area \r\n");  
		//rt_kprintf("\r\n   Current== Latitude:   %d     Longitude: %d     Point1== Latitude:   %d     Longitude: %d     Point2== Latitude:   %d     Longitude: %d\r\n",Cur_Lat,Cur_Longi,P1_Lat,P1_Longi,P2_Lat,P2_Longi);   

        //  3. 将差值差算成实际距离
             #if 0
		    x2=x2*0.0853; // a/1000000*85300=a 853/10000 m =a x 0.0853
                 y1=y1/9; //  a/1000000*111000=a/9.009	除以一百万得到度数 再乘以 111000 米得到实际距离 
                 x0=x0*0.0853;     
		   y0=y0/9; //  a/1000000*111000=a/9.009	    
	      #else
		   fx2=fx2*0.0853; // a/1000000*85300=a 853/10000 m =a x 0.0853
                 fy1=fy1/9; //  a/1000000*111000=a/9.009	除以一百万得到度数 再乘以 111000 米得到实际距离 
                 fx0=fx0*0.0853;     
		   fy0=fy0/9; //  a/1000000*111000=a/9.009	      
             #endif

	 //  4. 计算距离 
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
		if(i==0)   //第一包
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
		else if(i==(num-1))  //最后一包
			{
            buffer_temp[0]=0;
	        buffer_temp[1]=50;
	        DF_ReadFlash(50,0,&buffer_temp[2],PageSIZE);
			FileTCB_CRC16=((unsigned short int)buffer_temp[512]<<8)+(unsigned short int)buffer_temp[513];
			crc_fcs=CRC16_1(buffer_temp,512,Last_crc);
			rt_kprintf("\r\ni=%d,j=%d,Last_crc=%x ReadCrc=%x ",i,j,crc_fcs,FileTCB_CRC16);
			}
		else  
			{             // 中间的包
			 j=i+51;
			 buffer_temp[0]=(char)(j>>8);
			 buffer_temp[1]=(char)j;
			 DF_ReadFlash(j,0,&buffer_temp[2],PageSIZE);
			 WatchDog_Feed(); 
			Last_crc=CRC16_1(buffer_temp,514,Last_crc);
			//rt_kprintf("\r\ni=%d,j=%d,Last_crc=%d",i,j,Last_crc);
			}
	   }
rt_kprintf("\r\n  校验结果 %x",crc_fcs);
return crc_fcs;
#endif
		  return 1;
}



//-------  JT808  Related   Save  Process---------
void  Save_Status(u8 year,u8 mon,u8 day,u8 hour,u8 min,u8 sec)
{   // 存储事故疑点 最近20s车辆的状态字，和当前最近的有效位置信息  

       
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
					 cur=save_sensorCounter;  //20s的事故疑点
					 for(j=0;j<100;j++)
					 {
                         Statustmp[wr_add++]=Sensor_buf[cur].DOUBTspeed;    //速度
						 Statustmp[wr_add++]=Sensor_buf[cur].DOUBTstatus;   //状态    
                         cur++;
						 if(cur>100)
						 	cur=0;                            
					 }           
					 //---------------------------------------------------------------------------------------
					 FCS = 0;  
					 for( j=0;j<wr_add;j++)    
					 {
						FCS ^=Statustmp[j];	            
					 }				//求上边数据的异或和      不算校验 信息长度为   206
					 //------------------------------------------------------------------------------
					 Statustmp[wr_add++]=FCS; 
					   //----------------------------------------------------------------------------- 	 
					   rt_kprintf("\r\n 存储行车记录  Record  write: %d    SaveLen: %d    \r\n",Recorder_write,wr_add); 
                                      Api_DFdirectory_Write(doubt_data,Statustmp, wr_add);

					   
}
//-----------------------------------------------------------------------------------
void Save_AvrgSpdPerMin(void) 
{ /*
     Note: 存储单位小时每分钟平均速度 
  */
  u8   content[70];
  u8   saveLen=0,FCS=0,i;   
    
	 saveLen=0; 
	 memset(content,0,70);
	 memcpy(content+saveLen,Avrgspd_Mint.datetime_Bak,5);    //用BAKtime
	 saveLen+=5;
	 memcpy(content+saveLen,Avrgspd_Mint.avgrspd,60);    //用BAKtime
	 saveLen+=60;  
  //---------------------------------------------------------------------------------------
  FCS = 0;
  for ( i = 0; i < 69; i++ ) 
  {
	 FCS ^=content[i];    
  } 			 //求上边数据的异或和   
  content[69] = FCS;	 // 第8字节	       
  //----------------------------------------------------------------------------------------  
  Api_DFdirectory_Write(spdpermin, content, 70); 
  //-----------------------------------------------------------------
  memset(Avrgspd_Mint.avgrspd,0,60); // 存储后清除记录     
   if(DispContent)
        rt_kprintf("\r\n   存储单位小时每分钟平均速度   Save Time %X-%X-%X %x:%x     \r\n",\
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
	   }			  //求上边数据的异或和
	   content[wr_add++] = FCS;	  // 第31字节 
	   
       Api_DFdirectory_Write(spd_warn, (u8 *)content, 32); 
	  //----------- debug -----------------------
	  rt_kprintf("\r\n 超速报警  %X-%X-%X %X:%X:%X,MaxSpd=%d\r\n",speed_Exd.ex_endTime[0],speed_Exd.ex_endTime[1],speed_Exd.ex_endTime[2],speed_Exd.ex_endTime[3],speed_Exd.ex_endTime[4],speed_Exd.ex_endTime[5],speed_Exd.current_maxSpd); 
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
	   }			  //求上边数据的异或和
	   content[wr_add++] = FCS;	  // 第31字节 
	   
       Api_DFdirectory_Write(tired_warn, (u8 *)content, 32); 
       //-----  clear status ---------------  
	  TIRED_Drive_Init();      
	  Status_TiredwhRst=0;
	 // DF_WriteFlashSector(DF_TiredStartTime_Page,0,(u8*)&Status_TiredwhRst,1); //清除O后要写入		
	   TIRED_Drive_Init(); //	休息了 
	   Warn_Status[3]&=~0x04;  //BIT(2)	疲劳驾驶  

}


void  JT808_Related_Save_Process(void)
{
       if(DF_LOCK)
	   	 return ;

	      if(Dev_Voice.CMD_Type!='1')  // 录音时不做以下处理  
      {           

		//-------------------------
		//    存储疑点数据
		if(1==NandsaveFlg.Doubt_SaveFlag) 
		{
			if(sensor_writeOverFlag==1)	
			{ 
			  time_now=Get_RTC();     //  RTC  相关 
			  Save_Status(time_now.year,time_now.month,time_now.day,time_now.hour,time_now.min,time_now.sec);
			  NandsaveFlg.Doubt_SaveFlag=0;
			  return;
			} 
		} 							 
		//  存储每分钟速度信息  
		if(1==Avrgspd_Mint.saveFlag) 
		{
			Save_AvrgSpdPerMin();
			Avrgspd_Mint.saveFlag=0; 
			return;
		}							  	  

		//-----------------  存储疲劳驾驶记录 ---------------			
		if( TiredConf_struct.Tired_drive.Tireddrv_status==2)   
		{
	             Save_TiredDrive_Record();
			return;	 
		}			
		//-----------------  超速报警 ----------------------
		if(speed_Exd.excd_status==2)
		{
		   Spd_Exp_Wr();   
		   return;
		}   
		 /*
		if(NandsaveFlg.Setting_SaveFlag==1)   // 参数 修改记录
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
	//	 定时存储里程
	if((Vehicle_RunStatus)&&((Systerm_Reset_counter&0xff)==0xff))
	{   //  如果车辆在行驶过程中，每255 秒存储一次里程数据    
                Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct)); 
		  return;		
	} 
			
	 //--------------------------------------------------------------

}

/*
    打印输出 HEX 信息，Descrip : 描述信息 ，instr :打印信息， inlen: 打印长度
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
  rt_kprintf("\r\n 手动设置上报时间间隔 %d s\r\n",Current_SD_Duration);
  
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
    rt_kprintf("手动发送:%s",SMS_Service.SMS_sd_Content);    
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
	 //车辆VIN
	memset(JT808Conf_struct.Vechicle_Info.Vech_VIN,0,sizeof(JT808Conf_struct.Vechicle_Info.Vech_VIN));
	memcpy(JT808Conf_struct.Vechicle_Info.Vech_VIN,instr,strlen(instr)); 
	Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
       rt_kprintf("\r\n 手动设置vin:%s \r\n",instr); 

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
