
//================================================================
/*         ��дAT45DB16������ͷ�ļ�
MSP430 IAR application builder : 2007-04-15 9:00:00
Target : MSP430F149
Crystal: 3.6864Mhz
*/
//================================================================
//#include "common.h" 
//#include "71x_type.h"

#ifndef _H_AT45
#define _H_AT45

#define    PageSIZE     512 

//================================================================
/* 
  Flash Chip : SSST25VF032B-50-4I-S2AF 
  ChipSize       : 4MBytes       PageSize(vitual): 512Bytes  SectorSize:4K<=>8 Pages    Chip: 1024Sectors<=>8192Pages
  
  Regulation :
                  

                    
<һ>   ϵͳ���� �Լ� Ӧ�ò���  �г���¼��ص�ַ�洢����
*/


/*               Dataflash Page  �滮   ------->    Start          */

 /*  0. Page 0~9     Producet Info */
 #define DF_ProInfo_Page      0

#define    ConfigStart_offset                         8        //   Block   ��ʼλ��  Conifg  Struct Save      Sector 1 
#define    JT808Start_offset                          16        //   Block   ��ʼλ��  Conifg  Struct Save    Sector 2
#define    TiredCondifg_offset                       24        //   Block   ��ʼλ��  Conifg  Struct Save      Sector 3 
 


 /*  1. page 10 -903     ISP   */
  
#define DF_BL_PageNo		                 40             /*DF_BL_RAM run PageNo:   10  ~ 49  page */
#define DF_APP1_PageNo		                 50             /*DF_APP_flah run PageNo:   50  ~ 903  page*/
                                                                                   /* 512K  -->1072 Page */ 
                                                                                          

 /*  2. Page  904 - 912           ״̬��Ϣ    */
 #define DF_Format_Page 	                     1072     // 904~911    Sector  114    (x8)=912


 /*  3. Page  920 - 943        	GPSӦ�� ���������    */   //���ٶ�д  
 #define  DF_socket_all                          1080     // Block��ʼ    socket 1 ,2, 3 
 #define  DF_APN_page			                 1088	 //
 #define  DF_ACC_ON_dur_Page                     1092     // ACC ��ʱ���ͼ��     
 #define  DF_ACC_OFF_dur_Page					 1093 	 // ACC ��ʱ���ͼ�� 		   
 #define  DF_TCP_sd_Dur_Page                     1094     // TCP ���ͼ��
 #define  DF_TrigSDStatus_Page                   1095     // �����������ϱ�״̬
 #define  DF_CycleAdd_Page                       1096     // Block ��ʼ-- ��¼ѭ���洢��дƫ�Ƶ�ַ��page
 #define  DF_PhotoAdd_Page                      1104     // Block ��ʼ--��¼��Ƭ�洢��дƫ�Ƶ�ַ��page


 /*  4. ��ͬ�ͻ���ƷӦ�����й��ܲ���           */
 #define  DF_DevConfirmCode_Page                 1112     // Block ��ʼ-- ����αIP
 #define  DF_ListenNum_Page                      1113     // ���ļ�������    
 #define  DF_Distance_Page                       1120     // Block ��ʼ-- �����ۼ���ʻ���
 #define  DF_LoadState_Page                      1128     // Block ��ʼ-- ��������״̬ 
 #define  DF_Speed_GetType_Page               1136     // Block ��ʼ--�洢�ٶȻ�ȡ��ʽ 1Ϊ �ٶȴ����� 0ΪGPS
 #define  DF_K_adjust_Page                          1144     // Block ��ʼ--�洢��ʶ�Ƿ�����ϵ���Ѿ����Զ�У׼   1.У׼��  0:��δУ׼
 #define  DF_ACCONFFcounter_Page              1152     // Block ��ʼ--�쳣��λʱ�洢ACCON_Off�ļ�����ֵ
 #define  DF_TiredStartTime_Page                 1160     // Block ��ʼ--ƣ�ͼ�ʻ�����󣬼�¼ƣ�ͼ�ʻ����ʼʱ�� 
                                                         /*
                                                                        Byte1 Flag :   0 :ͣ��  1:ͣ����û���� 2:�����˻�û����
                                                                        Byte2 TiredDrvStatus  Tired_drive.Tireddrv_status
                                                                        Byte3 On->off Flag
                                                                        Byte4~8: starttimeBCD 
                                                                  */ 
 #define  DF_OutGPS_Page                          1168    // Block ��ʼ -- ���ⲿGPS�ź�Դ״̬��־   
 #define  DF_BD_Extend_Page                    1176   //  ������չ

  //��������� �� 1023  ( ��1024Page)                                                         


 /*  5.�г���¼����ز���  */ 
#define       DF_VehicleID_Page                        1192                           // Block ��ʼ-���ƺ���
#define       DF_VehicleType_Page                      1200                           // Block ��ʼ-��������
#define       DF_PropertiValue_Page                    1208                           // Block ��ʼ-- ����ϵ��
#define       DF_DriverID_Page                         1216                           // Block ��ʼ--��ʻԱID ������
#define       DF_TiredDrvAdd_Page                      1224                           // Block ��ʼ--ƣ�ͼ�ʻ��ַ
#define       DF_ExpSpdAdd_Page                        1232                           // Block ��ʼ--���ٱ�����¼ƫ�Ƶ�ַ
#define       DF_AccFireRecAdd_Page                    1240                           // Block ��ʼ--��������¼��ַƫ��
#define       DF_AvrgSpdPerMinAdd_Page                 1248                           // Block ��ʼ--ÿ����ƽ���ٶ�
#define       DF_AbnormalLogAdd_Page                   1256                           // Block ��ʼ--�쳣Log�洢 
#define       DF_RecordAdd_Page                        1264                           // Block ��ʼ--�г���¼�������洢��¼ƫ�Ƶ�ַ 
#define       DF_MaxSpdPerDay_Page                     1272                           // Block ��ʼ--��������ٶ�
#define       DF_DayDistance_Page                      1280                           // Block ��ʼ--�������  
#define       DF_DoubtAdd_Page                         1288                           // Block �¹��ɵ����
#define       DF_AvrgSpdSec_Page                       1296                           // Block ��ʼ-ÿ����ƽ���ٶ�
#define       DF_Login_Page				               1304                           // Block ��ʼ-��¼��¼
#define       DF_Powercut_Page		                   1312                           // Block ��ʼ-�ⲿ��Դ�Ͽ�
#define       DF_Settingchg_Page	                   1320                           // Block ��ʼ-�����޸� 
#define       DF_Minpos_Page                           1328                           // Block ��ʼ-ÿ����λ�ô洢
#define       DF_StdVer_Page                           1336                            // Block ��ʼ-���ұ�׼�汾
#define       DF_1stDate_page                          1344                           // Block ��ʼ-���ΰ�װʱ�� 
#define       DF_DevOnlyID_Page                        1368                           // Block ��ʼ-��¼��Ψһ���
#define       DF_SpeedLimt_Page                        1376                           // Block ��ʼ-����ٶ���С�ٶ�
#define       DF_TiredDoor_Page                        1384                           // Block ��ʼ-ƣ�ͼ�ʻ����  
                                                                                       /*
                                                                                                       ������ʻʱ�䡢�����ۼƼ�ʻʱ�䡢��С��Ϣʱ�䡢�ͣ��ʱ��
                                                                                                      */
#define       DF_SDTime_Page                           1392                           // Block ��ʼ-��ʱ��ʽ���                
#define       DF_SDDistance_Page                       1400                           // Block ��ʼ-���෢�;���
#define       DF_SDMode_Page                           1408                           // Block ��ʼ- �ն����ݷ��ͷ�ʽ
#define       DF_RTLock_Page                           1416                           // Block ��ʼ- ʵʱ�ϱ� --
#define       DF_Event_Page                            1424                           // Block ��ʼ- �¼����  
#define       DF_Msg_Page                              1432                           // Block ��ʼ- ��Ϣ���  
#define       DF_PhoneBook_Page                        1440                           // Block ��ʼ- �绰�����
#define       DF_CircleRail_Page                       1448                           // Block ��ʼ- Բ��Χ��

#define       DF_RectangleRail_Page                    3000                           // Block ��ʼ- ����Χ��  1288  --���������7000 �� 24�� 

#define       DF_PolygenRail_Page                      1464                           // Block ��ʼ- �����Χ��
#define       DF_PicIndex_Page                         1480                           // Block ��ʼ- ͼ�����
#define       DF_SoundIndex_Page                       1488                           // Block ��ʼ- ��Ƶ����        
#define       DF_FlowNum_Page                          1496                           // Block ��ʼ- ��ˮ��
   
// 16  �ı���Ϣ
#define       TextStart_offdet                         1504

//17.  ����
#define       DF_DomainName_Page                        1512            

#define       DF_question_Page                         4000        //16m 4000  32m 4600                     // ������Ϣ�洢��ʼҳ               

    //���������     

/*                    
<��>   ѭ���洢�ϱ�  �г���¼����ع��� ���ݴ洢��
*/

/*  I.  Function App Area                  ע: ����Page �滮����   SST25VF16     ������ʱTest                   */

//                     Name                                     PageNum                	 	                     Description                             
// 1.  Cycle Save Send Area
#define       CycleStart_offset                       1768                          // ѭ���洢�ϱ��洢����(Basic �����ر�)        1 record=32 Bytes




// 2. Vehicle  Status Record Area 
#define       VehicleRecStart_offset                  2792                         // �г���¼�����ݴ洢����(�¹��ɵ�����)  1 record=256Bytes

// 3. Average Speed Per Minute
#define       AverageSpdStart_offset                  2992                         // ����ÿ����ƽ���ٶ�(Ҫ���¼����360h)          1 record =70 Bytes

// 4. Tired Driving Record
#define       TiredDrvStart_offset                    3392                          //  ƣ�ͼ�ʻ��¼��ʼƫ��

// 5. Exp  Speed  Record
#define       ExpSpdStart_offset                      3400                          //  ���ٱ���ƫ��

// 6. Average Minte position
#define      AvrgMintPosit_offset                     3408                         // ����ÿСʱ��ÿ����λ�ü�¼   1 record =512 Bytes


// 7. Average Speed Per Second
#define      AvrgSpdSec_offset                        3432                         // ����ÿ����ƽ���ٶȼ�¼       1 record =70 Bytes

// 8. Acc  Work  On  Record
#define       AccWorkOnStart_offset                 3512                          // ����¼

// 9. Abnormal  Record 
#define       AbNormalStart_offset                    3420                         // �豸�쳣Log�洢

// 10.  LogIn  Record
#define      LogIn_offset                             3528                         // �û���¼��¼

// 11. PowerCut Record
#define      PowerCut_offset                          3536                         // �ⲿ��Դ�Ͽ���¼


// 12. Setting Change Record
#define     SettingChg_offset                         3544                    // �����޸ļ�¼


// 13. Picture   Area
                                                     /* 
                                                                filename            cameraNum    size
                                                                   19                         1             4
                                                     */
#define       PicStart_offset                          4096                          // Block ��ʼλ�� ͼƬ�洢����(Current Save) ����Ҫ�ŵ�TF����
#define       PicStart_offset2                        4424                          // Block ��ʼλ�� ͼƬ2���� 
#define       PicStart_offset3                        4752                          // Block ��ʼλ�� ͼƬ3���� 
#define       PicStart_offset4                        5080                          // Block ��ʼλ�� ͼƬ4����  



// 14  Sound  Area
#define       SoundStart_offdet                      5248      //4200                 32K �ռ�        // Block ��ʼλ�� 15s�����洢����(Current Save) ����Ҫ�ŵ�TF����
			                                                                /*  
			                                                                             filesize              filename 
			                                                                                4  Bytes          5thstart   
			                                                                */
#define       SoundFileMax_Sectors                   5                              //  5 sect=5*8 pages =20s data



#define       DF_DeviceID_offset                      5400                 // Block ��ʼλ��   ����ID  12  λ BCD   


//  15 �ֿⲻ��Dataflash �˿�������



//----  ����
#define    DF_Broadcast_offset                      5300       //  Block   ��ʼλ��  ������ʼ��ַ
#define    DF_Route_Page                               5400      // 1304                           // Block ��ʼ- ·��
#define    DF_turnPoint_Page                         5500       //  �յ�
 #define   DF_AskQuestion_Page                    5600       //  ��������    
/*                Dataflash     <------------   End              */ 



//  =================  �г���¼�� ��� ============================
/*
    StartPage :    6320          Start Address offset :   0x316000       

    Area Size :
                          213   Sector       = 1704  pages
                           ----------------------
                           
				����               
				1                                      00-07H
				135                                   08H               
				64                                     09H  
				7                                      10H
				2                                      11H 
				2                                      12H
				1                                      13H
				1                                      14H  
				1                                      15H   

          ----------  ֻ������������---  ע�� ����������� Vdr.C 
				
*/



//-------------------------------------------------------


extern  u8   DF_LOCK;    //     Dataflash  Lock  


extern void	mDelaymS( u8 ms );
extern void DF_delay_us(u16 j);
extern void DF_delay_ms(u16 j);
extern void DF_ReadFlash(u16 page_counter,u16 page_offset,u8 *p,u16 length);
extern void DF_WriteFlash(u16 page_counter,u16 page_offset,u8 *p,u16 length);
extern void DF_ReadFlash(u16 page_counter,u16 page_offset,u8 *p,u16 length);
extern void DF_WriteFlashSector(u16 page_counter,u16 page_offset,u8 *p,u16 length);//512bytes ֱ�Ӵ洢
extern void DF_WriteFlashRemote(u16 page_counter,u16 page_offset,u8 *p,u16 length);//512bytes ֱ�Ӵ洢
extern void DF_WriteFlashDirect(u16 page_counter,u16 page_offset,u8 *p,u16 length);
extern void DF_Read_zk(u32 address,u8 *p,u16 length);//480 bytes ֱ�Ӷ�ȡ
extern void DF_EraseAppFile_Area(void);
extern void DF_ClearUpdate_Area(void);    // ���Զ�������������� 
extern void DF_init(void); 

#endif
