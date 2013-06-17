#ifndef Flash_Nand
#define  Flash_Nand
#include "App_moduleConfig.h"

//-----------------------------------------------------------------------------------------------------------------------------
//==============================================================================================================================
//   Add Type define
#define   Type_Idle                            0                     // ����   
#define   TYPE_CycleAdd                        1                     // ѭ��
#define   TYPE_PhotoAdd                        2                     // ͼƬ 
#define   TYPE_TiredDrvAdd                     3                     // ƣ�ͼ�ʻ
#define   TYPE_ExpSpdAdd                       4                     // ���ٱ�����¼ƫ�Ƶ�ַ
#define   TYPE_AccFireAdd                      5                     // ��������¼��ַƫ��
#define   TYPE_AvrgSpdAdd                      6                     // ÿ����ƽ���ٶ�
#define   TYPE_ErrorLogAdd                     7                     // �쳣Log�洢 
#define   TYPE_VechRecordAdd                   8                     // �����洢��¼ƫ�Ƶ�ַ     
#define   TYPE_DoubtAdd                        9                     // �¹��ɵ�ƫ�Ƶ�ַ
#define   TYPE_AvrgSpdSecAdd                   10                    // ������λ����ÿ��ƽ���ٶȼ�¼��ַ
#define   TYPE_LogInAdd                        11                    // ��¼��Ϣ��¼��ַ
#define   TYPE_PowerCutAdd                     12                    // �ⲿ��Դ�Ͽ���¼��ַ
#define   TYPE_SettingChgAdd                   13                    // �����޸ļ�¼��ַ   
#define   TYPE_MintPosAdd                      14                    // ƽ��ÿСʱÿ����λ�ü�¼��ַ
#define   TYPE_DayDistancAdd                   15                    // ÿ�������ʼ��Ŀ 
#define   TYPE_ACCONFFcounterAdd               16                    // �쳣��λʱ�洢ACCON_Off�ļ�����ֵ  
//-----------------------------------------------------------------------------------------------------------------------------

//---------  ˳���ȡ���� ��� define  -----------
#define   RdCycle_Idle                 0     // ����
#define   RdCycle_RdytoSD              1     // ׼������
#define   RdCycle_SdOver               2     // ������ϵȴ�����Ӧ��


//--------   ˳���ȡ�������  ------------
extern u8       ReadCycle_status;   
extern u8       ReadCycle_timer;   // ��ʱ�ж�

extern u32     cycle_write, cycle_read;  // ѭ���洢��¼
extern u32    AvrgSpdPerMin_write,AvrgSpdPerMin_Read; // ����ÿ����ƽ���ٶȼ�¼
extern u32    AvrgSpdPerSec_write,AvrgSpdPerSec_Read; // ����ÿ��ƽ���ٶȼ�¼   
extern u32    AvrgMintPosit_write,AvrgMintPosit_Read; // ������λСʱ��ÿ����λ�ü�¼ 
extern u32    ErrorLog_write,ErrorLog_Read;           // �豸�쳣��¼
extern u32    Recorder_write,Recorder_Read;           // �г���¼�Ǽ�¼
extern u32    Login_write,Login_Read;				   // ��¼��¼
extern u32    Powercut_write,Powercut_read;		   // �ⲿ��Դ�Ͽ�
extern u32    Settingchg_write,Settingchg_read;	   // �����޸� 
extern u32    TiredDrv_write, TiredDrv_read;  // ƣ�ͼ�ʻ�洢��¼
extern u32    ExpSpdRec_write, ExpSpdRec_read;  // ���ٱ����洢��¼
extern u32    OnFireRec_write, OnFireRec_read;  // �������洢��¼
extern u32    pic_write,pic_read,pic_current_page,pic_PageIn_offset,pic_size;       // ͼƬ�洢��¼ 
extern u32    Distance_m_u32;	 // �г���¼�����о���	  ��λ��
extern u32    DayStartDistance_32; //ÿ����ʼ�����Ŀ    



extern u8  SaveCycleGPS(u32 cycle_wr,u8 *content ,u16 saveLen);   
extern u8  ReadCycleGPS(u32 cycleread,u8 *content ,u16 ReadLen);    
extern u8  Save_DrvRecoder(u32 In_write,u8 *content ,u16 saveLen);
extern u8  Read_DrvRecoder(u32 In_read,u8 *content ,u16 ReadLen);
extern u8  Common_WriteContent(u32 In_write,u8 *content ,u16 saveLen, u8 Type); 
extern u8  Common_ReadContent(u32 In_read,u8 *content ,u16 ReadLen, u8 Type);    
extern u8  Save_PerMinContent(u32 In_wr,u8 *content ,u16 saveLen); 
extern u8  Read_PerMinContent(u32 In_read,u8 *content ,u16 ReadLen);
extern u8  Save_PerSecContent(u32 In_wr,u8 *content ,u16 saveLen);
extern u8  Read_PerSecContent(u32 In_read,u8 *content ,u16 ReadLen);  
extern u8  Save_MintPosition(u32 In_write,u8 *content ,u16 saveLen);
extern u8  Read_MintPosition(u32 In_read,u8 *content ,u16 ReadLen);		
extern void  CHK_ReadCycle_status(void);  
extern void  MediaIndex_Init(void) ; 
extern void Save_Common(u32 In_write,u8 Type);


#endif 

