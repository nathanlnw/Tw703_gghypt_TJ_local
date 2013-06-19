#ifndef _H_SMS
#define _H_SMS

#include "App_moduleConfig.h"
#include "SMS_PDU.h"


#define  SMS_ENABLE  

#define  SMS_ACK_msg          1      // ��Ӵ���ض�Ϣ
#define  SMS_ACK_none         0      // ����Ҫ���ض�Ϣ



typedef  struct _SMS
{
   	u8  SMIndex;    // ���ż�¼
	u8  SMS_read;   // ��ȡ���ű�־λ
	u8  SMS_delALL; // ɾ�����ж��ű�־λ
	u8  SMS_come;   // �ж��Ź�����
	u8  SMS_delayCounter; //������ʱ��
	u8  SMS_waitCounter;	///���ŵȴ�
	u8  SMSAtSend[45];    //����AT����Ĵ���   

	u8  SMS_destNum[15];  //  ���Ͷ�ϢĿ�ĺ��� 
	u8  SMS_sendFlag;  //  ��Ϣ���ͱ�־λ
	u8  SMS_sd_Content[150];  // ��Ϣ��������

	//------- self sms protocol  ----
	u8  MsgID[4];    //  �Զ����ϢID 
	SmsType Sms_Info;	//������PDU��Ϣ�Ĳ�����Ϣ
} SMS_Style;

extern SMS_Style SMS_Service;
/////////////////////////////////////////////////////////////////////////////////////////////////
///��ں���
/////////////////////////////////////////////////////////////////////////////////////////////////
extern void SMS_timer(void);
extern void SMS_Process(void);
extern void SMS_protocol(u8 *instr,u16 len, u8  ACKstate);
extern u8 SMS_Rx_PDU(char *instr,u16 len);
extern u8 SMS_Rx_Text(char *instr,char *strDestNum);
extern u8 SMS_Tx_PDU(char *strDestNum,char *s);
extern u8 SMS_Tx_Text(char *strDestNum,char *s);
extern u8 SMS_Rx_Notice(u16 indexNum);
#endif

