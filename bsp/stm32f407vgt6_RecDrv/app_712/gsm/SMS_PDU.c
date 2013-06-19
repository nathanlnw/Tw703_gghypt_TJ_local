#include  "stm32f4xx.h"
#include  <stdlib.h>//����ת�����ַ���
#include  <stdio.h>
#include  <string.h>
#include  "SMS_PDU.h"

const char GB_DATA[]="����������ش�������������ɽ��ʹ����������³���������ԥ��";
const char UCS2_CODE[]="4EAC6D256CAA5B816E1D743C85CF5DDD7CA497528D3595FD540996558499664B7518684291028D636D5982CF65B09C8176966E589ED18FBD4E918C6B5180";


/*��ȡһ��ָ���ַ���λ�ã������ַ���Ϊһ���ַ�����*/
static int StringFind(const char* string,const char* find,int number)
{
	char* pos;
	char* p;
	int count = 0;
	//pos=string;
	//p = string;
	pos=strstr(string,find);
	if (pos == (void *)0)
		return -1;
	count=pos-string;
	return count;
	#ifdef 0
	while (number > 0)
	{
		/*������ҵ����ַ�λ�õ�ָ�룬�Ա���ʱָ����б���*/
		pos = strstr(p,find);
		/*��λ��ָ��Ϊ0ʱ��˵��û���ҵ�����ַ�*/
		if (pos == (void *)0)
		return -1;
		/*��λ��ָ�����ʱָ�����˵����һ���ַ�����Ҫ�ҵ��ַ��������ʱָ��С��λ��ָ�룬����б����ַ�������������count��1*/
		while(p <= pos)
		{
			if(*p > 0x80 || *p < 0)
			{
				p++;
			}
			p++;
			count++;
		}
		/*��Ҫ���ҵĴ�����һ*/
		number--;
	}
	return count;
	#endif
}


u16 Hex_To_Ascii(const u8* pSrc, u8* pDst, u16 nSrcLength)
{
	
	const u8 tab[]="0123456789ABCDEF";    // 0x0-0xf���ַ����ұ�
	u16 i;
    
    for( i=0; i<nSrcLength; i++)
    {
		// �����4λ
        *pDst++ = tab[*pSrc >> 4];
                                        
        // �����4λ
        *pDst++ = tab[*pSrc & 0x0f];
    
        pSrc++;
	}
    
    // ����ַ����Ӹ�������
    *pDst = '\0';
    
    // ����Ŀ���ַ�������
    return (nSrcLength << 1);
}


u16 Ascii_To_Hex(const u8* pSrc, u8* pDst, u16 nSrcLength)
{
	u16 i;
	for(i=0; i<nSrcLength; i+=2)
    {
        // �����4λ
        if(*pSrc>='0' && *pSrc<='9')
        {
            *pDst = (*pSrc - '0') << 4;
		}
        else if(*pSrc>='A' && *pSrc<='F')
        {
            *pDst = (*pSrc - 'A' + 10) << 4;
        }
        else  if(*pSrc>='a' && *pSrc<='f')
        {
             *pDst = (*pSrc - 'a' + 10) << 4;
        
        }
        else
           return FALSE;
        pSrc++;
      // �����4λ
        if(*pSrc>='0' && *pSrc<='9')
        {
            *pDst |= *pSrc - '0';
        }
        else if(*pSrc>='A' && *pSrc<='F')
		{
            *pDst |= *pSrc - 'A' + 10;
        }
         else  if(*pSrc>='a' && *pSrc<='f')
        {
             *pDst |= (*pSrc - 'a' + 10);
        
        }
        else
           return FALSE;
        pSrc++;
        pDst++;
    }
    
    // ����Ŀ�����ݳ���
    return (nSrcLength >> 1);	
}



u16 GsmDecode8bit(const u8 *pSrc, u8 *pDst, u16 nSrcLength)
{
	u16 m;
	// �򵥸���
	for(m=0;m<nSrcLength;m++)
	   *pDst++=*pSrc++;
	// ����ַ����Ӹ�������
	*pDst = '\0';
	return nSrcLength;
}

u16  GsmEncode8bit(const u8 *pSrc, u8 *pDst, u16 nSrcLength)
{
	u16 m;
	// �򵥸���
	for(m=0;m<nSrcLength;m++)
	   *pDst++=*pSrc++;

	return nSrcLength;
}

u16 GsmDecodeUcs2_old(const u8* pSrc, u8* pDst, u16 nSrcLength)
{
	u16 nDstLength=nSrcLength;        // UNICODE���ַ���Ŀ
	u16 i;
   // INT16U wchar[128];      // UNICODE��������
    
    // �ߵ��ֽڶԵ���ƴ��UNICODE
    for(i=0; i<nSrcLength; i+=2)
    {
        // �ȸ�λ�ֽ�,��Ϊ�����ݡ����ֽ�Ϊ0
         pSrc++;
        // ���λ�ֽ�
        *pDst++= *pSrc++;
       
   
    }
        // UNICODE��-.�ַ���
    //nDstLength = ::WideCharToMultiByte(CP_ACP, 0, wchar, nSrcLength/2, pDst, 160, NULL, NULL);
        // ����ַ����Ӹ�������    
    //pDst[nDstLength] = '\0';    
        // ����Ŀ���ַ�������
    return (nDstLength>>1);
}

u16 GsmEncodeUcs2_old(const u8* pSrc, u8* pDst, u16 nSrcLength)
{
	u16 nDstLength=nSrcLength;        // UNICODE���ַ���Ŀ
	u16 i;
    //INT16U wchar[128];      // UNICODE��������
    
    // �ַ���-.UNICODE��
   // nDstLength = ::MultiByteToWideChar(CP_ACP, 0, pSrc, nSrcLength, wchar, 128);
 
    // �ߵ��ֽڶԵ������
    for(i=0; i<nDstLength; i++)
    {
         // �������λ�ֽ�
        *pDst++ = 0x00;
        // �������λ�ֽ�
        *pDst++ = * pSrc++;
       
    }
    
    // ����Ŀ����봮����
    return (nDstLength <<1);
}

u16 GsmDecodeUcs2(const u8* pSrc, u8* pDst, u16 nSrcLength)
{
	u16 nDstLength=0;        // UNICODE���ַ���Ŀ
	u16 i;
	s16 indexNum;
	char strTemp[5];
	const u8 *p;
	// INT16U wchar[128];      // UNICODE��������

	rt_kprintf("\r\n GsmDecodeUcs2");
    // �ߵ��ֽڶԵ���ƴ��UNICODE
    for(i=0; i<nSrcLength; i+=2)
    {
		if(*pSrc)		///���ֱ���
    		{
    		p=pSrc;
			Hex_To_Ascii(p,strTemp,2);
			strTemp[4]=0;
    		indexNum=StringFind(UCS2_CODE,strTemp,strlen(UCS2_CODE));
			rt_kprintf("\r\n index=%d,str=%s",indexNum,strTemp);
			if(indexNum>=0)
				{
				indexNum=indexNum>>1;
				*pDst++ = GB_DATA[indexNum];
				*pDst++ = GB_DATA[1+indexNum];
				}
			else
				{
				*pDst++ = 0x20;		///����ʶ��ĺ�����"��"��ʾ
				*pDst++ = 0x3B;
				//*pDst++ = *pSrc;		///����ʶ��ĺ�����UCS2Դ���ʾ
				//*pDst++ = *(pSrc+1);
				}
			pSrc+=2;
			nDstLength+=2;
    		}
		else						///Ӣ�ı���
			{
			
			// �ȸ�λ�ֽ�,��Ϊ�����ݡ����ֽ�Ϊ0
	        pSrc++;
	        // ���λ�ֽ�
	        *pDst++= *pSrc++;
			nDstLength++;
			}
   
    }
        // UNICODE��-.�ַ���
    //nDstLength = ::WideCharToMultiByte(CP_ACP, 0, wchar, nSrcLength/2, pDst, 160, NULL, NULL);
        // ����ַ����Ӹ�������    
    //pDst[nDstLength] = '\0';    
        // ����Ŀ���ַ�������
    return nDstLength;
}


u16 GsmEncodeUcs2(const u8* pSrc, u8* pDst, u16 nSrcLength)
{
	u16 nDstLength=nSrcLength;        // UNICODE���ַ���Ŀ
	u16 i;
	s16 indexNum;
	char strTemp[3];
    //INT16U wchar[128];      // UNICODE��������
    nDstLength=0;
    // �ַ���-.UNICODE��
   // nDstLength = ::MultiByteToWideChar(CP_ACP, 0, pSrc, nSrcLength, wchar, 128);
 
    // �ߵ��ֽڶԵ������
    for(i=0; i<nSrcLength; i++)
    {
    	if((*pSrc&0x80)==0x80)		///���ֱ���
    		{
    		strTemp[0]=*pSrc;
    		strTemp[1]=*(pSrc+1);
			strTemp[2]=0;
    		indexNum=StringFind(GB_DATA,strTemp,strlen(GB_DATA));
			if(indexNum>=0)
				{
				indexNum=indexNum*2;
				Ascii_To_Hex(&UCS2_CODE[indexNum],strTemp,4);
				*pDst++ = strTemp[0];
				*pDst++ = strTemp[1];
				}
			else		///����ʶ��ĺ�����"��"��ʾ
				{
				*pDst++ = 0x20;
				*pDst++ = 0x3B;
				}
			pSrc+=2;
			i++;
    		}
		else						///Ӣ�ı���
			{
	         // �������λ�ֽ�
	        *pDst++ = 0x00;
	        // �������λ�ֽ�
	        *pDst++ = * pSrc++;
			}
		nDstLength++;
       
    }
    
    // ����Ŀ����봮����
    return (nDstLength <<1);
}


u16 GsmDecode7bit(const u8* pSrc, u8* pDst, u16 nSrcLength)
{
    u16 nSrc;        // Դ�ַ����ļ���ֵ
    u16 nDst;        // Ŀ����봮�ļ���ֵ
    u16 nByte;       // ��ǰ���ڴ���������ֽڵ���ţ���Χ��0-6
    u8 nLeft;    // ��һ�ֽڲ��������
    
    // ����ֵ��ʼ��
    nSrc = 0;
    nDst = 0;
    
    // �����ֽ���źͲ������ݳ�ʼ��
    nByte = 0;
    nLeft = 0;
    
    // ��Դ����ÿ7���ֽڷ�Ϊһ�飬��ѹ����8���ֽ�
  // ѭ���ô�����̣�ֱ��Դ���ݱ�������
    // ������鲻��7�ֽڣ�Ҳ����ȷ����
    while(nSrc<nSrcLength)
    {
        // ��Դ�ֽ��ұ߲��������������ӣ�ȥ�����λ���õ�һ��Ŀ������ֽ�
        *pDst = ((*pSrc << nByte) | nLeft) & 0x7f;
        // �����ֽ�ʣ�µ���߲��֣���Ϊ�������ݱ�������
        nLeft = *pSrc >> (7-nByte);
    
        // �޸�Ŀ�괮��ָ��ͼ���ֵ
        pDst++;
        nDst++;
   // �޸��ֽڼ���ֵ
        nByte++;
    
        // ����һ������һ���ֽ�
        if(nByte == 7)
        {
            // ����õ�һ��Ŀ������ֽ�
            *pDst = nLeft;
    
            // �޸�Ŀ�괮��ָ��ͼ���ֵ
            pDst++;
            nDst++;
    
            // �����ֽ���źͲ������ݳ�ʼ��
            nByte = 0;
            nLeft = 0;
        }
    
        // �޸�Դ����ָ��ͼ���ֵ
        pSrc++;
        nSrc++;
	}
    
    *pDst = 0;
    
    // ����Ŀ�괮����
    return nDst;

	
}
//��ÿ��ascii8λ�����Bit8ȥ�������ν���7λ����ĺ�λ����Ƶ�ǰ�棬�γ��µ�8λ���롣 
u16 GsmEncode7bit(const u8* pSrc, u8* pDst, u16 nSrcLength)
{
    u16 nSrc;        // Դ�ַ����ļ���ֵ
    u16 nDst;        // Ŀ����봮�ļ���ֵ
    u16 nChar;       // ��ǰ���ڴ���������ַ��ֽڵ���ţ���Χ��0-7
    u8 nLeft;    // ��һ�ֽڲ��������
    
    // ����ֵ��ʼ��
    nSrc = 0;
    nDst = 0;
    
    // ��Դ��ÿ8���ֽڷ�Ϊһ�飬ѹ����7���ֽ�
    // ѭ���ô�����̣�ֱ��Դ����������
    // ������鲻��8�ֽڣ�Ҳ����ȷ����
    while(nSrc<nSrcLength+1)
    {
        // ȡԴ�ַ����ļ���ֵ�����3λ
        nChar = nSrc & 7;
		  // ����Դ����ÿ���ֽ�
        if(nChar == 0)
        {
            // ���ڵ�һ���ֽڣ�ֻ�Ǳ�����������������һ���ֽ�ʱʹ��
            nLeft = *pSrc;
        }
        else
        {
            // ���������ֽڣ������ұ߲��������������ӣ��õ�һ��Ŀ������ֽ�
            *pDst = (*pSrc << (8-nChar)) | nLeft;
      // �����ֽ�ʣ�µ���߲��֣���Ϊ�������ݱ�������
            nLeft = *pSrc >> nChar;
            // �޸�Ŀ�괮��ָ��ͼ���ֵ pDst++;
            pDst++;
            nDst++; 
        } 
        
        // �޸�Դ����ָ��ͼ���ֵ
        pSrc++; nSrc++;
    }
    
    // ����Ŀ�괮����
    return nDst; 
	
}




// �����ߵ����ַ���ת��Ϊ����˳����ַ���
// �磺"8613693092030" -. "683196032930F0"
// pSrc: Դ�ַ���ָ��
// pDst: Ŀ���ַ���ָ��
// nSrcLength: Դ�ַ�������
// ����: Ŀ���ַ�������
u8 Hex_Num_Encode(const u8 *pSrc,u8 *pDst,u8 nSrcLength)
{
  u8 nDstLength=nSrcLength;
	u8 i;
  if(nDstLength&0x01)
    nDstLength+=1;
  for(i=0;i<nDstLength;i+=2)
  {
   *pDst=(*pSrc<<4)|(*pSrc>>4);
    pDst++;
    pSrc++;
  }
  if(nSrcLength&1)
    {
     *(pDst-1)&=0x0f;
     *(pDst-1)|=0xf0;
    }
   return (nDstLength>>1);
}

// �����ߵ����ַ���ת��Ϊ����˳����ַ���
// �磺"8613693092030" -. "683196032930F0"
// pSrc: Դ�ַ���ָ��
// pDst: Ŀ���ַ���ָ��
// nSrcLength: Դ�ַ�������
// ����: Ŀ���ַ�������
u8 Hex_Num_Decode(const u8 *pSrc,u8 *pDst,u8 nSrcLength)
{
  u8 nDstLength=nSrcLength;
  u8 i;
  if(nDstLength&0x01)
    nDstLength+=1;
  for( i=0;i<nDstLength;i+=2)
  {
    *pDst=(*pSrc<<4)|(*pSrc>>4);
    pDst++;
    pSrc++;
  }
  if(nSrcLength&1)
    {
     *(pDst-1)&=0xf0;
     *(pDst-1)|=0x0f;
    }
  return (nDstLength>>1);
}


//���غ��볤��
u8 Que_Number_Length(const u8 *Src)
  {
     u8 n=0;
	 u8 m;
     for(m=0;m<8;m++)
     {
       if((Src[m]&0xf0)==0xf0)
        break;
        n++;
      if((Src[m]&0x0f)==0x0f)
         break;
         n++;
     }
     return n;
  
  }


/*********************************************************************************
*��������:u16 SetPhoneNumToPDU(u8 *pDest,u8 *pSrc,u16 len)
*��������:���ַ�����ʽת����BCD���ʽ��PDU�绰���룬��ԭʼ����Ϊ"13820554863"תΪ13820554863F
*��    ��:pSrc ԭʼ���ݣ�len�������󳤶�
*��    ��:pDest ����ַ���
*�� �� ֵ:	��Ч�ĺ��볤��
*��    ��:������
*��������:2013-05-28
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
*********************************************************************************/
u16 SetPhoneNumToPDU(u8 *pDest,char *pSrc,u16 len)
{
	u16 i,j;
	
    memset(pDest,0xff,len);
    for( i=0; i<len; i++)
    {
		if((*pSrc>=0x30)&&(*pSrc<=0x39))
			{
			if(i%2==0)
				{
				*pDest &=0x0F;
				*pDest |= ((*pSrc-0x30)<<4);
				}
			else
				{
				*pDest &=0xF0;
				*pDest |= *pSrc-0x30;
				pDest++;
				}
			}
		else
			{
			return i;
			}
		pSrc++;
	}
	return i;
}
/*********************************************************************************
*��������:void GetPhoneNumFromPDU(u8 *pDest,u8 *pSrc,u16 len)
*��������:��BCD���ʽ��PDU�绰����ת�����ַ�����ʽ����ԭʼ����Ϊ13820554863FתΪ"13820554863"
*��    ��:pSrc ԭʼ���ݣ�len�������󳤶�
*��    ��:pDest ����ַ���
*�� �� ֵ:	��Ч�ĺ��볤��
*��    ��:������
*��������:2013-05-28
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
*********************************************************************************/
u16 GetPhoneNumFromPDU(char *pDest,u8 *pSrc,u16 len)
{
	u16 i,j;
	for(j=0,i=0;i<len;i++)
		{
		if((pSrc[i]&0x0f)!=0x0f)
			{
			pDest[j++]=(pSrc[i]>>4)+0x30;
			pDest[j++]=(pSrc[i]&0x0f)+0x30;
			}
		else
			{
			pDest[j++]=(pSrc[i]>>4)+0x30;
			pDest[j]=0;
			break;
			}
		}
	return j;
}

//ok_bym
//�������
u16   GsmDecodePdu(const u8* pSrc,u16 pSrcLength,SmsType *pSmstype,u8 *DataDst)
{
	u8 nDstLength=0;          // Ŀ��PDU������
	u8 tmp;       // �ڲ��õ���ʱ�ֽڱ���
	u8 buf[256];  // �ڲ��õĻ�����
	u16 templen=0;
	u16 tmp16;
    ///0891683108200245F320 0D90 683128504568F3 0008315032419430235E00540057003700300033002300440045005600490043004500490044002800310033003300340035003600370034003800360033002900230049005000310028003100320035002E00330038002E003100380032002E0036003000296D25
    //---SCA
    // SMSC��ַ��Ϣ��
    Ascii_To_Hex(pSrc, &tmp, 2);    // ȡ����
    if(tmp>0&&tmp<=12)
    {
    
    tmp = (tmp - 1) * 2;    // SMSC���봮����,ȥ����91;
    pSrc += 4;              // ָ�����,���������ֽڣ�91�����ֽڡ���4���ֽ�
    templen+=4;
    Ascii_To_Hex(pSrc, buf, tmp);    // ת��SMSC���뵽Ŀ��PDU��
    Hex_Num_Decode(buf,(*pSmstype).SCA,tmp);//ȡ�������ĺ���,���棬�ظ�ʱ��,��HEX�뱣���  
    pSrc += tmp;        // ָ�����,��ʱ����PDUType
    templen+=tmp;
  
    
    // TPDU�λ����������ظ���ַ��
     //--PDUType
    Ascii_To_Hex(pSrc, &tmp, 2);    // ȡ��������
    pSrc += 2;        // ָ�����
    templen+=2;
    //--OA----star
        // �����ظ���ַ��ȡ�ظ���ַ��Ϣ
    Ascii_To_Hex(pSrc, &tmp, 2);    // ȡ����,OA�ĳ�����ָ����ĺ��볤�ȣ�
    if(tmp & 1) tmp += 1;    // ������ż��
    pSrc += 4;          // ָ����ƣ���ȥ���ȣ���91,��4���ֽ�
    templen+=4;
    if(tmp>0&&tmp<=12*2)
    {
    Ascii_To_Hex(pSrc, buf, tmp); 
    Hex_Num_Decode(buf,(*pSmstype).TPA,tmp) ;  // ȡTP-RA����,����ظ���ַ
    pSrc += tmp;        // ָ�����
    templen+=tmp;
    //--OA---End-------

    ///0891683108200245F320 0D90 683128504568F3 0008 31503241943023 5E 00540057003700300033002300440045005600490043004500490044002800310033003300340035003600370034003800360033002900230049005000310028003100320035002E00330038002E003100380032002E0036003000296D25
    //---SCA
    // TPDU��Э���ʶ�����뷽ʽ���û���Ϣ��
    Ascii_To_Hex(pSrc, (u8*)&(*pSmstype).TP_PID, 2);    // ȡЭ���ʶ(TP-PID)
    pSrc += 2;
    templen+=2;       // ָ�����
    Ascii_To_Hex(pSrc, (u8*)&(*pSmstype).TP_DCS, 2);    // ȡ���뷽ʽ(TP-DCS),�������ʱ�ͻظ�ʱ��
    pSrc += 2;        // ָ�����
    templen+=2;
    //GsmSerializeNumbers(pSrc, m_SmsType.TP_SCTS, 14);        // ����ʱ����ַ���(TP_SCTS) 
    pSrc += 14;       // ָ�����
    templen+=14;
    Ascii_To_Hex(pSrc, &tmp, 2);    // �û���Ϣ����(TP-UDL)
    pSrc += 2;        // ָ�����
    templen+=2;
   // pDst.TP_DCS=8;
    if(((*pSmstype).TP_DCS&0x0c) == GSM_7BIT)    
    {
        // 7-bit����
        tmp16=tmp%8 ? ((u16)tmp * 7 / 8 + 1) : ((u16)tmp * 7 / 8);
        tmp16*=2;
       if((templen+tmp16<=pSrcLength)&&(tmp16<400))
       { 
        nDstLength = Ascii_To_Hex(pSrc, buf,tmp16);  // ��ʽת��
        GsmDecode7bit(buf, DataDst, nDstLength);    // ת����TP-DU
        nDstLength = tmp;
       }
    }
    else if(((*pSmstype).TP_DCS&0x0c) == GSM_UCS2)
    {
        // UCS2����
       tmp16=tmp * 2;
        if((templen+tmp16<=pSrcLength)&&(tmp16<400))
        {
        nDstLength = Ascii_To_Hex(pSrc, buf,tmp16);        // ��ʽת��
        nDstLength = GsmDecodeUcs2(buf, DataDst, nDstLength);    // ת����TP-DU
        }
    }
    else
    {
        // 8-bit����
        tmp16=tmp * 2;
       if((templen+tmp16<=pSrcLength)&&(tmp16<512))
        {
        nDstLength = Ascii_To_Hex(pSrc, buf,tmp16);        // ��ʽת��
        nDstLength = GsmDecode8bit(buf, DataDst, nDstLength);    // ת����TP-DU
		}
		
    }
   }
   }
    // ����Ŀ���ַ�������
    return nDstLength;
}
 
 //���Ӷ������ĺ���
u16   GsmEncodePdu_NoCenter(const SmsType pSrc,const u8 *DataSrc,u16 datalen, u8* pDst)
{
	
	u16 nLength;             // �ڲ��õĴ�����
    u16 nDstLength=0;          // Ŀ��PDU������
    u8 buf[256];  // �ڲ��õĻ�����
    
    // SMSC��ַ��Ϣ��
    buf[0]=0;//SCA
     nDstLength += Hex_To_Ascii(buf, pDst, 1);
     // TPDU�λ���������Ŀ���ַ�� 
    buf[0]=0x11;//PDUTYPE
    buf[1]=0x00;//MR
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength], 2);
    // SMSC��ַ��Ϣ��
    nLength = Que_Number_Length(pSrc.TPA);// TP-DA��ַ�ַ����ĳ���
    buf[0] = (u8)nLength;   // Ŀ���ַ���ָ���(TP-DA��ַ�ַ�����ʵ����)
    buf[1] = 0x91;            // �̶�: �ù��ʸ�ʽ����
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength], 2);
    nLength=Hex_Num_Encode(pSrc.TPA,buf,nLength);
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength],nLength);  // ת��TP-DA��Ŀ��PDU��
    
    // TPDU��Э���ʶ�����뷽ʽ���û���Ϣ��
    buf[0] = 0;        // Э���ʶ(TP-PID)
    buf[1] = pSrc.TP_DCS&0x0c;        // �û���Ϣ���뷽ʽ(TP-DCS)
    buf[2] = 0x8f;           // ��Ч��(TP-VP)Ϊ12Сʱ
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength],3);
    if((pSrc.TP_DCS&0x0c) == GSM_7BIT)    
    {
        // 7-bit���뷽ʽ
        buf[0] = datalen;            // ����ǰ����.7λ��ʽ��ʾ����ǰ�ĳ���
        nLength = GsmEncode7bit(DataSrc, &buf[1], datalen); 
        nLength+=1;
		// ת��		TP-DA��Ŀ��PDU��
    }
    else if((pSrc.TP_DCS&0x0c)  == GSM_UCS2)
    {
        // UCS2���뷽ʽ
        buf[0] = GsmEncodeUcs2(DataSrc, &buf[1], datalen);    // ת��TP-DA��Ŀ��PDU��
        nLength = buf[0] + 1;        // nLength���ڸö����ݳ���
    }
    else
    {
        // 8-bit���뷽ʽ
        buf[0] = GsmEncode8bit(DataSrc, &buf[1], datalen);    // ת��TP-DA��Ŀ��PDU��
        nLength = buf[0] + 1;        // nLength���ڸö����ݳ���
    }
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength], nLength);        // ת���ö����ݵ�Ŀ��PDU��
    
    // ����Ŀ���ַ�������
    return nDstLength;
}


u16   AnySmsEncode_NoCenter(const u8 *SrcNumber,u8 type,const u8 *DataSrc,u16 datalen, u8* pDst)
{
   SmsType  tmpSms;
   u8 len8;
   len8=Que_Number_Length(SrcNumber);
   if(*SrcNumber==0x86)		///��0x86���
   	{
	   len8=(len8+1)>>1;
	   //tmpSms.TPA[0]=0x86;
	   memcpy(&tmpSms.TPA[0],SrcNumber,len8);
   	}
   else						///û��0x86���
   	{
	   len8=(len8+1)>>1;
	   tmpSms.TPA[0]=0x86;
	   memcpy(&tmpSms.TPA[1],SrcNumber,len8);
   	}
   tmpSms.TP_DCS=type&0x0c;
   tmpSms.TP_PID=0;
    return (GsmEncodePdu_NoCenter(tmpSms,DataSrc,datalen,pDst));
}


u16 GsmEncodePdu_Center(const SmsType pSrc,const u8 *DataSrc,u16 datalen, u8* pDst)
{
	
	u16 nLength;             // �ڲ��õĴ�����
	u16 nDstLength=0;          // Ŀ��PDU������
	u8 buf[256];  // �ڲ��õĻ�����
    
     // SMSC��ַ��Ϣ��
    nLength = Que_Number_Length(pSrc.SCA);    // SMSC��ַ�ַ����ĳ���    
    buf[0] = (u8)((nLength & 1) == 0 ? nLength : nLength + 1) / 2 + 1;    // SMSC��ַ��Ϣ����
    buf[1] = 0x91;        // �̶�: �ù��ʸ�ʽ����
    nDstLength = Hex_To_Ascii(buf, pDst, 2);        // ת��2���ֽڵ�Ŀ��PDU��
    nLength=Hex_Num_Encode(pSrc.SCA,buf,nLength);
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength],nLength);     // ת��SMSC��Ŀ��PDU��
     // TPDU�λ���������Ŀ���ַ��
    buf[0]=0x11;//PDUTYPE
    buf[1]=0x00;//MR
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength], 2);
    // SMSC��ַ��Ϣ��
    nLength = Que_Number_Length(pSrc.TPA);// TP-DA��ַ�ַ����ĳ���
    buf[0] = (u8)nLength;   // Ŀ���ַ���ָ���(TP-DA��ַ�ַ�����ʵ����)
    buf[1] = 0x91;            // �̶�: �ù��ʸ�ʽ����
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength], 2);
    nLength=Hex_Num_Encode(pSrc.TPA,buf,nLength);
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength],nLength);  // ת��TP-DA��Ŀ��PDU��
    
    // TPDU��Э���ʶ�����뷽ʽ���û���Ϣ��
    buf[0] = 0;        // Э���ʶ(TP-PID)
    buf[1] = pSrc.TP_DCS&0x0c;        // �û���Ϣ���뷽ʽ(TP-DCS)
    buf[2] = 0x8f;            // ��Ч��(TP-VP)Ϊ12Сʱ
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength],3);
    if((pSrc.TP_DCS&0x0c) == GSM_7BIT)    
    {
        // 7-bit���뷽ʽ
        buf[0] = datalen;            // ����ǰ����.7λ��ʽ��ʾ����ǰ�ĳ���
        nLength = GsmEncode7bit(DataSrc, &buf[1], datalen); 
        nLength+=1;
		// ת��		TP-DA��Ŀ��PDU��
    }
    else if((pSrc.TP_DCS&0x0c)== GSM_UCS2)
    {
        // UCS2���뷽ʽ
        buf[0] = GsmEncodeUcs2(DataSrc, &buf[1], datalen);    // ת��TP-DA��Ŀ��PDU��
        nLength = buf[0] + 1;        // nLength���ڸö����ݳ���
    }
    else
    {
        // 8-bit���뷽ʽ
        buf[0] = GsmEncode8bit(DataSrc, &buf[1], datalen);    // ת��TP-DA��Ŀ��PDU��
        nLength = buf[0] + 1;        // nLength���ڸö����ݳ���
    }
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength], nLength);        // ת���ö����ݵ�Ŀ��PDU��
    
    // ����Ŀ���ַ�������
    return nDstLength;
}
/*
// �����ߵ����ַ���ת��Ϊ����˳����ַ���
// �磺"8613693092030" -. "683196032930F0"
// pSrc: Դ�ַ���ָ��
// pDst: Ŀ���ַ���ָ��
// nSrcLength: Դ�ַ�������
// ����: Ŀ���ַ�������
  INT16U  GsmSerializeNumbers(const INT8U* pSrc, INT8U* pDst, INT16U nSrcLength)
{
	
	INT16U nDstLength;   // Ŀ���ַ�������
    INT8U ch;          // ���ڱ���һ���ַ�
    
    // ���ƴ�����
    nDstLength = nSrcLength;
	  // �����ߵ�
    for(INT16U i=0; i<nSrcLength;i+=2)
    {
        ch = *pSrc++;        // �����ȳ��ֵ��ַ�
        *pDst++ = *pSrc++;   // ���ƺ���ֵ��ַ�
        *pDst++ = ch;        // �����ȳ��ֵ��ַ�
    }
    
    // �����ַ���'F'��
    if(*(pDst-1) == 'F')
    {
        pDst--;
        nDstLength--;        // Ŀ���ַ������ȼ�1
    }
    
    // ����ַ����Ӹ�������
    *pDst = '\0';
    
    // ����Ŀ���ַ�������
    return nDstLength;
}

//PDU���еĺ����ʱ�䣬���������ߵ����ַ����������������������ɽ��������任��
// ����˳����ַ���ת��Ϊ�����ߵ����ַ�����������Ϊ��������'F'�ճ�ż��
// �磺"8613693092030" -. "683196032930F0"
// pSrc: Դ�ַ���ָ��
// pDst: Ŀ���ַ���ָ��
// nSrcLength: Դ�ַ�������
// ����: Ŀ���ַ�������
 INT16U  GsmInvertNumbers(const INT8U* pSrc, INT8U* pDst, INT16U nSrcLength)
{
	
	 INT16U nDstLength;   // Ŀ���ַ�������
    INT8U ch;          // ���ڱ���һ���ַ�
    
    // ���ƴ�����
    nDstLength = nSrcLength;
    
    // �����ߵ�
    for(INT16U i=0; i<nSrcLength;i+=2)
    {
        ch = *pSrc++;        // �����ȳ��ֵ��ַ�
        *pDst++ = *pSrc++;   // ���ƺ���ֵ��ַ�
        *pDst++ = ch;        // �����ȳ��ֵ��ַ�
    }
    
    // Դ��������������
    if(nSrcLength & 1)
   {
        *(pDst-2) = 'F';     // ��'F'
        nDstLength++;        // Ŀ�괮���ȼ�1
    }
    
    // ����ַ����Ӹ�������
    *pDst = '\0';
    
    // ����Ŀ���ַ�������
    return nDstLength;
}

//�Ӷ������ĺ���
  INT16U  GsmEncodePdu(const SM_PARAM* pSrc, INT8U* pDst)
{
	
	INT16U nLength;             // �ڲ��õĴ�����
    INT16U nDstLength;          // Ŀ��PDU������
    INT8U buf[256];  // �ڲ��õĻ�����
    
    // SMSC��ַ��Ϣ��
    nLength = strlen(pSrc.SCA);    // SMSC��ַ�ַ����ĳ���    
    buf[0] = (INT8U)((nLength & 1) == 0 ? nLength : nLength + 1) / 2 + 1;    // SMSC��ַ��Ϣ����
    buf[1] = 0x91;        // �̶�: �ù��ʸ�ʽ����
    nDstLength = Hex_To_Ascii(buf, pDst, 2);        // ת��2���ֽڵ�Ŀ��PDU��
    nDstLength += GsmInvertNumbers(pSrc.SCA, &pDst[nDstLength], nLength);    // ת��SMSC��Ŀ��PDU��
     // TPDU�λ���������Ŀ���ַ��
    nLength = strlen(pSrc.TPA);    // TP-DA��ַ�ַ����ĳ���
	CString rec_number=CString(pSrc.TPA);
    buf[0] = 0x11;            // �Ƿ��Ͷ���(TP-MTI=01)��TP-VP����Ը�ʽ(TP-VPF=10)
    buf[1] = 0;               // TP-MR=0
    buf[2] = (INT8U)nLength;   // Ŀ���ַ���ָ���(TP-DA��ַ�ַ�����ʵ����)
    buf[3] = 0x91;            // �̶�: �ù��ʸ�ʽ����
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength], 4);  // ת��4���ֽڵ�Ŀ��PDU��
    nDstLength += GsmInvertNumbers(pSrc.TPA, &pDst[nDstLength], nLength); // ת��TP-DA��Ŀ��PDU��
    
    // TPDU��Э���ʶ�����뷽ʽ���û���Ϣ��
    nLength = strlen(pSrc.TP_UD);    // �û���Ϣ�ַ����ĳ���
    buf[0] = pSrc.TP_PID;        // Э���ʶ(TP-PID)
    buf[1] = pSrc.TP_DCS;        // �û���Ϣ���뷽ʽ(TP-DCS)
    buf[2] = 0;            // ��Ч��(TP-VP)Ϊ5����
    if(pSrc.TP_DCS == GSM_7BIT)    
    {
        // 7-bit���뷽ʽ
        buf[3] = nLength;            // ����ǰ����
        nLength = GsmEncode7bit(pSrc.TP_UD, &buf[4], nLength+1) + 4; 
		// ת��		TP-DA��Ŀ��PDU��
    }
    else if(pSrc.TP_DCS == GSM_UCS2)
    {
        // UCS2���뷽ʽ
        buf[3] = GsmEncodeUcs2(pSrc.TP_UD, &buf[4], nLength);    // ת��TP-DA��Ŀ��PDU��
        nLength = buf[3] + 4;        // nLength���ڸö����ݳ���
    }
    else
    {
        // 8-bit���뷽ʽ
        buf[3] = GsmEncode8bit(pSrc.TP_UD, &buf[4], nLength);    // ת��TP-DA��Ŀ��PDU��
        nLength = buf[3] + 4;        // nLength���ڸö����ݳ���
    }
    nDstLength += Hex_To_Ascii(buf, &pDst[nDstLength], nLength);        // ת���ö����ݵ�Ŀ��PDU��
    
    // ����Ŀ���ַ�������
    return nDstLength;
}
*/


