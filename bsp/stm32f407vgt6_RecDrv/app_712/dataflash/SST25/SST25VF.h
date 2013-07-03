
//#include <includes.h>
#ifndef __SST25V_H
#define __SST25V_H
#include <stm32f4xx.h>

//  1.   Total  Define 
#define   DF_SPI           SPI2
//------------------------------------------------------------------
#define Read_Data                     0x03       //��ȡ�洢������
#define HighSpeedReadData             0x0B       //���ٶ�ȡ�洢������
#define SectorErace_4KB               0x20       //��������
#define BlockErace_32KB               0x52       //32KB�����
#define BlockErace_64KB               0xD8       //64KB�����
#define ChipErace                     0xC7       //Ƭ����
 
#define Byte_Program                  0x02       //ҳ����--д����  
#define AAI_WordProgram               0xAD
#define ReadStatusRegister            0x05       //��״̬�Ĵ���
#define EnableWriteStatusRegister     0x50
#define WriteStatusRegister           0x01       //д״̬�Ĵ���

#define WriteEnable                   0x06       //дʹ�ܣ�����״̬�Ĵ���
#define WriteDisable                  0x04       //д��ֹ
#define ReadDeviceID                  0xAB       //��ȡ�豸ID��Ϣ

#define ReadJedec_ID                  0x9F       //JEDEC��ID��Ϣ

#define EBSY                          0X70
#define DBSY                          0X80

#define Dummy_Byte                    0xFF

//  3.   PIN   
#define SST25V_CS_LOW()      GPIO_ResetBits(GPIOD,GPIO_Pin_14)  
#define SST25V_CS_HIGH()     GPIO_SetBits(GPIOD,GPIO_Pin_14)

#define SST25V_WP_HIGH()   GPIO_SetBits(GPIOD,GPIO_Pin_15)
#define SST25V_WP_LOW()    GPIO_SetBits(GPIOD,GPIO_Pin_15) 







extern void SST25V_Init(void);
extern u8   SST25V_ByteRead(u32 ReadAddr);
extern void SST25V_strWrite(u8 *p, u32 WriteAddr,u16 length);    
extern u8  SST25V_OneSector_Write(u8 *p,  u32  addr,  u32 len);

void SST25V_BufferRead(u8* pBuffer, u32 ReadAddr, u16 NumByteToRead);
void SST25V_HighSpeedBufferRead(u8* pBuffer, u32 ReadAddr, u16 NumByteToRead);
u8 SST25V_HighSpeedRead(u32 ReadAddr);

//extern u8 SPI_Flash_SendByte(u8 byte);
//extern u8 SPI_Flash_ReceiveByte(void);
extern void SST25V_ByteWrite(u8 Byte, u32 WriteAddr);
extern void SST25V_BufferWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToRead); 
void AutoAddressIncrement_WordProgramA(u8 Byte1, u8 Byte2, u32 Addr);
void AutoAddressIncrement_WordProgramB(u8 state,u8 Byte1, u8 Byte2) ;

void SST25V_Wait_Busy_AAI(void);
extern void SST25V_SectorErase_4KByte(u32 Addr);
void SST25V_BlockErase_32KByte(u32 Addr);
void SST25V_BlockErase_64KByte(u32 Addr);
void SST25V_ChipErase(void);

u8 SST25V_ReadStatusRegister(void);
void SST25V_WriteEnable(void);
void SST25V_WriteDisable(void);

void SST25V_EnableWriteStatusRegister(void);
void SST25V_WriteStatusRegister(u8 Byte);
void SST25V_WaitForWriteEnd(void);

u32 SST25V_ReadJedecID(void);

u16 SST25V_ReadManuID_DeviceID(u32 ReadManu_DeviceID_Addr);

extern u8 SPI_Flash_SendByte(u8 data);
extern u8 SPI_Flash_ReceiveByte(void);   



#endif 

