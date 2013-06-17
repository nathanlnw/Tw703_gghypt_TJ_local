#ifndef H_DATAFLASH
#define H_DATAFLASH


//-------------------------------------------------------------


//flash ��Ϣ�ṹ��
#pragma pack(1)
typedef struct
{
 u8 All_Picture_num;           //��ǰ���ݴ洢�ܵ�ͼƬ��(���Ϊ30 ��ΧΪ0~30)
 u8 first_index_num;           //��һ�δ洢��������λ�� ��ΧΪ 1~30
 u8 last_index_num;            //���һ�δ洢��������λ�� ��ΧΪ 1~30
 u16  page_start;              //��ǰ�Ѿ��洢ͼ�����ʼҳ
 u16  page_end;                //��ǰ�Ѿ��洢ͼ���ĩβҳ
 u8   current_photoNum;             //��ǰ����ͼƬ�����
} DF_picture_Status;

typedef struct
{
 u8		Picture_num;       //ͼƬ���
 u8		last_package; 	 //���һ��package�ı�ǣ����һ��packageʱΪ1
 u8		block_num;		    //��ǰ�洢��������ͼƬ�еĿ�����
 u16	data_long;         //ͼƬ��ǰ�洢��ҳ���ֽ���
}DF_Picture_block;

typedef struct 
{
 u16    page_start;      //��ʼҳ
 u16    page_end;        //����ҳ
 u8	    block_total;       //ͼƬռ�õ�ҳ������
 u8     User_information[17]; 
 }DF_index;

typedef struct
{
 u16    page_start;     //��ʼҳ
 u16    page_end; 		  //����ҳ
 u8	   block_total;	  //ͼƬռ�õ�ҳ������
 u8	   Have_picture;	  // 1:��ͼƬ ��0:��ͼƬ
} DF_temp_picture_Status;
#pragma pack()


extern void Write_IPandPort_Setting(void);
extern void  Read_IPandPort_Setting(void); 


//====================================================
extern u8  DF_Write_RecordAdd(u32 Wr_Address,u32 Rd_Address, u8 Type);
extern u8  DF_Read_RecordAdd(u32 Wr_Address,u32 Rd_Address, u8 Type); 



#endif
