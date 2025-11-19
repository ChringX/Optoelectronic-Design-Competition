//#include "iic.h"
//#include "eeprom.h"
//#include "stdio.h"
//uint8_t Send_Byte(uint16_t address,uint8_t pBuffer)
//{
//	IIC_Start();
//	IIC_Send_Byte(Address_Write);
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	
//	IIC_Send_Byte(address>>8);
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	
//	IIC_Send_Byte((uint8_t)(address&0xFF));
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	
//	IIC_Send_Byte(pBuffer);
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	
//	IIC_Stop();
//	delay_ms(50);
//	return 1;
//}	

//uint8_t Read_Byte(uint16_t address,uint8_t *data)
//{
//	EEPROM_wait();
//	IIC_Start();
//	IIC_Send_Byte(Address_Write);
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	
//	IIC_Send_Byte(address>>8);
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	
//	IIC_Send_Byte((uint8_t)(address&0xFF));
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	IIC_Start();
//	IIC_Send_Byte(Address_Read);
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	
//	*data = IIC_Read_Byte(0);
//	
//	IIC_Stop();
//	return 1;
//}	

////返回值为1：等待超时；返回值为0：正常
//uint8_t EEPROM_wait(void)//等待EEPROM内部时序完成
//{
//	uint16_t cycle = 0;
//	
//	while(EEPROM_check_device())//EEPROM无应答
//	{
//		cycle++;
//		if(cycle>=10000) return 1;//等待超时
//	}
//	
//	return 0;//完成等待
//}

//uint8_t EEPROM_check_device(void)//检测EEPROM是否存在
//{
//	uint8_t result;
//	
//	IIC_Start();//发送起始信号
//	IIC_Send_Byte(Address_Write);
//	
//	if(IIC_Wait_Ack())//检测响应信号
//	{
//		result = 1; 
//	}		//表示EEPROM不存在，无应答
//	else
//	{
//		result = 0;
//	}		//表示EEPROM存在
//	
//	IIC_NAck();//如果是读模式下，还要我们自己给EEPROM返回一个非应答信号才能停止
//	IIC_Stop();
//	
//	return result;
//}

///************************************************
//eeprom 一次性最多写入八个字节，每八个字节为一页
//size:要写入多少字节
//************************************************/
////返回值为1：写入正常；返回值为0：写入错误
//uint8_t EEPROM_write_bytes(uint16_t address,uint8_t *data,uint16_t size)
//{
//	uint16_t i;
// 
//	for(i = 0;i < size;i++)
//	{
//		if(i == 0 || address%8 == 0)//第一次或者每八次，i是从0-7的
//		//采用地址对齐的方法，每8个字节（0-7）一页，在每一页的开头发送起始信号
//		{
//			/*每写完一页的结束，先发送一次stop信号，结束前面的写入操作
//			如果是第一次写入也不影响，因为后面有等待EEPROM内部时序完成的程序*/
//			IIC_Stop();
//			if(IIC_Wait_Ack() == 1)
//			{
//				IIC_Stop();
//				return 0;
//			}
//			
//			IIC_Start();//发送起始信号
//			IIC_Send_Byte(Address_Write);
//			if(IIC_Wait_Ack())
//			{
//				IIC_Stop();
//				return 0;
//			}
//			IIC_Send_Byte(address>>8);
//			if(IIC_Wait_Ack())
//			{
//				IIC_Stop();
//				return 0;
//			}
//			
//			IIC_Send_Byte((uint8_t)(address&0xFF));
//			if(IIC_Wait_Ack())
//			{
//				IIC_Stop();
//				return 0;
//			}
//		}//在第一页或者每写完8个字节都要走一遍上面的程序
// 
//		IIC_Send_Byte(*data);//向 EEPROM写入数据，一个字节一个字节的输入
//		if(IIC_Wait_Ack() == 1)
//		{
//			IIC_Stop();
//			return 0;
//		}
//		data++;
//		address++;
//	}
// 
//	/*全部输入完成之后*/
//	IIC_Stop();
//	if(IIC_Wait_Ack() == 1)
//	{
//		IIC_Stop();
//		return 0;
//	}
//	return 1;
//}


////返回值为1：读取正常；返回值为0：读取错误
//uint8_t EEPROM_read_bytes(uint16_t address,uint8_t *data,uint16_t size)//从EEPROM里面读取n个字节
//{
//	IIC_Start(); //发送第一次起始信号
//	IIC_Send_Byte(Address_Write);
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	
//	IIC_Send_Byte(address>>8);
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	
//	IIC_Send_Byte((uint8_t)(address&0xFF));
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	
//	IIC_Start(); //发送第二次起始信号
//		
//	IIC_Send_Byte(Address_Read);
//	if(IIC_Wait_Ack())
//	{
//		IIC_Stop();
//		return 0;
//	}
//	
//	uint16_t i;
//	for(i = 0;i < size;i++)
//	{
//		if(i == size - 1)
//		{
//			*data = IIC_Read_Byte(0);
//		}//数据接收够了
//		else
//		{
//			*data = IIC_Read_Byte(1);
//		}
//		data++;//指针指向下一个数据
//	}
//			
//	IIC_Stop();	
//	return 1;
//}

#include "iic.h"
#include "eeprom.h"

uint8_t Send_Byte(uint16_t address,uint8_t pBuffer)
{
	IIC_Start();
	IIC_Send_Byte(Address_Write);
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 0;
	}
	
	IIC_Send_Byte(address>>8);
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 0;
	}
	
	IIC_Send_Byte((uint8_t)(address&0xFF));
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 0;
	}
	
	IIC_Send_Byte(pBuffer);
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 0;
	}
	
	IIC_Stop();
	delay_ms(50);
	return 1;
}	

uint8_t Read_Byte(uint16_t address,uint8_t *data)
{
	EEPROM_wait();
	
	IIC_Start();
	IIC_Send_Byte(Address_Write);
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 5;
	}
	
	IIC_Send_Byte(address>>8);
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 0;
	}
	
	IIC_Send_Byte((uint8_t)(address&0xFF));
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 0;
	}
	IIC_Start();
	IIC_Send_Byte(Address_Read);
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 0;
	}
	
	*data = IIC_Read_Byte(0);
	
	IIC_Stop();
	return 1;
}	

uint8_t EEPROM_check_device(void)//检测EEPROM是否存在
{
	uint8_t result;
	
	IIC_Start();//发送起始信号
	IIC_Send_Byte(Address_Write);
	
	if(IIC_Wait_Ack())//检测响应信号
	{
		result = 1; 
	}		//表示EEPROM不存在，无应答
	else
	{
		result = 0;
	}		//表示EEPROM存在
	
	IIC_NAck();//如果是读模式下，还要我们自己给EEPROM返回一个非应答信号才能停止
	IIC_Stop();
	
	return result;
}

//返回值为1：等待超时；返回值为0：正常
uint8_t EEPROM_wait(void)//等待EEPROM内部时序完成
{
	uint16_t cycle = 0;
	
	while(EEPROM_check_device())//EEPROM无应答
	{
		cycle++;
		if(cycle>=10000) return 1;//等待超时
	}
	
	return 0;//完成等待
}

/************************************************
eeprom 一次性最多写入八个字节，每八个字节为一页
size:要写入多少字节
************************************************/
//返回值为1：写入正常；返回值为0：写入错误
uint8_t EEPROM_write_bytes(uint16_t address,uint8_t *data,uint16_t size)
{
	uint16_t i;
 
	for(i = 0;i < size;i++)
	{
		if(i == 0 || address%8 == 0)//第一次或者每八次，i是从0-7的
		//采用地址对齐的方法，每8个字节（0-7）一页，在每一页的开头发送起始信号
		{
			/*每写完一页的结束，先发送一次stop信号，结束前面的写入操作
			如果是第一次写入也不影响，因为后面有等待EEPROM内部时序完成的程序*/
			IIC_Stop();
			if(IIC_Wait_Ack() == 1)
			{
				IIC_Stop();
				return 0;
			}
			
			IIC_Start();//发送起始信号
			IIC_Send_Byte(Address_Write);
			if(IIC_Wait_Ack())
			{
				IIC_Stop();
				return 0;
			}
			IIC_Send_Byte(address>>8);
			if(IIC_Wait_Ack())
			{
				IIC_Stop();
				return 0;
			}
			
			IIC_Send_Byte((uint8_t)(address&0xFF));
			if(IIC_Wait_Ack())
			{
				IIC_Stop();
				return 0;
			}
		}//在第一页或者每写完8个字节都要走一遍上面的程序
 
		IIC_Send_Byte(*data);//向 EEPROM写入数据，一个字节一个字节的输入
		if(IIC_Wait_Ack() == 1)
		{
			IIC_Stop();
			return 0;
		}
		data++;
		address++;
	}
 
	/*全部输入完成之后*/
	IIC_Stop();
	if(IIC_Wait_Ack() == 1)
	{
		IIC_Stop();
		return 0;
	}
	return 1;
}


//返回值为1：读取正常；返回值为0：读取错误
uint8_t EEPROM_read_bytes(uint16_t address,uint8_t *data,uint16_t size)//从EEPROM里面读取n个字节
{
	IIC_Start(); //发送第一次起始信号
	IIC_Send_Byte(Address_Write);
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 0;
	}
	
	IIC_Send_Byte(address>>8);
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 0;
	}
	
	IIC_Send_Byte((uint8_t)(address&0xFF));
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 0;
	}
	
	IIC_Start(); //发送第二次起始信号
		
	IIC_Send_Byte(Address_Read);
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 0;
	}
	
	uint16_t i;
	for(i = 0;i < size;i++)
	{
		if(i == size - 1)
		{
			*data = IIC_Read_Byte(0);
		}//数据接收够了
		else
		{
			*data = IIC_Read_Byte(1);
		}
		data++;//指针指向下一个数据
	}
			
	IIC_Stop();	
	return 1;
}
