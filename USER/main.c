#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "mpu6050.h"
#include "lcd.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

void USART_Send_Char(u8 data)
{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==0);
	USART_SendData(USART1,data);
}

/*传送数据给匿名四轴上位机软件(V2.6版本)
//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
*/
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//最多28字节数据 
	send_buf[len+3]=0;	//校验数置零
	send_buf[0]=0X88;	//帧头
	send_buf[1]=fun;	//功能字
	send_buf[2]=len;	//数据长度
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//复制数据
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//计算校验和	
	for(i=0;i<len+4;i++)USART_Send_Char(send_buf[i]);	//发送数据到串口1 
}

void MPU6050_Send_Data(short ax,short ay,short az,short gx,short gy,short gz)
{
	u8 buf[12];
	buf[0]=(ax>>8)|0xFF;
	buf[1]=ax|0xFF;
	buf[2]=(ay>>8)|0xFF;
	buf[3]=ay|0xFF;
	buf[4]=(az>>8)|0xFF;
	buf[5]=az|0xFF;
	buf[6]=(gx>>8)|0xFF;
	buf[7]=gx|0xFF;
	buf[8]=(gy>>8)|0xFF;
	buf[9]=gy|0xFF;
	buf[10]=(gz>>8)|0xFF;
	buf[11]=gz|0xFF;
	usart1_niming_report(0xA1,buf,12);
}

//通过串口1上报结算后的姿态数据给电脑
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
//roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
//pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
//yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
	u8 tbuf[28]; 
	u8 i;
	for(i=0;i<28;i++)tbuf[i]=0;//清0
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;	
	tbuf[18]=(roll>>8)&0XFF;
	tbuf[19]=roll&0XFF;
	tbuf[20]=(pitch>>8)&0XFF;
	tbuf[21]=pitch&0XFF;
	tbuf[22]=(yaw>>8)&0XFF;
	tbuf[23]=yaw&0XFF;
	usart1_niming_report(0XAF,tbuf,28);//飞控显示帧,0XAF
}

int main(void)
{
	float pitch,roll,yaw;
	short ax,ay,az;
	short gx,gy,gz;
//	short temp;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	LCD_Init();
	delay_init(168);
	uart_init(500000);
	MPU6050_Init();
	while(mpu_dmp_init())
	{
		
	}
  while(1)
	{
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{
//			temp=MPU6050_Read_Temperature();
			MPU6050_Get_GyroValue(&gx,&gy,&gz);
			MPU6050_Get_Accelerometer(&ax,&ay,&az);
			MPU6050_Send_Data(ax,ay,az,gx,gy,gz);
			usart1_report_imu(ax,ay,az,gx,gy,gz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
		}
	}
}
