#include "exti.h"
#include "delay.h" 
#include "led.h" 
#include "key.h"
#include "beep.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK精英STM32F103开发板V1
//外部中断 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/1/10
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   


//外部中断初始化程序
//初始化PA0/PE2/PE3/PE4为中断输入.
void EXTIX_Init(void)
{
	KEY_Init();
	Ex_NVIC_Config(GPIO_A,0,RTIR); 	//上升沿触发
	Ex_NVIC_Config(GPIO_E,2,FTIR); 	//下降沿触发
	Ex_NVIC_Config(GPIO_E,3,FTIR); 	//下降沿触发
 	Ex_NVIC_Config(GPIO_E,4,FTIR); 	//下降沿触发
	MY_NVIC_Init(2,3,EXTI0_IRQn,2);	//抢占2，子优先级3，组2
	MY_NVIC_Init(2,2,EXTI2_IRQn,2);	//抢占2，子优先级2，组2	   
	MY_NVIC_Init(2,1,EXTI3_IRQn,2);	//抢占2，子优先级1，组2	   
	MY_NVIC_Init(2,0,EXTI4_IRQn,2);	//抢占2，子优先级0，组2	   
}












