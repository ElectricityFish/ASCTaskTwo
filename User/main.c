#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "RP.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include <string.h>
#include <math.h>


uint8_t KeyNum;
uint8_t State;
/*定义变量*/
float Target, Actual, Out;			//目标值，实际值，输出值
float Kp=0, Ki=0,Kd=0;					//比例项，积分项，微分项的权重
float Error0, Error1, Error2;		//本次误差，上次误差，上上次误差

int main(void)
{
	/*模块初始化*/
	OLED_Init();		//OLED初始化
	Key_Init();			//非阻塞式按键初始化
	Motor_Init();		//电机初始化
	Encoder_Init();		//编码器初始化
	Serial_Init();		//串口初始化，波特率9600
	
	Timer_Init();		//定时器初始化，定时中断时间1ms
	
	/*OLED打印一个标题*/
	OLED_Printf(0, 0, OLED_8X16, "Speed Control");
	OLED_Update();
	
	while (1)
	{
		if(State==0)//定速控制
		{	
		Kp=0.6, Ki=0.15,Kd=0.01;
		OLED_Printf(0, 0, OLED_8X16, "Speed Control");
	    OLED_Update();	
		if (Serial_RxFlag == 1)//收到对应格式的文本信息
				{
					if (strstr(Serial_Rxpacket, "speed%") != NULL) {
						int16_t speed;
						// 从字符串中提取“speed%”后面的数字
						sscanf(Serial_Rxpacket, "speed%%%hd", &speed);
						Target = speed ;
						if (Target >= 250)Target = 250;
						if (Target <= -250)Target = -250;
					} 
					Serial_RxFlag = 0;//重置标志位
				}
		}
		OLED_Printf(64, 16, OLED_8X16, "Tar:%+04.0f", Target);	//显示目标值
		OLED_Printf(64, 32, OLED_8X16, "Act:%+04.0f", Actual);	//显示实际值
		OLED_Printf(64, 48, OLED_8X16, "Out:%+04.0f", Out);		//显示输出值
		
		OLED_Update();	//OLED更新，调用显示函数后必须调用此函数更新，否则显示的内容不会更新到OLED上
		
		Serial_Printf("%f,%f,%f\r\n", Target, Actual, Out);		//串口打印目标值、实际值和输出值
																//配合绘图软件，可以显示数据的波形
	}
}

void TIM1_UP_IRQHandler(void)
{
	/*定义静态变量（默认初值为0，函数退出后保留值和存储空间）*/
	static uint16_t Count;		//用于计次分频
	
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		/*每隔1ms，程序执行到这里一次*/
		
		Key_Tick();				//调用按键的Tick函数
		KeyNum =Key_GetNum ();
		if(KeyNum ==1)
		{
			State=!State ;//模式改变，要清除之前的数据
			Out=0;
			Error0 =0;
			Error1 =0;
			Error2 =0;
			Target=0;
			Actual =0;
		}
		
		/*计次分频*/
		Count ++;				//计次自增
		if (Count >= 40)		//如果计次40次，则if成立，即if每隔40ms进一次
		{
			Count = 0;			//计次清零，便于下次计次
			if(State==0)
			{
			/*获取实际速度值*/
			/*Encoder_Get函数，可以获取两次读取编码器的计次值增量*/
			/*此值正比于速度，所以可以表示速度，但它的单位并不是速度的标准单位*/
			/*此处每隔40ms获取一次计次值增量，电机旋转一周的计次值增量约为2600*/
			/*因此如果想转换为标准单位，比如转/秒*/
			/*则可将此句代码改成Actual = Encoder_Get() / 408.0 / 0.04;*/
			Actual = Encoder_Get();
			
			/*获取本次误差、上次误差和上上次误差*/
			Error2 = Error1;			//获取上上次误差
			Error1 = Error0;			//获取上次误差
			Error0 = Target - Actual;	//获取本次误差，目标值减实际值，即为误差值
			
			/*PID计算*/
			/*使用增量式PID公式，计算得到输出值*/
			Out += Kp * (Error0 - Error1) + Ki * Error0
					+ Kd * (Error0 - 2 * Error1 + Error2);
			
			/*输出限幅*/
			if (Out > 100) {Out = 100;}		//限制输出值最大为100
			if (Out < -100) {Out = -100;}	//限制输出值最小为100
			
			/*执行控制*/
			/*输出值给到电机PWM*/
			/*因为此函数的输入范围是-100~100，所以上面输出限幅，需要给Out值限定在-100~100*/
			Motor_SetPWM(Out);
			}
			
			
//			if(State==1)
//			{
//				Target=
//			}
		}
		
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

