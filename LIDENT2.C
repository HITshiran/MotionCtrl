/* **************************************************************
系统辨识程序2
version 1.0
1、添加了部分注释；
2、增加了从硬盘读入文件作为输入信号功能；
3、将伺服设置为电流环模式进行辨识
******************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <conio.h>
#include <bios.h>
#include <alloc.h>
#include <process.h>
#include <dos.h>
#include <math.h>
#include "userlib.h"

#define   PI 3.1415926
#define   Rand (rand()%100)/100.0
#define   axno_x 1
#define   axno_y 2

char   SavePath[]="C:\\srmotion\\7.dat";
int    f1=20;//扫频信号最大频率
long   Total=2000;//记录的点个数，也等于运行的时间=Total*中断时间(1ms)
float  In_K1=1500;//输入数据的缩放系数
float  In_K2=1;//X轴输入缩放系数
float  In_K3=1;//Y轴输入缩放系数

int    flag_ForB=1;//辨识运行方向，2取反
int    flag_input=1;//辨识输入信号选择


short  rtn;
float  time=0;
long   count=0;
long   input_x;
long   input_y;
int    input_limit=10;

FILE   *sd;//文件存盘
FILE   *IdIn;//文件读入输入数据

long	actl_pos_x;
long	actl_pos_y;
float * Apos_x;
float  tempp_x;
float * Inpt_x;
float  tempi_x;
float * Apos_y;
float  tempp_y;
float * Inpt_y;
float  tempi_y;

GT_ISR oldisr;
short  IntrTm=5;//中断时间配置
// For test_signal chaos
float  z1=0;
float  z2=0;

//Y轴测速程序相关变量
long Vel_Y_New=0,Vel_Y_Old=0,Vel_Y_Delta=0;
float Vel_Y=0;
float * Avel_y;

//X轴测速程序相关变量
long Vel_X_New=0,Vel_X_Old=0,Vel_X_Delta=0;
float Vel_X=0;
float * Avel_x;

float *idinput;

void error(short rtn)
{
   switch (rtn)
   {
      case -1:
	   printf("error: communciation error\n");
	   break;
      case 0:
	   /* No error */
	   break;
      case 1:
	   printf("error: command error\n");
	   break;
      case 2:
	   printf("error: radius is zero or too large\n");
	   break;
      case 3:
	   printf("error: the length of line is zero or too large\n");
	   break;
      case 4:
	   printf("error: acceleration or velocity is zero or too large\n");
	   break;
      case 5:
	   printf("error: parameter conflict\n");
	   break;
      case 6:
	   printf("error: nonexistence solution for the equation\n");
	   break;
      case 7:
	   printf("error: process parameter error\n");
	   break;
      default:
	   break;
   }
}

void Initial(short axisno)
{
		//rtn=GT_Reset();          error(rtn);
		//rtn=GT_LmtsOff();        error(rtn);
		rtn=GT_Axis(axisno);     error(rtn);
		rtn=GT_ClrSts();         error(rtn);
		rtn=GT_OpenLp();         error(rtn);
		rtn=GT_Update();         error(rtn);
		//rtn=GT_AxisOn();         error(rtn);
}
// PRBS
float test_signal1(float time)
{
	if(Rand>0.5)
		return 1;
	else return 0;
}


//Sine
float test_signal2(float time)
{
	int f0=0;
     //	int f1=10;
	float f;
	float T=0.001*Total;
	f=f0+(f1-f0)*time/T;
	return (0.1+0.1*(0.2*sin(PI*f*time)));
}

//Chaos
float test_signal3(float time)
{
	float w=2*PI*17;
	float z1_dot;
	float z2_dot;

    z1_dot=w*z2;
    z2_dot=w*(z1-z1*z1*z1-0.25*z2+0.3*sin(w*time));
    z1=z1+z1_dot*0.001;
    z2=z2+z2_dot*0.001;

    if (z1*8/3.0+4<8)
	return z1*8/3.0+4;
    else return 8;
}
float test_signal4(float time)
{

	float ff=1;
	return (0.002*sin(2*PI*ff*time));
}



void GetnSet()
{
	// actual time
	time=count*IntrTm*0.0002; // 5*0.0002

	//X轴的位置(要根据光栅尺分辨率进行单位换算)
	GT_Axis(axno_x);
	rtn=GT_GetAtlPos(&actl_pos_x);     error(rtn);
	actl_pos_x=-actl_pos_x;
	Apos_x[count]=actl_pos_x;

	GT_Axis(axno_y);
	rtn=GT_GetAtlPos(&actl_pos_y);     error(rtn);
	actl_pos_y=-actl_pos_y;
	Apos_y[count]=actl_pos_y;

	//简单写的X轴和Y轴的测速程序
	Vel_Y_New=actl_pos_y;
	Vel_Y_Delta=Vel_Y_New-Vel_Y_Old;
	Vel_Y=Vel_Y_Delta*0.0001;
	Vel_Y_Old=Vel_Y_New;
    Avel_y[count]=Vel_Y;//速度单位?

	Vel_X_New=actl_pos_x;
	Vel_X_Delta=Vel_X_New-Vel_X_Old;
	Vel_X=Vel_X_Delta*0.0001;
	Vel_X_Old=Vel_X_New;
    Avel_x[count]=Vel_X;

	// Calculate the input signals
	switch(flag_input)//取值范围-32767~32767
	{
		case 1: input_x=(long)(test_signal1(time)*32767.0); break;
		case 2: input_x=(long)(test_signal2(time)*32767.0); break;
		case 3: input_x=(long)(test_signal3(time)*32767.0); break;
		case 4: input_x=(long)(test_signal4(time)*32767.0); break;
	}
	if(flag_ForB==2)
	{
		input_x=-input_x;//输入反向
	}

/*	//采用内部生成数据时
	input_x=input_x*In_K2;
	input_y=input_x*In_K3;



	// 将输入数字量转换为电压值,用于数据存储
	tempi_x=input_x*10.0/32767.0;
	tempi_y=input_y*10.0/32767.0;
	Inpt_x[count]=tempi_y;
	Inpt_y[count]=tempi_y;
*/

	Inpt_x[count]=idinput[count];
	Inpt_y[count]=idinput[count];

	GT_Axis(axno_x);
	rtn=GT_SetMtrCmd(500);//idinput[count]);        error(rtn);

	GT_Axis(axno_y);
	//rtn=GT_SetMtrCmd(idinput[count]);         error(rtn);
	rtn=GT_SetMtrCmd(500);         error(rtn);

	GT_MltiUpdt(0xff);

	count++;
}

void interrupt OnInterrupt()
{
	GT_ClearInt(0);
	GetnSet();
	outportb(0x20,0x20);
	outportb(0xa0,0x20);
}
 main()
{
	int   j;
	short nret;

	z1=Rand;
	z2=Rand;
	// Memory to store the actual positions
	Apos_x = (float *)malloc(Total*sizeof(float));
	Apos_y = (float *)malloc(Total*sizeof(float));
	// Memory to store the input voltages
	Inpt_x = (float *)malloc(Total*sizeof(float));
	Inpt_y = (float *)malloc(Total*sizeof(float));
	// Memory to store the velocity
	Avel_y = (float *)malloc(Total*sizeof(float));
	Avel_x= (float *)malloc(Total*sizeof(float));
	idinput= (float *)malloc(Total*sizeof(float));

	//从硬盘读入数据
	/*IdIn=fopen("C:\\srmotion\\sr11.txt","r");
	for(j=0;j<Total;j++)
    {
		fscanf(IdIn,"%f",&idinput[j]);
		//使用外部输入数据时
	 idinput[j]=(idinput[j]+1)*In_K1;

    }
    fclose(IdIn);
*/
	//Open the GT controller
	nret=GT_Open();

	GT_Reset();
	Initial(axno_x);
	Initial(axno_y);

	GT_Axis(axno_x);
	GT_AxisOn();
	GT_Axis(axno_y);
	GT_AxisOn();

	clrscr();

	oldisr=GT_HookIsr(OnInterrupt);
	nret=GT_SetIntrTm(IntrTm); //设定定时中断的定时周期5*200us


	nret=GT_TmrIntr();//采用定时中断

	//Just for show
	while(!kbhit()&&count<Total)
	{
		printf("The Interrupt Number is %d\n ",count);
	    printf("count:%d  X:%f Y:%f\n",count,Apos_x[count],Apos_y[count]);
	}


	//Close the interrupt
	nret=GT_EvntIntr();
	GT_UnhookIsr(oldisr);
	GT_Axis(axno_x);
	rtn=GT_AxisOff();           error(rtn);
	GT_Axis(axno_y);
	GT_AxisOff();
	rtn=GT_Reset();  	        error(rtn);
	rtn=GT_Close();            // error(rtn);//会报错rtn=1

	// Record the data
	sd=fopen(SavePath,"w+");
	for (j=0;j<Total;j++)
	{
		fprintf(sd,"%f %f %f %f %f %f\n",Inpt_x[j],Apos_x[j],Inpt_y[j],Apos_y[j],Avel_x[j],Avel_y[j]);
	}

	fclose(sd);
	printf("\n Good Work!!!");

	free(Apos_x);
	free(Inpt_x);
	free(Apos_y);
	free(Inpt_y);
	free(Avel_y);
	free(Avel_x);
	free(idinput);
	printf("\n WELL FREE!!!");
	return 0;
 }
