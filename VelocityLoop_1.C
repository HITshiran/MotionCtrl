/* **************************************************************

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

char   SavePath[]="C:\\srmotion\\1.dat";
int    f1=20;//扫频信号最大频率
long   Total=2000;//记录的点个数，也等于运行的时间=Total*中断时间(1ms)
float  In_K1=0.1;//输入数据的缩放系数
float  In_K2=1;//X轴输入缩放系数
float  In_K3=1;//Y轴输入缩放系数
float  Vel_Given=0.1;//velocity given m/s
float  Vel_Ref=0;//velocity reference input variable
float  Acc_Time=1/0.5;// acceleration and decelerate time(for simplify calculation ) 1/s
float  X_I_max=1/
int    flag_ForB=1;//辨识运行方向，2取反
int    flag_input=2;//辨识输入信号选择


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
short  IntrTm=2;//中断时间配置
// For test_signal chaos
float  z1=0;
float  z2=0;

//Y轴测速程序相关变量
long Vel_Y_New=0,Vel_Y_Old=0,Vel_Y_Delta=0;
float Vel_Y=0;
float * Avel_y;

//X轴测速程序相关变量
long Vel_X_New=0,Vel_X_Old=0,Vel_X_Delta=0;
float Vel_X=0,Vel_X_Filter=0;
float Filter_Time=0.286; // 1ms filter
float * Avel_x;

float *idinput;

void GetnSet()
{
	// actual time,Unit s
	time=count*IntrTm*0.0002; 

	//the position of AXIS_X
	GT_Axis(axno_x);
	rtn=GT_GetAtlPos(&actl_pos_x);    
	actl_pos_x=-actl_pos_x;
	Apos_x[count]=actl_pos_x;

	//Speed Measure,method M
	Vel_X_New=actl_pos_x;
	Vel_X_Delta=Vel_X_New-Vel_X_Old;
	Vel_X=Vel_X_Delta*0.001;
	Vel_X_Old=Vel_X_New;
    Vel_X_Filter+=Filter_Time*(Vel_X-Vel_X_Filter);//Low pass filter
	Avel_x[count]=Vel_X_Filter;
	
    //simple trapezoid velocity
    Vel_Ref=time*Vel_Given*Acc_Time;

	//single Axis PID,Incremental PID
	V_Err=Vel_Ref-Vel_X_Filter;
	V_Temp1=V_Err-V_ErrLast;
    V_Up=V_Kp*V_Temp1;
	V_Ui=V_Ki*V_Err;
	V_Temp1=V_Err-2*V_ErrLast+V_ErrLast1;
	V_Ud=V_Kd*V_Temp1;  

	V_OutPre=V_Up+V_Ui+V_Ud+V_OutLast;

	V_Out = V_Out_pre;

    V_ErrLast1 = V_ErrLast;	
    V_ErrLast = V_Err;
    V_OutLast = V_Out;

	//convert to analog control input,10V means 50Amps current,MIPEAK=19.1Amps
	Inpt_x[count]=V_Out*32767*0.02;
	if (Inpt_x[count] > 2000)
      Inpt_x[count] =  2000;
    else if (V_Out_pre < -2000)
      Inpt_x[count] =  -2000;
	else
      Inpt_x[count] = Inpt_x[count];
	
	GT_Axis(axno_x);
	rtn=GT_SetMtrCmd(Inpt_x[count]);        error(rtn);
	
	GT_Axis(axno_y);
	rtn=GT_SetMtrCmd(0);                    error(rtn);

	GT_MltiUpdt(0xff);

	count++;
}

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

void interrupt OnInterrupt()
{
	GT_ClearInt(0);
	GetnSet();//Interrupt function
	outportb(0x20,0x20);
	outportb(0xa0,0x20);
}


 main()
{
	int   j;
	short nret;

	z1=Rand;
	z2=Rand;
	// Config Memory to store variables
	Apos_x = (float *)malloc(Total*sizeof(float));
	Apos_y = (float *)malloc(Total*sizeof(float));

	Inpt_x = (float *)malloc(Total*sizeof(float));
	Inpt_y = (float *)malloc(Total*sizeof(float));

	Avel_y = (float *)malloc(Total*sizeof(float));
	Avel_x= (float *)malloc(Total*sizeof(float));

	idinput= (float *)malloc(Total*sizeof(float));

	//Copy Datas from DISK
 	IdIn=fopen("C:\\srmotion\\sr_ident.txt","r");
	for(j=0;j<Total;j++)
    {
		fscanf(IdIn,"%f",&idinput[j]);
		idinput[j]=(idinput[j]+1)*32767.0*In_K1;
	}
    fclose(IdIn);

	//Open the GT controller
	nret=GT_Open();
	GT_Reset();
	Initial(axno_x);
	Initial(axno_y);
	GT_Axis(axno_x);
	GT_AxisOn();
	GT_Axis(axno_y);
	GT_AxisOn();

	clrscr();//Clear Screen

	oldisr=GT_HookIsr(OnInterrupt);
	nret=GT_SetIntrTm(IntrTm); //Set Timed Cycle=200us*IntrTm=200us*2=400us
	nret=GT_TmrIntr();

	//Just for show
	while(!kbhit()&&count<Total)//any butten pressed or count over
	{
	    printf("Interrupt NUM:%d\n",count);
	}

	//Close the interrupt
	nret=GT_EvntIntr();
	GT_UnhookIsr(oldisr);
	GT_Axis(axno_x);
	rtn=GT_AxisOff();          
	GT_Axis(axno_y);
	GT_AxisOff();
	rtn=GT_Reset();  	       
	rtn=GT_Close();           

	// Record the data
	sd=fopen(SavePath,"w+");
	for (j=0;j<Total;j++)
	{
		fprintf(sd,"%f %f %f %f %f %f\n",Inpt_x[j],Apos_x[j],Inpt_y[j],Apos_y[j],Avel_x[j],Avel_y[j]);
	}
	fclose(sd);
	printf("\n *******************WELL DONE*********************");

	//Free the memory
	free(Apos_x);
	free(Inpt_x);
	free(Apos_y);
	free(Inpt_y);
	free(Avel_y);
	free(Avel_x);
	return 0;
 }

