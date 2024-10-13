#ifndef MOTORCTRL_H
#define MOTORCTRL_H
#include "main.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "PID_MODEL.h"
#include "bsp_can.h"
#include "motorctrl.h"
#include "stdio.h"
#include "math.h"


#define CH1_1 0
#define CH1_2 1
#define CH1_3 2
#define CH1_4 3
#define CH1_5 4
#define CH1_6 5
#define CH1_7 6

#define CH2_1 7
#define CH2_2 8
#define CH2_3 9
#define CH2_4 10
#define CH2_5 11
#define CH2_6 12
#define CH2_7 13

extern double output[16];
typedef struct
{
	int dir;
	double thetal;
	double thetan;
	double thetas;
	double err;
	double err1;
	double err2;
	double errl;
	
} ang_dir;

void get_msgn(void);
void assign_output(void);
void set_mode(int mode_CH1_1, int mode_CH1_2, int mode_CH1_3, int mode_CH1_4, int mode_CH1_5, int mode_CH1_6, int mode_CH1_7, 
              int mode_CH2_1, int mode_CH2_2, int mode_CH2_3, int mode_CH2_4, int mode_CH2_5, int mode_CH2_6, int mode_CH2_7);

void cala_d(int i);
void ctrlmotor(double Vx, double Vy, double omega,int dir1,int dir2,int dir3,int dir4,int flag) ;
void PID_Speed_Para_Init(int channel, int motor, double kp, double ki, double kd);
void PID_Angle_S_Para_Init(int channel, int motor, double kp, double ki, double kd);
void PID_Angle_A_Para_Init(int channel, int motor, double kp, double ki, double kd);


#endif
