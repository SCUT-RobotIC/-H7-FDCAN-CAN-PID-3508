#include "motorctrl.h"
#include "UPPER_LOCATION.h"
ang_dir MotorSignal[4];
extern motor_measure_t *motor_data[8];
extern motor_measure_t *motor_data1[8];
double output[16] = {0};
int BrakeFlag = 0;
int BrakeAng[4] = {0};
double mult = 1;
int dirflag=0;
int MAXVEL=6000;

void ctrlmotor(double Vx, double Vy, double omega,int dir1,int dir2,int dir3,int dir4,int flag) {

	// for(int i=0;i<4;i++){
	// 			if(fabs((double)((int)MotorSignal[i].thetas%360-90))<1)
	// 				MotorSignal[i].thetas=90+(int)MotorSignal[i].thetas/360*360;
	// 			if(fabs((double)((int)MotorSignal[i].thetas%360+90))<1)
	// 				MotorSignal[i].thetas=-90+(int)MotorSignal[i].thetas/360*360;
	// 			if(fabs((double)((int)MotorSignal[i].thetas%360-270))<1)
	// 				MotorSignal[i].thetas=270+(int)MotorSignal[i].thetas/360*360;
	// 			if(fabs((double)((int)MotorSignal[i].thetas%360+270))<1)
	// 				MotorSignal[i].thetas=-270+(int)MotorSignal[i].thetas/360*360;
	// }
	// 	if(flag==1&&
	// 		(fabs(motor_data1[0]->ecd+motor_data1[0]->circle*8191-rtU.yaw_target_CH2_1)<45*19*8191/360*93/35&&
	// 		fabs(motor_data1[1]->ecd+motor_data1[1]->circle*8191-rtU.yaw_target_CH2_2)<45*19*8191/360*93/35&&
	// 		fabs(motor_data1[2]->ecd+motor_data1[2]->circle*8191-rtU.yaw_target_CH2_3)<45*19*8191/360*93/35&&
	// 		fabs(motor_data1[3]->ecd+motor_data1[3]->circle*8191-rtU.yaw_target_CH2_4)<45*19*8191/360*93/35)||omega!=0){
	// 		while(  (fabs(sqrt(pow((Vy-omega*cos(atan(1))),2)+pow((Vx-omega*sin(atan(1))),2))*mult)>MAXVEL)||
	// 					(fabs(sqrt(pow((Vy+omega*cos(atan(1))),2)+pow((Vx-omega*sin(atan(1))),2))*mult)>MAXVEL)||
	// 					(fabs(sqrt(pow((Vy+omega*cos(atan(1))),2)+pow((Vx+omega*sin(atan(1))),2))*mult)>MAXVEL)||
	// 					(fabs(sqrt(pow((Vy-omega*cos(atan(1))),2)+pow((Vx+omega*sin(atan(1))),2))*mult)>MAXVEL))
	// 		{
	// 			mult=0.98*mult;
	// 		}
	
	// 		BrakeFlag=0;
	// 		rtU.yaw_status_CH1_1=1;
	// 		rtU.yaw_status_CH1_2=1;
	// 		rtU.yaw_status_CH1_3=1;
	// 		rtU.yaw_status_CH1_4=1;
				
				
				
	// 		if(dir1==1)
	// 			rtU.yaw_target_CH1_1= -sqrt(pow((Vy-omega*cos(atan(1))),2)+pow((Vx-omega*sin(atan(1))),2))*mult;
	// 		else
	// 			rtU.yaw_target_CH1_1= sqrt(pow((Vy-omega*cos(atan(1))),2)+pow((Vx-omega*sin(atan(1))),2))*mult;
	
	// 		if(dir2==1)	
	// 			rtU.yaw_target_CH1_2 =-sqrt(pow((Vy-omega*cos(atan(1))),2)+pow((Vx+omega*sin(atan(1))),2))*mult;
	// 		else
	// 			rtU.yaw_target_CH1_2 =sqrt(pow((Vy-omega*cos(atan(1))),2)+pow((Vx+omega*sin(atan(1))),2))*mult;
	
	// 		if(dir3==1)
	// 			rtU.yaw_target_CH1_3= -sqrt(pow((Vy+omega*cos(atan(1))),2)+pow((Vx+omega*sin(atan(1))),2))*mult;	
	// 		else
	// 			rtU.yaw_target_CH1_3= sqrt(pow((Vy+omega*cos(atan(1))),2)+pow((Vx+omega*sin(atan(1))),2))*mult;	
	
	// 		if(dir4==1)
	// 			rtU.yaw_target_CH1_4 =-sqrt(pow((Vy+omega*cos(atan(1))),2)+pow((Vx-omega*sin(atan(1))),2))*mult;
	// 		else
	// 			rtU.yaw_target_CH1_4 =sqrt(pow((Vy+omega*cos(atan(1))),2)+pow((Vx-omega*sin(atan(1))),2))*mult;
	// 		// if(RC.y>6700)
	// 		// {
	// 		// 	if(RC.theta<0&&RC.theta>-180&&RC.action!=6)
	// 		// 	{
	// 		// 		rtU.yaw_target_CH2_5=-15000;
	// 		// 		rtU.yaw_target_CH2_6=-15000;
	// 		// 	}
	// 		// 	else if(RC.theta<180&&RC.theta>0&&RC.action!=6)
	// 		// 	{
	// 		// 		rtU.yaw_target_CH2_5=15000;
	// 		// 		rtU.yaw_target_CH2_6=15000;
	// 		// 	}	
	// 		// 	else if(RC.action==6){
	// 		// 		rtU.yaw_target_CH2_5=0;
	// 		// 		rtU.yaw_target_CH2_6=0;
	// 		// 	}
	// 		// }				
	// 	}
	// 		else
	// 				{
	
	
	// 				if(BrakeFlag==0&&motor_data[0]->ecd<8191&&motor_data[1]->ecd<8191
	// 		&&motor_data[2]->ecd<8191&&motor_data[3]->ecd<8191)
	// 				{
							
	// 						rtDW.Integrator_DSTATE_lm=0;
	// 						rtDW.Integrator_DSTATE_mwe=0;
	// 						rtDW.Integrator_DSTATE_cf=0;
	// 						rtDW.Integrator_DSTATE_mw=0;
	// 						rtU.yaw_status_CH1_1=2;
	// 						rtU.yaw_status_CH1_2=2;
	// 						rtU.yaw_status_CH1_3=2;
	// 						rtU.yaw_status_CH1_4=2;
	// 						BrakeAng[0]=motor_data[0]->ecd+motor_data[0]->circle*8191;
	// 						BrakeAng[1]=motor_data[1]->ecd+motor_data[1]->circle*8191;
	// 						BrakeAng[2]=motor_data[2]->ecd+motor_data[2]->circle*8191;
	// 						BrakeAng[3]=motor_data[3]->ecd+motor_data[3]->circle*8191;
	// 						BrakeFlag=1;
	// 				}
	// 				rtU.yaw_target_CH1_1=BrakeAng[0];
	// 				rtU.yaw_target_CH1_2=BrakeAng[1];
	// 				rtU.yaw_target_CH1_3=BrakeAng[2];
	// 				rtU.yaw_target_CH1_4=BrakeAng[3];
					
	// 			 rtU.yaw_target_CH2_5=0;
	// 		   rtU.yaw_target_CH2_6=0;
	// 				}
	
	// mult=1;
}

void cala_d(int i){
	
		if((MotorSignal[i].thetan<=180&&MotorSignal[i].thetan>90)||(MotorSignal[i].thetan<-90&&MotorSignal[i].thetan>-180))
				MotorSignal[i].dir=1;
			else
				MotorSignal[i].dir=0;
 if(MotorSignal[i].thetan<-90&&MotorSignal[i].thetan>-180)
		 {
			 MotorSignal[i].thetan=MotorSignal[i].thetan+180;

		 }
		 else if(MotorSignal[i].thetan<=180&&MotorSignal[i].thetan>90)
		 {
			 MotorSignal[i].thetan=MotorSignal[i].thetan-180;

		 }

		 MotorSignal[i].err=MotorSignal[i].thetan-MotorSignal[i].thetal;
      if(MotorSignal[i].thetan-MotorSignal[i].thetal<0)
			{
				MotorSignal[i].err1=MotorSignal[i].err+180;
				MotorSignal[i].err2=MotorSignal[i].err;
				
			}
			else if(MotorSignal[i].thetal-MotorSignal[i].thetan<=0)
			{
				MotorSignal[i].err1=MotorSignal[i].err-180;
				MotorSignal[i].err2=MotorSignal[i].err;
				
			}
			if(fabs(MotorSignal[i].err1)>=fabs(MotorSignal[i].err2)){
				MotorSignal[i].err=MotorSignal[i].err2;
			}
			else{
				MotorSignal[i].err=MotorSignal[i].err1;
        MotorSignal[i].dir=1-MotorSignal[i].dir;
			}
			MotorSignal[i].thetas+=MotorSignal[i].err;
			

//			if((dirflag==0)&&(((int)MotorSignal[i].thetas%360<=270&&(int)MotorSignal[i].thetas%360>=90)||((int)MotorSignal[i].thetas%360<=-90&&(int)MotorSignal[i].thetas%360>=-270))){
//					MotorSignal[i].dir=1-MotorSignal[i].dir;
//					dirflag=1;
//				}
//			
			if((((int)MotorSignal[i].thetas%360<=270&&(int)MotorSignal[i].thetas%360>=90)||((int)MotorSignal[i].thetas%360<=-90&&(int)MotorSignal[i].thetas%360>=-270))){
					MotorSignal[i].dir=1-MotorSignal[i].dir;
				}
//			else if(!((int)MotorSignal[i].thetas%360<=270&&(int)MotorSignal[i].thetas%360>=90)||((int)MotorSignal[i].thetas%360<=-90&&(int)MotorSignal[i].thetas%360>=-270)){
//			dirflag=0;
//			}

}


void get_msgn(void)
{
  rtU.yaw_speed_rpm_CH1_1 = motor_data[0]->speed_rpm;
  rtU.yaw_ecd_CH1_1 = motor_data[0]->ecd;
  rtU.yaw_last_ecd_CH1_1 = motor_data[0]->last_ecd;
  rtU.yaw_circle_CH1_1 = motor_data[0]->circle;

  rtU.yaw_speed_rpm_CH1_2 = motor_data[1]->speed_rpm;
  rtU.yaw_ecd_CH1_2 = motor_data[1]->ecd;
  rtU.yaw_last_ecd_CH1_2 = motor_data[1]->last_ecd;
  rtU.yaw_circle_CH1_2 = motor_data[1]->circle;

  rtU.yaw_speed_rpm_CH1_3 = motor_data[2]->speed_rpm;
  rtU.yaw_ecd_CH1_3 = motor_data[2]->ecd;
  rtU.yaw_last_ecd_CH1_3 = motor_data[2]->last_ecd;
  rtU.yaw_circle_CH1_3 = motor_data[2]->circle;

  rtU.yaw_speed_rpm_CH1_4 = motor_data[3]->speed_rpm;
  rtU.yaw_ecd_CH1_4 = motor_data[3]->ecd;
  rtU.yaw_last_ecd_CH1_4 = motor_data[3]->last_ecd;
  rtU.yaw_circle_CH1_4 = motor_data[3]->circle;

  rtU.yaw_speed_rpm_CH1_5 = motor_data[4]->speed_rpm;
  rtU.yaw_ecd_CH1_5 = motor_data[4]->ecd;
  rtU.yaw_last_ecd_CH1_5 = motor_data[4]->last_ecd;
  rtU.yaw_circle_CH1_5 = motor_data[4]->circle;

  rtU.yaw_speed_rpm_CH1_6 = motor_data[5]->speed_rpm;
  rtU.yaw_ecd_CH1_6 = motor_data[5]->ecd;
  rtU.yaw_last_ecd_CH1_6 = motor_data[5]->last_ecd;
  rtU.yaw_circle_CH1_6 = motor_data[5]->circle;

  rtU.yaw_speed_rpm_CH1_7 = motor_data[6]->speed_rpm;
  rtU.yaw_ecd_CH1_7 = motor_data[6]->ecd;
  rtU.yaw_last_ecd_CH1_7 = motor_data[6]->last_ecd;
  rtU.yaw_circle_CH1_7 = motor_data[6]->circle;

  // rtU.yaw_speed_rpm_CH2_1 = motor_data1[0]->speed_rpm;
  // rtU.yaw_ecd_CH2_1 = motor_data1[0]->ecd;
  // rtU.yaw_last_ecd_CH2_1 = motor_data1[0]->last_ecd;
  // rtU.yaw_circle_CH2_1 = motor_data1[0]->circle;

  // rtU.yaw_speed_rpm_CH2_2 = motor_data1[1]->speed_rpm;
  // rtU.yaw_ecd_CH2_2 = motor_data1[1]->ecd;
  // rtU.yaw_last_ecd_CH2_2 = motor_data1[1]->last_ecd;
  // rtU.yaw_circle_CH2_2 = motor_data1[1]->circle;

  // rtU.yaw_speed_rpm_CH2_3 = motor_data1[2]->speed_rpm;
  // rtU.yaw_ecd_CH2_3 = motor_data1[2]->ecd;
  // rtU.yaw_last_ecd_CH2_3 = motor_data1[2]->last_ecd;
  // rtU.yaw_circle_CH2_3 = motor_data1[2]->circle;

  // rtU.yaw_speed_rpm_CH2_4 = motor_data1[3]->speed_rpm;
  // rtU.yaw_ecd_CH2_4 = motor_data1[3]->ecd;
  // rtU.yaw_last_ecd_CH2_4 = motor_data1[3]->last_ecd;
  // rtU.yaw_circle_CH2_4 = motor_data1[3]->circle;

  // rtU.yaw_speed_rpm_CH2_5 = motor_data1[4]->speed_rpm;
  // rtU.yaw_ecd_CH2_5 = motor_data1[4]->ecd;
  // rtU.yaw_last_ecd_CH2_5 = motor_data1[4]->last_ecd;
  // rtU.yaw_circle_CH2_5 = motor_data1[4]->circle;

  // rtU.yaw_speed_rpm_CH2_6 = motor_data1[5]->speed_rpm;
  // rtU.yaw_ecd_CH2_6 = motor_data1[5]->ecd;
  // rtU.yaw_last_ecd_CH2_6 = motor_data1[5]->last_ecd;
  // rtU.yaw_circle_CH2_6 = motor_data1[5]->circle;

  // rtU.yaw_speed_rpm_CH2_7 = motor_data1[6]->speed_rpm;
  // rtU.yaw_ecd_CH2_7 = motor_data1[6]->ecd;
  // rtU.yaw_last_ecd_CH2_7 = motor_data1[6]->last_ecd;
  // rtU.yaw_circle_CH2_7 = motor_data1[6]->circle;
}

void assign_output(void)
{
  if (rtU.yaw_status_CH1_1 == 1)
    output[CH1_1] = rtY.yaw_SPD_OUT_CH1_1 + 100;
  else
    output[CH1_1] = rtY.yaw_ANG_OUT_CH1_1;

  if (rtU.yaw_status_CH1_2 == 1)
    output[CH1_2] = rtY.yaw_SPD_OUT_CH1_2 + 100;
  else
    output[CH1_2] = rtY.yaw_ANG_OUT_CH1_2;

  if (rtU.yaw_status_CH1_3 == 1)
    output[CH1_3] = rtY.yaw_SPD_OUT_CH1_3 + 100;
  else
    output[CH1_3] = rtY.yaw_ANG_OUT_CH1_3;

  if (rtU.yaw_status_CH1_4 == 1)
    output[CH1_4] = rtY.yaw_SPD_OUT_CH1_4 + 100;
  else
    output[CH1_4] = rtY.yaw_ANG_OUT_CH1_4;

  if (rtU.yaw_status_CH1_5 == 1)
    output[CH1_5] = rtY.yaw_SPD_OUT_CH1_5;
  else
    output[CH1_5] = rtY.yaw_ANG_OUT_CH1_5;

  if (rtU.yaw_status_CH1_6 == 1)
    output[CH1_6] = rtY.yaw_SPD_OUT_CH1_6;
  else
    output[CH1_6] = rtY.yaw_ANG_OUT_CH1_6;

  if (rtU.yaw_status_CH1_7 == 1)
    output[CH1_7] = rtY.yaw_SPD_OUT_CH1_7;
  else
    output[CH1_7] = rtY.yaw_ANG_OUT_CH1_7;

  if (rtU.yaw_status_CH2_1 == 1)
    output[CH2_1] = rtY.yaw_SPD_OUT_CH2_1;
  else
    output[CH2_1] = rtY.yaw_ANG_OUT_CH2_1;

  if (rtU.yaw_status_CH2_2 == 1)
    output[CH2_2] = rtY.yaw_SPD_OUT_CH2_2;
  else
    output[CH2_2] = rtY.yaw_ANG_OUT_CH2_2;

  if (rtU.yaw_status_CH2_3 == 1)
    output[CH2_3] = rtY.yaw_SPD_OUT_CH2_3;
  else
    output[CH2_3] = rtY.yaw_ANG_OUT_CH2_3;

  if (rtU.yaw_status_CH2_4 == 1)
    output[CH2_4] = rtY.yaw_SPD_OUT_CH2_4;
  else
    output[CH2_4] = rtY.yaw_ANG_OUT_CH2_4;

  if (rtU.yaw_status_CH2_5 == 1)
    output[CH2_5] = rtY.yaw_SPD_OUT_CH2_5;
  else
    output[CH2_5] = rtY.yaw_ANG_OUT_CH2_5;

  if (rtU.yaw_status_CH2_6 == 1)
    output[CH2_6] = rtY.yaw_SPD_OUT_CH2_6;
  else
    output[CH2_6] = rtY.yaw_ANG_OUT_CH2_6;

  if (rtU.yaw_status_CH2_7 == 1)
    output[CH2_7] = rtY.yaw_SPD_OUT_CH2_7;
  else
    output[CH2_7] = rtY.yaw_ANG_OUT_CH2_7;

  CAN1_cmd_chassis(output[CH1_1], output[CH1_2], output[CH1_3], output[CH1_4], output[CH1_5], output[CH1_6], output[CH1_7], 0);
//  CAN2_cmd_chassis(output[CH2_1], output[CH2_2], output[CH2_3], output[CH2_4], output[CH2_5], output[CH2_6], output[CH2_7], 0);
}
void set_mode(int mode_CH1_1, int mode_CH1_2, int mode_CH1_3, int mode_CH1_4, int mode_CH1_5, int mode_CH1_6, int mode_CH1_7,
              int mode_CH2_1, int mode_CH2_2, int mode_CH2_3, int mode_CH2_4, int mode_CH2_5, int mode_CH2_6, int mode_CH2_7)
{
  rtU.yaw_status_CH1_1 = mode_CH1_1;
  rtU.yaw_status_CH1_2 = mode_CH1_2;
  rtU.yaw_status_CH1_3 = mode_CH1_3;
  rtU.yaw_status_CH1_4 = mode_CH1_4;
  rtU.yaw_status_CH1_5 = mode_CH1_5;
  rtU.yaw_status_CH1_6 = mode_CH1_6;
  rtU.yaw_status_CH1_7 = mode_CH1_7;

  rtU.yaw_status_CH2_1 = mode_CH2_1;
  rtU.yaw_status_CH2_2 = mode_CH2_2;
  rtU.yaw_status_CH2_3 = mode_CH2_3;
  rtU.yaw_status_CH2_4 = mode_CH2_4;
  rtU.yaw_status_CH2_5 = mode_CH2_5;
  rtU.yaw_status_CH2_6 = mode_CH2_6;
  rtU.yaw_status_CH2_7 = mode_CH2_7;
}
void PID_Speed_Para_Init(int channel, int motor, double kp, double ki, double kd)
{
  switch (channel)
  {
  case 1:
    switch (motor)
    {
    case 1:
      rtP.SPD_D_CH1_1 = kd;
      rtP.SPD_I_CH1_1 = ki;
      rtP.SPD_P_CH1_1 = kp;
      break;
    case 2:
      rtP.SPD_D_CH1_2 = kd;
      rtP.SPD_I_CH1_2 = ki;
      rtP.SPD_P_CH1_2 = kp;
      break;
    case 3:
      rtP.SPD_D_CH1_3 = kd;
      rtP.SPD_I_CH1_3 = ki;
      rtP.SPD_P_CH1_3 = kp;
      break;
    case 4:
      rtP.SPD_D_CH1_4 = kd;
      rtP.SPD_I_CH1_4 = ki;
      rtP.SPD_P_CH1_4 = kp;
      break;
    case 5:
      rtP.SPD_D_CH1_5 = kd;
      rtP.SPD_I_CH1_5 = ki;
      rtP.SPD_P_CH1_5 = kp;
      break;
    case 6:
      rtP.SPD_D_CH1_6 = kd;
      rtP.SPD_I_CH1_6 = ki;
      rtP.SPD_P_CH1_6 = kp;
      break;
    case 7:
      rtP.SPD_D_CH1_7 = kd;
      rtP.SPD_I_CH1_7 = ki;
      rtP.SPD_P_CH1_7 = kp;
      break;
    }
    break;
  case 2:
    switch (motor)
    {
    case 1:
      rtP.SPD_D_CH2_1 = kd;
      rtP.SPD_I_CH2_1 = ki;
      rtP.SPD_P_CH2_1 = kp;
      break;
    case 2:
      rtP.SPD_D_CH2_2 = kd;
      rtP.SPD_I_CH2_2 = ki;
      rtP.SPD_P_CH2_2 = kp;
      break;
    case 3:
      rtP.SPD_D_CH2_3 = kd;
      rtP.SPD_I_CH2_3 = ki;
      rtP.SPD_P_CH2_3 = kp;
      break;
    case 4:
      rtP.SPD_D_CH2_4 = kd;
      rtP.SPD_I_CH2_4 = ki;
      rtP.SPD_P_CH2_4 = kp;
      break;
    case 5:
      rtP.SPD_D_CH2_5 = kd;
      rtP.SPD_I_CH2_5 = ki;
      rtP.SPD_P_CH2_5 = kp;
      break;
    case 6:
      rtP.SPD_D_CH2_6 = kd;
      rtP.SPD_I_CH2_6 = ki;
      rtP.SPD_P_CH2_6 = kp;
      break;
    case 7:
      rtP.SPD_D_CH2_7 = kd;
      rtP.SPD_I_CH2_7 = ki;
      rtP.SPD_P_CH2_7 = kp;
      break;
    }
    break;
  }
}

void PID_Angle_S_Para_Init(int channel, int motor, double kp, double ki, double kd)
{
  switch (channel)
  {
  case 1:
    switch (motor)
    {
    case 1:
      rtP.ANG_S_P_CH1_1 = kp;
      rtP.ANG_S_I_CH1_1 = ki;
      rtP.ANG_S_D_CH1_1 = kd;
      break;
    case 2:
      rtP.ANG_S_P_CH1_2 = kp;
      rtP.ANG_S_I_CH1_2 = ki;
      rtP.ANG_S_D_CH1_2 = kd;
      break;
    case 3:
      rtP.ANG_S_P_CH1_3 = kp;
      rtP.ANG_S_I_CH1_3 = ki;
      rtP.ANG_S_D_CH1_3 = kd;
      break;
    case 4:
      rtP.ANG_S_P_CH1_4 = kp;
      rtP.ANG_S_I_CH1_4 = ki;
      rtP.ANG_S_D_CH1_4 = kd;
      break;
    case 5:
      rtP.ANG_S_P_CH1_5 = kp;
      rtP.ANG_S_I_CH1_5 = ki;
      rtP.ANG_S_D_CH1_5 = kd;
      break;
    case 6:
      rtP.ANG_S_P_CH1_6 = kp;
      rtP.ANG_S_I_CH1_6 = ki;
      rtP.ANG_S_D_CH1_6 = kd;
      break;
    case 7:
      rtP.ANG_S_P_CH1_7 = kp;
      rtP.ANG_S_I_CH1_7 = ki;
      rtP.ANG_S_D_CH1_7 = kd;
      break;
    }
    break;
  case 2:
    switch (motor)
    {
    case 1:
      rtP.ANG_S_P_CH2_1 = kp;
      rtP.ANG_S_I_CH2_1 = ki;
      rtP.ANG_S_D_CH2_1 = kd;
      break;
    case 2:
      rtP.ANG_S_P_CH2_2 = kp;
      rtP.ANG_S_I_CH2_2 = ki;
      rtP.ANG_S_D_CH2_2 = kd;
      break;
    case 3:
      rtP.ANG_S_P_CH2_3 = kp;
      rtP.ANG_S_I_CH2_3 = ki;
      rtP.ANG_S_D_CH2_3 = kd;
      break;
    case 4:
      rtP.ANG_S_P_CH2_4 = kp;
      rtP.ANG_S_I_CH2_4 = ki;
      rtP.ANG_S_D_CH2_4 = kd;
      break;
    case 5:
      rtP.ANG_S_P_CH2_5 = kp;
      rtP.ANG_S_I_CH2_5 = ki;
      rtP.ANG_S_D_CH2_5 = kd;
      break;
    case 6:
      rtP.ANG_S_P_CH2_6 = kp;
      rtP.ANG_S_I_CH2_6 = ki;
      rtP.ANG_S_D_CH2_6 = kd;
      break;
    case 7:
      rtP.ANG_S_P_CH2_7 = kp;
      rtP.ANG_S_I_CH2_7 = ki;
      rtP.ANG_S_D_CH2_7 = kd;
      break;
    }
    break;
  }
}

void PID_Angle_A_Para_Init(int channel, int motor, double kp, double ki, double kd)
{
  switch (channel)
  {
  case 1:
    switch (motor)
    {
    case 1:
      rtP.ANG_A_P_CH1_1 = kp;
      rtP.ANG_A_I_CH1_1 = ki;
      rtP.ANG_A_D_CH1_1 = kd;
      break;
    case 2:
      rtP.ANG_A_P_CH1_2 = kp;
      rtP.ANG_A_I_CH1_2 = ki;
      rtP.ANG_A_D_CH1_2 = kd;
      break;
    case 3:
      rtP.ANG_A_P_CH1_3 = kp;
      rtP.ANG_A_I_CH1_3 = ki;
      rtP.ANG_A_D_CH1_3 = kd;
      break;
    case 4:
      rtP.ANG_A_P_CH1_4 = kp;
      rtP.ANG_A_I_CH1_4 = ki;
      rtP.ANG_A_D_CH1_4 = kd;
      break;
    case 5:
      rtP.ANG_A_P_CH1_5 = kp;
      rtP.ANG_A_I_CH1_5 = ki;
      rtP.ANG_A_D_CH1_5 = kd;
      break;
    case 6:
      rtP.ANG_A_P_CH1_6 = kp;
      rtP.ANG_A_I_CH1_6 = ki;
      rtP.ANG_A_D_CH1_6 = kd;
      break;
    case 7:
      rtP.ANG_A_P_CH1_7 = kp;
      rtP.ANG_A_I_CH1_7 = ki;
      rtP.ANG_A_D_CH1_7 = kd;
      break;
    }
    break;
  case 2:
    switch (motor)
    {
    case 1:
      rtP.ANG_A_P_CH2_1 = kp;
      rtP.ANG_A_I_CH2_1 = ki;
      rtP.ANG_A_D_CH2_1 = kd;
      break;
    case 2:
      rtP.ANG_A_P_CH2_2 = kp;
      rtP.ANG_A_I_CH2_2 = ki;
      rtP.ANG_A_D_CH2_2 = kd;
      break;
    case 3:
      rtP.ANG_A_P_CH2_3 = kp;
      rtP.ANG_A_I_CH2_3 = ki;
      rtP.ANG_A_D_CH2_3 = kd;
      break;
    case 4:
      rtP.ANG_A_P_CH2_4 = kp;
      rtP.ANG_A_I_CH2_4 = ki;
      rtP.ANG_A_D_CH2_4 = kd;
      break;
    case 5:
      rtP.ANG_A_P_CH2_5 = kp;
      rtP.ANG_A_I_CH2_5 = ki;
      rtP.ANG_A_D_CH2_5 = kd;
      break;
    case 6:
      rtP.ANG_A_P_CH2_6 = kp;
      rtP.ANG_A_I_CH2_6 = ki;
      rtP.ANG_A_D_CH2_6 = kd;
      break;
    case 7:
      rtP.ANG_A_P_CH2_7 = kp;
      rtP.ANG_A_I_CH2_7 = ki;
      rtP.ANG_A_D_CH2_7 = kd;
      break;
    }
    break;
  }
}
