//#include "UPPER_LOCATION.h" // Device header

//TGT_COOR TC;
//REAL_COOR RC;
//extern uint8_t USART2_RX_BUF[100];
//extern motor_measure_t *motor_data[8];
//uint8_t data[10];

//double factors1 = 2;
//double factors2 = 50;
//double por=2.5;

//double deadband = 50;
//double top = 4000;
//double a1, a2, a3, a4, a5, a6;

//// void Receive(uint8_T RX, uint8_T RY, uint8_T Rtheta, uint8_T TX, uint8_T TY, uint8_T Ttheta ,
////	           uint8_T RX1,uint8_T RY1,uint8_T Rtheta1,uint8_T TX1,uint8_T TY1,uint8_T Ttheta1){

////
////	RC.theta=(Rtheta << 8) | Rtheta1;
////	RC.x=    (RX << 8)     | RX1;
////	RC.y=    (RY << 8)     | RY1;
////	TC.x=    (TX << 8)     | TX1;
////	TC.y=    (TY << 8)     | TY1;
////	TC.theta=(Ttheta << 8) |Ttheta1;
////
//////
//////	RC.theta= Rtheta;
//////	RC.x=     RX;
//////	RC.y=     RY;
//////	TC.x=     TX;
//////	TC.y=     TY;
//////	TC.theta=Ttheta;
//////
//////
//////	RC.theta = (RC.theta & 0x8000) ? (RC.theta | 0xFFFF0000) : RC.theta;
//////  RC.x = (RC.x & 0x8000) ? (RC.x | 0xFFFF0000) : RC.x;
//////  RC.y = (RC.y & 0x8000) ? (RC.y | 0xFFFF0000) : RC.y;
//////  TC.x = (TC.x & 0x8000) ? (TC.x | 0xFFFF0000) : TC.x;
//////  TC.y = (TC.y & 0x8000) ? (TC.y | 0xFFFF0000) : TC.y;
//////  TC.theta = (TC.theta & 0x8000) ? (TC.theta | 0xFFFF0000) : TC.theta;
////}
//void Receive()
//{
//  RC.x = (USART2_RX_BUF[2] << 8) | USART2_RX_BUF[1];
//  RC.y = (USART2_RX_BUF[4] << 8) | USART2_RX_BUF[3];
//  RC.theta = (USART2_RX_BUF[6] << 8) | USART2_RX_BUF[5];
//	
//	RC.xll=RC.xlast;
//	RC.yll=RC.ylast;
//	
//  RC.xlast = RC.x;
//  RC.ylast = RC.y;
//  TC.x = (USART2_RX_BUF[8] << 8) | USART2_RX_BUF[7];
//  TC.y = (USART2_RX_BUF[10] << 8) | USART2_RX_BUF[9];
//  TC.theta = (USART2_RX_BUF[12] << 8) | USART2_RX_BUF[11];

//  RC.action = (USART2_RX_BUF[14] << 8) | USART2_RX_BUF[13];
//}
//void Reach_TGT()
//{
//	RC.distll=RC.distlast;
//	RC.distlast=RC.dist;
//  RC.dist = sqrt(pow(TC.y - RC.y, 2) + pow((TC.x - RC.x), 2));
//  TC.XYtheta = atan2(TC.y - RC.y, TC.x - RC.x) * 180 / PI;

//  if (fabs((double)RC.dist) >= deadband && fabs((double)RC.dist) < 300)
//  {
//		factors1=2;
//    RC.dist = 1300 / factors1;
//  }
//	else if(fabs((double)RC.dist) >300)
//	{
//		factors1=4;
//	}
//  if (fabs((double)RC.dist) > top)
//  {

//    RC.dist = top;
//  }

//  RC.RE_theta = TC.XYtheta - RC.theta;
//  if (fabs((double)RC.RE_theta) < 1)
//  {
//    RC.RE_theta = 0;
//  }
//  if (fabs((double)RC.dist) < deadband)
//  {
//		RC.Vx=0;
//		RC.Vy=0;
//  }else{
//  RC.Vx = ((RC.dist+(RC.dist-RC.distlast)*por) * cos(RC.RE_theta * PI / 180)) * factors1;
//  RC.Vy = ((RC.dist+(RC.dist-RC.distlast)*por) * sin(RC.RE_theta * PI / 180)) * factors1;
//	}
//  //  a1=cos(RC.RE_theta*PI/180);
//  //	a2=sin(RC.RE_theta*PI/180);
//  //	    if(RC.omega!=0)
//  //		{
//  //			RC.Vx=0;
//  //			RC.Vy=0;
//  //		}
//  if (TC.theta - RC.theta > 181)
//    TC.theta = TC.theta - 360;
//  if (TC.theta - RC.theta < -181)
//    TC.theta = TC.theta + 360;
//  TC.theta = TC.theta % 360;

//  if (fabs((double)TC.theta - RC.theta) > 3)
//    RC.omega = (TC.theta - RC.theta) * factors2;
//  else
//    RC.omega = 0;

//  if (fabs((double)RC.omega) > 5000)
//  {
//    if (RC.omega > 0)
//      RC.omega = 5000;
//    else
//      RC.omega = -5000;
//  }
//  else if (fabs((double)RC.omega) < 650)
//  {
//    if (RC.omega > 0)
//      RC.omega = 650;
//    else if (RC.omega < 0)
//      RC.omega = -650;
//    else
//      RC.omega = 0;
//  }
//  RC.omega = -RC.omega;
//} // 最后把RC.Vx，RC.Vy，RC.omege三个参数放进main接收就行

//void CAL_TXMESSAGE(void)
//{

//  data[0] = 0x0F;
//  for (int i = 0; i < 4; i++)
//  {

//    data[2 * i + 1] = motor_data[i]->speed_rpm >> 8;
//    data[2 * i + 2] = motor_data[i]->speed_rpm;
//  }
//  data[9] = 0xAA;
//}
