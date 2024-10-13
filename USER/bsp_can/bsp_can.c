/**
 *   @brief ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
 **/
#include "bsp_can.h"
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1; // CAN����1


static motor_measure_t motor_chassis[8];

FDCAN_RxHeaderTypeDef temp;

motor_measure_t *motor_data[8];


static FDCAN_TxHeaderTypeDef can1_tx_message_front; // can1前八个
static FDCAN_TxHeaderTypeDef can1_tx_message_last; // can1前八个


static uint8_t can1_send_front_data[8]; // can1前八个
static uint8_t can1_send_last_data[8]; // can2前八个
//static uint8_t can2_send__front_data[8]; // can1后八个
//static uint8_t can2_send_last_data[8]; // can2后八个

/// @brief
#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
  }

void circle_cc(motor_measure_t *ptr)
{

  if (((ptr)->ecd - (ptr)->last_ecd) > 5000)
  {
    ptr->circle -= 1;
  }
  else if (((ptr)->ecd - (ptr)->last_ecd) < -5000)
  {

    ptr->circle += 1;
  }
}

/*
������ݣ�      0:���̵��1 3508���,              1:���̵��2 3508���,
                2:���̵��3 3508���,              3:���̵��4 3508���;
                4:yaw��̨��� 6020���;            5:pitch��̨��� 6020���;
                6:������� 2006���
*/

/// @brief hal��CAN�ص�����,���յ������
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)//接收回调函数
{
  if (hfdcan == &hfdcan1)
  {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
		temp=rx_header;
    switch (rx_header.Identifier)
    {
    case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    case CAN_3508_M5_ID:
    case CAN_3508_M6_ID:
    case CAN_3508_M7_ID:
    case CAN_3508_M8_ID:
    {

      static uint8_t i = 0;
      // get motor id
      i = rx_header.Identifier - CAN_3508_M1_ID;
      get_motor_measure(&motor_chassis[i], rx_data);
      circle_cc(&motor_chassis[i]);
      break;
    }
    default:
    {
      break;
    }
    }
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);



  }

 
}

/**
 * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
 * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
 */

void CAN1_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4, int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
  can1_tx_message_front.Identifier = CAN_CHASSIS_ALL_ID;
	can1_tx_message_front.IdType = FDCAN_STANDARD_ID;
  can1_tx_message_front.TxFrameType = FDCAN_DATA_FRAME;
  can1_tx_message_front.DataLength = FDCAN_DLC_BYTES_8;
  can1_send_front_data[0] = motor1 >> 8;
  can1_send_front_data[1] = motor1;
  can1_send_front_data[2] = motor2 >> 8;
  can1_send_front_data[3] = motor2;
  can1_send_front_data[4] = motor3 >> 8;
  can1_send_front_data[5] = motor3;
  can1_send_front_data[6] = motor4 >> 8;
  can1_send_front_data[7] = motor4;

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can1_tx_message_front, can1_send_front_data);

  can1_tx_message_last.Identifier = CAN_GIMBAL_ALL_ID;
	can1_tx_message_last.IdType = FDCAN_STANDARD_ID;
  can1_tx_message_last.TxFrameType = FDCAN_DATA_FRAME;
  can1_tx_message_last.DataLength = FDCAN_DLC_BYTES_8;
  can1_send_last_data[0] = motor5 >> 8;
  can1_send_last_data[1] = motor5;
  can1_send_last_data[2] = motor6 >> 8;
  can1_send_last_data[3] = motor6;
  can1_send_last_data[4] = motor7 >> 8;
  can1_send_last_data[5] = motor7;
  can1_send_last_data[6] = motor8 >> 8;
  can1_send_last_data[7] = motor8;

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can1_tx_message_last, can1_send_last_data);
}

// ���� 3508�������ָ��
motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis[(i)];
}




// void FDCAN1_RX_Filter_Init(void)
// {
// 	 FDCAN_FilterTypeDef hfdcan1_RX_Filter;   /* FDCAN1滤波器0对象句柄 */
   
// 	 hfdcan1_RX_Filter.IdType = FDCAN_STANDARD_ID;              /* 只接收标准帧ID */
//    hfdcan1_RX_Filter.FilterIndex = 0;                         /* 滤波器索引0 */
//    hfdcan1_RX_Filter.FilterType = FDCAN_FILTER_MASK;          /* 滤波器类型 */
//    hfdcan1_RX_Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  /* 滤波器关联到RXFIFO0 */
//    hfdcan1_RX_Filter.FilterID1 = 0x111;                       /* 滤波ID1: 0x00 */
//    hfdcan1_RX_Filter.FilterID2 = 0x7FF; /* 滤波ID2: 0x00 */
//    HAL_FDCAN_ConfigFilter(&hfdcan1,&hfdcan1_RX_Filter);       /* 看看滤波器有没有创建成功 */
//    /* HAL_FDCAN_ConfigGlobalFilter()
//     * 参数2：设置标准帧ID，接收的报文ID没有匹配上滤波器时，选择拒绝接收(没有匹配上时,可以选择放入FIFO0或者FIFO1)。
//     * 参数3：设置拓展帧ID，接收的报文ID没有匹配上滤波器时，选择拒绝接收。
//     * 参数4：设置是否拒绝远程标准帧，ENABLE代表拒绝接收。
//     * 参数5：设置是否拒绝远程拓展帧，ENABLE代表拒绝接收。
//     */
//    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,DISABLE,ENABLE); /* 设置FDCAN1滤波器0全局配置  */
// 	 HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
// }
void can_filter_init(void)
{
   FDCAN_FilterTypeDef filter;                   	//< 声明局部变量 can过滤器结构体
	filter.IdType       = FDCAN_STANDARD_ID;       	//< id设置为标准id
	filter.FilterIndex  = 0;                      	//< 设值筛选器的编号，标准id选择0-127
	filter.FilterType   = FDCAN_FILTER_MASK;       	//< 设置工作模式为掩码模式
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; 	//< 将经过过滤的数据存储到 fifo0
	filter.FilterID1    = 0x000;                   	//< 筛选器的id
	filter.FilterID2    = 0x000;
	
	HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);   //< 配置过滤器	
  HAL_FDCAN_Start(&hfdcan1);                   //< 使能can
    //该check来测试can控制器是否使能，可以把该赋值去掉	
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // 使能fifo0接收到新信息中断

  // HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter_st);
  // HAL_FDCAN_Start(&hfdcan2);
  // HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

//void FDCAN2_RX_Filter_Init(void)
//{
//	 FDCAN_FilterTypeDef hfdcan2_RX_Filter;   /* FDCAN2滤波器0对象句柄 */
//   
//	 hfdcan2_RX_Filter.IdType = FDCAN_STANDARD_ID;              /* 只接收标准帧ID */
//   hfdcan2_RX_Filter.FilterIndex = 0;                         /* 滤波器索引0 */
//   hfdcan2_RX_Filter.FilterType = FDCAN_FILTER_MASK;          /* 滤波器类型(允许接收报文的ID范围是FilterID1至FilterID2 */
//   hfdcan2_RX_Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;  /* 滤波器关联到RXFIFO1 */
//   hfdcan2_RX_Filter.FilterID1 = 0x111;                       /* 滤波ID1: 0x00 */
//   hfdcan2_RX_Filter.FilterID2 = 0x7FF;                      /* 滤波ID2:   0x00 */
//   HAL_FDCAN_ConfigFilter(&hfdcan2,&hfdcan2_RX_Filter);       /* 看看滤波器有没有创建成功 */
// 
//   /* HAL_FDCAN_ConfigGlobalFilter()
//    * 参数2：设置标准帧ID，接收的报文ID没有匹配上滤波器时，选择拒绝接收(没有匹配上时,可以选择放入FIFO0或者FIFO1)。
//    * 参数3：设置拓展帧ID，接收的报文ID没有匹配上滤波器时，选择拒绝接收。
//    * 参数4：设置是否拒绝远程标准帧，ENABLE代表拒绝接收。
//    * 参数5：设置是否拒绝远程拓展帧，ENABLE代表拒绝接收。
//    */
//   HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,FDCAN_REJECT,FDCAN_REJECT,DISABLE,ENABLE); /* 设置FDCAN1滤波器1全局配置  */
//	 HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO1_NEW_MESSAGE,0);
//}



void motor_state_update()
{

  motor_data[0] = get_chassis_motor_measure_point(0);
  motor_data[1] = get_chassis_motor_measure_point(1);
  motor_data[2] = get_chassis_motor_measure_point(2);
  motor_data[3] = get_chassis_motor_measure_point(3);
  motor_data[4] = get_chassis_motor_measure_point(4);
  motor_data[5] = get_chassis_motor_measure_point(5);
  motor_data[6] = get_chassis_motor_measure_point(6);
  motor_data[7] = get_chassis_motor_measure_point(7);
}


//*******************************************************************�¼�
