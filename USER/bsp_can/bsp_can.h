#ifndef BSP_CAN_H
#define BSP_CAN_H
#include <stdint.h>

typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
    CAN_3508_M5_ID = 0x205,
    CAN_3508_M6_ID = 0x206,
    CAN_3508_M7_ID = 0x207,
    CAN_3508_M8_ID = 0x208,

    CAN_2006_M1_ID = 0x201,
    CAN_2006_M2_ID = 0x202,
    CAN_2006_M3_ID = 0x203,
    CAN_2006_M4_ID = 0x204,
    CAN_2006_M5_ID = 0x205,
    CAN_2006_M6_ID = 0x206,
    CAN_2006_M7_ID = 0x207,
    CAN_2006_M8_ID = 0x208,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;

// Modify here to control different motors

// rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    int circle;
} motor_measure_t;

void CAN1_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4, int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);
void CAN2_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4, int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);

void FDCAN1_RX_Filter_Init(void);
void can_filter_init(void);

extern motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
extern motor_measure_t *get_chassis_motor_measure_point1(uint8_t i);

void motor_state_update(void);
void motor_state_update1(void);

#endif
