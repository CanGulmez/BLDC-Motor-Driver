/**
 * BLDC Motor Driver
 */

#include <math.h>
#include <stm32f4xx_hal.h>

/* User-defined global constants */

#define MOTOR_ID              0x01

#define M_PI                  3.14159265358979323846
#define A                     0.001129148
#define B                     0.000234125
#define C                     0.0000000876741
#define FIX_R                 10000.0
#define SHUNT_R               0.001
#define GAIN                  20

#define MAX_TEMP              100.0
#define MAX_CURR              100.0
#define MAX_SPEED             100.0             /* km/h */
#define MAX_TORQUE            100.0
#define MAX_VOLTAGE           100.0
#define MAX_SPEED_INTEGRAL    50.0
#define MAX_CURRENT_INTEGRAL  50.0
#define MAX_TORQUE_INTEGRAL   50.0
 
#define WHEEL_RADIUS          43.18
#define GEAR_RATIO            1.0
#define TORQUE_CONST          1.0
#define VBUS                  60.0
#define RDS_ON                pow(3.7, -3)
#define T_RISE                pow(40, -9)
#define T_FALL                pow(70, -9)
#define VGS                   10.0
#define QGATE                 pow(260, -9)
#define POLE_PAIRS            15
#define PWM_PERIOD            1 / 10000         /* 100us */

#define BLDC_ROTOR_POS_1      0b001             /* 0 to 60 */
#define BLDC_ROTOR_POS_2      0b011             /* 60 to 120 */
#define BLDC_ROTOR_POS_3      0b010             /* 120 to 180 */
#define BLDC_ROTOR_POS_4      0b110             /* 180 to 240 */
#define BLDC_ROTOR_POS_5      0b100             /* 240 to 300 */
#define BLDC_ROTOR_POS_6      0b101             /* 300 to 360 */

#define BLDC_STATE_OFF        0x00
#define BLDC_STATE_ON         0x01
#define BLDC_STATE_FAILURE    0x02

#define BLDC_FRAME_1          0x01
#define BLDC_FRAME_2          0x02

#define DEG(rad)              (rad * 180.0 / M_PI)
#define RAD(deg)              (deg * M_PI / 180.0)
#define VOLT(x)               (x / 4095.0 * 3.3)
#define THROTTLE(x)           (x)                  /* 0.0 - 1.0 */
#define R_NTC(x)              ((VOLT(x) * FIX_R) / (3.3 - VOLT(x)))
#define TEMP(x)               (1.0 / (A + B * log(R_NTC(x)) + C * \
                               pow(log(R_NTC(x)), 3)) - 273.15)
#define CURR(x)               (VOLT(x) / 20 / SHUNT_R)
#define SECTOR(x)             (x)                  /* 1-6 */
#define COUNTER(x)            (__HAL_TIM_GET_COUNTER(x))
#define SPEED(x)              (2.0 * M_PI * WHEEL_RADIUS * x / 60.0 * 3.6) /* km/h */

/* User-defined peripheral configurations */

RCC_OscInitTypeDef iosc;
RCC_ClkInitTypeDef iclk;
GPIO_InitTypeDef igpio;
TIM_HandleTypeDef htim2, htim3, htim1;
TIM_MasterConfigTypeDef sMasterConfig;
TIM_OC_InitTypeDef sConfigOC;
ADC_HandleTypeDef hadc1;
ADC_ChannelConfTypeDef sConfig;
DMA_HandleTypeDef hdma_adc1;
UART_HandleTypeDef huart3;
CAN_HandleTypeDef hcan2;
CAN_TxHeaderTypeDef txHeader;
CAN_RxHeaderTypeDef rxHeader;
CAN_FilterTypeDef filterConfig;

/* User-defined enumerations and structures */

typedef enum {
   FALSE, 
   TRUE
} bool_t;

typedef struct {
   uint8_t frame;
   uint8_t state;
   uint8_t temp;
   uint8_t currA;
   uint8_t currB;
   uint8_t currC;
   uint8_t speed;
   uint8_t torque;
} bldc_mdata1_t;
 
typedef struct {
   uint8_t frame;
   uint8_t RPM;
   uint8_t inputPower;
   uint8_t driverPower;
   uint8_t outputPower;
   uint8_t efficiency;
   uint8_t NA[2];
} bldc_mdata2_t;
 