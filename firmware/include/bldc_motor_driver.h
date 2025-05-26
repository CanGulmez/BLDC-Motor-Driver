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
#define WHEEL_RADIUS          43.18
#define GEAR_RATIO            1.0
#define TORQUE_CONST          1.0
#define VBUS                  60.0
#define POLE_PAIRS            15

#define BLDC_ROTOR_POS_1      0b001             /* 0 to 60 */
#define BLDC_ROTOR_POS_2      0b011             /* 60 to 120 */
#define BLDC_ROTOR_POS_3      0b010             /* 120 to 180 */
#define BLDC_ROTOR_POS_4      0b110             /* 180 to 240 */
#define BLDC_ROTOR_POS_5      0b100             /* 240 to 300 */
#define BLDC_ROTOR_POS_6      0b101             /* 300 to 360 */

#define BLDC_STATE_OFF        0x00
#define BLDC_STATE_ON         0x01
#define BLDC_STATE_FAILURE    0x02

#define DEG(rad)              (rad * 180.0 / M_PI)
#define RAD(deg)              (deg * M_PI / 180.0)
#define VOLT(x)               (x / 4095.0 * 3.3)
#define R_NTC(x)              ((VOLT(x) * FIX_R) / (3.3 - VOLT(x)))
#define TEMP(x)               (1.0 / (A + B * log(R_NTC(x)) + C * \
                               pow(log(R_NTC(x)), 3)) - 273.15)
#define CURR(x)               (VOLT(x) / 20 / SHUNT_R)
#define SECTOR(x)             ()
#define COUNTER(x)            (__HAL_TIM_GET_COUNTER(x))
#define SPEED(x)              (2.0 * M_PI * WHEEL_RADIUS * x / 60.0)

/* User-defined peripheral configurations */

RCC_OscInitTypeDef iosc;
RCC_ClkInitTypeDef iclk;
GPIO_InitTypeDef igpio;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_MasterConfigTypeDef sMasterConfig;
ADC_HandleTypeDef hadc1;
ADC_ChannelConfTypeDef sConfig;
DMA_HandleTypeDef hdma_adc1;
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
   int a;
} bldc_mdata_t;

/* User-defined peripheral configuration methods */

void bldc_perip_oscillator(void);
void bldc_perip_clock(void);
void bldc_perip_timer(void);
void bldc_perip_analog(void);
void bldc_perip_digital(void);
void bldc_perip_comm(void);

