/**
 * BLDC Motor Driver 
 */

#include "bldc_motor_driver.h"

/* Initial motor configurations and variables that set by HAL */

bool_t failure                = FALSE;
char state                    = BLDC_STATE_OFF;
uint32_t buffer[6]            = {0};
bldc_mdata_t mdata            = {0};

/* These variables will be changed constantly so that they are defined 
   as 'volatile'. This will prevent the compiler from optimizing them. */

volatile double tempA, tempB, tempC;                  /* Phase temperature */
volatile double currA, currB, currC;                  /* Phase current */   
volatile double clarkeA, clarkeB;                     /* Clarke transformation */ 
volatile double parkD, parkQ;                         /* Park transformation */  
volatile char hallA, hallB, hallC, hallPos, sector;   /* Hall sensor states */
volatile uint32_t currStamp, diffStamp, lastStamp;    /* Timing stuffs */
volatile uint32_t lastSector;                         /* Rotor sector */
volatile double eRPM, mRPM, speed;                    /* RPM and speed */ 
volatile double posDeg, posRad;                       /* Position in deg and rad */
volatile double torque;                               /* Torque */

void main(void)
{
   HAL_Init(); 

   /* Enable the peripheral clocks */
   __HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_GPIOB_CLK_ENABLE();
   __HAL_RCC_GPIOC_CLK_ENABLE();
   __HAL_RCC_ADC1_CLK_ENABLE();
   __HAL_RCC_DMA1_CLK_ENABLE();
   __HAL_RCC_TIM2_CLK_ENABLE();
   __HAL_RCC_TIM3_CLK_ENABLE();
   __HAL_RCC_CAN2_CLK_ENABLE();
   __HAL_RCC_SYSCFG_CLK_ENABLE();
 
   /* Initialize and configure peripherals */
   bldc_perip_oscillator();
   bldc_perip_clock();
   bldc_perip_timer();
   bldc_perip_analog();
   bldc_perip_digital();
   bldc_perip_comm();

   /* Start the peripherals */
   HAL_TIM_Base_Start(&htim2);
   HAL_TIM_Base_Start(&htim3);
   HAL_CAN_Start(&hcan2);
   HAL_ADC_Start_DMA(&hadc1, &buffer, 6);
   
   /* Make the default settings until any failure condition occurs */
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
   state = BLDC_STATE_ON;

   while (1) {

      /* Send the motor data to vehicle control system */

      /* Stop the motor driver due to detected shutdown state */
      if (failure == TRUE) {
         state = BLDC_STATE_FAILURE;
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
         HAL_TIM_Base_Stop(&htim2);
         HAL_TIM_Base_Stop(&htim3);
         HAL_CAN_Stop(&hcan2);
         HAL_ADC_Stop_DMA(&hadc1); 
      }
      
      /* Delay the each interation by 1 second */
      HAL_Delay(1000);
   }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
   if (hadc->Instance == ADC1) {
      
      /* Extract and calculate the required parameters */
      tempA = TEMP(buffer[0]);
      tempB = TEMP(buffer[1]);
      tempC = TEMP(buffer[2]);
      currA = CURR(buffer[3]);
      currB = CURR(buffer[4]);
      currC = CURR(buffer[5]);

      posDeg = ((lastSector - 1) * 60.0) + 
         (((COUNTER(&htim3) - currStamp) / diffStamp) * 60.0);
      if (posDeg > 360.0)
         posDeg -= 360.0;
      posRad = RAD(posDeg);

      clarkeA = (1.0 / 3.0) (2 * currA - currB - currC);
      clarkeB = (sqrt(3.0) / 3.0) * (currB - currC);

      parkD = clarkeA * cos(posRad) + clarkeB * sin(posRad);
      parkQ = -clarkeA * sin(posRad) + clarkeB * cos(posRad);

      torque = parkQ * TORQUE_CONST;
 
      /* Run the motor control loop */

      /* Pack the required data into a frame to send over CAN */
 
      /* Check the possible shutdown state */
      if (tempA > MAX_TEMP || tempB > MAX_TEMP || tempC > MAX_TEMP || 
          currA > MAX_CURR || currB > MAX_CURR || currC > MAX_CURR)
         failure = TRUE;
   }
}

void EXTI9_5_IRQHandler(void)
{
   /* IRQs for all hall sensors (PA6, PA7, PA8) */
   HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
   HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
   HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
}
 
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
   if (GPIO_Pin == GPIO_PIN_6 || GPIO_Pin == GPIO_PIN_7 || 
       GPIO_Pin == GPIO_PIN_8) {

      /* Read the Hall sensors and then map the position */
      hallA = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
      hallB = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
      hallC = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);
      hallPos = (hallA << 2) | (hallB << 1) | hallC;

      /* Make some calculations using the hall transations */
      currStamp = COUNTER(&htim3);
      diffStamp = currStamp - lastStamp;
      lastStamp = currStamp;
      eRPM = 60.0 / (6.0 * diffStamp / 1e6);
      mRPM = eRPM / POLE_PAIRS;
      speed = SPEED(mRPM);
      lastSector = SECTOR(hallPos);
   }
}
 