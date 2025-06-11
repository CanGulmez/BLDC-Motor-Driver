/**
 * BLDC Motor Driver 
 */

#include "bldc_motor_driver.h"

/* Initial motor configurations and variables that set by HAL */

volatile bool_t failure       = FALSE;
volatile char state           = BLDC_STATE_OFF;
uint32_t buffer[5]            = {0};
bldc_mdata1_t mdata1          = {0};
bldc_mdata2_t mdata2          = {0};
uint32_t txMailbox            = {0};

/* These variables will be changed constantly so that they are defined 
   as 'volatile'. This will prevent the compiler from optimizing them. */

volatile double temp;                                
volatile double currA, currB, currC, maxCurr;                  
volatile double throttle;                              
volatile double clarkeA, clarkeB;                       
volatile double actParkD, actParkQ;
volatile double refParkD, refParkQ;
volatile char hallA, hallB, hallC, hallPos, sector;   
volatile uint32_t currStamp, diffStamp;                
static uint32_t lastStamp, lastSector;               
volatile double eRPM, mRPM, actSpeed, refSpeed;
volatile double posDeg, posRad;                       
volatile double actTorque, refTorque;
volatile double speedErr, parkDErr, parkQErr;                  
static double speedIn, parkDIn, parkQIn;
volatile double Ki, Kp;
volatile double VParkD, VParkQ, VClarkeA, VClarkeB, Va, Vb, Vc;
volatile double VMax, VMin, VOffset, dutyA, dutyB, dutyC;
volatile double driverPower, inputPower, outputPower, efficiency;

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
   __HAL_RCC_TIM1_CLK_ENABLE();
   __HAL_RCC_CAN1_CLK_ENABLE();
   __HAL_RCC_CAN2_CLK_ENABLE();
   __HAL_RCC_SYSCFG_CLK_ENABLE();
 
   /* Initialize and configure peripherals */
   bldc_perip_oscillator();
   bldc_perip_clock();
   bldc_perip_timer();
   bldc_perip_analog();
   bldc_perip_digital();
   bldc_perip_serial();
   bldc_perip_comm();

   /* Set the initial conditon for PWMs */
   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0.0);
   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0.0);
   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0.0);
   
   /* Start the peripherals */
   HAL_TIM_Base_Start(&htim2);
   HAL_TIM_Base_Start(&htim3);
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
   HAL_ADC_Start_DMA(&hadc1, &buffer, 5);
   HAL_CAN_Start(&hcan2);
   HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
   
   /* Make the default settings until any failure condition occurs */
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
   state = BLDC_STATE_ON;

   while (1) {

      /* Pack the required motor data to send over CAN */
      mdata1.frame = (uint8_t) BLDC_FRAME_1;
      mdata1.state = (uint8_t) state;
      mdata1.temp = (uint8_t) temp;
      mdata1.currA = (uint8_t) currA;
      mdata1.currB = (uint8_t) currB;
      mdata1.currC = (uint8_t) currC;
      mdata1.speed = (uint8_t) actSpeed;
      mdata1.torque = (uint8_t) actTorque;
      mdata2.RPM = (uint8_t) mRPM;
      maxCurr = fmax(fabs(currA), fmax(fabs(currB), fabs(currC)));
      mdata2.inputPower = (uint8_t) (VBUS * maxCurr / 1000.0); 
      mdata2.driverPower = (uint8_t) (pow(maxCurr, 2) * RDS_ON + 0.5 * VBUS * 
         maxCurr * (T_RISE + T_FALL) * 10000.0 + QGATE * VGS * 10000) / 1000.0;
      mdata2.outputPower = (uint8_t) (mdata2.inputPower - mdata2.driverPower);
      mdata2.efficiency = (uint8_t) ((mdata2.outputPower / mdata2.inputPower));
      mdata2.NA[0] = 0;
      mdata2.NA[1] = 0;
      mdata2.NA[2] = 0;

      /* Send the motor data to vehicle control system */
      txHeader.StdId = MOTOR_ID;
      txHeader.ExtId = 0;
      txHeader.IDE = CAN_ID_STD;
      txHeader.RTR = CAN_RTR_DATA;
      txHeader.DLC = sizeof(bldc_mdata1_t);
      txHeader.TransmitGlobalTime = DISABLE;
      HAL_CAN_AddTxMessage(&hcan2, &txHeader, (uint8_t*) &mdata1, &txMailbox);
      HAL_CAN_AddTxMessage(&hcan2, &txHeader, (uint8_t*) &mdata2, &txMailbox);

      /* Stop the motor driver due to detected shutdown state */
      if (failure == TRUE) {
         state = BLDC_STATE_FAILURE;
         speedIn = 0;
         parkDIn = 0; 
         parkQIn = 0;
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
         HAL_TIM_Base_Stop(&htim2);
         HAL_TIM_Base_Stop(&htim3);
         HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
         HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
         HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
         HAL_CAN_Stop(&hcan2);
         HAL_ADC_Stop_DMA(&hadc1); 
      }
      
      /* Delay for a while... */
      HAL_Delay(1000);
   }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
   if (hadc->Instance == ADC1) {
      
      /* Convert the ADC buffer to corresponding data */
      currA = CURR(buffer[0]);
      currB = CURR(buffer[1]);
      currC = CURR(buffer[2]);
      throttle = THROTTLE(buffer[3]);
      temp = TEMP(buffer[4]);

      /* Calculate the position in radians and degrees */
      posDeg = ((lastSector - 1) * 60.0) + 
         (((COUNTER(&htim3) - currStamp) / diffStamp) * 60.0);
      if (posDeg > 360.0)
         posDeg -= 360.0;
      posRad = RAD(posDeg);

      /* Apply Clarke and Park transformations onto phase currents */
      clarkeA = (1.0 / 3.0) * (2 * currA - currB - currC);
      clarkeB = (sqrt(3.0) / 3.0) * (currB - currC);
      actParkD = clarkeA * cos(posRad) + clarkeB * sin(posRad);
      actParkQ = -clarkeA * sin(posRad) + clarkeB * cos(posRad);
 
      /* Indicate the referance speed and actual torque */
      refSpeed = MAX_SPEED * throttle;
      actTorque = actParkQ * TORQUE_CONST;

      Kp = 0.1;
      Ki = 0.02;
      speedIn = 0.0;
      speedErr = refSpeed - actSpeed;
      speedIn += speedErr;
      if (speedIn > MAX_SPEED_INTEGRAL) speedIn = MAX_SPEED_INTEGRAL;
      if (speedIn < -MAX_SPEED_INTEGRAL) speedIn = -MAX_SPEED_INTEGRAL;
      refTorque = Kp * speedErr + Ki * speedIn;
      if (refTorque > MAX_TORQUE) refTorque = MAX_TORQUE;
      if (refTorque < -MAX_TORQUE) refTorque = -MAX_TORQUE;

      refParkD = 0.0;
      refParkQ = refTorque / TORQUE_CONST;
 
      Kp = 2.0;
      Ki = 100.0;
      parkDErr = refParkD - actParkD;
      parkQErr = refParkQ - actParkQ;
      parkDIn += parkDErr;
      parkQIn += parkQErr;
      if (parkDIn > MAX_CURRENT_INTEGRAL) parkDIn = MAX_CURRENT_INTEGRAL;
      if (parkDIn < -MAX_CURRENT_INTEGRAL) parkDIn = -MAX_CURRENT_INTEGRAL;
      if (parkQIn > MAX_CURRENT_INTEGRAL) parkQIn = MAX_CURRENT_INTEGRAL;
      if (parkQIn < -MAX_CURRENT_INTEGRAL) parkQIn = -MAX_CURRENT_INTEGRAL;
      VParkD = Kp * parkDErr + Ki * parkDIn;
      VParkQ = Kp * parkQErr + Ki * parkQIn;

      if (VParkD > MAX_VOLTAGE) VParkD = MAX_VOLTAGE;
      if (VParkD < MAX_VOLTAGE) VParkD = -MAX_VOLTAGE;
      if (VParkQ > MAX_VOLTAGE) VParkQ = MAX_VOLTAGE;
      if (VParkQ < MAX_VOLTAGE) VParkQ = -MAX_VOLTAGE;

      /* Apply inverse Clarke/Park transformations */
      VClarkeA = VParkD * cos(posRad) - VParkQ * sin(posRad);
      VClarkeB = VParkD * sin(posRad) + VParkQ * cos(posRad);
      Va = VClarkeA;
      Vb = -0.5 * VClarkeA + (sqrt(3.0) / 2.0) * VClarkeB;
      Vc = -0.5 * VClarkeA - (sqrt(3.0) / 2.0) * VClarkeB;

      /* Set the voltages which will be applied */
      VMax = fmax(Va, fmax(Vb, Vc));
      VMin = fmin(Va, fmin(Vb, Vc));
      VOffset = 0.5 * (VMax + VMin);
      Va -= VOffset;
      Vb -= VOffset;
      Vc -= VOffset; 

      /* Determine the phase duty cycles using phase voltages */
      dutyA = 0.5 + (Va / VBUS);
      dutyB = 0.5 + (Vb / VBUS);
      dutyC = 0.5 + (Vc / VBUS);      
      if (dutyA > 1.0) dutyA = 1.0;
      if (dutyA < 0.0) dutyA = 0.0;
      if (dutyB > 1.0) dutyB = 1.0;
      if (dutyB < 0.0) dutyB = 0.0;
      if (dutyC > 1.0) dutyC = 1.0;
      if (dutyC < 0.0) dutyC = 0.0;

      /* Update the duty cycles of timer for three PWM channels  */
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dutyA * 
         (htim1.Init.Period + 1));
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dutyB * 
         (htim1.Init.Period + 1));
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dutyC * 
         (htim1.Init.Period + 1));

      /* Check the possible shutdown state */
      if (temp > MAX_TEMP || currA > MAX_CURR || 
          currB > MAX_CURR || currC > MAX_CURR)
         failure = TRUE;
   }
}

void EXTI9_5_IRQHandler(void)
{
   /* IRQs for all hall sensors (PC6, PC7, PC8) */
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
      lastSector = SECTOR(hallPos);

      /* Make some calculations using the hall transations */
      currStamp = COUNTER(&htim3);
      diffStamp = currStamp - lastStamp;
      lastStamp = currStamp;
      eRPM = 60.0 / (6.0 * diffStamp / 1e6);
      mRPM = eRPM / POLE_PAIRS;
      actSpeed = SPEED(mRPM);
   }
}
 
void SysTick_Handler(void) 
{
   HAL_IncTick();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
   if (hcan->Instance == CAN2) {

      /* Receive the message from EV control system over CAN */
      CAN_RxHeaderTypeDef RxHeader;
      uint8_t RxData[8];
   }
}

void CAN2_RX0_IRQHandler(void) 
{
   HAL_CAN_IRQHandler(&hcan2);
}
