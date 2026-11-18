#include "stm32f1xx.h"                                                     
#include "system_stm32f1xx.h"                                                           
#include <stdint.h>                                                      
#include <stdio.h>                                                  

#define LCD_I2C_ADDRESS       0x27U                                                   
#define ADC_FULL_SCALE        4095U                                       
#define ADC_VDDA_MV           3200U                                                                    
#define TPS_DIVIDER_SCALE_NUM 3U                                                                         
#define TPS_DIVIDER_SCALE_DEN 1U                                       
#define TEMP_SIM_MAX_C        150U                                             
#define FAN_ON_TEMP_C         80U                                             
#define TPS_VTA1_IDLE_RAW     424U                                                       
#define PEDAL_IDLE_RAW        80U                                                     
#define PEDAL_ADC_MIN_RAW     0U                                                              
#define PEDAL_ADC_MAX_RAW     4059U                                                           
#define TPS_VTA1_OPEN_RAW     2280U                                                 
#define TPS_VTA2_IDLE_RAW     1092U                                                 
#define TPS_VTA2_OPEN_RAW     2740U                                                 
#define TPS_VTA2_MIN_RAW      800U                                                             
#define TPS_VTA2_MAX_RAW      2957U                                                      
#define TPS_VTA1_MIN_RAW      300U                                                             
#define TPS_VTA1_MAX_RAW      2559U                                                      
#define TPS_CONTROL_PERIOD_MS 20U                                                   
#define TPS_ADC_AVERAGE_SAMPLES 20U                                                       
#define TPS_POSITION_DEADBAND 0.02f                                                       
#define TPS_PWM_MAX_PERMILLE  600U                                                       
#define TPS_PWM_MIN_PERMILLE  100U                                                  
#define TPS_PID_KP            80.0f                                       
#define TPS_PID_KI            2.0f                                       
#define TPS_PID_KD            1.0f                                       
#define TPS_PID_I_LIMIT       30.0f                                       
#define UART_BAUD_RATE        115200U                                            
#define UART_TX_TIMEOUT       1000000U                                             
#define I2C_TIMEOUT           100000U                                          
#define SWITCH_PIN            0U                                                               
#define LCD_BACKLIGHT_BIT     0x08U                                             
#define LCD_ENABLE_BIT        0x04U                                          
#define LCD_REGISTER_SELECT   0x01U                                      

static volatile uint16_t g_adc_samples[4];                                                                      
static uint16_t g_tps_adc_history[3][TPS_ADC_AVERAGE_SAMPLES];                                                 
static uint32_t g_tps_adc_sum[3];                                        
static uint16_t g_tps_adc_average[3];                                                      
static uint8_t g_tps_adc_index;                                             
static uint8_t g_tps_adc_count;                                     
static uint8_t g_tps_sensor_fault;                                                
static uint8_t g_tps_fault_waiting;                                                           
static uint8_t g_tps_armed;                                                           
static volatile uint8_t g_switch_event;                                               
static uint8_t g_engine_running;                                                    
static uint8_t g_lcd_available;                                                      
static uint8_t g_tps_sensor_ok;                                                               
static uint8_t g_tps_motor_active;                                                               
static uint8_t g_tps_direction;                                                            
static uint16_t g_tps_feedback_percent;                                                  
static float g_tps_pid_integral;                                        
static float g_tps_previous_position;                                           
static uint16_t g_tps_control_elapsed_ms;                                               

static void gpio_set_config(GPIO_TypeDef *port, uint8_t pin, uint8_t config);                                            
static int system_clock_init(void);                                             
static void tim2_delay_init(void);                                              
static void delay_ms(uint32_t milliseconds);                                          
static void gpio_init(void);                                                         
static void exti0_init(void);                                                     
static void tim3_pwm_init(void);                                          
static void tim3_set_duty(uint16_t permille);                                              
static void tim4_throttle_pwm_init(void);                                                               
static void throttle_bridge_disable(void);                                                           
static void throttle_bridge_drive(uint8_t direction, uint16_t duty_permille);                                               
static uint8_t throttle_sensor_check(uint16_t vta1_raw, uint16_t vta2_raw);                                                       
static float throttle_normalize_position(uint16_t raw, uint16_t rest, uint16_t full);                                   
static uint8_t throttle_feedback_percent(uint16_t vta1_raw, uint16_t vta2_raw);                                                 
static void throttle_adc_average_update(void);                                      
static void throttle_control_update(uint16_t pedal_raw, uint16_t vta1_raw, uint16_t vta2_raw);                                                         
static void adc1_dma_init(void);                                                     
static void i2c1_init(void);                                          
static int i2c_wait_sr1(uint32_t flag);                                                 
static int pcf8574_write(uint8_t value);                                     
static int lcd_write_nibble(uint8_t nibble, uint8_t rs);                                                   
static int lcd_write_byte(uint8_t value, uint8_t rs);                                                   
static void lcd_init(void);                                        
static int lcd_set_cursor(uint8_t row, uint8_t column);                                             
static int lcd_write_line(uint8_t row, const char *text);                                               
static void usart1_init(void);                                                  
static void usart1_write_string(const char *text);                                  
static void process_switch(void);                                                   

                                                                              
static void gpio_set_config(GPIO_TypeDef *port, uint8_t pin, uint8_t config)
{
  uint32_t shift = (uint32_t)(pin & 7U) * 4U;                                          
  volatile uint32_t *configuration_register = (pin < 8U) ? &port->CRL : &port->CRH;                                   
  *configuration_register = (*configuration_register & ~(0xFU << shift)) | ((uint32_t)config << shift);                                     
}

                                                                          
static int system_clock_init(void)
{
  uint32_t timeout = 1000000U;                                                                       

  FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;                                                            
  RCC->CR |= RCC_CR_HSEON;                               
  while (((RCC->CR & RCC_CR_HSERDY) == 0U) && (timeout > 0U)) { timeout--; }                                    
  if (timeout == 0U) { return 0; }                                       

  RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_SW | RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2 | RCC_CFGR_ADCPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL)) | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_ADCPRE_DIV6 | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;                                                            

  RCC->CR |= RCC_CR_PLLON;                                                        
  timeout = 1000000U;                                     
  while (((RCC->CR & RCC_CR_PLLRDY) == 0U) && (timeout > 0U)) { timeout--; }                            
  if (timeout == 0U) { RCC->CR &= ~RCC_CR_PLLON; return 0; }                                              

  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;                                 
  timeout = 1000000U;                                               
  while (((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) && (timeout > 0U)) { timeout--; }                                            
  if (timeout == 0U) { return 0; }                                             

  SystemCoreClockUpdate();                                                             
  return (SystemCoreClock == 72000000U);                                          
}

                                                                    
static void tim2_delay_init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;                               
  TIM2->CR1 = 0U;                                          
  TIM2->PSC = 71U;                                                    
  TIM2->ARR = 999U;                                              
  TIM2->CNT = 0U;                       
  TIM2->EGR = TIM_EGR_UG;                                                       
  TIM2->SR = 0U;                            
  TIM2->CR1 = TIM_CR1_CEN;                                     
}

                                                                    
static void delay_ms(uint32_t milliseconds)
{
  while (milliseconds > 0U) {                                             
    TIM2->CNT = 0U;                               
    TIM2->SR = (uint16_t)~TIM_SR_UIF;                                             
    while ((TIM2->SR & TIM_SR_UIF) == 0U) {                              }                             
    milliseconds--;                                     
  }
}

                                                             
static void gpio_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;                                         

  gpio_set_config(GPIOA, 0U, 0x8U);                                                 
  GPIOA->ODR |= GPIO_ODR_ODR0;                                                  
  gpio_set_config(GPIOA, 1U, 0x0U);                                                 
  gpio_set_config(GPIOA, 2U, 0x0U);                                                    
  gpio_set_config(GPIOA, 3U, 0x0U);                                                                  
  gpio_set_config(GPIOA, 6U, 0xAU);                                                           
  GPIOA->BSRR = (1U << (8U + 16U)) | (1U << (11U + 16U));                                                                      
  gpio_set_config(GPIOA, 8U, 0x2U);                                                                  
  gpio_set_config(GPIOA, 11U, 0x2U);                                                                   
  gpio_set_config(GPIOA, 9U, 0xBU);                                                             
  gpio_set_config(GPIOA, 10U, 0x4U);                                         

  gpio_set_config(GPIOB, 0U, 0x0U);                                                                   
  gpio_set_config(GPIOB, 8U, 0xBU);                                                                 
  gpio_set_config(GPIOB, 9U, 0xBU);                                                                 
}

                                                                    
static void exti0_init(void)
{
  AFIO->EXTICR[0] &= ~AFIO_EXTICR1_EXTI0;                                      
  EXTI->IMR &= ~EXTI_IMR_MR0;                                     
  EXTI->RTSR |= EXTI_RTSR_TR0;                                             
  EXTI->FTSR |= EXTI_FTSR_TR0;                                              
  EXTI->PR = EXTI_PR_PR0;                                             
  NVIC_SetPriority(EXTI0_IRQn, 2U);                                                    
  NVIC_EnableIRQ(EXTI0_IRQn);                                 
  EXTI->IMR |= EXTI_IMR_MR0;                                                
}

                                                                                  
void EXTI0_IRQHandler(void)
{
  if ((EXTI->PR & EXTI_PR_PR0) != 0U) {                                     
    EXTI->PR = EXTI_PR_PR0;                                      
    EXTI->IMR &= ~EXTI_IMR_MR0;                                                       
    g_switch_event = 1U;                                                      
  }
}

                                                
static void tim3_pwm_init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;                               
  TIM3->CR1 = 0U;                                         
  TIM3->PSC = 71U;                                                    
  TIM3->ARR = 999U;                                          
  TIM3->CCR1 = 0U;                              
  TIM3->CCMR1 = (TIM3->CCMR1 & ~(TIM_CCMR1_OC1M | TIM_CCMR1_CC1S)) | (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;                                              
  TIM3->CCER |= TIM_CCER_CC1E;                                            
  TIM3->CR1 |= TIM_CR1_ARPE;                                               
  TIM3->EGR = TIM_EGR_UG;                                          
  TIM3->SR = 0U;                                                 
  TIM3->CR1 |= TIM_CR1_CEN;                        
}

                                                                     
static void tim3_set_duty(uint16_t permille)
{
  if (permille > 1000U) { permille = 1000U; }                                          
  TIM3->CCR1 = permille;                                                          
}

                                                                          
static void tim4_throttle_pwm_init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;                               
  TIM4->CR1 = 0U;                                           
  TIM4->PSC = 0U;                                                                 
  TIM4->ARR = 3599U;                                                                     
  TIM4->CCR3 = 0U;                                
  TIM4->CCR4 = 0U;                                
  TIM4->CCMR2 = (TIM4->CCMR2 & ~(TIM_CCMR2_OC3M | TIM_CCMR2_CC3S | TIM_CCMR2_OC4M | TIM_CCMR2_CC4S)) | (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE | (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;                                              
  TIM4->CCER = (TIM4->CCER & ~(TIM_CCER_CC3P | TIM_CCER_CC4P)) | TIM_CCER_CC3E | TIM_CCER_CC4E;                                                           
  TIM4->CR1 |= TIM_CR1_ARPE;                                                       
  TIM4->EGR = TIM_EGR_UG;                                                      
  TIM4->SR = 0U;                                                 
  TIM4->CR1 |= TIM_CR1_CEN;                                                               
  throttle_bridge_disable();                                                                   
}

                                                                     
static void throttle_bridge_disable(void)
{
  GPIOA->BSRR = (1U << (8U + 16U)) | (1U << (11U + 16U));                                                      
  TIM4->CCR3 = 0U;                          
  TIM4->CCR4 = 0U;                          
  g_tps_motor_active = 0U;                                
  g_tps_direction = 0U;                                                    
}

                                                                           
static void throttle_bridge_drive(uint8_t direction, uint16_t duty_permille)
{
  if (duty_permille > TPS_PWM_MAX_PERMILLE) { duty_permille = TPS_PWM_MAX_PERMILLE; }                                               
  if (direction != 1U && direction != 2U) { throttle_bridge_disable(); return; }                                                           
  if (g_tps_direction != 0U && g_tps_direction != direction) { throttle_bridge_disable(); delay_ms(2U); }                                                  
  GPIOA->BSRR = (1U << (8U + 16U)) | (1U << (11U + 16U));                                                      
  if (direction == 1U) { TIM4->CCR3 = 0U; TIM4->CCR4 = ((uint32_t)TIM4->ARR * duty_permille) / 1000U; }                                            
  else { TIM4->CCR4 = 0U; TIM4->CCR3 = ((uint32_t)TIM4->ARR * duty_permille) / 1000U; }                                              
  g_tps_direction = direction;                                                           
  GPIOA->BSRR = (1U << 8U) | (1U << 11U);                                                 
  g_tps_motor_active = 1U;                                           
}

                                                                                  
static uint8_t throttle_sensor_check(uint16_t vta1_raw, uint16_t vta2_raw)
{
  if (vta1_raw < TPS_VTA1_MIN_RAW || vta1_raw > TPS_VTA1_MAX_RAW) { return 0U; }                                                      
  if (vta2_raw < TPS_VTA2_MIN_RAW || vta2_raw > TPS_VTA2_MAX_RAW) { return 0U; }                                                       
  return 1U;                                                        
}

                                                              
static float throttle_normalize_position(uint16_t raw, uint16_t rest, uint16_t full)
{
  float position = ((float)raw - (float)rest) / ((float)full - (float)rest);                                             
  if (position < 0.0f) { position = 0.0f; }                          
  if (position > 1.0f) { position = 1.0f; }                                          
  return position;                            
}

                                                                              
static uint8_t throttle_feedback_percent(uint16_t vta1_raw, uint16_t vta2_raw)
{
  float position = (throttle_normalize_position(vta1_raw, TPS_VTA1_IDLE_RAW, TPS_VTA1_OPEN_RAW) + throttle_normalize_position(vta2_raw, TPS_VTA2_IDLE_RAW, TPS_VTA2_OPEN_RAW)) * 0.5f;                                       
  return (uint8_t)(position * 100.0f + 0.5f);                                                 
}

                                                                    
static void throttle_adc_average_update(void)
{
  const uint16_t samples[3] = {g_adc_samples[0], g_adc_samples[2], g_adc_samples[3]};                                                   
  for (uint8_t channel = 0U; channel < 3U; channel++) {                                                 
    if (g_tps_adc_count == TPS_ADC_AVERAGE_SAMPLES) { g_tps_adc_sum[channel] -= g_tps_adc_history[channel][g_tps_adc_index]; }                                             
    g_tps_adc_history[channel][g_tps_adc_index] = samples[channel];                                
    g_tps_adc_sum[channel] += samples[channel];                             
    g_tps_adc_average[channel] = (uint16_t)(g_tps_adc_sum[channel] / (g_tps_adc_count < TPS_ADC_AVERAGE_SAMPLES ? (g_tps_adc_count + 1U) : TPS_ADC_AVERAGE_SAMPLES));                                      
  }
  g_tps_adc_index++;                                   
  if (g_tps_adc_index >= TPS_ADC_AVERAGE_SAMPLES) { g_tps_adc_index = 0U; }                            
  if (g_tps_adc_count < TPS_ADC_AVERAGE_SAMPLES) { g_tps_adc_count++; }                               
}

                                                                                 
static void throttle_control_update(uint16_t pedal_raw, uint16_t vta1_raw, uint16_t vta2_raw)
{
  if (g_tps_adc_count > 0U) { g_tps_sensor_ok = throttle_sensor_check(vta1_raw, vta2_raw); g_tps_sensor_fault = (uint8_t)(g_tps_sensor_ok == 0U); g_tps_feedback_percent = throttle_feedback_percent(vta1_raw, vta2_raw); }                                                                     
  if (!g_engine_running) { g_tps_pid_integral = 0.0f; g_tps_control_elapsed_ms = 0U; throttle_bridge_disable(); return; }
  g_tps_control_elapsed_ms++;                                     
  if (g_tps_control_elapsed_ms < TPS_CONTROL_PERIOD_MS) { return; }                                  
  g_tps_control_elapsed_ms = 0U;                                  
  if (g_tps_adc_count < TPS_ADC_AVERAGE_SAMPLES) { throttle_bridge_disable(); return; }                                                       

  if (!g_tps_sensor_ok) { throttle_bridge_disable(); g_tps_pid_integral = 0.0f; g_tps_fault_waiting = 1U; return; }                                               

  float pos1 = throttle_normalize_position(vta1_raw, TPS_VTA1_IDLE_RAW, TPS_VTA1_OPEN_RAW);                               
  float pos2 = throttle_normalize_position(vta2_raw, TPS_VTA2_IDLE_RAW, TPS_VTA2_OPEN_RAW);                               
  float actual = (pos1 + pos2) * 0.5f;                                             

  if (g_tps_fault_waiting != 0U) { throttle_bridge_disable(); if (pedal_raw <= PEDAL_IDLE_RAW) { g_tps_fault_waiting = 0U; g_tps_pid_integral = 0.0f; g_tps_previous_position = actual; } return; }                                                        
  if (g_tps_armed == 0U) { throttle_bridge_disable(); if (pedal_raw <= PEDAL_IDLE_RAW) { g_tps_armed = 1U; g_tps_pid_integral = 0.0f; g_tps_previous_position = actual; } return; }                                   

  uint16_t bounded_pedal = pedal_raw;                                              
  if (bounded_pedal < PEDAL_ADC_MIN_RAW) { bounded_pedal = PEDAL_ADC_MIN_RAW; }
  if (bounded_pedal > PEDAL_ADC_MAX_RAW) { bounded_pedal = PEDAL_ADC_MAX_RAW; }
  float pedal_fraction = (float)(bounded_pedal - PEDAL_ADC_MIN_RAW) / (float)(PEDAL_ADC_MAX_RAW - PEDAL_ADC_MIN_RAW);                                                                
  float target = pedal_fraction;                                                   
  float error = target - actual;                                         
  float rate = (actual - g_tps_previous_position) / ((float)TPS_CONTROL_PERIOD_MS / 1000.0f);                                        
  g_tps_previous_position = actual;                                              
  if ((error > -TPS_POSITION_DEADBAND) && (error < TPS_POSITION_DEADBAND)) { error = 0.0f; }                                             

  float next_integral = g_tps_pid_integral + TPS_PID_KI * error * ((float)TPS_CONTROL_PERIOD_MS / 1000.0f);                              
  if (next_integral > TPS_PID_I_LIMIT) { next_integral = TPS_PID_I_LIMIT; }                                
  if (next_integral < -TPS_PID_I_LIMIT) { next_integral = -TPS_PID_I_LIMIT; }                             
  float output = TPS_PID_KP * error + next_integral - TPS_PID_KD * rate;                             
  float output_limit = (float)TPS_PWM_MAX_PERMILLE / 10.0f;                                                    
  float limited = output;                                       
  if (limited > output_limit) { limited = output_limit; }                                    
  if (limited < -output_limit) { limited = -output_limit; }                                      
  if ((output == limited) || ((output > output_limit) && (error < 0.0f)) || ((output < -output_limit) && (error > 0.0f))) { g_tps_pid_integral = next_integral; }                                                         
  else { output = TPS_PID_KP * error + g_tps_pid_integral - TPS_PID_KD * rate; limited = output; if (limited > output_limit) { limited = output_limit; } if (limited < -output_limit) { limited = -output_limit; } }                                      

  uint8_t requested_direction = 0U;                           
  uint16_t duty = 0U;                            
  if (limited > 0.0f) { requested_direction = 1U; float percent = limited; float min_percent = (float)TPS_PWM_MIN_PERMILLE / 10.0f; if (percent < min_percent) { percent = min_percent; } duty = (uint16_t)(percent * 10.0f + 0.5f); }                                  
  else if (limited < 0.0f) { requested_direction = 2U; float percent = -limited; float min_percent = (float)TPS_PWM_MIN_PERMILLE / 10.0f; if (percent < min_percent) { percent = min_percent; } duty = (uint16_t)(percent * 10.0f + 0.5f); }                                    
  if ((pedal_raw <= PEDAL_IDLE_RAW) && (actual <= TPS_POSITION_DEADBAND)) { g_tps_pid_integral = 0.0f; requested_direction = 0U; duty = 0U; }                                                               
  if ((requested_direction == 1U) && ((vta1_raw >= TPS_VTA1_OPEN_RAW) || (vta2_raw >= TPS_VTA2_OPEN_RAW))) { g_tps_pid_integral = 0.0f; requested_direction = 0U; duty = 0U; }                                                        
  if ((requested_direction == 2U) && (vta1_raw <= (TPS_VTA1_IDLE_RAW + 14U)) && (vta2_raw <= (TPS_VTA2_IDLE_RAW + 14U))) { g_tps_pid_integral = 0.0f; requested_direction = 0U; duty = 0U; }                                                     

  throttle_bridge_drive(requested_direction, duty);                                             
}

                                                                    
static void adc1_dma_init(void)
{
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;                              
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;                               
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV6;                                    

  DMA1_Channel1->CCR &= ~DMA_CCR_EN;                                              
  while ((DMA1_Channel1->CCR & DMA_CCR_EN) != 0U) {                                           }                                          
  DMA1->IFCR = DMA_IFCR_CGIF1;                                             
  DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;                                                        
  DMA1_Channel1->CMAR = (uint32_t)g_adc_samples;                                       
  DMA1_Channel1->CNDTR = 4U;                                                             
  DMA1_Channel1->CCR = DMA_CCR_PL_0 | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_CIRC;                                                            
  DMA1_Channel1->CCR |= DMA_CCR_EN;                                                

  ADC1->CR1 = ADC_CR1_SCAN;                                             
  ADC1->CR2 = 0U;                                                         
  ADC1->SMPR2 = (3U << ADC_SMPR2_SMP1_Pos) | (3U << ADC_SMPR2_SMP2_Pos) | (3U << ADC_SMPR2_SMP3_Pos) | (3U << ADC_SMPR2_SMP8_Pos);                                                              
  ADC1->SQR1 = (3U << ADC_SQR1_L_Pos);                                                                 
  ADC1->SQR3 = (1U << ADC_SQR3_SQ1_Pos) | (2U << ADC_SQR3_SQ2_Pos) | (3U << ADC_SQR3_SQ3_Pos) | (8U << ADC_SQR3_SQ4_Pos);                                          
  ADC1->SR = 0U;                                
  ADC1->CR2 = ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_EXTTRIG | (7U << ADC_CR2_EXTSEL_Pos);                                                               
  ADC1->CR2 |= ADC_CR2_ADON;                                          
  delay_ms(1U);                                         
  ADC1->CR2 |= ADC_CR2_RSTCAL;                                           
  while ((ADC1->CR2 & ADC_CR2_RSTCAL) != 0U) {                                          }                                         
  ADC1->CR2 |= ADC_CR2_CAL;                              
  while ((ADC1->CR2 & ADC_CR2_CAL) != 0U) {                                }                               
  ADC1->CR2 |= ADC_CR2_SWSTART;                                                        
}

static void i2c1_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;                                   
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;                               
  gpio_set_config(GPIOB, 6U, 0xDU);                                                        
  gpio_set_config(GPIOB, 7U, 0xDU);                                                        
  I2C1->CR1 = I2C_CR1_SWRST;                                                          
  I2C1->CR1 = 0U;                                                    
  I2C1->CR2 = 36U;                                                  
  I2C1->CCR = 180U;                                               
  I2C1->TRISE = 37U;                                                              
  I2C1->CR1 = I2C_CR1_PE;                                               
}

static int i2c_wait_sr1(uint32_t flag)
{
  uint32_t timeout = I2C_TIMEOUT;                                                   
  while (((I2C1->SR1 & flag) == 0U) && ((I2C1->SR1 & (I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR)) == 0U) && (timeout > 0U)) { timeout--; }                                   
  if ((I2C1->SR1 & (I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR)) != 0U) { return 0; }                                                  
  return ((I2C1->SR1 & flag) != 0U);                                                                           
}

                                                                   
static int pcf8574_write(uint8_t value)
{
  I2C1->CR1 |= I2C_CR1_START;                                     
  if (!i2c_wait_sr1(I2C_SR1_SB)) { goto i2c_error; }                                    
  I2C1->DR = (uint8_t)(LCD_I2C_ADDRESS << 1U);                                             
  if (!i2c_wait_sr1(I2C_SR1_ADDR)) { goto i2c_error; }                                        
  (void)I2C1->SR1;                                                     
  (void)I2C1->SR2;                                             
  if (!i2c_wait_sr1(I2C_SR1_TXE)) { goto i2c_error; }                                                     
  I2C1->DR = value;                                       
  if (!i2c_wait_sr1(I2C_SR1_BTF)) { goto i2c_error; }                                     
  I2C1->CR1 |= I2C_CR1_STOP;                                                 
  uint32_t stop_timeout = I2C_TIMEOUT;                                                 
  while (((I2C1->CR1 & I2C_CR1_STOP) != 0U) && (stop_timeout > 0U)) { stop_timeout--; }                                               
  if (stop_timeout == 0U) { goto i2c_error; }                                           
  return 1;                                

i2c_error:
  I2C1->CR1 |= I2C_CR1_STOP;                                                   
  I2C1->SR1 &= ~(I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR);                                                     
  return 0;                                           
}

static int lcd_write_nibble(uint8_t nibble, uint8_t rs)
{
  uint8_t value = (uint8_t)((nibble << 4U) | LCD_BACKLIGHT_BIT | (rs ? LCD_REGISTER_SELECT : 0U));                                                
  if (!pcf8574_write((uint8_t)(value | LCD_ENABLE_BIT))) { return 0; }                                            
  if (!pcf8574_write(value)) { return 0; }                                             
  return 1;                                              
}

                                                             
static int lcd_write_byte(uint8_t value, uint8_t rs)
{
  if (!lcd_write_nibble((uint8_t)(value >> 4U), rs)) { return 0; }                                                     
  if (!lcd_write_nibble((uint8_t)(value & 0x0FU), rs)) { return 0; }                                                      
  if ((rs == 0U) && ((value == 0x01U) || (value == 0x02U))) { delay_ms(2U); }                                                      
  return 1;                                                
}

                                                                  
static void lcd_init(void)
{
  g_lcd_available = 1U;                                                           
  delay_ms(50U);                                         
  if (!lcd_write_nibble(0x03U, 0U)) { goto lcd_unavailable; }                                                       
  delay_ms(5U);                                           
  if (!lcd_write_nibble(0x03U, 0U)) { goto lcd_unavailable; }                                             
  delay_ms(1U);                                  
  if (!lcd_write_nibble(0x03U, 0U)) { goto lcd_unavailable; }                                            
  delay_ms(1U);                                 
  if (!lcd_write_nibble(0x02U, 0U)) { goto lcd_unavailable; }                                       
  if (!lcd_write_byte(0x28U, 0U)) { goto lcd_unavailable; }                                        
  if (!lcd_write_byte(0x0CU, 0U)) { goto lcd_unavailable; }                                   
  if (!lcd_write_byte(0x06U, 0U)) { goto lcd_unavailable; }                                        
  if (!lcd_write_byte(0x01U, 0U)) { goto lcd_unavailable; }                                               
  return;                                                          

lcd_unavailable:
  g_lcd_available = 0U;                                                                        
}

                                                 
static int lcd_set_cursor(uint8_t row, uint8_t column)
{
  uint8_t address = (row == 0U) ? (uint8_t)(0x80U + column) : (uint8_t)(0xC0U + column);                                                 
  return lcd_write_byte(address, 0U);                                                           
}

                                                                         
static int lcd_write_line(uint8_t row, const char *text)
{
  if (!g_lcd_available) { return 0; }                                                               
  if (!lcd_set_cursor(row, 0U)) { return 0; }                                                 
  for (uint8_t column = 0U; column < 16U; column++) {                                    
    char character = (text[column] != '\0') ? text[column] : ' ';                                                          
    if (!lcd_write_byte((uint8_t)character, 1U)) { return 0; }                                                        
  }
  return 1;                                     
}

                                                               
static void usart1_init(void)
{
  USART1->CR1 = 0U;                                        
  USART1->CR2 = 0U;                                  
  USART1->CR3 = 0U;                                        
  USART1->BRR = 625U;                                                                      
  USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;                                    
}

                                                           
static void usart1_write_string(const char *text)
{
  while (*text != '\0') {                                           
    uint32_t timeout = UART_TX_TIMEOUT;                                           
    while (((USART1->SR & USART_SR_TXE) == 0U) && (timeout > 0U)) { timeout--; }                                  
    if (timeout == 0U) { return; }                                               
    USART1->DR = (uint8_t)*text;                                                
    text++;                                 
  }
  uint32_t timeout = UART_TX_TIMEOUT;                                                      
  while (((USART1->SR & USART_SR_TC) == 0U) && (timeout > 0U)) { timeout--; }                                     
}

                                                                     
static void process_switch(void)
{
  if (g_switch_event == 0U) { return; }                                            
  g_switch_event = 0U;                                            
  delay_ms(20U);                                       
  g_engine_running = ((GPIOA->IDR & (1U << SWITCH_PIN)) == 0U) ? 1U : 0U;                                        
  EXTI->PR = EXTI_PR_PR0;                                                   
  EXTI->IMR |= EXTI_IMR_MR0;                                              
}

                                                                             
int main(void)
{
  if (!system_clock_init()) { while (1) { } }                                                          
  tim2_delay_init();                                                 
  gpio_init();                                                          
  g_engine_running = ((GPIOA->IDR & (1U << SWITCH_PIN)) == 0U) ? 1U : 0U;                                                      
  exti0_init();                                                           
  tim3_pwm_init();                                             
  tim4_throttle_pwm_init();                                                                 
  adc1_dma_init();                                                    
  i2c1_init();                                                        
  usart1_init();                                                  
  lcd_init();                         

  uint32_t ui_elapsed_ms = 0U;                                     
  uint32_t uart_elapsed_ms = 0U;                                 
  char uart_line[256];                                                                         

  while (1) {                                                 
    process_switch();                                                     

    uint16_t temperature_raw = g_adc_samples[1];                                             
    uint16_t vta1_raw = g_adc_samples[2];                                            
    uint16_t vta2_raw = g_adc_samples[3];                                             
    throttle_adc_average_update();                                               
    uint16_t throttle_filtered = g_tps_adc_average[0];                                                     
    uint16_t vta1_filtered = g_tps_adc_average[1];                                    
    uint16_t vta2_filtered = g_tps_adc_average[2];                                    
    uint32_t vta1_sensor_mv = ((uint32_t)vta1_raw * ADC_VDDA_MV * TPS_DIVIDER_SCALE_NUM) / (ADC_FULL_SCALE * TPS_DIVIDER_SCALE_DEN);                                                                               
    uint32_t vta2_sensor_mv = ((uint32_t)vta2_raw * ADC_VDDA_MV * TPS_DIVIDER_SCALE_NUM) / (ADC_FULL_SCALE * TPS_DIVIDER_SCALE_DEN);                                                              
    uint8_t throttle_percent = (uint8_t)(((uint32_t)throttle_filtered * 100U) / ADC_FULL_SCALE);                                                    
    uint16_t temperature_c = (uint16_t)(((uint32_t)temperature_raw * TEMP_SIM_MAX_C) / ADC_FULL_SCALE);                                                                 
    uint8_t fan_percent = (g_engine_running && (temperature_c > FAN_ON_TEMP_C)) ? 100U : 0U;                                                                                      

    throttle_control_update(throttle_filtered, vta1_filtered, vta2_filtered);                                                      
    tim3_set_duty((uint16_t)fan_percent * 10U);                                                  
    if (ui_elapsed_ms >= 250U) {                                            
      char row0[17];                                                          
      char row1[17];                                                           
      (void)snprintf(row0, sizeof(row0), "P:%3u T:%3u F:%3u", (unsigned int)throttle_percent, (unsigned int)g_tps_feedback_percent, (unsigned int)fan_percent);                                                     
      (void)snprintf(row1, sizeof(row1), "TMP:%3uC %s", (unsigned int)temperature_c, g_engine_running ? "RUN" : "STOP");                                                 
      if (!lcd_write_line(0U, row0) || !lcd_write_line(1U, row1)) { g_lcd_available = 0U; }                                                           
      ui_elapsed_ms = 0U;                                       
    }

    if (uart_elapsed_ms >= 500U) {                                                  
      int length = snprintf(uart_line, sizeof(uart_line), "ENGINE=%s,PEDAL=%u%%,TBODY=%u%%,TEMP=%uC,FAN=%u%%,VTA_RAW=%u,VTA_mV~%u,VTA2_RAW=%u,VTA2_mV~%u,TPS_OK=%u,TPS_ARMED=%u,TPS_SENSOR_FAULT=%u,TPS_WAITING=%u,MOTOR=%u\r\n", g_engine_running ? "RUN" : "STOP", (unsigned int)throttle_percent, (unsigned int)g_tps_feedback_percent, (unsigned int)temperature_c, (unsigned int)fan_percent, (unsigned int)vta1_raw, (unsigned int)vta1_sensor_mv, (unsigned int)vta2_raw, (unsigned int)vta2_sensor_mv, (unsigned int)g_tps_sensor_ok, (unsigned int)g_tps_armed, (unsigned int)g_tps_sensor_fault, (unsigned int)g_tps_fault_waiting, (unsigned int)g_tps_motor_active);                                                                           
      if (length > 0) { usart1_write_string(uart_line); }                                           
      uart_elapsed_ms = 0U;                                   
    }

    delay_ms(1U);                                                                    
    ui_elapsed_ms++;                                       
    uart_elapsed_ms++;                                   
  }
}
