/*
 * ECU mô phỏng quản lý động cơ, viết bare-metal bằng CMSIS và thanh ghi STM32F103.
 * PA1 đọc biến trở bàn đạp; PA2 đọc biến trở nhiệt; PA3/PB0 đọc VTA/VTA2 qua cầu chia áp.
 * PA6 xuất PWM quạt; PA0 là nút Start/Stop; PB6/PB7 nối LCD I2C.
 * USART1 PA9/PA10 gửi telemetry 115200 baud.
 */
#include "stm32f1xx.h"                 /* Định nghĩa thanh ghi STM32F1 và các bit CMSIS. */
#include "system_stm32f1xx.h"          /* Khai báo biến SystemCoreClock và hàm cập nhật clock. */
#include <stdint.h>                     /* Cung cấp các kiểu số nguyên có độ rộng cố định. */
#include <stdio.h>                      /* Cung cấp snprintf để tạo dòng dữ liệu UART. */

#define LCD_I2C_ADDRESS       0x27U     /* Địa chỉ 7-bit phổ biến của backpack PCF8574. */
#define ADC_FULL_SCALE        4095U     /* Giá trị lớn nhất của ADC 12-bit. */
#define ADC_VDDA_MV           3200U     /* VDDA đo được khoảng 3,2 V; hiệu chỉnh lại nếu nguồn thay đổi. */
#define TPS_DIVIDER_SCALE_NUM 3U        /* Nghịch đảo hệ số cầu chia áp 1/3: 20 kΩ nối tiếp, 10 kΩ xuống GND. */
#define TPS_DIVIDER_SCALE_DEN 1U        /* Mẫu số của hệ số nghịch đảo 3/1. */
#define TEMP_SIM_MAX_C        150U      /* Thang nhiệt độ mô phỏng bằng biến trở. */
#define FAN_ON_TEMP_C         80U       /* Ngưỡng bật quạt khi động cơ đang chạy. */
#define TPS_VTA1_IDLE_RAW     424U      /* Mẫu PA3/VTA ở vị trí nghỉ theo log bench đã gửi. */
#define PEDAL_IDLE_RAW        80U       /* Ngưỡng bàn đạp nhả để cho phép arm điều khiển. */
#define PEDAL_LEVEL_COUNT     10U       /* Chia hành trình bàn đạp thành 10 mức mục tiêu. */
#define TPS_VTA1_OPEN_RAW     2004U     /* Mẫu PA3/VTA mở hết theo file test_throttle. */
#define TPS_VTA2_IDLE_RAW     1092U     /* Mẫu PB0/VTA2 nghỉ theo file test_throttle. */
#define TPS_VTA2_OPEN_RAW     2409U     /* Mẫu PB0/VTA2 mở hết theo file test_throttle. */
#define TPS_VTA2_MIN_RAW      800U      /* Giới hạn thấp rộng cho track VTA2, có dự phòng sai số. */
#define TPS_VTA2_MAX_RAW      2600U     /* Giới hạn cao rộng theo file test_throttle. */
#define TPS_VTA1_MIN_RAW      300U      /* Giới hạn thấp rộng cho track VTA1, có dự phòng sai số. */
#define TPS_VTA1_MAX_RAW      2250U     /* Giới hạn cao rộng theo file test_throttle. */
#define TPS_CONTROL_PERIOD_MS 20U       /* Chu kỳ cập nhật PID theo file test_throttle. */
#define TPS_ADC_AVERAGE_SAMPLES 20U     /* Số mẫu trung bình trượt theo file test_throttle. */
#define TPS_POSITION_DEADBAND 0.02f    /* Vùng chết 2% hành trình theo file test_throttle. */
#define TPS_PWM_MAX_PERMILLE  600U      /* Giới hạn duty PID ở 60% theo file test_throttle. */
#define TPS_PWM_MIN_PERMILLE  100U      /* Duty tối thiểu 10% theo file test_throttle. */
#define TPS_PID_KP            80.0f     /* Hệ số P theo file test_throttle. */
#define TPS_PID_KI            2.0f      /* Hệ số I theo file test_throttle. */
#define TPS_PID_KD            1.0f      /* Hệ số D theo file test_throttle. */
#define TPS_PID_I_LIMIT       30.0f     /* Giới hạn tích phân chống windup. */
#define TPS_STALL_LIMIT_MS    1500U     /* Cho motor thêm thời gian chạy ở giới hạn PWM bench trước khi chốt lỗi đứng yên. */
#define UART_BAUD_RATE        115200U   /* Tốc độ UART gửi dữ liệu lên máy tính. */
#define UART_TX_TIMEOUT       1000000U  /* Giới hạn chờ UART để tránh kẹt vô hạn. */
#define I2C_TIMEOUT           100000U   /* Giới hạn chờ mỗi cờ trạng thái I2C. */
#define BUTTON_PIN            0U        /* Nút Start/Stop nối PA0, nhấn sẽ kéo xuống GND. */
#define LCD_BACKLIGHT_BIT     0x08U     /* P3 của PCF8574 điều khiển đèn nền LCD. */
#define LCD_ENABLE_BIT        0x04U     /* P2 của PCF8574 nối chân Enable LCD. */
#define LCD_REGISTER_SELECT   0x01U     /* P0 của PCF8574 nối chân RS LCD. */

static volatile uint16_t g_adc_samples[4]; /* DMA: [0]=PA1 bàn đạp, [1]=PA2 nhiệt, [2]=PA3 VTA, [3]=PB0 VTA2. */
static uint16_t g_tps_adc_history[3][TPS_ADC_AVERAGE_SAMPLES]; /* Lịch sử bàn đạp và hai track để lọc nhiễu. */
static uint32_t g_tps_adc_sum[3]; /* Tổng mẫu của pedal, VTA1 và VTA2. */
static uint16_t g_tps_adc_average[3]; /* Giá trị trung bình trượt đưa vào bộ điều khiển. */
static uint8_t g_tps_adc_index; /* Vị trí ghi kế tiếp trong vòng đệm mẫu. */
static uint8_t g_tps_adc_count; /* Số mẫu đã thu được, tối đa 20. */
static uint8_t g_tps_sensor_fault; /* Lỗi raw của VTA1/VTA2 theo cửa sổ hợp lệ. */
static uint8_t g_tps_fault_waiting; /* Chờ cảm biến hợp lệ và bàn đạp nhả sau lỗi cảm biến. */
static uint8_t g_tps_armed; /* Chỉ điều khiển sau khi bàn đạp nhả từ lúc khởi động. */
static volatile uint8_t g_button_event;     /* ISR đặt cờ khi phát hiện cạnh nhấn nút. */
static uint8_t g_engine_running;            /* Trạng thái động cơ do vòng lặp chính quản lý. */
static uint8_t g_lcd_available;             /* Cờ cho biết LCD/I2C đã phản hồi trong khởi tạo. */
static uint8_t g_tps_sensor_ok;              /* Cờ plausibility: cả VTA và VTA2 đều nằm trong dải bench. */
static uint8_t g_tps_motor_active;           /* Cờ cho biết bridge đang được enable để điều khiển motor. */
static uint8_t g_tps_direction;               /* Hướng hiện tại: 1 mở bướm ga, 2 đóng bướm ga, 0 dừng. */
static uint16_t g_tps_feedback_percent;       /* Vị trí VTA đã chuẩn hóa theo hai mẫu bench. */
static uint8_t g_tps_fault_latched;            /* Chốt lỗi kẹt/không chuyển động đến khi người dùng Stop rồi Start lại. */
static float g_tps_pid_integral; /* Tích phân PID của vị trí bướm ga. */
static float g_tps_previous_position; /* Vị trí chuẩn hóa ở chu kỳ PID trước. */
static uint16_t g_tps_control_elapsed_ms; /* Thời gian tích lũy tới chu kỳ PID 20 ms. */

static void gpio_set_config(GPIO_TypeDef *port, uint8_t pin, uint8_t config); /* Đặt cấu hình 4 bit cho một chân GPIO. */
static int system_clock_init(void);          /* Khởi tạo HSE/PLL và clock tree 72 MHz. */
static void tim2_delay_init(void);            /* Cấu hình TIM2 làm bộ đếm thời gian chờ. */
static void delay_ms(uint32_t milliseconds);  /* Tạo delay theo mili giây bằng TIM2. */
static void gpio_init(void);                  /* Cấu hình GPIO cho nút, LED, PWM và UART. */
static void exti0_init(void);                 /* Cấu hình PA0 làm ngắt ngoài cạnh xuống. */
static void tim3_pwm_init(void);              /* Cấu hình TIM3_CH1 trên PA6 ở 1 kHz. */
static void tim3_set_duty(uint16_t permille); /* Đặt duty PWM quạt trong khoảng 0..1000. */
static void tim4_throttle_pwm_init(void);      /* Cấu hình TIM4_CH3/CH4 trên PB8/PB9 ở 20 kHz cho BTS7960. */
static void throttle_bridge_disable(void);    /* Đưa hai EN xuống thấp và xóa cả hai PWM của BTS7960. */
static void throttle_bridge_drive(uint8_t direction, uint16_t duty_permille); /* Chạy một phía BTS7960 với duty giới hạn. */
static uint8_t throttle_sensor_check(uint16_t vta1_raw, uint16_t vta2_raw); /* Kiểm tra hai track feedback nằm trong dải bench. */
static float throttle_normalize_position(uint16_t raw, uint16_t rest, uint16_t full); /* Chuẩn hóa một track về 0..1. */
static uint8_t throttle_feedback_percent(uint16_t vta1_raw, uint16_t vta2_raw); /* Trung bình vị trí chuẩn hóa của hai track. */
static void throttle_adc_average_update(void); /* Lọc 20 mẫu pedal, VTA1 và VTA2. */
static void throttle_control_update(uint16_t pedal_raw, uint16_t vta1_raw, uint16_t vta2_raw); /* Điều khiển vị trí throttle theo pedal và feedback. */
static void adc1_dma_init(void);               /* Đọc PA1/PA2/PA3/PB0 liên tục qua ADC1 và DMA1. */
static void i2c1_init(void);                   /* Cấu hình I2C1 100 kHz trên PB6/PB7. */
static int i2c_wait_sr1(uint32_t flag);        /* Chờ cờ I2C hoặc trả lỗi khi hết thời gian. */
static int pcf8574_write(uint8_t value);       /* Gửi một byte tới backpack LCD. */
static int lcd_write_nibble(uint8_t nibble, uint8_t rs);  /* Gửi một nibble 4-bit tới LCD và báo lỗi bus. */
static int lcd_write_byte(uint8_t value, uint8_t rs);     /* Gửi một byte lệnh hoặc ký tự LCD và báo lỗi. */
static void lcd_init(void);                    /* Khởi tạo LCD 16x2 ở chế độ 4-bit. */
static int lcd_set_cursor(uint8_t row, uint8_t column);   /* Đặt vị trí con trỏ LCD và báo lỗi bus. */
static int lcd_write_line(uint8_t row, const char *text); /* Ghi đủ 16 ký tự lên một dòng và báo lỗi. */
static void usart1_init(void);                 /* Cấu hình USART1 trên PA9/PA10, 115200 baud. */
static void usart1_write_string(const char *text); /* Gửi chuỗi ASCII qua USART1. */
static void process_button(void);              /* Debounce và xử lý nhấn nút ở main loop. */

/* Gán giá trị cấu hình 4-bit trực tiếp vào CRL hoặc CRH của một GPIO port. */
static void gpio_set_config(GPIO_TypeDef *port, uint8_t pin, uint8_t config)
{
  uint32_t shift = (uint32_t)(pin & 7U) * 4U; /* Mỗi chân chiếm 4 bit trong CRL/CRH. */
  volatile uint32_t *configuration_register = (pin < 8U) ? &port->CRL : &port->CRH; /* Chọn thanh ghi theo số chân. */
  *configuration_register = (*configuration_register & ~(0xFU << shift)) | ((uint32_t)config << shift); /* Thay đúng trường của chân này. */
}

/* Khởi tạo clock hệ thống: HSE 8 MHz x PLL 9, AHB 72 MHz, APB1 36 MHz. */
static int system_clock_init(void)
{
  uint32_t timeout = 1000000U; /* Bộ đếm giới hạn để lỗi thạch anh không treo chương trình vô hạn. */

  FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2; /* Bật prefetch và đặt 2 wait-state trước khi tăng xung. */
  RCC->CR |= RCC_CR_HSEON; /* Bật thạch anh HSE 8 MHz. */
  while (((RCC->CR & RCC_CR_HSERDY) == 0U) && (timeout > 0U)) { timeout--; } /* Chờ HSE ổn định hoặc timeout. */
  if (timeout == 0U) { return 0; } /* Báo lỗi nếu HSE không khởi động. */

  RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_SW | RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2 | RCC_CFGR_ADCPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL)) | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_ADCPRE_DIV6 | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9; /* Đặt AHB /1, APB1 /2, APB2 /1, ADC /6 và PLL = HSE x9. */

  RCC->CR |= RCC_CR_PLLON; /* Bật PLL sau khi nạp cấu hình nguồn và hệ số nhân. */
  timeout = 1000000U; /* Nạp lại giới hạn chờ khóa PLL. */
  while (((RCC->CR & RCC_CR_PLLRDY) == 0U) && (timeout > 0U)) { timeout--; } /* Chờ PLL khóa ổn định. */
  if (timeout == 0U) { RCC->CR &= ~RCC_CR_PLLON; return 0; } /* Tắt PLL và báo lỗi nếu không khóa được. */

  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL; /* Chọn PLL làm nguồn SYSCLK. */
  timeout = 1000000U; /* Nạp lại giới hạn chờ đổi clock hệ thống. */
  while (((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) && (timeout > 0U)) { timeout--; } /* Chờ phần cứng xác nhận SYSCLK từ PLL. */
  if (timeout == 0U) { return 0; } /* Báo lỗi nếu chưa chuyển được sang PLL. */

  SystemCoreClockUpdate(); /* Cập nhật biến CMSIS SystemCoreClock theo RCC hiện tại. */
  return (SystemCoreClock == 72000000U); /* Xác nhận CPU đang chạy đúng 72 MHz. */
}

/* Cấu hình TIM2 đếm 1 MHz và tràn mỗi 1 ms để làm delay polling. */
static void tim2_delay_init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; /* Cấp clock APB1 cho TIM2. */
  TIM2->CR1 = 0U; /* Giữ bộ đếm dừng trong lúc cấu hình. */
  TIM2->PSC = 71U; /* Chia clock timer 72 MHz cho 72 để được 1 MHz. */
  TIM2->ARR = 999U; /* Đếm 1000 tick 1 MHz để tạo chu kỳ 1 ms. */
  TIM2->CNT = 0U; /* Đặt bộ đếm về 0. */
  TIM2->EGR = TIM_EGR_UG; /* Nạp ngay prescaler và auto-reload vào phần cứng. */
  TIM2->SR = 0U; /* Xóa cờ trạng thái cũ. */
  TIM2->CR1 = TIM_CR1_CEN; /* Bắt đầu cho TIM2 đếm liên tục. */
}

/* Chờ số mili giây yêu cầu bằng cách polling cờ update của TIM2. */
static void delay_ms(uint32_t milliseconds)
{
  while (milliseconds > 0U) { /* Lặp một lần cho mỗi mili giây cần chờ. */
    TIM2->CNT = 0U; /* Bắt đầu chu kỳ 1 ms mới. */
    TIM2->SR = (uint16_t)~TIM_SR_UIF; /* Xóa cờ update bằng cách ghi 0 vào UIF. */
    while ((TIM2->SR & TIM_SR_UIF) == 0U) { /* Đợi TIM2 đếm hết 1 ms. */ }
    milliseconds--; /* Giảm bộ đếm thời gian còn lại. */
  }
}

/* Cấu hình chân GPIO của cảm biến, nút, PWM và UART. */
static void gpio_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN; /* Cấp clock GPIOA/B, AFIO và USART1. */

  gpio_set_config(GPIOA, 0U, 0x8U); /* PA0 input pull-up cho nút nhấn active-low. */
  GPIOA->ODR |= GPIO_ODR_ODR0; /* Chọn pull-up nội cho PA0. */
  gpio_set_config(GPIOA, 1U, 0x0U); /* PA1 analog input ADC1_IN1 cho biến trở ga. */
  gpio_set_config(GPIOA, 2U, 0x0U); /* PA2 analog input ADC1_IN2 cho biến trở nhiệt. */
  gpio_set_config(GPIOA, 3U, 0x0U); /* PA3 analog input ADC1_IN3 cho tín hiệu VTA sau cầu chia áp. */
  gpio_set_config(GPIOA, 6U, 0xAU); /* PA6 alternate-function push-pull 2 MHz cho TIM3_CH1. */
  GPIOA->BSRR = (1U << (8U + 16U)) | (1U << (11U + 16U)); /* Kéo R_EN/L_EN của BTS7960 xuống thấp trước khi cấu hình output. */
  gpio_set_config(GPIOA, 8U, 0x2U); /* PA8 output push-pull làm R_EN, trạng thái mặc định disable. */
  gpio_set_config(GPIOA, 11U, 0x2U); /* PA11 output push-pull làm L_EN, trạng thái mặc định disable. */
  gpio_set_config(GPIOA, 9U, 0xBU); /* PA9 alternate-function push-pull 50 MHz làm USART1_TX. */
  gpio_set_config(GPIOA, 10U, 0x4U); /* PA10 input floating làm USART1_RX. */

  gpio_set_config(GPIOB, 0U, 0x0U); /* PB0 analog input ADC1_IN8 cho tín hiệu VTA2 sau cầu chia áp. */
  gpio_set_config(GPIOB, 8U, 0xBU); /* PB8 alternate-function push-pull 50 MHz cho TIM4_CH3/RPWM. */
  gpio_set_config(GPIOB, 9U, 0xBU); /* PB9 alternate-function push-pull 50 MHz cho TIM4_CH4/LPWM. */
}

/* Cấu hình EXTI0 để nút PA0 báo sự kiện khi được nhấn. */
static void exti0_init(void)
{
  AFIO->EXTICR[0] &= ~AFIO_EXTICR1_EXTI0; /* Chọn GPIOA làm nguồn cho EXTI0. */
  EXTI->IMR &= ~EXTI_IMR_MR0; /* Khóa EXTI0 trong lúc cấu hình. */
  EXTI->RTSR &= ~EXTI_RTSR_TR0; /* Không kích hoạt ở cạnh lên. */
  EXTI->FTSR |= EXTI_FTSR_TR0; /* Kích hoạt ngắt tại cạnh xuống khi nhấn nút. */
  EXTI->PR = EXTI_PR_PR0; /* Xóa pending flag cũ trước khi mở ngắt. */
  NVIC_SetPriority(EXTI0_IRQn, 2U); /* Đặt ưu tiên cho EXTI0 bằng CMSIS NVIC helper. */
  NVIC_EnableIRQ(EXTI0_IRQn); /* Cho phép EXTI0 trong NVIC. */
  EXTI->IMR |= EXTI_IMR_MR0; /* Mở mặt nạ EXTI0 sau khi đã cấu hình xong. */
}

/* ISR chỉ ghi nhận sự kiện và khóa EXTI để debounce trong main loop. */
void EXTI0_IRQHandler(void)
{
  if ((EXTI->PR & EXTI_PR_PR0) != 0U) { /* Xác nhận ngắt đến từ line PA0. */
    EXTI->PR = EXTI_PR_PR0; /* Xóa cờ pending bằng cách ghi 1. */
    EXTI->IMR &= ~EXTI_IMR_MR0; /* Tạm khóa line để tránh bounce tạo nhiều sự kiện. */
    g_button_event = 1U; /* Báo main loop xử lý nút sau khi ISR kết thúc. */
  }
}

/* Cấu hình TIM3_CH1 xuất PWM 1 kHz trên PA6. */
static void tim3_pwm_init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; /* Cấp clock APB1 cho TIM3. */
  TIM3->CR1 = 0U; /* Dừng timer trước khi đổi thông số. */
  TIM3->PSC = 71U; /* Chia timer clock 72 MHz cho 72 để được 1 MHz. */
  TIM3->ARR = 999U; /* Đếm 1000 tick tạo chu kỳ PWM 1 kHz. */
  TIM3->CCR1 = 0U; /* Khởi động với quạt tắt. */
  TIM3->CCMR1 = (TIM3->CCMR1 & ~(TIM_CCMR1_OC1M | TIM_CCMR1_CC1S)) | (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE; /* Chọn PWM mode 1 và bật preload cho CH1. */
  TIM3->CCER |= TIM_CCER_CC1E; /* Cho phép tín hiệu CH1 đi ra chân PA6. */
  TIM3->CR1 |= TIM_CR1_ARPE; /* Bật preload cho ARR để cập nhật đồng bộ. */
  TIM3->EGR = TIM_EGR_UG; /* Nạp PSC/ARR ngay và đồng bộ bộ đếm. */
  TIM3->SR = 0U; /* Xóa cờ timer phát sinh trong lúc cập nhật. */
  TIM3->CR1 |= TIM_CR1_CEN; /* Bắt đầu phát PWM. */
}

/* Đặt duty TIM3_CH1 theo phần nghìn và chặn giá trị vượt quá ARR. */
static void tim3_set_duty(uint16_t permille)
{
  if (permille > 1000U) { permille = 1000U; } /* Giới hạn duty trong khoảng 0..100%. */
  TIM3->CCR1 = permille; /* Cập nhật compare value, 1000 tương ứng 100% chu kỳ. */
}

/* Cấu hình hai kênh PWM 20 kHz cho RPWM/LPWM của BTS7960 trên PB8/PB9. */
static void tim4_throttle_pwm_init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; /* Cấp clock APB1 cho TIM4. */
  TIM4->CR1 = 0U; /* Dừng timer trước khi đặt chu kỳ PWM. */
  TIM4->PSC = 0U; /* Không chia clock timer 72 MHz để có độ phân giải duty cao. */
  TIM4->ARR = 3599U; /* 72 MHz/(3599+1) = 20 kHz, nằm dưới giới hạn 25 kHz của module. */
  TIM4->CCR3 = 0U; /* Bắt đầu với RPWM bằng 0%. */
  TIM4->CCR4 = 0U; /* Bắt đầu với LPWM bằng 0%. */
  TIM4->CCMR2 = (TIM4->CCMR2 & ~(TIM_CCMR2_OC3M | TIM_CCMR2_CC3S | TIM_CCMR2_OC4M | TIM_CCMR2_CC4S)) | (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE | (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE; /* Chọn PWM mode 1 và preload cho CH3/CH4. */
  TIM4->CCER = (TIM4->CCER & ~(TIM_CCER_CC3P | TIM_CCER_CC4P)) | TIM_CCER_CC3E | TIM_CCER_CC4E; /* Bật hai output active-high, compare vẫn đang bằng 0. */
  TIM4->CR1 |= TIM_CR1_ARPE; /* Bật preload ARR để chu kỳ luôn cập nhật đồng bộ. */
  TIM4->EGR = TIM_EGR_UG; /* Nạp prescaler và chu kỳ PWM vào phần cứng ngay. */
  TIM4->SR = 0U; /* Xóa các cờ timer phát sinh do lệnh update. */
  TIM4->CR1 |= TIM_CR1_CEN; /* Bắt đầu timer; output vẫn bằng 0 vì hai CCR đang bằng 0. */
  throttle_bridge_disable(); /* Giữ BTS7960 disable cho đến khi main loop kiểm tra cảm biến. */
}

/* Disable cầu H và xóa PWM để motor không thể tiếp tục nhận lệnh. */
static void throttle_bridge_disable(void)
{
  GPIOA->BSRR = (1U << (8U + 16U)) | (1U << (11U + 16U)); /* Kéo R_EN và L_EN xuống thấp để tắt hai nửa cầu. */
  TIM4->CCR3 = 0U; /* Xóa PWM hướng RPWM. */
  TIM4->CCR4 = 0U; /* Xóa PWM hướng LPWM. */
  g_tps_motor_active = 0U; /* Ghi nhận bridge đang tắt. */
  g_tps_direction = 0U; /* Xóa hướng cũ để lần chạy sau bắt đầu an toàn. */
}

/* Chạy một hướng của BTS7960; chiều mở dùng LPWM, chiều đóng dùng RPWM. */
static void throttle_bridge_drive(uint8_t direction, uint16_t duty_permille)
{
  if (duty_permille > TPS_PWM_MAX_PERMILLE) { duty_permille = TPS_PWM_MAX_PERMILLE; } /* Chặn duty ở 60% theo file test_throttle. */
  if (direction != 1U && direction != 2U) { throttle_bridge_disable(); return; } /* Hướng không hợp lệ luôn đưa cầu H về trạng thái tắt. */
  if (g_tps_direction != 0U && g_tps_direction != direction) { throttle_bridge_disable(); delay_ms(2U); } /* Chèn khoảng nghỉ trước khi đảo chiều motor. */
  GPIOA->BSRR = (1U << (8U + 16U)) | (1U << (11U + 16U)); /* Disable hai nửa cầu trong lúc cập nhật compare. */
  if (direction == 1U) { TIM4->CCR3 = 0U; TIM4->CCR4 = ((uint32_t)TIM4->ARR * duty_permille) / 1000U; } /* Mở: PB9/LPWM có duty, PB8/RPWM giữ 0. */
  else { TIM4->CCR4 = 0U; TIM4->CCR3 = ((uint32_t)TIM4->ARR * duty_permille) / 1000U; } /* Đóng: PB8/RPWM có duty, PB9/LPWM giữ 0. */
  g_tps_direction = direction; /* Lưu chiều đang được chọn để nhận biết lúc đảo chiều. */
  GPIOA->BSRR = (1U << 8U) | (1U << 11U); /* Enable cầu H sau khi PWM chỉ còn một phía. */
  g_tps_motor_active = 1U; /* Ghi nhận motor đang được điều khiển. */
}

/* Kiểm tra hai track cảm biến có nằm trong dải điện áp đã quan sát trên bench. */
static uint8_t throttle_sensor_check(uint16_t vta1_raw, uint16_t vta2_raw)
{
  if (vta1_raw < TPS_VTA1_MIN_RAW || vta1_raw > TPS_VTA1_MAX_RAW) { return 0U; } /* Tắt motor nếu VTA1 hở, chập hoặc ngoài vùng đo. */
  if (vta2_raw < TPS_VTA2_MIN_RAW || vta2_raw > TPS_VTA2_MAX_RAW) { return 0U; } /* Tắt motor nếu track kiểm tra VTA2 ngoài vùng đo. */
  return 1U; /* Cả hai track đang nằm trong cửa sổ bench đã chọn. */
}

/* Chuẩn hóa raw của một track thành vị trí từ 0.0 đến 1.0. */
static float throttle_normalize_position(uint16_t raw, uint16_t rest, uint16_t full)
{
  float position = ((float)raw - (float)rest) / ((float)full - (float)rest); /* Nội suy theo điểm nghỉ và điểm mở hết. */
  if (position < 0.0f) { position = 0.0f; } /* Kẹp dưới điểm nghỉ. */
  if (position > 1.0f) { position = 1.0f; } /* Kẹp trên điểm mở hết đã hiệu chuẩn. */
  return position; /* Trả vị trí chuẩn hóa. */
}

/* Trung bình vị trí chuẩn hóa của VTA1 và VTA2 để hiển thị phản hồi chung. */
static uint8_t throttle_feedback_percent(uint16_t vta1_raw, uint16_t vta2_raw)
{
  float position = (throttle_normalize_position(vta1_raw, TPS_VTA1_IDLE_RAW, TPS_VTA1_OPEN_RAW) + throttle_normalize_position(vta2_raw, TPS_VTA2_IDLE_RAW, TPS_VTA2_OPEN_RAW)) * 0.5f; /* Trung bình hai track cùng chiều. */
  return (uint8_t)(position * 100.0f + 0.5f); /* Đổi hành trình trung bình thành phần trăm. */
}

/* Cập nhật trung bình trượt 20 mẫu cho bàn đạp và hai track VTA. */
static void throttle_adc_average_update(void)
{
  const uint16_t samples[3] = {g_adc_samples[0], g_adc_samples[2], g_adc_samples[3]}; /* Chụp PA1 pedal, PA3 VTA1 và PB0 VTA2 từ DMA. */
  for (uint8_t channel = 0U; channel < 3U; channel++) { /* Cập nhật từng ngõ vào dùng trong vòng kín. */
    if (g_tps_adc_count == TPS_ADC_AVERAGE_SAMPLES) { g_tps_adc_sum[channel] -= g_tps_adc_history[channel][g_tps_adc_index]; } /* Bỏ mẫu cũ khỏi tổng khi bộ đệm đã đầy. */
    g_tps_adc_history[channel][g_tps_adc_index] = samples[channel]; /* Ghi mẫu mới vào vòng đệm. */
    g_tps_adc_sum[channel] += samples[channel]; /* Cộng mẫu mới vào tổng. */
    g_tps_adc_average[channel] = (uint16_t)(g_tps_adc_sum[channel] / (g_tps_adc_count < TPS_ADC_AVERAGE_SAMPLES ? (g_tps_adc_count + 1U) : TPS_ADC_AVERAGE_SAMPLES)); /* Tính trung bình số mẫu hiện có. */
  }
  g_tps_adc_index++; /* Chuyển sang ô mẫu tiếp theo. */
  if (g_tps_adc_index >= TPS_ADC_AVERAGE_SAMPLES) { g_tps_adc_index = 0U; } /* Quay vòng sau 20 mẫu. */
  if (g_tps_adc_count < TPS_ADC_AVERAGE_SAMPLES) { g_tps_adc_count++; } /* Tăng số mẫu đã tích lũy. */
}

/* Điều khiển vị trí PID theo bàn đạp, trung bình VTA1/VTA2 và trạng thái ECU. */
static void throttle_control_update(uint16_t pedal_raw, uint16_t vta1_raw, uint16_t vta2_raw)
{
  static uint16_t last_progress_raw; /* Mẫu VTA gần nhất dùng để xác nhận motor đang làm cánh bướm di chuyển. */
  static uint16_t stalled_elapsed_ms; /* Thời gian motor chạy nhưng chưa thấy tiến triển đáng kể. */
  static uint8_t monitored_direction; /* Hướng của chu kỳ giám sát chuyển động hiện tại. */
  if (g_tps_adc_count > 0U) { g_tps_sensor_ok = throttle_sensor_check(vta1_raw, vta2_raw); g_tps_sensor_fault = (uint8_t)(g_tps_sensor_ok == 0U); g_tps_feedback_percent = throttle_feedback_percent(vta1_raw, vta2_raw); } /* Cập nhật hiển thị và trạng thái cảm biến cả khi ECU đang STOP. */
  if (!g_engine_running) { g_tps_fault_latched = 0U; g_tps_pid_integral = 0.0f; g_tps_control_elapsed_ms = 0U; stalled_elapsed_ms = 0U; monitored_direction = 0U; throttle_bridge_disable(); return; } /* STOP tắt cầu H và xóa lỗi kẹt theo hành vi ECU hiện có. */
  g_tps_control_elapsed_ms++; /* Main loop chạy theo nhịp 1 ms. */
  if (g_tps_control_elapsed_ms < TPS_CONTROL_PERIOD_MS) { return; } /* Chỉ cập nhật PID mỗi 20 ms. */
  g_tps_control_elapsed_ms = 0U; /* Bắt đầu chu kỳ PID kế tiếp. */
  if (g_tps_adc_count < TPS_ADC_AVERAGE_SAMPLES) { throttle_bridge_disable(); return; } /* Chờ tích lũy đủ 20 mẫu ADC trước khi điều khiển. */

  if (!g_tps_sensor_ok) { throttle_bridge_disable(); g_tps_pid_integral = 0.0f; g_tps_fault_waiting = 1U; return; } /* Lỗi cảm biến tắt cầu H và chờ pedal nhả. */

  float pos1 = throttle_normalize_position(vta1_raw, TPS_VTA1_IDLE_RAW, TPS_VTA1_OPEN_RAW); /* Chuẩn hóa phản hồi VTA1. */
  float pos2 = throttle_normalize_position(vta2_raw, TPS_VTA2_IDLE_RAW, TPS_VTA2_OPEN_RAW); /* Chuẩn hóa phản hồi VTA2. */
  float actual = (pos1 + pos2) * 0.5f; /* Tính vị trí thực trung bình hai track. */

  if (g_tps_fault_waiting != 0U) { throttle_bridge_disable(); if (pedal_raw <= PEDAL_IDLE_RAW) { g_tps_fault_waiting = 0U; g_tps_pid_integral = 0.0f; g_tps_previous_position = actual; } return; } /* Chỉ thoát chờ khi cảm biến tốt và bàn đạp đã nhả. */
  if (g_tps_armed == 0U) { throttle_bridge_disable(); if (pedal_raw <= PEDAL_IDLE_RAW) { g_tps_armed = 1U; g_tps_pid_integral = 0.0f; g_tps_previous_position = actual; } return; } /* Chỉ arm sau khi bàn đạp nhả. */
  if (g_tps_fault_latched != 0U) { throttle_bridge_disable(); return; } /* Giữ cầu H tắt nếu đã phát hiện kẹt motor. */

  uint16_t bounded_pedal = pedal_raw; /* Giới hạn mẫu bàn đạp về dải ADC 12-bit. */
  if (bounded_pedal > ADC_FULL_SCALE) { bounded_pedal = ADC_FULL_SCALE; } /* Kẹp giá trị cao nhất. */
  uint8_t level = (uint8_t)(((uint32_t)bounded_pedal * PEDAL_LEVEL_COUNT) / (ADC_FULL_SCALE + 1U)); /* Chia ADC bàn đạp thành 10 mức rời rạc. */
  if (level >= PEDAL_LEVEL_COUNT) { level = PEDAL_LEVEL_COUNT - 1U; } /* Bảo đảm mức cao nhất bằng 9. */
  float target = (float)level / (float)(PEDAL_LEVEL_COUNT - 1U); /* Quy đổi mức bàn đạp thành mục tiêu 0..100% hành trình. */
  float error = target - actual; /* Sai số vị trí đặt trừ vị trí thực. */
  float rate = (actual - g_tps_previous_position) / ((float)TPS_CONTROL_PERIOD_MS / 1000.0f); /* Tốc độ thay đổi vị trí theo giây. */
  g_tps_previous_position = actual; /* Lưu vị trí cho thành phần D chu kỳ sau. */
  if ((error > -TPS_POSITION_DEADBAND) && (error < TPS_POSITION_DEADBAND)) { error = 0.0f; } /* Triệt sai lệch nằm trong vùng chết 2%. */

  float next_integral = g_tps_pid_integral + TPS_PID_KI * error * ((float)TPS_CONTROL_PERIOD_MS / 1000.0f); /* Tính tích phân dự kiến. */
  if (next_integral > TPS_PID_I_LIMIT) { next_integral = TPS_PID_I_LIMIT; } /* Giới hạn tích phân dương. */
  if (next_integral < -TPS_PID_I_LIMIT) { next_integral = -TPS_PID_I_LIMIT; } /* Giới hạn tích phân âm. */
  float output = TPS_PID_KP * error + next_integral - TPS_PID_KD * rate; /* PID dương mở, âm đóng. */
  float output_limit = (float)TPS_PWM_MAX_PERMILLE / 10.0f; /* Quy đổi duty tối đa permille thành phần trăm. */
  float limited = output; /* Bản sao dùng để giới hạn đầu ra. */
  if (limited > output_limit) { limited = output_limit; } /* Chặn lệnh mở tại duty tối đa. */
  if (limited < -output_limit) { limited = -output_limit; } /* Chặn lệnh đóng tại duty tối đa. */
  if ((output == limited) || ((output > output_limit) && (error < 0.0f)) || ((output < -output_limit) && (error > 0.0f))) { g_tps_pid_integral = next_integral; } /* Chống tích phân tăng khi đầu ra bão hòa sai hướng. */
  else { output = TPS_PID_KP * error + g_tps_pid_integral - TPS_PID_KD * rate; limited = output; if (limited > output_limit) { limited = output_limit; } if (limited < -output_limit) { limited = -output_limit; } } /* Tính lại bằng I cũ khi bão hòa. */

  uint8_t requested_direction = 0U; /* Mặc định dừng motor. */
  uint16_t duty = 0U; /* Mặc định duty bằng 0. */
  if (limited > 0.0f) { requested_direction = 1U; float percent = limited; float min_percent = (float)TPS_PWM_MIN_PERMILLE / 10.0f; if (percent < min_percent) { percent = min_percent; } duty = (uint16_t)(percent * 10.0f + 0.5f); } /* Lệnh mở LPWM, duty sàn 10%. */
  else if (limited < 0.0f) { requested_direction = 2U; float percent = -limited; float min_percent = (float)TPS_PWM_MIN_PERMILLE / 10.0f; if (percent < min_percent) { percent = min_percent; } duty = (uint16_t)(percent * 10.0f + 0.5f); } /* Lệnh đóng RPWM, duty sàn 10%. */
  if ((pedal_raw <= PEDAL_IDLE_RAW) && (actual <= TPS_POSITION_DEADBAND)) { g_tps_pid_integral = 0.0f; requested_direction = 0U; duty = 0U; } /* Ở pedal nghỉ và vị trí nghỉ thì không giữ motor có điện. */
  if ((requested_direction == 1U) && ((vta1_raw >= TPS_VTA1_OPEN_RAW) || (vta2_raw >= TPS_VTA2_OPEN_RAW))) { g_tps_pid_integral = 0.0f; requested_direction = 0U; duty = 0U; } /* Chặn mở khi một trong hai track chạm điểm mở hết. */
  if ((requested_direction == 2U) && (vta1_raw <= (TPS_VTA1_IDLE_RAW + 14U)) && (vta2_raw <= (TPS_VTA2_IDLE_RAW + 14U))) { g_tps_pid_integral = 0.0f; requested_direction = 0U; duty = 0U; } /* Chặn đóng khi cả hai track về gần vị trí nghỉ. */

  if (requested_direction == 0U) { stalled_elapsed_ms = 0U; monitored_direction = 0U; throttle_bridge_disable(); return; } /* Khi PID không ra lệnh thì dừng bridge. */
  if (monitored_direction != requested_direction) { monitored_direction = requested_direction; stalled_elapsed_ms = 0U; last_progress_raw = vta1_raw; } /* Bắt đầu giám sát chuyển động sau khi đổi chiều. */
  uint16_t movement = (vta1_raw >= last_progress_raw) ? (uint16_t)(vta1_raw - last_progress_raw) : (uint16_t)(last_progress_raw - vta1_raw); /* Tính độ đổi feedback theo trị tuyệt đối. */
  if (movement >= 8U) { last_progress_raw = vta1_raw; stalled_elapsed_ms = 0U; } /* Có chuyển động đủ rõ thì khởi động lại bộ đếm kẹt. */
  else if (stalled_elapsed_ms < TPS_STALL_LIMIT_MS) { stalled_elapsed_ms = (uint16_t)(stalled_elapsed_ms + TPS_CONTROL_PERIOD_MS); } /* Cộng 20 ms nếu chưa thấy motor di chuyển. */
  if (stalled_elapsed_ms >= TPS_STALL_LIMIT_MS) { g_tps_fault_latched = 1U; throttle_bridge_disable(); return; } /* Chốt lỗi và ngắt motor nếu đứng yên quá thời hạn. */
  throttle_bridge_drive(requested_direction, duty); /* Cấp PWM PID theo chiều cần hiệu chỉnh. */
}

/* Cấu hình ADC1 scan PA1/PA2/PA3/PB0 và DMA1 Channel 1 circular. */
static void adc1_dma_init(void)
{
  RCC->AHBENR |= RCC_AHBENR_DMA1EN; /* Cấp clock AHB cho DMA1. */
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; /* Cấp clock APB2 cho ADC1. */
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV6; /* Chọn ADC clock 72/6 = 12 MHz. */

  DMA1_Channel1->CCR &= ~DMA_CCR_EN; /* Tắt DMA channel trước khi nạp cấu hình. */
  while ((DMA1_Channel1->CCR & DMA_CCR_EN) != 0U) { /* Chờ phần cứng xác nhận DMA đã dừng. */ }
  DMA1->IFCR = DMA_IFCR_CGIF1; /* Xóa các cờ DMA1 Channel 1 còn sót lại. */
  DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR; /* Đặt địa chỉ ngoại vi nguồn là ADC1 data register. */
  DMA1_Channel1->CMAR = (uint32_t)g_adc_samples; /* Đặt bộ đệm RAM chứa bốn mẫu ADC. */
  DMA1_Channel1->CNDTR = 4U; /* Mỗi chu kỳ ghi bốn mẫu theo thứ tự PA1, PA2, PA3, PB0. */
  DMA1_Channel1->CCR = DMA_CCR_PL_0 | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_CIRC; /* Halfword, tăng địa chỉ RAM, circular, ưu tiên medium. */
  DMA1_Channel1->CCR |= DMA_CCR_EN; /* Bật DMA trước khi ADC bắt đầu chuyển đổi. */

  ADC1->CR1 = ADC_CR1_SCAN; /* Bật scan để ADC lần lượt đọc bốn kênh. */
  ADC1->CR2 = 0U; /* Đặt các chế độ ADC về mặc định trước khi cấu hình. */
  ADC1->SMPR2 = (3U << ADC_SMPR2_SMP1_Pos) | (3U << ADC_SMPR2_SMP2_Pos) | (3U << ADC_SMPR2_SMP3_Pos) | (3U << ADC_SMPR2_SMP8_Pos); /* Chọn sample time 28.5 chu kỳ cho các kênh 1, 2, 3 và 8. */
  ADC1->SQR1 = (3U << ADC_SQR1_L_Pos); /* Đặt độ dài regular sequence thành bốn phép đo (L = 4 - 1). */
  ADC1->SQR3 = (1U << ADC_SQR3_SQ1_Pos) | (2U << ADC_SQR3_SQ2_Pos) | (3U << ADC_SQR3_SQ3_Pos) | (8U << ADC_SQR3_SQ4_Pos); /* Đọc PA1, PA2, PA3/VTA rồi PB0/VTA2. */
  ADC1->SR = 0U; /* Xóa cờ trạng thái ADC cũ. */
  ADC1->CR2 = ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_EXTTRIG | (7U << ADC_CR2_EXTSEL_Pos); /* Bật liên tục, DMA, trigger ngoài và chọn software start. */
  ADC1->CR2 |= ADC_CR2_ADON; /* Cấp nguồn cho ADC trước hiệu chuẩn. */
  delay_ms(1U); /* Chờ ADC ổn định sau khi bật nguồn. */
  ADC1->CR2 |= ADC_CR2_RSTCAL; /* Yêu cầu ADC xóa hệ số hiệu chuẩn cũ. */
  while ((ADC1->CR2 & ADC_CR2_RSTCAL) != 0U) { /* Chờ phần cứng xóa hiệu chuẩn xong. */ }
  ADC1->CR2 |= ADC_CR2_CAL; /* Bắt đầu hiệu chuẩn ADC. */
  while ((ADC1->CR2 & ADC_CR2_CAL) != 0U) { /* Chờ hiệu chuẩn hoàn tất. */ }
  ADC1->CR2 |= ADC_CR2_SWSTART; /* Bắt đầu chuỗi chuyển đổi và để CONT lặp liên tục. */
}

/* Cấu hình PB6/PB7 và I2C1 ở Standard Mode 100 kHz. */
static void i2c1_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; /* Cấp clock GPIOB cho PB6/PB7. */
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; /* Cấp clock APB1 cho I2C1. */
  gpio_set_config(GPIOB, 6U, 0xDU); /* PB6 alternate-function open-drain 10 MHz làm SCL. */
  gpio_set_config(GPIOB, 7U, 0xDU); /* PB7 alternate-function open-drain 10 MHz làm SDA. */
  I2C1->CR1 = I2C_CR1_SWRST; /* Reset bộ điều khiển I2C để xóa trạng thái trước đó. */
  I2C1->CR1 = 0U; /* Thả reset và giữ I2C tắt khi cấu hình timing. */
  I2C1->CR2 = 36U; /* Ghi tần số PCLK1 là 36 MHz vào trường FREQ. */
  I2C1->CCR = 180U; /* Standard mode: 36 MHz/(2*180) = 100 kHz. */
  I2C1->TRISE = 37U; /* Thời gian lên tối đa theo PCLK1 36 MHz ở Standard mode. */
  I2C1->CR1 = I2C_CR1_PE; /* Bật ngoại vi I2C1 sau khi cấu hình xong. */
}

/* Chờ cờ SR1 hoặc trả lỗi nếu I2C báo lỗi hay hết timeout. */
static int i2c_wait_sr1(uint32_t flag)
{
  uint32_t timeout = I2C_TIMEOUT; /* Đặt giới hạn thời gian chờ cho thao tác này. */
  while (((I2C1->SR1 & flag) == 0U) && ((I2C1->SR1 & (I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR)) == 0U) && (timeout > 0U)) { timeout--; } /* Chờ cờ hoàn tất hoặc cờ lỗi. */
  if ((I2C1->SR1 & (I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR)) != 0U) { return 0; } /* Báo lỗi nếu ACK/bus/arbitration gặp vấn đề. */
  return ((I2C1->SR1 & flag) != 0U); /* Trả theo trạng thái cờ để cả trường hợp sát ngưỡng timeout vẫn đúng. */
}

/* Gửi một byte cho PCF8574 và kết thúc giao dịch I2C bằng STOP. */
static int pcf8574_write(uint8_t value)
{
  I2C1->CR1 |= I2C_CR1_START; /* Phát điều kiện START trên bus. */
  if (!i2c_wait_sr1(I2C_SR1_SB)) { goto i2c_error; } /* Chờ START hoặc xử lý lỗi bus. */
  I2C1->DR = (uint8_t)(LCD_I2C_ADDRESS << 1U); /* Gửi địa chỉ 7-bit cộng bit ghi bằng 0. */
  if (!i2c_wait_sr1(I2C_SR1_ADDR)) { goto i2c_error; } /* Chờ LCD/PCF8574 xác nhận địa chỉ. */
  (void)I2C1->SR1; /* Đọc SR1 theo trình tự bắt buộc để xóa cờ ADDR. */
  (void)I2C1->SR2; /* Đọc SR2 hoàn tất thao tác xóa cờ ADDR. */
  if (!i2c_wait_sr1(I2C_SR1_TXE)) { goto i2c_error; } /* Chờ data register trống trước khi ghi dữ liệu. */
  I2C1->DR = value; /* Gửi byte điều khiển tới PCF8574. */
  if (!i2c_wait_sr1(I2C_SR1_BTF)) { goto i2c_error; } /* Chờ byte truyền xong trên bus. */
  I2C1->CR1 |= I2C_CR1_STOP; /* Phát điều kiện STOP để kết thúc giao dịch. */
  uint32_t stop_timeout = I2C_TIMEOUT; /* Đặt giới hạn chờ phần cứng phát xong STOP. */
  while (((I2C1->CR1 & I2C_CR1_STOP) != 0U) && (stop_timeout > 0U)) { stop_timeout--; } /* Đợi STOP tự xóa trước giao dịch kế tiếp. */
  if (stop_timeout == 0U) { goto i2c_error; } /* Báo lỗi nếu bus không hoàn tất STOP. */
  return 1; /* Báo giao dịch thành công. */

i2c_error:
  I2C1->CR1 |= I2C_CR1_STOP; /* Cố gắng giải phóng bus sau lỗi hoặc timeout. */
  I2C1->SR1 &= ~(I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR); /* Xóa các cờ lỗi để lần gửi tiếp có thể thử lại. */
  return 0; /* Báo giao dịch thất bại cho tầng LCD. */
}

/* Gửi nibble dữ liệu và tạo xung Enable cho backpack PCF8574. */
static int lcd_write_nibble(uint8_t nibble, uint8_t rs)
{
  uint8_t value = (uint8_t)((nibble << 4U) | LCD_BACKLIGHT_BIT | (rs ? LCD_REGISTER_SELECT : 0U)); /* Ánh xạ D4..D7, đèn nền và RS lên PCF8574. */
  if (!pcf8574_write((uint8_t)(value | LCD_ENABLE_BIT))) { return 0; } /* Dừng nếu lần ghi Enable cao thất bại. */
  if (!pcf8574_write(value)) { return 0; } /* Dừng nếu lần ghi Enable thấp thất bại. */
  return 1; /* Xác nhận nibble đã được gửi thành công. */
}

/* Gửi một byte bằng hai nibble theo chế độ 4-bit của LCD. */
static int lcd_write_byte(uint8_t value, uint8_t rs)
{
  if (!lcd_write_nibble((uint8_t)(value >> 4U), rs)) { return 0; } /* Dừng và báo lỗi nếu nibble cao không gửi được. */
  if (!lcd_write_nibble((uint8_t)(value & 0x0FU), rs)) { return 0; } /* Dừng và báo lỗi nếu nibble thấp không gửi được. */
  if ((rs == 0U) && ((value == 0x01U) || (value == 0x02U))) { delay_ms(2U); } /* Lệnh clear/home cần thời gian thực thi dài hơn. */
  return 1; /* Xác nhận byte LCD đã được gửi thành công. */
}

/* Đưa LCD về chế độ 4-bit, hai dòng, font 5x8 và bật hiển thị. */
static void lcd_init(void)
{
  g_lcd_available = 1U; /* Cho phép thử giao tiếp LCD trong quá trình khởi tạo. */
  delay_ms(50U); /* Chờ LCD ổn định sau khi cấp nguồn. */
  if (!lcd_write_nibble(0x03U, 0U)) { goto lcd_unavailable; } /* Bắt đầu khởi tạo; bỏ LCD nếu bus không phản hồi. */
  delay_ms(5U); /* Chờ LCD xử lý lệnh khởi tạo lần một. */
  if (!lcd_write_nibble(0x03U, 0U)) { goto lcd_unavailable; } /* Lặp lại reset lần hai và kiểm tra bus. */
  delay_ms(1U); /* Chờ LCD xử lý lệnh lần hai. */
  if (!lcd_write_nibble(0x03U, 0U)) { goto lcd_unavailable; } /* Lặp lại reset lần ba và kiểm tra bus. */
  delay_ms(1U); /* Chờ LCD xử lý lệnh lần ba. */
  if (!lcd_write_nibble(0x02U, 0U)) { goto lcd_unavailable; } /* Chuyển giao tiếp LCD sang 4-bit. */
  if (!lcd_write_byte(0x28U, 0U)) { goto lcd_unavailable; } /* Chọn 4-bit, hai dòng và font 5x8. */
  if (!lcd_write_byte(0x0CU, 0U)) { goto lcd_unavailable; } /* Bật màn hình và tắt con trỏ. */
  if (!lcd_write_byte(0x06U, 0U)) { goto lcd_unavailable; } /* Tăng địa chỉ DDRAM sau mỗi ký tự. */
  if (!lcd_write_byte(0x01U, 0U)) { goto lcd_unavailable; } /* Xóa màn hình trước khi hiển thị dữ liệu. */
  return; /* Kết thúc khởi tạo khi mọi giao dịch đều thành công. */

lcd_unavailable:
  g_lcd_available = 0U; /* Bỏ qua LCD khi I2C không phản hồi để các chức năng khác tiếp tục. */
}

/* Đặt con trỏ LCD theo hàng 0/1 và cột 0..15. */
static int lcd_set_cursor(uint8_t row, uint8_t column)
{
  uint8_t address = (row == 0U) ? (uint8_t)(0x80U + column) : (uint8_t)(0xC0U + column); /* Chọn địa chỉ DDRAM của hàng 1 hoặc hàng 2. */
  return lcd_write_byte(address, 0U); /* Gửi lệnh set DDRAM address và trả kết quả giao dịch. */
}

/* Ghi trọn một hàng 16 ký tự và tự thêm khoảng trắng để xóa ký tự cũ. */
static int lcd_write_line(uint8_t row, const char *text)
{
  if (!g_lcd_available) { return 0; } /* Bỏ qua cập nhật nếu LCD đã được đánh dấu không sẵn sàng. */
  if (!lcd_set_cursor(row, 0U)) { return 0; } /* Đưa con trỏ về đầu hàng; dừng nếu I2C lỗi. */
  for (uint8_t column = 0U; column < 16U; column++) { /* Ghi đúng chiều rộng LCD 16x2. */
    char character = (text[column] != '\0') ? text[column] : ' '; /* Dùng khoảng trắng cho phần chuỗi ngắn hơn 16 ký tự. */
    if (!lcd_write_byte((uint8_t)character, 1U)) { return 0; } /* Dừng cả hàng ngay khi giao dịch I2C đầu tiên lỗi. */
  }
  return 1; /* Xác nhận ghi xong đủ 16 ký tự. */
}

/* Cấu hình USART1 8N1, 115200 baud, dùng PA9 TX và PA10 RX. */
static void usart1_init(void)
{
  USART1->CR1 = 0U; /* Tắt USART trong lúc nạp cấu hình. */
  USART1->CR2 = 0U; /* Chọn một stop bit mặc định. */
  USART1->CR3 = 0U; /* Tắt flow control và DMA cho UART. */
  USART1->BRR = 625U; /* Với PCLK2 72 MHz, USARTDIV=625 cho baud 115200 oversampling 16. */
  USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE; /* Bật USART, bộ phát và bộ thu. */
}

/* Gửi chuỗi ký tự qua USART1 với timeout cho từng byte. */
static void usart1_write_string(const char *text)
{
  while (*text != '\0') { /* Duyệt từng ký tự cho đến cuối chuỗi. */
    uint32_t timeout = UART_TX_TIMEOUT; /* Đặt timeout riêng cho byte hiện tại. */
    while (((USART1->SR & USART_SR_TXE) == 0U) && (timeout > 0U)) { timeout--; } /* Chờ thanh ghi truyền trống. */
    if (timeout == 0U) { return; } /* Bỏ phần còn lại nếu UART không phản hồi. */
    USART1->DR = (uint8_t)*text; /* Nạp byte hiện tại vào data register UART. */
    text++; /* Chuyển sang ký tự kế tiếp. */
  }
  uint32_t timeout = UART_TX_TIMEOUT; /* Đặt timeout chờ bit cuối cùng rời khỏi chân TX. */
  while (((USART1->SR & USART_SR_TC) == 0U) && (timeout > 0U)) { timeout--; } /* Chờ truyền xong toàn bộ chuỗi. */
}

/* Debounce nút ngoài ISR để ISR luôn ngắn và không block. */
static void process_button(void)
{
  if (g_button_event == 0U) { return; } /* Không làm gì khi chưa có sự kiện nút. */
  g_button_event = 0U; /* Nhận và xóa sự kiện để xử lý đúng một lần. */
  delay_ms(20U); /* Chờ 20 ms để lọc rung tiếp điểm. */
  if ((GPIOA->IDR & (1U << BUTTON_PIN)) == 0U) { /* Xác nhận nút vẫn đang được nhấn mức thấp. */
    g_engine_running ^= 1U; /* Đảo trạng thái Start/Stop của động cơ. */
    while ((GPIOA->IDR & (1U << BUTTON_PIN)) == 0U) { delay_ms(5U); } /* Chờ người dùng nhả nút để không nhận lặp. */
    delay_ms(20U); /* Lọc rung tiếp điểm khi nhả nút. */
  }
  EXTI->PR = EXTI_PR_PR0; /* Xóa cạnh phát sinh trong thời gian debounce. */
  EXTI->IMR |= EXTI_IMR_MR0; /* Mở lại EXTI0 cho lần nhấn kế tiếp. */
}

/* Vòng lặp chính đọc cảm biến, điều khiển quạt/LCD và gửi telemetry UART. */
int main(void)
{
  if (!system_clock_init()) { while (1) { } } /* Dừng an toàn nếu clock 72 MHz không khởi động được. */
  tim2_delay_init(); /* Khởi động timer nền cho các delay đồng bộ. */
  gpio_init(); /* Cấu hình toàn bộ chân GPIO được dùng trong project. */
  exti0_init(); /* Kích hoạt nút Start/Stop PA0. */
  tim3_pwm_init(); /* Bắt đầu PWM quạt ở trạng thái duty 0%. */
  tim4_throttle_pwm_init(); /* Chuẩn bị PWM BTS7960 nhưng giữ hai chân Enable ở mức thấp. */
  adc1_dma_init(); /* Bắt đầu đọc biến trở ga và nhiệt độ liên tục. */
  i2c1_init(); /* Khởi động bus I2C1 để giao tiếp với LCD backpack. */
  usart1_init(); /* Khởi động UART1 để gửi dữ liệu về máy tính. */
  lcd_init(); /* Khởi tạo LCD 16x2. */

  uint32_t ui_elapsed_ms = 0U; /* Bộ đếm thời gian cập nhật LCD. */
  uint32_t uart_elapsed_ms = 0U; /* Bộ đếm thời gian gửi UART. */
  char uart_line[256]; /* Bộ đệm telemetry gồm pedal, feedback, trạng thái an toàn và cầu H. */

  while (1) { /* Chạy lặp liên tục như firmware bare-metal. */
    process_button(); /* Xử lý nút Start/Stop nếu ISR đã ghi nhận sự kiện. */

    uint16_t temperature_raw = g_adc_samples[1]; /* Chụp mẫu ADC kênh PA2 do DMA cập nhật. */
    uint16_t vta1_raw = g_adc_samples[2]; /* Chụp mẫu ADC PA3/VTA sau cầu chia áp. */
    uint16_t vta2_raw = g_adc_samples[3]; /* Chụp mẫu ADC PB0/VTA2 sau cầu chia áp. */
    throttle_adc_average_update(); /* Lọc 20 mẫu bàn đạp và hai track bướm ga. */
    uint16_t throttle_filtered = g_tps_adc_average[0]; /* Lấy bàn đạp đã lọc cho điều khiển và hiển thị. */
    uint16_t vta1_filtered = g_tps_adc_average[1]; /* Lấy VTA1 đã lọc cho vòng kín. */
    uint16_t vta2_filtered = g_tps_adc_average[2]; /* Lấy VTA2 đã lọc cho vòng kín. */
    uint32_t vta1_sensor_mv = ((uint32_t)vta1_raw * ADC_VDDA_MV * TPS_DIVIDER_SCALE_NUM) / (ADC_FULL_SCALE * TPS_DIVIDER_SCALE_DEN); /* Ước lượng VTA trước cầu chia áp theo VDDA đã đo và cầu chia 20 kΩ/10 kΩ. */
    uint32_t vta2_sensor_mv = ((uint32_t)vta2_raw * ADC_VDDA_MV * TPS_DIVIDER_SCALE_NUM) / (ADC_FULL_SCALE * TPS_DIVIDER_SCALE_DEN); /* Ước lượng VTA2 trước cầu chia áp theo cùng hệ số đã đo. */
    uint8_t throttle_percent = (uint8_t)(((uint32_t)throttle_filtered * 100U) / ADC_FULL_SCALE); /* Hiển thị biến trở ga đã lọc theo dải 0..100%. */
    uint16_t temperature_c = (uint16_t)(((uint32_t)temperature_raw * TEMP_SIM_MAX_C) / ADC_FULL_SCALE); /* Quy đổi biến trở nhiệt tuyến tính thành 0..150°C mô phỏng. */
    uint8_t fan_percent = (g_engine_running && (temperature_c > FAN_ON_TEMP_C)) ? 100U : 0U; /* Bật quạt 100% khi máy chạy và nhiệt độ trên 80°C; các trường hợp khác tắt quạt. */

    throttle_control_update(throttle_filtered, vta1_filtered, vta2_filtered); /* Điều khiển PID mỗi 20 ms bằng pedal/VTA đã lọc. */
    tim3_set_duty((uint16_t)fan_percent * 10U); /* Chuyển 0..100% thành duty 0..1000 của TIM3. */
    if (ui_elapsed_ms >= 250U) { /* Cập nhật LCD định kỳ để giảm tải I2C. */
      char row0[17]; /* Bộ đệm hàng đầu LCD gồm 16 ký tự và ký tự kết thúc. */
      char row1[17]; /* Bộ đệm hàng dưới LCD gồm 16 ký tự và ký tự kết thúc. */
      (void)snprintf(row0, sizeof(row0), "P:%3u T:%3u F:%3u", (unsigned int)throttle_percent, (unsigned int)g_tps_feedback_percent, (unsigned int)fan_percent); /* Hiển thị bàn đạp, vị trí bướm ga thật và quạt. */
      (void)snprintf(row1, sizeof(row1), "TMP:%3uC %s", (unsigned int)temperature_c, g_engine_running ? "RUN" : "STOP"); /* Hiển thị nhiệt mô phỏng và trạng thái máy. */
      if (!lcd_write_line(0U, row0) || !lcd_write_line(1U, row1)) { g_lcd_available = 0U; } /* Tắt cập nhật LCD sau lỗi I2C để UART không bị nghẽn. */
      ui_elapsed_ms = 0U; /* Bắt đầu lại chu kỳ cập nhật LCD. */
    }

    if (uart_elapsed_ms >= 500U) { /* Gửi một gói dữ liệu dạng text mỗi nửa giây. */
      int length = snprintf(uart_line, sizeof(uart_line), "ENGINE=%s,PEDAL=%u%%,TBODY=%u%%,TEMP=%uC,FAN=%u%%,VTA_RAW=%u,VTA_mV~%u,VTA2_RAW=%u,VTA2_mV~%u,TPS_OK=%u,TPS_ARMED=%u,TPS_SENSOR_FAULT=%u,TPS_WAITING=%u,MOTOR=%u,STALL=%u\r\n", g_engine_running ? "RUN" : "STOP", (unsigned int)throttle_percent, (unsigned int)g_tps_feedback_percent, (unsigned int)temperature_c, (unsigned int)fan_percent, (unsigned int)vta1_raw, (unsigned int)vta1_sensor_mv, (unsigned int)vta2_raw, (unsigned int)vta2_sensor_mv, (unsigned int)g_tps_sensor_ok, (unsigned int)g_tps_armed, (unsigned int)g_tps_sensor_fault, (unsigned int)g_tps_fault_waiting, (unsigned int)g_tps_motor_active, (unsigned int)g_tps_fault_latched); /* Gửi hai track, phản hồi vị trí và các cờ an toàn của bướm ga. */
      if (length > 0) { usart1_write_string(uart_line); } /* Chỉ gửi nếu snprintf tạo được chuỗi. */
      uart_elapsed_ms = 0U; /* Bắt đầu lại chu kỳ gửi UART. */
    }

    delay_ms(1U); /* Lấy mẫu thường xuyên để bộ lọc 20 mẫu có cửa sổ khoảng 20 ms. */
    ui_elapsed_ms++; /* Cộng 1 ms cho lịch cập nhật LCD. */
    uart_elapsed_ms++; /* Cộng 1 ms cho lịch gửi UART. */
  }
}
