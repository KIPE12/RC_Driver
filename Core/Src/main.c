/**
 * @mainpage RC_Driver
 *
 * <br>
 * <img src="전력전자학회로고1.png" width="400px" style="display:block; margin:0;"/>
 *
 * <br>
 * @section overview 프로젝트 개요
 * <b>RC_Driver</b>는 STM32G474RET6 마이크로컨트롤러 기반의
 * <b>14V / 10A PMSM(영구자석 동기 전동기) 인버터 드라이버</b>입니다.\n
 * RC카용 브러시리스 모터를 FOC(Field Oriented Control) 방식으로 구동하며,
 * Hall 센서 기반 위치/속도 추정과 센서리스 EEMF 옵저버를 지원합니다.
 *
 * @section hw 하드웨어 구성
 * | 항목 | 내용 |
 * |------|------|
 * | MCU | STM32G474RET6 (Cortex-M4, 170MHz, LQFP64) |
 * | 게이트 드라이버 | DRV8300DPWR – 3상 상하 암 6채널 구동 |
 * | 전력 스위치 | ISG0613N04NM6HATMA1 N-MOSFET × 6 |
 * | 전류 센서 | ACS725LLCTR-20AB-T (±20A, 66mV/A) × 3상 |
 * | 전원 공급 | TPS54202DDCR 벅 컨버터 (14V → 3.3V) |
 * | 홀 센서 인터페이스 | 74LVC125AD 레벨 시프터 + TPS60150 차지 펌프 (5V) |
 * | 입력 전압 | 14V (최대), 외부 12V IC 전원 사용 가능 |
 * | 정격 전류 | 10A |
 *
 * @section sw 소프트웨어 구조
 * 제어 루프는 TIM2 인터럽트(10kHz, 100µs 주기)에서 실행됩니다.
 * | 파일 | 역할 |
 * |------|------|
 * | main.c | 시스템 초기화, 파라미터 설정, 제어기 초기화 |
 * | control.c | 메인 제어 루프 – ADC 샘플링 → 보호 → 모드 분기 |
 * | inv.c | FOC 핵심 알고리즘 – Clarke/Park 변환, PI 제어, SVPWM, Hall 옵저버 |
 * | adc.c | ADC1 초기화 및 3상 전류(Ia, Ib, Ic) / DC링크 전압(Vdc) 측정 |
 * | speed_observer.c | 전체차원 속도 옵저버 및 EEMF 기반 센서리스 위치 추정 |
 * | fault.c | 하드웨어/소프트웨어 고장 감지 및 PWM 즉시 차단 |
 * | flag.c | 운전 모드 플래그 정의 (READY, FAULT, INV_RUN 등) |
 * | variable.c | 모듈 간 공유 전역 변수 |
 * | tim.c | TIM1(PWM 20kHz), TIM2(제어 10kHz), TIM15 초기화 |
 * | gpio.c | PWM_EN 출력 및 Hall 센서 입력 GPIO 설정 |
 *
 * @section algo 제어 알고리즘
 * - <b>Clarke / Park 변환</b>: 3상 정지 좌표계 → α-β → d-q 동기 좌표계
 * - <b>PI 전류 제어기</b>: d축(자속), q축(토크) 독립 제어, Anti-windup 포함
 * - <b>PI 속도 제어기</b>: 속도 기준 경사 제한(Ramp), Anti-windup 포함
 * - <b>SVPWM</b>: min-max 오프셋 방식, NLC(비선형 보상) 전압 추가
 * - <b>Hall PLL 옵저버</b>: 3비트 Hall 상태 → 전기각 매핑 → PI-PLL 속도 추정
 * - <b>EEMF 센서리스 옵저버</b>: 확장 동기 좌표계 역기전력 기반 위치 오차 추정
 *
 * @section modes 지원 운전 모드
 * | 플래그 | 모드 설명 |
 * |--------|-----------|
 * | INV_RUN | 폐루프 속도 제어 (Hall PLL + FOC 전류 제어) |
 * | INV_OLC | 전류 개루프 제어 (d/q 전류 기준 직접 지정) |
 * | INV_VOLC | 전압 개루프 제어 (d/q 전압 기준 직접 지정) |
 * | INV_ALIGN | 초기 d축 정렬 (4단계 상태 머신) |
 * | Param_Estimation | 파라미터 추정 (d축 구형파 전압 주입) |
 * | HALL_POS_TEST | Hall 센서 위치 확인 (기본 전압 벡터 순차 인가) |
 * | DUTY_TEST | PWM 듀티 직접 설정 테스트 |
 *
 * @section clk 시스템 클럭
 * HSI(16MHz) → PLL(×85/4/2) → SYSCLK <b>170MHz</b>\n
 * APB1/APB2: 85MHz, ADC1: SYSCLK/4 = 42.5MHz
 */

/**
 * @file    main.c
 * @brief   PMSM 인버터 제어 메인 진입점
 *
 * STM32G474RET6 기반의 PMSM(영구자석 동기 전동기) 인버터 제어 펌웨어
 * 메인 파일이다. 시스템 클럭, 주변장치(GPIO, ADC, TIM) 초기화 및
 * 제어 파라미터 설정 후 무한 루프로 진입한다.
 * 실제 제어 루프는 TIM2 인터럽트(Control())에서 실행된다.
 *
 * @details
 * - 시스템 클럭: HSI(16MHz) + PLL → 170MHz (SYSCLK)
 * - 제어 주기:   100µs (10kHz, TIM2 OC2 인터럽트)
 * - PWM 주파수:  약 20kHz (TIM1 Center-Aligned, ARR=4200, 170MHz)
 *
 * @author  HALAB_G
 * @date    2026
 *
 * @attention
 * Copyright (c) 2026 STMicroelectronics. All rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/**
 * @brief RC 수신기 입력 듀티 사이클 값 [0.0, 1.0]
 *
 * TIM15 입력 캡처로부터 계산된 PWM 듀티비.
 * Control() 함수 내에서 속도 또는 토크 기준으로 변환된다.
 */
float dutyCycle = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  애플리케이션 진입점 (메인 함수)
 *
 * 다음 순서로 초기화를 수행한 뒤 무한 루프로 진입한다:
 * 1. HAL 라이브러리 초기화 (HAL_Init)
 * 2. 시스템 클럭 설정 (SystemClock_Config)
 * 3. GPIO, ADC1, TIM1/TIM2/TIM15 초기화
 * 4. 전동기 파라미터, 제어기, 옵저버 초기화
 * 5. ADC 인터럽트 및 TIM2 OC 인터럽트 시작
 *
 * @retval int  항상 0을 반환하지만 실제로는 반환되지 않는다.
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration -------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM15_Init();

  /* USER CODE BEGIN 2 */

  /*
   * 전동기 파라미터 초기화 예시 (주석 처리된 항목은 다른 모터 설정):
   *
   * InitParameter(&INV,  Rs,     Ld,       Lq,       Lamf,    PP,  Jm,    Bm,
   *                      Idsr_align, Is_rated, Is_limit, Wrpm_rated, Te_rated)
   *
   * XERUN 13.5T 설정 사용:
   */
  InitParameter(&INV, 19e-3, 3.2e-6, 3.2e-6, 2e-3, 1.f, 1e-6, 1e-6, 2.f, 50.f, 50.f, 10000.f, 3);

  /**
   * @note 속도 제어기 초기화
   *       Wsc = 2π×25 rad/s, ζ = 0.707 (임계 감쇠)
   */
  InitSpeedController(&INV, PI2 * 25.f, 0.707f);

  /**
   * @note 전류 제어기 초기화
   *       Wcc = 2π×1000 rad/s
   */
  InitCurrentController(&INV, PI2 * 1000.f);

  /**
   * @note 속도 PLL 옵저버 초기화
   *       Ws = 2π×20 rad/s
   */
  Init_Spd_PLL(&INV, PI2 * 20.f);

  /**
   * @note 확장 EEMF 센서리스 옵저버 초기화
   *       Wc = 2π×200 rad/s
   */
  initExtended_Sensorless_Synchronous_Frame(&EXT_1, PI2 * 200.f, INV.Rs, INV.Ld, INV.Lq);

  /** ADC1 Injected 인터럽트 시작 (TIM2 TRGO 동기) */
  HAL_ADC_Start_IT(&hadc1);
  HAL_ADCEx_InjectedStart_IT(&hadc1);

  /**
   * TIM2 CH2 OC 인터럽트 시작.
   * Pulse=2100, Period=16999 → 100µs 주기로 Control() 호출
   */
  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop -----------------------------------------------------------*/
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 백그라운드 작업 없음. 모든 제어는 TIM2 인터럽트에서 수행 */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief  시스템 클럭 설정
 *
 * HSI(16MHz)를 PLL 소스로 사용하여 SYSCLK = 170MHz를 생성한다.
 *
 * @details PLL 설정:
 * - PLLM = /4  → VCO 입력 = 4MHz
 * - PLLN = ×85 → VCO 출력 = 340MHz
 * - PLLR = /2  → SYSCLK = 170MHz
 * - APB1 = HCLK/2 = 85MHz
 * - APB2 = HCLK/2 = 85MHz
 *
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN            = 85;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) { Error_Handler(); }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  오류 처리 함수
 *
 * HAL API 호출 실패 시 진입하며, 인터럽트를 비활성화하고
 * 무한 루프에서 정지한다.
 * 디버거를 연결하면 이 위치에서 중단점을 통해 원인을 파악할 수 있다.
 *
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) { }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  assert_param 오류 발생 시 소스 파일명과 라인 번호를 보고한다.
 *
 * @param  file  오류가 발생한 소스 파일명 포인터
 * @param  line  오류가 발생한 라인 번호
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */