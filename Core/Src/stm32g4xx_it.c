/**
 * @file    stm32g4xx_it.c
 * @brief   STM32G4xx 인터럽트 서비스 루틴(ISR) 모듈
 *
 * Cortex-M4 예외 핸들러 및 STM32G4xx 주변장치 인터럽트 핸들러를 정의한다.
 *
 * @details 사용 중인 주요 인터럽트:
 * | 인터럽트                    | 핸들러                    | 동작                         |
 * |-----------------------------|---------------------------|------------------------------|
 * | ADC1/ADC2                   | ADC1_2_IRQHandler         | HAL ADC 처리                 |
 * | TIM1 Break / TIM15          | TIM1_BRK_TIM15_IRQHandler | HAL TIM 처리                 |
 * | TIM2                        | TIM2_IRQHandler           | 카운터 증가 + **Control()** 호출 |
 * | SysTick                     | SysTick_Handler           | HAL 틱 카운터 증가           |
 *
 * @note TIM2_IRQHandler 내에서 Control()을 직접 호출한다.
 *       TIM2 주기(100µs)가 곧 제어 샘플링 주기(Tsamp)이다.
 *
 * @author  STMicroelectronics / HALAB_G
 * @date    2026
 *
 * @attention
 * Copyright (c) 2026 STMicroelectronics. All rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"

/* USER CODE BEGIN Includes */
#include "control.h"
#include "inv.h"
#include "variable.h"
#include "speed_observer.h"
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint32_t risingVal;   ///< RC 입력 캡처: 상승 에지 타임스탬프
extern uint32_t fallingVal;  ///< RC 입력 캡처: 하강 에지 타임스탬프
extern uint32_t highTime;    ///< RC 입력: 하이 구간 폭 [타이머 카운트]
extern uint32_t period;      ///< RC 입력: 전체 주기 [타이머 카운트]
extern float    dutyCycle;   ///< RC 입력 듀티 사이클 [0.0, 1.0]
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;   ///< ADC1 핸들 (adc.c에서 정의)
extern TIM_HandleTypeDef htim1;   ///< TIM1 핸들 (tim.c에서 정의)
extern TIM_HandleTypeDef htim2;   ///< TIM2 핸들 (tim.c에서 정의)
extern TIM_HandleTypeDef htim15;  ///< TIM15 핸들 (tim.c에서 정의)

/* USER CODE BEGIN EV */
/**
 * @brief TIM2 인터럽트 발생 횟수 카운터 (디버깅용)
 *
 * 누산 오버플로우는 uint32_t 범위 내에서 자연적으로 처리된다.
 */
uint32_t tim2_cnt;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 프로세서 예외 핸들러                                   */
/******************************************************************************/

/**
 * @brief  Non-Maskable Interrupt(NMI) 핸들러
 * @retval None
 */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1) { }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief  Hard Fault 핸들러
 *
 * 잘못된 메모리 접근, 정렬 오류, 버스 오류 등 치명적 예외 발생 시 진입한다.
 * @retval None
 */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
 * @brief  Memory Management Fault 핸들러
 *
 * MPU 보호 위반 시 진입한다.
 * @retval None
 */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
 * @brief  Bus Fault 핸들러
 *
 * 프리페치 실패, 메모리 접근 오류 시 진입한다.
 * @retval None
 */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
 * @brief  Usage Fault 핸들러
 *
 * 미정의 명령어, 잘못된 상태 등 사용 오류 발생 시 진입한다.
 * @retval None
 */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
 * @brief  SVC(SuperVisor Call) 핸들러
 *
 * SWI 명령어로 발생하는 소프트웨어 인터럽트를 처리한다.
 * RTOS 사용 시 커널 진입점으로 활용된다.
 * @retval None
 */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief  Debug Monitor 핸들러
 * @retval None
 */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief  PendSV 핸들러
 *
 * 소프트웨어 트리거 가능한 지연 예외 처리.
 * RTOS 컨텍스트 스위칭에 주로 사용된다.
 * @retval None
 */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief  SysTick 타이머 핸들러
 *
 * HAL_IncTick()을 호출하여 HAL 내부 ms 틱 카운터를 증가시킨다.
 * HAL_Delay() 및 타임아웃 관련 함수의 기준 클럭이다.
 * @retval None
 */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/*           STM32G4xx 주변장치 인터럽트 핸들러                               */
/******************************************************************************/

/**
 * @brief  ADC1 및 ADC2 글로벌 인터럽트 핸들러
 *
 * Regular 또는 Injected 변환 완료 시 호출된다.
 * HAL_ADC_IRQHandler()를 통해 콜백(HAL_ADCEx_InjectedConvCpltCallback 등)이
 * 연결된다.
 *
 * @retval None
 */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
 * @brief  TIM1 Break 및 TIM15 글로벌 인터럽트 핸들러
 *
 * TIM1과 TIM15는 IRQ를 공유한다.
 * 두 핸들 모두에 HAL_TIM_IRQHandler()를 호출하여 각자의 플래그를 처리한다.
 *
 * @retval None
 */
void TIM1_BRK_TIM15_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM15_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim15);
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM15_IRQn 1 */
}

/**
 * @brief  TIM2 글로벌 인터럽트 핸들러 ★ 제어 루프 진입점 ★
 *
 * TIM2 CH2 OC(Output Compare) 인터럽트가 100µs 주기로 발생한다.
 * HAL 처리 후 **Control()** 을 직접 호출하여 PMSM 제어 루프를 실행한다.
 *
 * @details 실행 순서:
 * 1. @c tim2_cnt 카운터 증가 (디버깅/모니터링용)
 * 2. HAL_TIM_IRQHandler() → 플래그 클리어 및 콜백 처리
 * 3. Control() → ADC 읽기, 좌표 변환, PI 제어, PWM 출력
 *
 * @warning Control()의 실행 시간이 100µs를 초과하면 제어 주기 위반이 발생한다.
 *
 * @retval None
 */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  tim2_cnt++;  ///< 디버깅용 인터럽트 카운터 증가
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  Control();   ///< PMSM 메인 제어 루프 실행 (100µs 주기)
  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
