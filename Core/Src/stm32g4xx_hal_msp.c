/**
 * @file    stm32g4xx_hal_msp.c
 * @brief   HAL MSP(MCU Support Package) 초기화 및 해제 모듈
 *
 * HAL 라이브러리가 내부적으로 호출하는 MSP 콜백 함수를 정의한다.
 * MCU에 종속적인 저수준 초기화(클럭, GPIO, NVIC 등)를 수행한다.
 *
 * @note 각 주변장치별 MSP 함수(HAL_ADC_MspInit, HAL_TIM_Base_MspInit 등)는
 *       해당 주변장치 파일(adc.c, tim.c)에 위치하며, 이 파일에는
 *       전역 MSP 초기화(HAL_MspInit)만 정의한다.
 *
 * @author  STMicroelectronics / HALAB_G
 * @date    2026
 *
 * @attention
 * Copyright (c) 2026 STMicroelectronics. All rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  전역 MSP 초기화 (HAL_Init() 내부에서 호출)
 *
 * 모든 주변장치에 공통으로 적용되는 저수준 초기화를 수행한다:
 * - SYSCFG 및 PWR 클럭 활성화
 * - UCPD Dead Battery 핀 Pull-Down 비활성화
 *   (UCPD를 사용하지 않는 경우 데드 배터리 모드 방지)
 *
 * @retval None
 */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* UCPD Dead Battery 핀 내부 Pull-Down 비활성화 */
  HAL_PWREx_DisableUCPDDeadBattery();

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
