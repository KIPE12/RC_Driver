/**
 * @file    tim.c
 * @brief   타이머(TIM1, TIM2, TIM15) 초기화 및 MSP 설정 모듈
 *
 * PMSM 인버터 제어에 사용되는 세 개의 타이머를 설정한다.
 *
 * @details 타이머 역할 요약:
 * | 타이머 | 역할                          | 주기/주파수             |
 * |--------|-------------------------------|-------------------------|
 * | TIM1   | 3상 PWM 출력 (Center-Aligned) | ARR=4200 → ~20kHz       |
 * | TIM2   | 제어 루프 트리거 + ADC TRGO   | Period=16999 → 100µs    |
 * | TIM15  | 보조 타이머 (RC 입력 등)       | Prescaler=4200, Period=9999 |
 *
 * @section tim1_detail TIM1 상세
 * - Center-Aligned Mode 2 (업/다운 카운트, 다운 시 CCx 인터럽트)
 * - CH1~CH3: 3상 PWM 상단 암 (PA8, PA9, PA10)
 * - CH1N~CH3N: 3상 PWM 하단 암 (PA11, PA12, PB15)
 * - Dead-Time: 35 (≈ 35/170MHz ≈ 206ns)
 * - TRGO2: OC4REF → ADC 동기화
 * - Slave Mode: RESET, 트리거 ITR1 (TIM2 TRGO)
 *
 * @section tim2_detail TIM2 상세
 * - Up Count, Period=16999 → 170MHz/17000 = 10kHz (100µs)
 * - CH2 OC Timing: Pulse=2100 → 제어 루프 인터럽트 발생 시점
 * - TRGO: OC1 → ADC Injected 트리거 소스
 * - Master Mode 활성화 (TIM1 슬레이브 동기)
 *
 * @section tim15_detail TIM15 상세
 * - Prescaler=4200, Period=9999
 * - 저속 보조 타이머 (RC 수신기 PWM 측정 등)
 *
 * @author  STMicroelectronics / HALAB_G
 * @date    2026
 *
 * @attention
 * Copyright (c) 2026 STMicroelectronics. All rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/** @defgroup TIM_Handles 타이머 핸들 전역 변수 */
/** @{ */
TIM_HandleTypeDef htim1;   ///< TIM1 핸들: 3상 PWM 출력
TIM_HandleTypeDef htim2;   ///< TIM2 핸들: 제어 루프 / ADC 트리거
TIM_HandleTypeDef htim15;  ///< TIM15 핸들: 보조 타이머
/** @} */

/**
 * @brief  TIM1 초기화 (3상 Center-Aligned PWM)
 *
 * TIM1을 3상 인버터 PWM 출력용으로 설정한다.
 * 슬레이브 모드(RESET/ITR1)로 TIM2와 동기화되며,
 * TRGO2(OC4REF)로 ADC 변환을 트리거한다.
 *
 * @retval None
 */
void MX_TIM1_Init(void)
{
  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef       sClockSourceConfig  = {0};
  TIM_SlaveConfigTypeDef       sSlaveConfig        = {0};
  TIM_MasterConfigTypeDef      sMasterConfig       = {0};
  TIM_OC_InitTypeDef           sConfigOC           = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */

  /* TIM1 기본 설정 */
  htim1.Instance               = TIM1;
  htim1.Init.Prescaler         = 0;                          ///< 프리스케일러 없음 (170MHz)
  htim1.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED2; ///< Center-Aligned Mode 2
  htim1.Init.Period            = 4200;                       ///< 170MHz / (2×4200) ≈ 20.24kHz
  htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) { Error_Handler(); }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) { Error_Handler(); }

  /* 슬레이브 모드: TIM2 TRGO(ITR1)로 카운터 리셋 */
  sSlaveConfig.SlaveMode    = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK) { Error_Handler(); }

  /* 마스터 출력: TRGO/TRGO2 = OC4REF → ADC 트리거 */
  sMasterConfig.MasterOutputTrigger  = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;
  sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }

  /* CH1~CH3: PWM Mode 1, 초기 듀티 0 */
  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }

  /* Dead-Time 및 Break 설정 */
  sBreakDeadTimeConfig.OffStateRunMode   = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode  = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel         = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime          = 35;  ///< ≈ 206ns @170MHz
  sBreakDeadTimeConfig.BreakState        = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity     = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter       = 0;
  sBreakDeadTimeConfig.BreakAFMode       = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State       = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity    = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter      = 0;
  sBreakDeadTimeConfig.Break2AFMode      = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput   = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) { Error_Handler(); }

  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief  TIM2 초기화 (제어 루프 트리거 / ADC TRGO)
 *
 * TIM2를 10kHz 제어 루프 인터럽트 소스 및 ADC Injected 트리거로 설정한다.
 *
 * @details
 * - Period = 16999: 170MHz / 17000 = 10kHz (100µs 주기)
 * - CH2 Pulse = 2100: 카운터 중간 시점에서 Control() 인터럽트 발생
 * - TRGO = OC1: ADC Injected 채널 외부 트리거
 * - Master Mode 활성화: TIM1 슬레이브와 동기
 *
 * @retval None
 */
void MX_TIM2_Init(void)
{
  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig      = {0};
  TIM_OC_InitTypeDef      sConfigOC          = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */

  htim2.Instance               = TIM2;
  htim2.Init.Prescaler         = 0;                        ///< 170MHz 직접 사용
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = 16999;                    ///< 100µs 주기
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }

  if (HAL_TIM_OC_Init(&htim2) != HAL_OK) { Error_Handler(); }

  /* TRGO = OC1 → ADC Injected 트리거 */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }

  /* CH2: OC Timing 모드, Pulse=2100 → 인터럽트 발생 시점 */
  sConfigOC.OCMode     = TIM_OCMODE_TIMING;
  sConfigOC.Pulse      = 2100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }

  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief  TIM15 초기화 (보조 타이머)
 *
 * RC 수신기 PWM 측정 등 저속 보조 목적으로 사용되는 타이머를 설정한다.
 *
 * @details
 * - Prescaler = 4200: 170MHz / 4201 ≈ 40.47kHz 타이머 클럭
 * - Period    = 9999: 약 247ms 주기 (오버플로우 기준)
 *
 * @retval None
 */
void MX_TIM15_Init(void)
{
  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig      = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */

  htim15.Instance               = TIM15;
  htim15.Init.Prescaler         = 4200;
  htim15.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim15.Init.Period            = 9999;
  htim15.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK) { Error_Handler(); }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK) { Error_Handler(); }

  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
}

/**
 * @brief  TIM MSP 초기화 (클럭 활성화 및 NVIC 설정)
 *
 * HAL_TIM_Base_Init() 내부에서 자동 호출된다.
 * 각 타이머 인스턴스에 대해 클럭을 활성화하고 인터럽트 우선순위를 설정한다.
 *
 * | 타이머 | IRQ                    | 우선순위 |
 * |--------|------------------------|----------|
 * | TIM1   | TIM1_BRK_TIM15_IRQn   | 0, 0     |
 * | TIM2   | TIM2_IRQn              | 0, 0     |
 * | TIM15  | TIM1_BRK_TIM15_IRQn   | 0, 0     |
 *
 * @param[in] tim_baseHandle 초기화 중인 타이머 핸들 포인터
 * @retval None
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_baseHandle)
{
  if (tim_baseHandle->Instance == TIM1)
  {
    /* USER CODE BEGIN TIM1_MspInit 0 */
    /* USER CODE END TIM1_MspInit 0 */
    __HAL_RCC_TIM1_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
    /* USER CODE BEGIN TIM1_MspInit 1 */
    /* USER CODE END TIM1_MspInit 1 */
  }
  else if (tim_baseHandle->Instance == TIM2)
  {
    /* USER CODE BEGIN TIM2_MspInit 0 */
    /* USER CODE END TIM2_MspInit 0 */
    __HAL_RCC_TIM2_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    /* USER CODE BEGIN TIM2_MspInit 1 */
    /* USER CODE END TIM2_MspInit 1 */
  }
  else if (tim_baseHandle->Instance == TIM15)
  {
    /* USER CODE BEGIN TIM15_MspInit 0 */
    /* USER CODE END TIM15_MspInit 0 */
    __HAL_RCC_TIM15_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
    /* USER CODE BEGIN TIM15_MspInit 1 */
    /* USER CODE END TIM15_MspInit 1 */
  }
}

/**
 * @brief  TIM MSP 초기화 후처리 (GPIO 대체 기능 설정)
 *
 * HAL_TIM_PWM_Init() 완료 후 자동 호출된다.
 * TIM1에 대해 PWM 출력 핀의 GPIO 대체 기능(AF)을 설정한다.
 *
 * @details TIM1 GPIO 핀 매핑:
 * | 핀   | 채널    | AF          | 설명        |
 * |------|---------|-------------|-------------|
 * | PB15 | CH3N   | GPIO_AF4_TIM1 | C상 하단 암 |
 * | PA8  | CH1    | GPIO_AF6_TIM1 | A상 상단 암 |
 * | PA9  | CH2    | GPIO_AF6_TIM1 | B상 상단 암 |
 * | PA10 | CH3    | GPIO_AF6_TIM1 | C상 상단 암 |
 * | PA11 | CH1N   | GPIO_AF6_TIM1 | A상 하단 암 |
 * | PA12 | CH2N   | GPIO_AF6_TIM1 | B상 하단 암 |
 *
 * @param[in] timHandle 초기화 완료된 타이머 핸들 포인터
 * @retval None
 */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (timHandle->Instance == TIM1)
  {
    /* USER CODE BEGIN TIM1_MspPostInit 0 */
    /* USER CODE END TIM1_MspPostInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PB15 → TIM1_CH3N (AF4) */
    GPIO_InitStruct.Pin       = GPIO_PIN_15;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PA8~PA12 → TIM1_CH1~CH3, CH1N~CH2N (AF6) */
    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10
                              | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM1_MspPostInit 1 */
    /* USER CODE END TIM1_MspPostInit 1 */
  }
}

/**
 * @brief  TIM MSP 해제 (클럭 비활성화 및 NVIC 해제)
 *
 * HAL_TIM_Base_DeInit() 내부에서 자동 호출된다.
 *
 * @param[in] tim_baseHandle 해제 중인 타이머 핸들 포인터
 * @retval None
 */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *tim_baseHandle)
{
  if (tim_baseHandle->Instance == TIM1)
  {
    /* USER CODE BEGIN TIM1_MspDeInit 0 */
    /* USER CODE END TIM1_MspDeInit 0 */
    __HAL_RCC_TIM1_CLK_DISABLE();
    /* TIM1_BRK_TIM15_IRQn은 TIM15와 공유하므로 주의하여 비활성화 */
    /* USER CODE BEGIN TIM1_MspDeInit 1 */
    /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if (tim_baseHandle->Instance == TIM2)
  {
    /* USER CODE BEGIN TIM2_MspDeInit 0 */
    /* USER CODE END TIM2_MspDeInit 0 */
    __HAL_RCC_TIM2_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
    /* USER CODE BEGIN TIM2_MspDeInit 1 */
    /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if (tim_baseHandle->Instance == TIM15)
  {
    /* USER CODE BEGIN TIM15_MspDeInit 0 */
    /* USER CODE END TIM15_MspDeInit 0 */
    __HAL_RCC_TIM15_CLK_DISABLE();
    /* TIM1_BRK_TIM15_IRQn은 TIM1과 공유하므로 주의하여 비활성화 */
    /* USER CODE BEGIN TIM15_MspDeInit 1 */
    /* USER CODE END TIM15_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
