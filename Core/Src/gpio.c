/**
 * @file    gpio.c
 * @brief   GPIO 핀 초기화 모듈
 *
 * 인버터 제어에 사용되는 모든 GPIO 핀을 설정한다.
 *
 * @details 핀 배치:
 * | 핀          | 방향   | 기능               | 초기값    |
 * |-------------|--------|--------------------|-----------|
 * | PWM_EN_Pin  | Output | DRV8300 /OE 제어   | HIGH (활성) |
 * | HALL_A_Pin  | Input  | Hall 센서 A (GPIOC) | Pull-Up  |
 * | HALL_B_Pin  | Input  | Hall 센서 B (GPIOC) | Pull-Up  |
 * | HALL_C_Pin  | Input  | Hall 센서 C         | Pull-Up  |
 *
 * @note PWM_EN_Pin을 HIGH로 초기화하면 DRV8300의 출력 버퍼가 활성화된다.
 *       LOW로 설정하면 PWM 출력이 Hi-Z 상태가 된다.
 *
 * @author  STMicroelectronics / HALAB_G
 * @date    2026
 *
 * @attention
 * Copyright (c) 2026 STMicroelectronics. All rights reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
 * @brief  사용되는 모든 GPIO 핀을 초기화한다.
 *
 * 다음 순서로 초기화를 수행한다:
 * 1. GPIOA/B/C/D 클럭 활성화
 * 2. PWM_EN_Pin → Push-Pull 출력, 초기값 HIGH
 * 3. HALL_A_Pin, HALL_B_Pin → 디지털 입력, Pull-Up (GPIOC)
 * 4. HALL_C_Pin → 디지털 입력, Pull-Up (HALL_C_GPIO_Port)
 *
 * @retval None
 */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO 포트 클럭 활성화 */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* PWM_EN_Pin 초기 출력 레벨: HIGH (인버터 게이트 드라이버 활성) */
  HAL_GPIO_WritePin(PWM_EN_GPIO_Port, PWM_EN_Pin, GPIO_PIN_SET);

  /* PWM_EN_Pin: Push-Pull 출력, No Pull, Low Speed */
  GPIO_InitStruct.Pin   = PWM_EN_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWM_EN_GPIO_Port, &GPIO_InitStruct);

  /* HALL_A_Pin, HALL_B_Pin: 디지털 입력, 내부 Pull-Up */
  GPIO_InitStruct.Pin  = HALL_A_Pin | HALL_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* HALL_C_Pin: 디지털 입력, 내부 Pull-Up */
  GPIO_InitStruct.Pin  = HALL_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HALL_C_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
