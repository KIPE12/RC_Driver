/**
 * @file    adc.c
 * @brief   ADC1 초기화 및 전류/전압 측정 처리 모듈
 *
 * STM32G4xx의 ADC1을 사용하여 3상 전류(Ia, Ib, Ic) 및
 * DC 링크 전압(Vdc)을 측정한다.
 * Injected 채널 방식으로 TIM2 TRGO에 동기화되어 샘플링된다.
 *
 * @details 하드웨어 연결:
 * | ADC 채널 | GPIO | 측정 대상 | 센서              |
 * |----------|------|-----------|-------------------|
 * | CH1      | PA0  | Ia        | ACS725 (66mV/A)   |
 * | CH2      | PA1  | Ib        | ACS725 (66mV/A)   |
 * | CH3      | PA2  | Ic        | ACS725 (66mV/A)   |
 * | CH4      | PA3  | Vdc       | 저항 분배기        |
 *
 * @details 물리량 변환 공식:
 * - 전류: I[A]   = (raw - offset) / 81.9
 * - 전압: Vdc[V] = raw / 203.4
 *
 * @author  HALAB_G
 * @date    2026
 */

#define ADC12BIT  4095.1f  ///< 12비트 ADC 최대값 (풀스케일)
#define ADC_RESOL ADC12BIT ///< ADC 분해능 설정값

#include "adc.h"
#include "stm32g4xx_it.h"
#include "inv.h"

/** @defgroup ADC_Variables ADC 전역 변수
 *  @{
 */

int AdInitFlag = 0; ///< ADC 오프셋 보정 완료 플래그 (0: 미완료, 1: 완료)

float adc1Val[4]     = {0.f, 0.f, 0.f, 0.f}; ///< ADC1 Injected 채널 원시값 [Ia, Ib, Ic, Vdc]
float ADC1_Result[4] = {0.f, 0.f, 0.f, 0.f}; ///< 오프셋/스케일 적용 후 변환값

float ADC1_Offset[3]             = {2048.f, 2048.f, 2048.f}; ///< 전류 채널 오프셋 (12bit 중간값 기본)
unsigned long ADC1_Offset_sum[3] = {0, 0, 0};                ///< 오프셋 누산 버퍼

float scale_comp = 1.0f; ///< ADC 스케일링 보정 계수

/**
 * @brief ADC 채널별 게인 보정 배열
 * - [0]: Ia 게인, [1]: Ib 게인, [2]: Ic 게인, [3]: Vdc 게인
 */
float ADCgain[4] = {1.0f, 1.0f, 1.0f, 1.0f};

float        Ia_arr[3000]; ///< Ia 데이터 저장 버퍼 (디버깅용, 최대 3000샘플)
int          store_cnt  = 0; ///< Ia_arr 저장 인덱스
int          store_flag = 0; ///< 데이터 저장 활성화 플래그 (1: 활성)
unsigned int AdOffCalcCnt = 0; ///< 오프셋 평균 계산 카운터

/** @} */

ADC_HandleTypeDef hadc1; ///< ADC1 HAL 핸들 구조체

/**
 * @brief  ADC1 초기화 (12비트, Injected 4채널, TIM2 TRGO 트리거)
 *
 * ADC1을 다음 조건으로 설정한다:
 * - 12비트 분해능, 우측 정렬
 * - Injected 4채널 (CH1~CH4): TIM2 TRGO 상승에지 외부 트리거
 * - CH1~CH3: 2.5 사이클 샘플링 (전류)
 * - CH4:    47.5 사이클 샘플링 (Vdc)
 *
 * @retval None
 */
void MX_ADC1_Init(void)
{
    ADC_MultiModeTypeDef     multimode       = {0};
    ADC_ChannelConfTypeDef   sConfig         = {0};
    ADC_InjectionConfTypeDef sConfigInjected = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.GainCompensation      = 0;
    hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait      = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode      = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }

    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) { Error_Handler(); }

    /* Regular CH1 (초기화만 수행) */
    sConfig.Channel      = ADC_CHANNEL_1;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

    /* Injected CH1~CH3 (Ia/Ib/Ic): TIM2 TRGO 트리거 */
    sConfigInjected.InjectedChannel               = ADC_CHANNEL_1;
    sConfigInjected.InjectedRank                  = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedSamplingTime          = ADC_SAMPLETIME_2CYCLES_5;
    sConfigInjected.InjectedSingleDiff            = ADC_SINGLE_ENDED;
    sConfigInjected.InjectedOffsetNumber          = ADC_OFFSET_NONE;
    sConfigInjected.InjectedOffset                = 0;
    sConfigInjected.InjectedNbrOfConversion       = 4;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.AutoInjectedConv              = DISABLE;
    sConfigInjected.QueueInjectedContext          = DISABLE;
    sConfigInjected.ExternalTrigInjecConv         = ADC_EXTERNALTRIGINJEC_T2_TRGO;
    sConfigInjected.ExternalTrigInjecConvEdge     = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    sConfigInjected.InjecOversamplingMode         = DISABLE;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) { Error_Handler(); }

    sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
    sConfigInjected.InjectedRank    = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) { Error_Handler(); }

    sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
    sConfigInjected.InjectedRank    = ADC_INJECTED_RANK_3;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) { Error_Handler(); }

    /* Injected CH4 (Vdc): 47.5 사이클 샘플링 */
    sConfigInjected.InjectedChannel      = ADC_CHANNEL_4;
    sConfigInjected.InjectedRank         = ADC_INJECTED_RANK_4;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) { Error_Handler(); }
}

/**
 * @brief  ADC1 MSP 초기화 (GPIO, 클럭, NVIC 설정)
 *
 * PA0~PA3를 아날로그 입력으로 구성하고 ADC1 인터럽트를 활성화한다.
 *
 * @param[in] adcHandle 초기화 중인 ADC 핸들 포인터
 * @retval None
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
{
    GPIO_InitTypeDef         GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit  = {0};

    if (adcHandle->Instance == ADC1)
    {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
        PeriphClkInit.Adc12ClockSelection  = RCC_ADC12CLKSOURCE_SYSCLK;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }

        __HAL_RCC_ADC12_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* PA0=Ia, PA1=Ib, PA2=Ic, PA3=Vdc: 아날로그 입력 */
        GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    }
}

/**
 * @brief  ADC1 MSP 해제
 * @param[in] adcHandle 해제 중인 ADC 핸들 포인터
 * @retval None
 */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle)
{
    if (adcHandle->Instance == ADC1)
    {
        __HAL_RCC_ADC12_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
        HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
    }
}

/**
 * @brief  ADC 원시값을 물리량(전류/전압)으로 변환한다.
 *
 * Injected 레지스터(JDR1~JDR4) 값에 오프셋 제거 및 스케일을 적용하고
 * INV 구조체의 Ia, Ib, Ic, Vdc 값을 갱신한다.
 * Vdc에는 1차 IIR 저역통과 필터(α = 0.999)를 적용한다.
 *
 * @note AdInitFlag == 1 이후에만 Control()에서 호출된다.
 * @retval None
 */
void AdcProcess(void)
{
    ADC1_Result[0] = (adc1Val[0] - ADC1_Offset[0]) / 81.9f;
    ADC1_Result[1] = (adc1Val[1] - ADC1_Offset[1]) / 81.9f;
    ADC1_Result[2] = (adc1Val[2] - ADC1_Offset[2]) / 81.9f;
    ADC1_Result[3] =  adc1Val[3]                   / 203.4f;

    INV.Ia  = ADCgain[0] * ADC1_Result[0] * scale_comp;
    INV.Ib  = ADCgain[1] * ADC1_Result[1] * scale_comp;
    INV.Ic  = ADCgain[2] * ADC1_Result[2] * scale_comp;
    INV.Vdc = ADCgain[3] * ADC1_Result[3] * scale_comp;

    const float alpha = 0.999f;
    INV.Vdc_control   = INV.Vdc * (1.0f - alpha) + alpha * INV.Vdc_control;
    INV.Vdc           = INV.Vdc_control;
    INV.INV_Vdc       = 1.0f / MAX(INV.Vdc_control, 1.f);

    if (store_flag == 1) {
        if (store_cnt < 3000) Ia_arr[store_cnt++] = INV.Ia;
        else                  store_flag = 0;
    }
}

/**
 * @brief  전류 센서 오프셋을 자동 측정한다.
 *
 * 인버터 출력이 없는 상태에서 5000회 더미 대기 후 5000회 누산하여
 * ADC1_Offset[]을 갱신하고 AdInitFlag = 1로 설정한다.
 * 총 소요 시간: (5000 + 5000) × 100µs = 1초.
 *
 * @note AdInitFlag == 0 동안 Control()에서 매 주기 호출된다.
 * @retval None
 */
void Offset(void)
{
    static unsigned int AdDummyCnt = 0;

    if (AdDummyCnt < 5000) {
        AdDummyCnt++;
    } else {
        AdOffCalcCnt++;

        ADC1_Offset_sum[0] += (unsigned long)adc1Val[0];
        ADC1_Offset_sum[1] += (unsigned long)adc1Val[1];
        ADC1_Offset_sum[2] += (unsigned long)adc1Val[2];

        if (AdOffCalcCnt == 5000) {
            ADC1_Offset[0] = (float)ADC1_Offset_sum[0] / 5000.f;
            ADC1_Offset[1] = (float)ADC1_Offset_sum[1] / 5000.f;
            ADC1_Offset[2] = (float)ADC1_Offset_sum[2] / 5000.f;
            AdInitFlag = 1;
        }
    }
}
