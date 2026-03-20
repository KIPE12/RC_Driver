/**
 * @file    Adc.c
 * @author  lsj50
 * @date    Aug 26, 2025
 * @brief   ADC 초기화, 외부 오프셋 캘리브레이션 및 ADC 데이터 스케일링을 처리하는 소스 파일
 */

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_adc.h"
#include "Globalvar.h"
#include "MotorControl.h"

/** @brief ADC 상태 전이를 관리하는 상태 변수 */
static uint16_t uCurrAdcState = ADC_EXTERNAL_OFFSET_CALIBRATION, uNextAdcState = ADC_EXTERNAL_OFFSET_CALIBRATION;

/** @brief ADC1 DMA 변환 결과가 저장되는 버퍼 */
volatile uint16_t uADC1Result[ADC1_CHANNEL_NUM];

/** @brief ADC 오프셋 캘리브레이션을 위한 카운터 변수들 */
static uint16_t uAdcOffsetCnt = 0u;       /**< 현재 오프셋 측정 카운트 */
static uint16_t uAdcStandbyCnt = 1000u;   /**< ADC 주변장치 안정화를 위한 대기 카운트 */
static uint16_t uAdcOffsetCntMax = 5000u; /**< 오프셋 값을 누적할 최대 횟수 (예: 0.5s) */

/** @brief ADC1 결과 임계값 (현재 미사용 또는 외부 참조용) */
uint16_t uADC1ResultTresh = 0u;

/** @brief ADC 스케일링 함수 호출 횟수를 누적하는 카운터 */
uint16_t uADCCnt = 0u;

/** @brief 메인 소스(또는 다른 파일)에서 정의된 ADC1 핸들러 외부 참조 */
extern ADC_HandleTypeDef hadc1;


/**
 * @brief  ADC 주변장치를 초기화하고 DMA를 통한 변환을 시작합니다.
 * @note   ADC 활성화 후 Single-Ended 모드로 내부 캘리브레이션을 수행하고,
 * uADC1Result 버퍼로 DMA 수신을 시작합니다.
 * @param  없음
 * @retval 없음
 */
void vInitAdc(void){
	ADC_Enable(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)uADC1Result, ADC1_CHANNEL_NUM);
}

/**
 * @brief  전류 센서의 외부 ADC 오프셋을 측정하고 평균값을 계산합니다.
 * @note   초기 안정화를 위해 일정 횟수 대기한 후, 지정된 횟수(uAdcOffsetCntMax)만큼
 * ADC 변환 값을 누적하여 평균 오프셋 수치를 도출합니다. 완료 시 다음 상태로 전이합니다.
 * @param  없음
 * @retval 없음
 */
void vAdcOffsetCalibration(void){
	if(uAdcOffsetCnt < uAdcStandbyCnt){		// Standby for ADC Peripheral on
		uAdcOffsetCnt ++;
	}else if(uAdcOffsetCnt < uAdcStandbyCnt + uAdcOffsetCntMax){		// ADC Offset Integration
		INV.ADC1Meas.fIaADC1Offset += uADC1Result[0];
		INV.ADC1Meas.fIbADC1Offset += uADC1Result[1];
		INV.ADC1Meas.fIcADC1Offset += uADC1Result[2];
		uAdcOffsetCnt ++;
	}else{
		INV.ADC1Meas.fIaADC1Offset = INV.ADC1Meas.fIaADC1Offset / (float)(uAdcOffsetCntMax);		// ADC Offset Calibration
		INV.ADC1Meas.fIbADC1Offset = INV.ADC1Meas.fIbADC1Offset / (float)(uAdcOffsetCntMax);
		INV.ADC1Meas.fIcADC1Offset = INV.ADC1Meas.fIcADC1Offset / (float)(uAdcOffsetCntMax);

		uAdcOffsetCnt = 0u;
		uNextAdcState = ADC_GET_SCALED_VALUE;
	}
}

/** @brief ADC 스케일링 시 미세 조정을 위한 튜닝 변수 */
float fIoAdcTun = 0.0f;

/**
 * @brief  원시(Raw) ADC 데이터를 실제 물리량(전류 및 DC 링크 전압)으로 변환합니다.
 * @note   이전에 계산된 오프셋을 차감한 뒤 전류 스케일 팩터를 곱하여 3상 전류 값을 계산합니다.
 * 전압의 경우 역수(fInvVdc)도 함께 계산하여 연산 효율을 높입니다.
 * @param  없음
 * @retval 없음
 */
void vScaleAdcValue(void){
	INV.CC.fIasHall = SCALE_ADC_CURR * ((float)(uADC1Result[0]) - INV.ADC1Meas.fIaADC1Offset) + 10e-3 * fIoAdcTun;
	INV.CC.fIbsHall = SCALE_ADC_CURR * ((float)(uADC1Result[1]) - INV.ADC1Meas.fIbADC1Offset) + 10e-3 * fIoAdcTun;
	INV.CC.fIcsHall = SCALE_ADC_CURR * ((float)(uADC1Result[2]) - INV.ADC1Meas.fIcADC1Offset) + 10e-3 * fIoAdcTun;

	fVdc = GAIN_TUNING_ADC_VDC * SCALE_ADC_VDC * (float)(uADC1Result[3]);
	//fVdc = 16.0f; // Unable to use PA2 in Launch Pad (NUCLEO-G474RE)

	if (fVdc < 1.)      fInvVdc = 1.;
	else                fInvVdc = 1. / fVdc;

	uADCCnt ++;
}

/**
 * @brief  ADC 상태 머신을 구동합니다.
 * @note   현재 상태(uCurrAdcState)에 따라 오프셋 캘리브레이션 또는
 * 스케일링 동작 중 알맞은 함수를 분기하여 실행합니다. 주기적인 타이머 인터럽트 등에서 호출됩니다.
 * @param  없음
 * @retval 없음
 */
void vAdcAction(void){
	uCurrAdcState = uNextAdcState;
	switch(uCurrAdcState){
	case ADC_EXTERNAL_OFFSET_CALIBRATION:
		vAdcOffsetCalibration();
		break;

	default: //case ADC_GET_SCALED_VALUE:
		vScaleAdcValue();
		break;
	}
}
