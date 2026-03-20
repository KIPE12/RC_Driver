/**
 * @file    IntDac.c
 * @author  lsj50
 * @date    2026. 3. 11.
 * @brief   MCU 내부 DAC를 이용한 실시간 모니터링 출력 구현 소스 파일
 * @details 제어 루프 내의 주요 변수(속도, 전류 등)를 아날로그 전압으로 출력하여
 * 오실로스코프 등을 통해 실시간으로 파형을 관측할 수 있도록 합니다.
 */

#include "Globalvar.h"
#include "MotorControl.h"
#include "IntDac.h"

/** @brief 테스트용 내부 변수 */
float fTest_Int = 0.0f;

/** @brief DAC 채널별 출력 스케일링 계수 배열 (기본값: 1500 RPM 기준 스케일링) */
float fIntDacScale[3] = {DAC_CODES_PER_VOLT / (1500.0f), DAC_CODES_PER_VOLT /(1500.0f), DAC_CODES_PER_VOLT / (1500.0f)};

/** @brief DAC 출력을 위해 참조할 데이터의 주소 포인터 배열 */
uint16_t *uIntDacDatAddr[3] = {0u, 0u, 0u};

/** @brief DAC 출력 데이터의 타입(모드) 설정 배열 */
uint16_t uIntDacDatType[3] = {0u, 0u, 0u};

/** @brief STM32 HAL DAC1 핸들러 외칭 참조 */
extern DAC_HandleTypeDef hdac1;
/** @brief STM32 HAL DAC2 핸들러 외칭 참조 */
extern DAC_HandleTypeDef hdac2;

/**
 * @brief  내부 DAC1, DAC2의 각 채널을 초기화하고 시작합니다.
 * @details
 * - 트리거 없음(None), 출력 버퍼 활성화(Enable) 설정
 * - DAC1 채널 1, 2 및 DAC2 채널 1을 시작하여 총 3채널 모니터링 준비
 * @retval 없음
 */
void vInitIntDac(){

	DAC_ChannelConfTypeDef sConfig = {0};

	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;

	HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

	HAL_DAC_ConfigChannel(&hdac2, &sConfig, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac2, DAC_CHANNEL_1);

	HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
}

/**
 * @brief  실시간 루프에서 호출되어 설정된 변수를 DAC 데이터 레지스터에 기록합니다.
 * @details
 * - 각 채널별로 타입(uIntDacDatType)에 따라 추정 속도 또는 사용자 지정 변수를 출력합니다.
 * - 12비트 DAC의 중간값인 2048(1.65V)을 오프셋으로 더하여 양/음의 신호를 모두 표현할 수 있습니다.
 * - 레지스터(DHR12R1, DHR12R2)에 직접 접근하여 처리 속도를 최적화하였습니다.
 * @retval 없음
 */
void vIntDacOut(){
	///Channel_1 (DAC1_CH1)
	if(uIntDacDatType[0] == 0){
		DAC1->DHR12R1 = (uint16_t)(fIntDacScale[0] * (INV.SO.fWrpmSC) + 2048u);
	}else{
		DAC1->DHR12R1 =(uint16_t)(fIntDacScale[0] * (*(int *)(uIntDacDatAddr[0])) + 2048u);
	}

	///Channel_2 (DAC1_CH2)
	if(uIntDacDatType[1] == 0){
		DAC1->DHR12R2 = (uint16_t)(fIntDacScale[1] * (INV.SO.fWrpmSC) + 2048u);
	}else{
		DAC1->DHR12R2 =(uint16_t)(fIntDacScale[1] * (*(int *)(uIntDacDatAddr[1])) + 2048u);
	}

	///Channel_3 (DAC2_CH1)
	if(uIntDacDatType[2] == 0){
		DAC2->DHR12R1 = (uint16_t)(fIntDacScale[2] * (INV.SO.fWrpmSC) + 2048u);
	}else{
		DAC2->DHR12R1 =(uint16_t)(fIntDacScale[2] * (*(int *)(uIntDacDatAddr[2])) + 2048u);
	}
}
