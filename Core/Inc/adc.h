/**
 * @file    Adc.h
 * @author  lsj50
 * @date    Aug 26, 2025
 * @brief   ADC(Analog-to-Digital Converter) 설정 및 데이터 처리를 위한 헤더 파일
 * @details 인버터의 상전류(Ia, Ib, Ic) 및 직류단 전압(Vdc) 측정을 위한
 * 스케일 상수를 정의하고, 오프셋 계산을 위한 구조체를 포함합니다.
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

/** @brief ADC1에서 사용하는 채널의 총 개수 */
#define ADC1_CHANNEL_NUM		4	//ADC 개수와 동일하게 설정
/** @brief ADC2에서 전류 센싱에 사용하는 채널의 개수 */
#define ADC2_CURRENT_SENSE_NUM  (ADC2_CHANNEL_NUM - 1u)

/** @brief 외부 오프셋 캘리브레이션 단계 상태 정의 */
#define ADC_EXTERNAL_OFFSET_CALIBRATION		0u
/** @brief ADC 원시 값을 실제 물리량(A, V)으로 변환하는 단계 정의 */
#define ADC_GET_SCALED_VALUE				1u

/** * @brief 전류 ADC 스케일링 상수
 * @details 계산식: 1.0 / 센서감도(0.066V/A) * (기준전압(3.3V) / 분해능(4096))
 */
#define SCALE_ADC_CURR			(0.01220703125f)			//1. / 0.066 * (3.3 / 4096)

/** * @brief 직류단 전압(Vdc) ADC 스케일링 상수
 * @details 계산식: (기준전압(3.3V) * 분배저항비(6.1)) / 분해능(4096)
 */
#define SCALE_ADC_VDC			(0.00491455078125f) 		//(3.3 * 6.1) / 4096.0

/** @brief 직류단 전압 정밀 측정을 위한 추가 게인 튜닝 변수 */
#define GAIN_TUNING_ADC_VDC 		(1.0f)

/**
 * @struct sADC1Meas
 * @brief  ADC1을 통해 측정된 각 상전류의 오프셋 값을 저장하는 구조체
 */
typedef struct{

	float fADC1Offset;    /**< ADC1 공통 오프셋 (필요 시 사용) */
	float fIaADC1Offset;  /**< A상 전류 오프셋 측정값 */
	float fIbADC1Offset;  /**< B상 전류 오프셋 측정값 */
	float fIcADC1Offset;  /**< C상 전류 오프셋 측정값 */

}sADC1Meas;

/**
 * @brief  ADC 관련 주변장치 및 변수를 초기화합니다.
 * @retval 없음
 */
extern void vInitAdc(void);

/**
 * @brief  ADC 변환 완료 후 호출되어 데이터를 스케일링하고 오프셋을 제거하는 실시간 처리 함수입니다.
 * @details 주로 ADC DMA 인터럽트 서비스 루틴 혹은 콜백 함수에서 호출됩니다.
 * @retval 없음
 */
extern void vAdcAction(void);


#endif /* INC_ADC_H_ */
