/**
 * @file    IntDac.h
 * @author  lsj50
 * @date    Sep 9, 2025
 * @brief   MCU 내부 DAC(Digital-to-Analog Converter) 제어를 위한 헤더 파일
 * @details 실시간 제어 변수(전류, 속도 등)를 오실로스코프로 관측하기 위해
 * 내부 DAC를 통해 아날로그 전압으로 출력하는 기능을 정의합니다.
 */

#ifndef INC_INTDAC_H_
#define INC_INTDAC_H_

/** @brief DAC의 기준 전압 (Reference Voltage) [V] */
#define VREF                3.3f

/** @brief DAC의 최대 해상도 (12-bit: 4096) */
#define DAC_CODES_FULL      4096.0f

/** @brief 전압당 DAC 코드 변환 상수 (Codes/Volt) */
#define DAC_CODES_PER_VOLT  (DAC_CODES_FULL / VREF)

/**
 * @brief  내부 DAC 주변장치를 초기화하고 출력 채널을 설정합니다.
 * @retval 없음
 */
void vInitIntDac();

/**
 * @brief  지정된 변수를 DAC 레지스터에 써서 아날로그 전압으로 출력합니다.
 * @details 모니터링이 필요한 변수를 스케일링하여 실시간 루프 내에서 호출됩니다.
 * @retval 없음
 */
extern void vIntDacOut();

#endif /* INC_INTDAC_H_ */
